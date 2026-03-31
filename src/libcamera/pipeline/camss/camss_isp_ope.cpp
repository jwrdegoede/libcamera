/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Qualcomm CAMSS OPE ISP class
 *
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include "camss_isp_ope.h"

#include <string.h>

#include <libcamera/base/log.h>

#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera_manager.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/dma_buf_allocator.h"
#include "libcamera/internal/framebuffer.h" // FIXME drop once softIPA (ab)use is gone
#include "libcamera/internal/ipa_manager.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/software_isp/debayer_params.h"
#include "libcamera/internal/software_isp/swstats_cpu.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "camss_frames.h"
#include "camss_params.h"
#include "camss_util.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(Camss)

namespace {

static const std::map<PixelFormat, uint32_t> inputFormatToMediaBus = {
	{ formats::SBGGR8, MEDIA_BUS_FMT_SBGGR8_1X8 },
	{ formats::SGBRG8, MEDIA_BUS_FMT_SGBRG8_1X8 },
	{ formats::SGRBG8, MEDIA_BUS_FMT_SGRBG8_1X8 },
	{ formats::SRGGB8, MEDIA_BUS_FMT_SRGGB8_1X8 },
	{ formats::SBGGR10_CSI2P, MEDIA_BUS_FMT_SBGGR10_1X10 },
	{ formats::SGBRG10_CSI2P, MEDIA_BUS_FMT_SGBRG10_1X10 },
	{ formats::SGRBG10_CSI2P, MEDIA_BUS_FMT_SGRBG10_1X10 },
	{ formats::SRGGB10_CSI2P, MEDIA_BUS_FMT_SRGGB10_1X10 },
};

static const std::map<PixelFormat, uint32_t> outputFormatToMediaBus = {
	{ formats::NV12, MEDIA_BUS_FMT_YUYV8_1_5X8 },
	{ formats::NV21, MEDIA_BUS_FMT_YVYU8_1_5X8 },
	{ formats::NV16, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::NV61, MEDIA_BUS_FMT_YUYV8_2X8 },
	{ formats::NV24, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::NV42, MEDIA_BUS_FMT_YUV8_1X24 },
	{ formats::R8, MEDIA_BUS_FMT_Y8_1X8 },
};

} /* namespace */

/**
 * \class CamssIspOpe
 * \brief CAMSS ISP class for the Offline Processing Engine (OPE) ISP
 */

/**
 * \brief Constructs CamssIspOpe object
 * \param[in] pipe The pipeline handler in use
 * \param[in] opeMediaDev The OPE MediaDevice
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
CamssIspOpe::CamssIspOpe(PipelineHandler *pipe, std::shared_ptr<MediaDevice> opeMediaDev,
			 const CameraSensor *sensor, CamssFrames *frameInfos,
			 ControlInfoMap *ispControls)
	: opeMediaDev_(std::move(opeMediaDev)), sensor_(sensor), frameInfos_(frameInfos)
{
	sharedParams_ = SharedMemObject<DebayerParams>("softIsp_params");
	if (!sharedParams_) {
		LOG(Camss, Error) << "Failed to create shared memory for parameters";
		return;
	}

	const CameraManager &cm = *pipe->cameraManager();

	stats_ = std::make_unique<SwStatsCpu>(cm);
	if (!stats_->isValid()) {
		LOG(Camss, Error) << "Failed to create SwStatsCpu object";
		return;
	}

	stats_->statsReady.connect(this,
				   [&](uint32_t frame, uint32_t statsBufferId) {
					   statsReady.emit(frame, statsBufferId);
				   });

	ipa_ = pipe->createIPA<ipa::soft::IPAProxySoft>("simple", 0, 0);
	if (!ipa_) {
		LOG(Camss, Error) << "Creating IPA failed";
		return;
	}

	/*
	 * The API tuning file is made from the sensor name. If the tuning file
	 * isn't found, fall back to the 'uncalibrated' file.
	 */
	std::string ipaTuningFile =
		ipa_->configurationFile(sensor->model() + ".yaml", "uncalibrated.yaml");

	IPACameraSensorInfo sensorInfo{};
	int ret = sensor->sensorInfo(&sensorInfo);
	if (ret) {
		LOG(Camss, Error) << "Camera sensor information not available";
		ipa_.reset();
		return;
	}

	bool ccmEnabled;
	ret = ipa_->init(IPASettings{ ipaTuningFile, sensor->model() },
			 stats_->getStatsFD(),
			 sharedParams_.fd(),
			 sensorInfo,
			 sensor->controls(),
			 ispControls,
			 &ccmEnabled);
	if (ret) {
		LOG(Camss, Error) << "IPA init failed";
		ipa_.reset();
		return;
	}

	ipa_->paramsComputed.connect(this, &CamssIspOpe::paramsComputed);
	ipa_->metadataReady.connect(this,
				    [&](uint32_t frame, const ControlList &metadata) {
					    metadataReady.emit(frame, metadata);
				    });
	ipa_->setSensorControls.connect(this,
					[&](const ControlList &sensorControls) {
						setSensorControls.emit(sensorControls);
					});
}

CamssIspOpe::~CamssIspOpe() = default;

bool CamssIspOpe::isValid()
{
	return !!ipa_;
}

Size CamssIspOpe::getMargins([[maybe_unused]] PixelFormat inputFormat)
{
	/* TODO: Do we really not need any margins on OPE? */
	return Size();
}

int CamssIspOpe::init()
{
	MediaEntity *paramsEnt, *inputEnt, *procEnt, *dispEnt, *dispOutputEnt;
	int ret;

	if (!isValid())
		return -ENOMEM;

	paramsEnt = opeMediaDev_->getEntityByName("ope_params");
	inputEnt = opeMediaDev_->getEntityByName("ope_input");
	procEnt = opeMediaDev_->getEntityByName("ope_proc");
	dispEnt = opeMediaDev_->getEntityByName("ope_disp");
	dispOutputEnt = opeMediaDev_->getEntityByName("ope_disp_output");

	if (!paramsEnt || !inputEnt || !procEnt || !dispEnt || !dispOutputEnt) {
		LOG(Camss, Error) << "Did not find expected entities";
		return -EINVAL;
	}

	params_ = std::make_unique<V4L2VideoDevice>(paramsEnt);
	ret = params_->open();
	if (ret)
		return ret;

	params_->bufferReady.connect(this, &CamssIspOpe::parameterBufferReady);

	input_ = std::make_unique<V4L2VideoDevice>(inputEnt);
	ret = input_->open();
	if (ret)
		return ret;

	input_->bufferReady.connect(this, [&](FrameBuffer *f) {
		inputBufferReady.emit(f);
	});

	proc_ = std::make_unique<V4L2Subdevice>(procEnt);
	ret = proc_->open();
	if (ret)
		return ret;

	disp_ = std::make_unique<V4L2Subdevice>(dispEnt);
	ret = disp_->open();
	if (ret)
		return ret;

	output_ = std::make_unique<V4L2VideoDevice>(dispOutputEnt);
	ret = output_->open();
	if (ret)
		return ret;

	output_->bufferReady.connect(this, [&](FrameBuffer *f) {
		outputBufferReady.emit(f);
	});

	return 0;
}

/* Note cfg is const / not modified when update_stride_and_size is false */
int CamssIspOpe::trySetVideoCfg(V4L2VideoDevice *v4l2Dev, StreamConfiguration &cfg,
				bool set, bool update_stride_and_size,
				const char *msgPrefix) const
{
	int ret;

	V4L2DeviceFormat v4l2Format;
	v4l2Format.fourcc = v4l2Dev->toV4L2PixelFormat(cfg.pixelFormat);
	v4l2Format.size = cfg.size;
	v4l2Format.planes[0].bpl = cfg.stride;
	v4l2Format.planesCount = 1;

	if (set)
		ret = v4l2Dev->setFormat(&v4l2Format);
	else
		ret = v4l2Dev->tryFormat(&v4l2Format);

	if (ret < 0) {
		LOG(Camss, Error) << msgPrefix << " error: " << strerror(-ret);
		return ret;
	}

	if (update_stride_and_size) {
		cfg.stride = v4l2Format.planes[0].bpl;
		cfg.frameSize = v4l2Format.planes[0].size;
	}

	if (!camssV4L2DeviceFormatMatchesStreamConfig(v4l2Format, cfg, msgPrefix))
		return -EINVAL;

	return 0;
}

StreamConfiguration CamssIspOpe::generateConfiguration(const StreamConfiguration &raw) const
{
	if (!inputFormatToMediaBus.count(raw.pixelFormat)) {
		LOG(Camss, Warning) << "Unsupported pixel format " << raw.pixelFormat;
		return {};
	}

	if (trySetVideoCfg(input_.get(), const_cast<StreamConfiguration &>(raw),
			   false, false, "OPE input try format"))
		return {};

	/* OPE always supports all output formats */
	std::vector<SizeRange> sizesVector = { SizeRange(kMinOutputSize, raw.size, 2, 2) };
	std::map<PixelFormat, std::vector<SizeRange>> formats;
	for (auto it = outputFormatToMediaBus.begin(); it != outputFormatToMediaBus.end(); it++)
		formats[it->first] = sizesVector;

	StreamConfiguration cfg{ StreamFormats{ formats } };
	cfg.size = raw.size;
	cfg.pixelFormat = formats::NV12; /* Default to NV12 output */
	cfg.bufferCount = kBufferCount;

	return cfg;
}

int CamssIspOpe::trySetSubdevFormat(V4L2Subdevice *subdev,
				    unsigned int pad,
				    const V4L2SubdeviceFormat &fmt,
				    V4L2Subdevice::Whence whence,
				    const char *msgPrefix) const
{
	V4L2SubdeviceFormat subdevFormat = fmt;

	int ret = subdev->setFormat(pad, &subdevFormat, whence);
	if (ret < 0) {
		LOG(Camss, Error) << msgPrefix << " try/set format error: " << strerror(-ret);
		return ret;
	}

	if (subdevFormat.size != fmt.size || subdevFormat.code != fmt.code) {
		LOG(Camss, Error) << msgPrefix << " mismatch asked "
				  << fmt.size << "/" << fmt.code << " got "
				  << subdevFormat.size << "/" << subdevFormat.code;
		return -EINVAL;
	}

	return 0;
}

int CamssIspOpe::trySetSubdevSelection(V4L2Subdevice *subdev,
				       unsigned int pad,
				       unsigned int target,
				       const Rectangle &rect,
				       V4L2Subdevice::Whence whence,
				       const char *msgPrefix) const
{
	struct Rectangle r = rect;

	int ret = subdev->setSelection(pad, target, &r, whence);
	if (ret < 0) {
		LOG(Camss, Error) << msgPrefix << " set selection error: " << strerror(-ret);
		return ret;
	}

	if (r != rect) {
		LOG(Camss, Error) << msgPrefix << " mismatch asked "
				  << rect << " got " << r;
		return -EINVAL;
	}

	return 0;
}

int CamssIspOpe::trySetPipelineConfig(const StreamConfiguration &inputCfg,
				      [[maybe_unused]] const StreamConfiguration &outputCfg,
				      V4L2Subdevice::Whence whence) const
{
	if (!inputFormatToMediaBus.count(inputCfg.pixelFormat)) {
		LOG(Camss, Warning) << "Unsupported pixel format " << inputCfg.pixelFormat;
		return -EINVAL;
	}

	V4L2SubdeviceFormat subdevFormat;
	subdevFormat.code = inputFormatToMediaBus.at(inputCfg.pixelFormat);
	subdevFormat.size = inputCfg.size;

	int ret = trySetSubdevFormat(proc_.get(), PROC_PAD_INPUT, subdevFormat,
				     whence, "OPE proc input");
	if (ret < 0)
		return ret;

	/*
	 * No input-crop, downscale instead to keep full field of view.
	 * \todo crop in x or y dimension to match requested output aspect,
	 * to keep square pixels after scaling.
	 */
	ret = trySetSubdevSelection(proc_.get(), PROC_PAD_INPUT, V4L2_SEL_TGT_CROP,
				    Rectangle(inputCfg.size), whence, "OPE input crop");
	if (ret < 0)
		return ret;

	ret = trySetSubdevSelection(proc_.get(), PROC_PAD_DISP, V4L2_SEL_TGT_COMPOSE,
				    Rectangle(outputCfg.size), whence, "OPE output compose");
	if (ret < 0)
		return ret;

	subdevFormat.code = outputFormatToMediaBus.at(outputCfg.pixelFormat);
	subdevFormat.size = outputCfg.size;

	ret = trySetSubdevFormat(proc_.get(), PROC_PAD_DISP, subdevFormat,
				 whence, "OPE proc disp out");
	if (ret < 0)
		return ret;

	ret = trySetSubdevFormat(disp_.get(), DISP_PAD_PROC, subdevFormat,
				 whence, "OPE disp proc in");
	if (ret < 0)
		return ret;

	ret = trySetSubdevFormat(disp_.get(), DISP_PAD_OUTPUT, subdevFormat,
				 whence, "OPE disp out");
	if (ret < 0)
		return ret;

	return 0;
}

StreamConfiguration CamssIspOpe::validate(const StreamConfiguration &raw, const StreamConfiguration &req) const
{
	if (trySetVideoCfg(input_.get(), const_cast<StreamConfiguration &>(raw),
			   false, false, "OPE input try format"))
		return {};

	StreamConfiguration cfg;

	for (auto it = outputFormatToMediaBus.begin(); it != outputFormatToMediaBus.end(); it++) {
		if (it->first == req.pixelFormat)
			cfg.pixelFormat = req.pixelFormat;
	}

	if (!cfg.pixelFormat.isValid())
		cfg.pixelFormat = formats::NV12; /* Default to NV12 output */

	if (SizeRange(kMinOutputSize, raw.size, 2, 2).contains(req.size))
		cfg.size = req.size;
	else
		cfg.size = raw.size;

	int ret = trySetPipelineConfig(raw, cfg, V4L2Subdevice::TryFormat);
	if (ret < 0)
		return {};

	ret = trySetVideoCfg(output_.get(), cfg, false, true, "OPE output try format");
	if (ret < 0)
		return {};

	cfg.bufferCount = std::max(kBufferCount, req.bufferCount);
	cfg.setStream(const_cast<Stream *>(&outStream_));

	return cfg;
}

int CamssIspOpe::configure(const StreamConfiguration &inputCfg,
			   const StreamConfiguration &outputCfg)
{
	ipa::soft::IPAConfigInfo configInfo;
	configInfo.sensorControls = sensor_->controls();

	int ret = ipa_->configure(configInfo);
	if (ret < 0)
		return ret;

	ret = stats_->configure(inputCfg);
	if (ret < 0)
		return ret;

	/* Use 2/3 center of image to reduce CPU load */
	Rectangle statsWindow;
	statsWindow.width = inputCfg.size.width * 2 / 3;
	statsWindow.height = inputCfg.size.height * 2 / 3;
	statsWindow.x = (inputCfg.size.width - statsWindow.width) / 2;
	statsWindow.y = (inputCfg.size.height - statsWindow.height) / 2;
	/* stats_->setWindow() takes care of necessary alignment itself */
	stats_->setWindow(statsWindow);

	ret = trySetVideoCfg(input_.get(), const_cast<StreamConfiguration &>(inputCfg),
			     true, false, "OPE input set format");
	if (ret < 0)
		return ret;

	ret = trySetPipelineConfig(inputCfg, outputCfg, V4L2Subdevice::ActiveFormat);
	if (ret < 0)
		return ret;

	ret = trySetVideoCfg(output_.get(), const_cast<StreamConfiguration &>(outputCfg),
			     true, false, "OPE output set format");
	if (ret < 0)
		return ret;

	inputBufferCount_ = inputCfg.bufferCount;
	outputBufferCount_ = outputCfg.bufferCount;

	return 0;
}

int CamssIspOpe::allocateBuffers(unsigned int bufferCount)
{
	int ret;

	ret = input_->importBuffers(inputBufferCount_);
	if (ret < 0)
		return ret;

	ret = output_->importBuffers(outputBufferCount_);
	if (ret < 0) {
		freeBuffers();
		return ret;
	}

	ret = params_->allocateBuffers(bufferCount, &paramBuffers_);
	if (ret < 0) {
		freeBuffers();
		return ret;
	}

	/* Map buffers to the IPA. */
	unsigned int ipaBufferId = 1;

	/* Lambda function for future re-use with statsBuffers. */
	auto pushBuffers = [&](const std::vector<std::unique_ptr<FrameBuffer>> &buffers) {
		for (const std::unique_ptr<FrameBuffer> &buffer : buffers) {
			Span<const FrameBuffer::Plane> planes = buffer->planes();

			buffer->setCookie(ipaBufferId++);
			ipaBuffers_.emplace_back(buffer->cookie(),
						 std::vector<FrameBuffer::Plane>{ planes.begin(),
										  planes.end() });
		}
	};

	pushBuffers(paramBuffers_);

	/* FIXME drop once using own camss IPA, which will map on IPA side */
	for (const IPABuffer &buffer : ipaBuffers_) {
		const FrameBuffer fb(buffer.planes);
		mappedIpaBuffers_.emplace(buffer.id,
					  MappedFrameBuffer(&fb, MappedFrameBuffer::MapFlag::ReadWrite));
	}

	frameInfos_->init(paramBuffers_, {}, true);
	return 0;
}

void CamssIspOpe::freeBuffers()
{
	frameInfos_->clear();

	/* FIXME drop once using own camss IPA, which will map on IPA side */
	mappedIpaBuffers_.clear();

	ipaBuffers_.clear();
	paramBuffers_.clear();
	params_->releaseBuffers();
	output_->releaseBuffers();
	input_->releaseBuffers();
}

int CamssIspOpe::exportOutputBuffers([[maybe_unused]] const Stream *stream, unsigned int count,
				     std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	return output_->exportBuffers(count, buffers);
}

void CamssIspOpe::queueBuffers(Request *request, FrameBuffer *inputBuffer)
{
	uint32_t frame = request->sequence();

	DmaSyncer inputDmaSync(inputBuffer->planes()[0].fd, DmaSyncer::SyncType::Read);

	MappedFrameBuffer input(inputBuffer, MappedFrameBuffer::MapFlag::Read);
	if (!input.isValid()) {
		LOG(Camss, Error) << "mmap-ing buffer(s) failed";
		return;
	}

	ipa_->queueRequest(frame, request->controls());
	stats_->processFrame(frame, 0, input);
	ipa_->computeParams(frame);
	/* paramsComputed() will queue all the buffers once the params are known. */
}

void CamssIspOpe::paramsComputed(uint32_t frame)
{
	CamssFrames::Info *info = frameInfos_->find(frame);
	if (!info)
		return;

	FrameBuffer *outputBuffer = nullptr;
	for (const auto &[stream, buffer] : info->request->buffers()) {
		if (stream == &outStream_)
			outputBuffer = buffer;
	}
	if (!outputBuffer) {
		LOG(Camss, Error) << "Missing output buffer";
		return;
	}

	/*
	 * FIXME drop once using own camss IPA, which will do this on IPA side.
	 * IPA paramsComputed signal should also pass a bytesused value back.
	 */
	uint32_t bufferId = info->paramBuffer->cookie();
	ipa::camss::CamssParams params(mappedIpaBuffers_.at(bufferId).planes()[0]);
	auto awbGains = params.block<ipa::camss::CamssBlocks::AwbGains>();
	DebayerParams debayerParams = *sharedParams_;
	/* Convert gains to 15uQ10 and store */
	awbGains->r_gain = std::clamp(debayerParams.gains.r(), 0.0f, 31.0f) * 1024;
	awbGains->g_gain = std::clamp(debayerParams.gains.g(), 0.0f, 31.0f) * 1024;
	awbGains->b_gain = std::clamp(debayerParams.gains.b(), 0.0f, 31.0f) * 1024;
	/* Convert blacklevel to 0-65535 (u16) and store */
	awbGains->r_sub = debayerParams.blackLevel.r() * 65535;
	awbGains->g_sub = debayerParams.blackLevel.g() * 65535;
	awbGains->b_sub = debayerParams.blackLevel.b() * 65535;
	/* Unused */
	awbGains->r_add = 0;
	awbGains->g_add = 0;
	awbGains->b_add = 0;

	info->paramBuffer->_d()->metadata().planes()[0].bytesused = params.bytesused();
	params_->queueBuffer(info->paramBuffer);
	output_->queueBuffer(outputBuffer);
	input_->queueBuffer(info->rawBuffer);
}

void CamssIspOpe::parameterBufferReady(FrameBuffer *f)
{
	CamssFrames::Info *info = frameInfos_->find(f);
	if (!info)
		return;

	info->paramDequeued = true;

	paramBufferReady.emit(f);
}

void CamssIspOpe::processStats(const uint32_t frame, const uint32_t statsBufferId,
			       const ControlList &sensorControls)
{
	ipa_->processStats(frame, statsBufferId, sensorControls);
}

int CamssIspOpe::start()
{
	int ret = ipa_->start();
	if (ret)
		return ret;

	ret = input_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = output_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	ret = params_->streamOn();
	if (ret < 0) {
		stop();
		return ret;
	}

	return 0;
}

void CamssIspOpe::stop()
{
	params_->streamOff();
	output_->streamOff();
	input_->streamOff();
	ipa_->stop();
}

/**
 * \brief Match media devices and create OPE instance if found
 * \param[in] pipe The pipeline handler in use
 * \param[in] sensor Pointer to the CameraSensor instance owned by the pipeline
 * \param[in] frameInfos Pointer to CamssFrames instance owned by the pipeline
 * \param[out] ControlInfoMap to which to add ISP provided controls
 */
std::unique_ptr<CamssIspOpe> CamssIspOpe::match(PipelineHandler *pipe,
						DeviceEnumerator *enumerator,
						const CameraSensor *sensor,
						CamssFrames *frameInfos,
						ControlInfoMap *ispControls)
{
	std::shared_ptr<MediaDevice> opeMediaDev;
	DeviceMatch opeDm("qcom-camss-ope");

	opeDm.add("ope_params");
	opeDm.add("ope_input");
	opeDm.add("ope_proc");
	opeDm.add("ope_disp");
	opeDm.add("ope_disp_output");

	opeMediaDev = pipe->acquireMediaDevice(enumerator, opeDm);
	if (!opeMediaDev) {
		LOG(Camss, Info) << "No OPE match for " << sensor->entity()->name();
		return nullptr;
	}

	std::unique_ptr<CamssIspOpe> ope =
		std::make_unique<CamssIspOpe>(pipe, std::move(opeMediaDev), sensor,
					      frameInfos, ispControls);

	if (ope->init() != 0)
		return nullptr;

	LOG(Camss, Info) << "Using OPE for " << sensor->entity()->name();

	return ope;
}

} /* namespace libcamera */
