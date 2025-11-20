/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Ideas On Board
 *
 * IPA module
 */

#pragma once

#include <list>
#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/utils.h>

#include "libcamera/internal/yaml_parser.h"

#include "algorithm.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(IPAModuleAlgo)

namespace ipa {

template<typename _Context, typename _FrameContext, typename _Config,
	 typename _Params, typename _Stats>
class Module : public Loggable
{
public:
	using Context = _Context;
	using FrameContext = _FrameContext;
	using Config = _Config;
	using Params = _Params;
	using Stats = _Stats;

	virtual ~Module() {}

	const std::list<std::unique_ptr<Algorithm<Module>>> &algorithms() const
	{
		return algorithms_;
	}

	int createAlgorithms(Context &context, const YamlObject &algorithms)
	{
		const auto &list = algorithms.asList();

		for (const auto &[i, algo] : utils::enumerate(list)) {
			if (!algo.isDictionary()) {
				LOG(IPAModuleAlgo, Error)
					<< "Invalid YAML syntax for algorithm " << i;
				algorithms_.clear();
				return -EINVAL;
			}

			int ret = createAlgorithm(context, algo);
			if (ret) {
				algorithms_.clear();
				return ret;
			}
		}

		return 0;
	}

	static void registerAlgorithm(AlgorithmFactoryBase<Module> *factory)
	{
		factories().push_back(factory);
	}

	/**
	 * \fn int Module::createSelfEnumeratingAlgorithm(Context &context, const std::string &name)
	 * \brief Create and initialise a self-enumerating algorithm by name
	 * 
	 * This function creates an algorithm instance from the registered algorithm
	 * factories using only the algorithm name, without requiring YAML configuration
	 * data.
	 * 
	 * This is useful for algorithms that don't require external configuration
	 * parameters and can self-configure or use default values.
	 * 
	 * \param[in] context The IPA context to pass to the algorithm's init function
	 * \param[in] name The name of the algorithm to instantiate
	 * 
	 * \return 0 on success, negative errno value on failure:
	 *         -EINVAL if the algorithm is not found in the factory registry
	 *         Other negative values if algorithm initialisation fails
	 */
	int createSelfEnumeratingAlgorithm(Context &context, const std::string &name)
	{
		std::unique_ptr<Algorithm<Module>> algo = createAlgorithm(name);
		if (!algo) {
			LOG(IPAModuleAlgo, Error)
				<< "Algorithm '" << name << "' not found";
			return -EINVAL;
		}

		int ret = algo->init(context);
		if (ret) {
			LOG(IPAModuleAlgo, Error)
				<< "Algorithm '" << name << "' failed to initialize";
			return ret;
		}

		LOG(IPAModuleAlgo, Debug)
			<< "Instantiated algorithm '" << name << "'";

		algorithms_.push_back(std::move(algo));
		return 0;
	}

private:
	int createAlgorithm(Context &context, const YamlObject &data)
	{
		const auto &[name, algoData] = *data.asDict().begin();
		std::unique_ptr<Algorithm<Module>> algo = createAlgorithm(name);
		if (!algo) {
			LOG(IPAModuleAlgo, Error)
				<< "Algorithm '" << name << "' not found";
			return -EINVAL;
		}

		int ret = algo->init(context, algoData);
		if (ret) {
			LOG(IPAModuleAlgo, Error)
				<< "Algorithm '" << name << "' failed to initialize";
			return ret;
		}

		LOG(IPAModuleAlgo, Debug)
			<< "Instantiated algorithm '" << name << "'";

		algorithms_.push_back(std::move(algo));
		return 0;
	}

	static std::unique_ptr<Algorithm<Module>> createAlgorithm(const std::string &name)
	{
		for (const AlgorithmFactoryBase<Module> *factory : factories()) {
			if (factory->name() == name)
				return factory->create();
		}

		return nullptr;
	}

	static std::vector<AlgorithmFactoryBase<Module> *> &factories()
	{
		/*
		 * The static factories map is defined inside the function to ensure
		 * it gets initialized on first use, without any dependency on
		 * link order.
		 */
		static std::vector<AlgorithmFactoryBase<Module> *> factories;
		return factories;
	}

	std::list<std::unique_ptr<Algorithm<Module>>> algorithms_;
};

} /* namespace ipa */

} /* namespace libcamera */
