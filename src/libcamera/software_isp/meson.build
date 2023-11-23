# SPDX-License-Identifier: CC0-1.0

libcamera_sources += files([
	'debayer.cpp',
	'debayer_cpu.cpp',
	'swstats.cpp',
	'swstats_cpu.cpp',
])

# Software ISP is enabled for 'simple' pipeline handler.
# The 'pipelines' option value should be
# 'simple/<Software ISP implementation>' e.g.:
#   -Dpipelines=simple/simple
# The source file should be named swisp_<Software ISP implementation>.cpp,
# e.g. 'swisp_simple.cpp'.

foreach pipeline : pipelines
    pipeline = pipeline.split('/')
    if pipeline.length() == 2 and pipeline[0] == 'simple'
        libcamera_sources += files([
            'swisp_' + pipeline[1] + '.cpp',
        ])
        # the 'break' below can be removed if/when multiple
        # Software ISP implementations are allowed in single build
        break
    endif
endforeach
