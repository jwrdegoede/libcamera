# SPDX-License-Identifier: CC0-1.0

softisp_enabled = pipelines.contains('simple') or pipelines.contains('atomisp')
summary({'SoftISP support' : softisp_enabled}, section : 'Configuration')

if not softisp_enabled
    subdir_done()
endif

libcamera_internal_sources += files([
    'benchmark.cpp',
    'debayer.cpp',
    'debayer_cpu.cpp',
    'software_isp.cpp',
    'swstats_cpu.cpp',
])
