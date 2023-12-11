# SPDX-License-Identifier: CC0-1.0

soft_ipa_common_sources = files([
    'soft_base.cpp',
])

soft_ipa_common_includes = [
    include_directories('..'),
]

soft_ipa_common_deps = [
    libcamera_private,
]

soft_ipa_common_lib = static_library('soft_ipa_common', soft_ipa_common_sources,
                                     include_directories : soft_ipa_common_includes,
                                     dependencies : soft_ipa_common_deps)
