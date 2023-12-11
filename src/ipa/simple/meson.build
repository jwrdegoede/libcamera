# SPDX-License-Identifier: CC0-1.0

subdir('common')

foreach pipeline : pipelines
    pipeline = pipeline.split('/')
    if pipeline.length() < 2 or pipeline[0] != 'simple'
        continue
    endif

    subdir(pipeline[1])
endforeach
