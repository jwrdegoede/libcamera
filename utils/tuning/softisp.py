#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas On Board
#
# Tuning script for software ISP

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
from libtuning.modules.lsc import LSCSoftisp
from libtuning.parsers import YamlParser

coloredlogs.install(level=logging.INFO, fmt='%(name)s %(levelname)s %(message)s')

lsc = LSCSoftisp(debug=[lt.Debug.Plot],
                 sector_shape=(17, 17),

                 sector_x_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),
                 sector_y_gradient=lt.gradient.Linear(lt.Remainder.DistributeFront),

                 # This is the function that will be used to average the pixels in
                 # each sector. This can also be a custom function.
                 sector_average_function=lt.average.Mean(),

                 # This is the function that will be used to smooth the color ratio
                 # values.  This can also be a custom function.
                 smoothing_function=lt.smoothing.MedianBlur(3),)

tuner = lt.Tuner('Softisp')
tuner.add([lsc])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())

tuner.set_output_order([lsc])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
