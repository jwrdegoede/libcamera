#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas On Board
#
# Tuning script for software ISP, based on rkisp1 tuning

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
# Re-use AWB + CCM from RKISP1k
from libtuning.modules.awb import AWBRkISP1
from libtuning.modules.ccm import CCMRkISP1
from libtuning.modules.lsc import LSCSoftisp
from libtuning.modules.static import StaticModule
from libtuning.parsers import YamlParser

coloredlogs.install(level=logging.INFO, fmt='%(name)s %(levelname)s %(message)s')

agc = StaticModule('Agc')
awb = AWBRkISP1(debug=[lt.Debug.Plot])
adj = StaticModule('Adjust')
blc = StaticModule('BlackLevel')
ccm = CCMRkISP1(debug=[lt.Debug.Plot])
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
tuner.add([agc, awb, adj, blc, ccm, lsc])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())

tuner.set_output_order([agc, awb, adj, blc, ccm, lsc])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
