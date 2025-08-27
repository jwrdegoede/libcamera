#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
# Copyright (C) 2024, Ideas On Board
#
# Tuning script for rkisp1

import logging
import sys

import coloredlogs
import libtuning as lt
from libtuning.generators import YamlOutput
from libtuning.modules.ccm import CCMRkISP1
from libtuning.modules.static import StaticModule
from libtuning.parsers import YamlParser

coloredlogs.install(level=logging.INFO, fmt='%(name)s %(levelname)s %(message)s')

ccm = CCMRkISP1(debug=[lt.Debug.Plot])
color_processing = StaticModule('ColorProcessing')
filter = StaticModule('Filter')
gamma_out = StaticModule('GammaOutCorrection', {'gamma': 2.2})

tuner = lt.Tuner('SoftISP')
tuner.add([ccm, color_processing, filter, gamma_out])
tuner.set_input_parser(YamlParser())
tuner.set_output_formatter(YamlOutput())

tuner.set_output_order([ccm, color_processing, filter, gamma_out])

if __name__ == '__main__':
    sys.exit(tuner.run(sys.argv))
