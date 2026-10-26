# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright (C) 2019, Raspberry Pi Ltd
# Copyright (C) 2022, Paul Elder <paul.elder@ideasonboard.com>
#
# LSC module for tuning softisp

from .lsc import LSC

import libtuning as lt
import libtuning.utils as utils

from numbers import Number
import numpy as np


class LSCSoftisp(LSC):
    hr_name = 'LSC (Softisp)'
    out_name = 'Lsc'
    # \todo Not sure if this is useful. Probably will remove later.
    compatible = ['softisp']

    def __init__(self, *args, **kwargs):
        super().__init__(**kwargs)

    # We don't actually need anything from the config file
    def validate_config(self, config: dict) -> bool:
        return True

    # @return Image color temperature, flattened array of red calibration table
    #         (containing {sector size} elements), flattened array of red
    #         calibration table, flattened array of green calibration
    #         table, flattened array of blue calibration table

    def _do_single_lsc(self, image: lt.Image):
        # Perform LSC on each colour channel independently. A future enhancement
        # worth investigating would be splitting the luminance and chrominance
        # LSC as done by Raspberry Pi.
        cr, _ = self._lsc_single_channel(image.channels[lt.Color.R], image)
        cgr, _ = self._lsc_single_channel(image.channels[lt.Color.GR], image)
        cgb, _ = self._lsc_single_channel(image.channels[lt.Color.GR], image)
        cb, _ = self._lsc_single_channel(image.channels[lt.Color.B], image)
        cg = (cgr + cgb) / 2

        return image.color, cr.flatten(), cg.flatten(), cb.flatten()

    # @return List of dictionaries of color temperature, red table, green
    #         table and blue table

    def _do_all_lsc(self, images: list) -> list:
        output_list = []
        output_map_func = lt.gradient.Linear().map

        # List of colour temperatures
        list_col = []
        # Associated calibration tables
        list_cr = []
        list_cg = []
        list_cb = []
        for image in self._enumerate_lsc_images(images):
            col, cr, cg, cb = self._do_single_lsc(image)
            list_col.append(col)
            list_cr.append(cr)
            list_cg.append(cg)
            list_cb.append(cb)

        # Convert to numpy array for data manipulation
        list_col = np.array(list_col)
        list_cr = np.array(list_cr)
        list_cg = np.array(list_cg)
        list_cb = np.array(list_cb)

        for color_temperature in sorted(set(list_col)):
            # Average tables for the same colour temperature
            indices = np.where(list_col == color_temperature)
            color_temperature = int(color_temperature)

            tables = []
            for lis in [list_cr, list_cg, list_cb]:
                table = np.mean(lis[indices], axis=0)
                table = output_map_func((1, 3), (1, 255), table)
                table = np.clip(table, 1, 255)
                table = np.round(table).astype('int32').tolist()
                tables.append(table)

            entry = {
                'ct': color_temperature,
                'r': tables[0],
                'g': tables[1],
                'b': tables[2],
            }

            output_list.append(entry)

        return output_list

    def process(self, config: dict, images: list, outputs: dict) -> dict:
        output = {}

        output['sets'] = self._do_all_lsc(images)

        if len(output['sets']) == 0:
            return None

        return output
