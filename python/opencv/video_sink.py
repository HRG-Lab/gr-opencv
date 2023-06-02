#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 Bailey Campbell.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
from gnuradio import gr
from gnuradio.gr import pmt

from pyqtgraph.widgets.RawImageWidget import RawImageWidget


class video_sink(gr.sync_block, RawImageWidget):
    """
    docstring for block video_sink
    """

    def __init__(self, parent=None):
        gr.sync_block.__init__(
            self,
            name="Video Sink",
            in_sig=None,
            out_sig=None
        )
        RawImageWidget.__init__(self, parent=parent)

        self.message_port_register_in(pmt.intern('frame'))
        self.set_msg_handler(pmt.intern('frame'), self.handle_frame)

    def handle_frame(self, msg):
        data = pmt.to_python(msg)
        frame = data[1].reshape(data[0]['shape'])
        self.setImage(frame)
