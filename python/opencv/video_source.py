#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 Bailey Campbell.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
from gnuradio import gr
import cv2 as cv

from pyqtgraph.Qt.QtCore import QObject, QThread, Signal
from pyqtgraph.widgets.RawImageWidget import RawImageWidget


class VideoThreadWorker(QObject):
    newFrame = Signal(object)

    def __init__(self, video_device):
        QObject.__init__(self)
        self.video_device = video_device
        self.cap = cv.VideoCapture(self.video_device)
        if not self.cap.isOpened():
            raise RuntimeError("Couldn't open camera")

    def run(self):
        import time
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            # gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
            frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
            self.newFrame.emit(np.transpose(frame_rgb, (1, 0, 2)))


class video_source(gr.sync_block, RawImageWidget):
    """
    docstring for block video_source
    """

    def __init__(self, video_device=0, parent=None):
        gr.sync_block.__init__(
            self,
            name="Video Source",
            in_sig=None,
            out_sig=None
        )
        RawImageWidget.__init__(self, parent=parent)

        self.thread = QThread()
        self.worker = VideoThreadWorker(video_device)
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.newFrame.connect(self.setImage)

        self.thread.start()
