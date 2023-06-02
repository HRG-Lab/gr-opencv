#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright 2023 Bailey Campbell.
#
# SPDX-License-Identifier: GPL-3.0-or-later
#


import numpy as np
import cv2 as cv
from cv2 import aruco
from gnuradio import gr
from gnuradio.gr import pmt

from pyqtgraph.Qt.QtCore import Signal, QObject, QThread
from pyqtgraph.widgets.RawImageWidget import RawImageWidget


class VideoThreadWorker(QObject):
    newInfo = Signal(object, object)

    def __init__(
        self,
        video_device,
        dictionary,
        marker_length,
        calibration_file,
    ):
        QObject.__init__(self)

        self.video_device = video_device
        self.dictionary = aruco.getPredefinedDictionary(dictionary)
        self.params = aruco.DetectorParameters()
        self.cal_file = cv.FileStorage(
            calibration_file, flags=cv.FileStorage_READ)
        self.cam_mtx = self.cal_file.getNode('camera_matrix').mat()
        self.dist_coeffs = self.cal_file.getNode(
            'distortion_coefficients').mat()
        self.marker_length = marker_length
        self.marker_pts = np.array([
            [-marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, marker_length / 2, 0],
            [marker_length / 2, -marker_length / 2, 0],
            [-marker_length / 2, -marker_length / 2, 0],
        ], dtype=np.float32)
        self.origin_id = 1

        self.cap = cv.VideoCapture(self.video_device)
        if not self.cap.isOpened():
            raise RuntimeError("Couldn't open camera")

        self.detector = cv.aruco.ArucoDetector(self.dictionary, self.params)

    def run(self):
        import time
        while True:
            ret, frame = self.cap.read()

            if not ret:
                print("Missed frame")
                continue

            self.detect_markers(frame)
            # time.sleep(0.1)

    def detect_markers(self, frame):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

        corners, ids, rejected = self.detector.detectMarkers(gray)

        ret = {}
        aruco.drawDetectedMarkers(frame, corners, ids)
        if len(corners) > 0:
            for (id, c) in zip(ids, corners):
                _, rvec, tvec = cv.solvePnP(
                    self.marker_pts,
                    c,
                    self.cam_mtx,
                    self.dist_coeffs,
                    False,
                    cv.SOLVEPNP_IPPE_SQUARE
                )
                ret[int(id[0])] = {
                    'rvec': rvec,
                    'tvec': tvec,
                }
                cv.drawFrameAxes(
                    frame, self.cam_mtx, self.dist_coeffs, rvec, tvec, 0.075
                )

        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)
        info = self.get_vecs_relative_to_origin(ret)
        self.newInfo.emit(np.transpose(frame_rgb, (1, 0, 2)), info)

    def get_vecs_relative_to_origin(self, frame_info):
        origin = frame_info.get(self.origin_id)
        if origin is None:
            return
        if len(frame_info) == 0:
            return

        for id, f in frame_info.items():
            r_rel, t_rel = self.relative_to(origin['rvec'], origin['tvec'], f['rvec'], f['tvec'])
            frame_info[id]['rvec'] = r_rel
            frame_info[id]['tvec'] = t_rel

        return frame_info

    def invert_perspective(self, rvec, tvec):
        r, _ = cv.Rodrigues(rvec)
        r = r.T
        t_inv = np.dot(r, tvec)
        r_inv, _ = cv.Rodrigues(r)
        return r_inv, t_inv

    def relative_to(self, rvec_1, tvec_1, rvec_2, tvec_2):
        r_inv, t_inv = self.invert_perspective(rvec_2, tvec_2)
        r_rel, t_rel, *_ = cv.composeRT(rvec_1, tvec_1, r_inv, t_inv)
        return r_rel, t_rel


class aruco_detector(gr.sync_block, RawImageWidget):
    """
    docstring for block aruco_detector
    """

    def __init__(
        self,
        video_device=0,
        dictionary=0,
        marker_length=0.1,
        calibration_file=None,
        parent=None,
        show=False
    ):
        gr.sync_block.__init__(
            self,
            name="Aruco Detector",
            in_sig=None,
            out_sig=None
        )
        RawImageWidget.__init__(self, parent=parent)

        self.show = show
        self.message_port_register_out(pmt.intern('info'))

        self.thread = QThread()
        self.worker = VideoThreadWorker(
            video_device,
            dictionary,
            marker_length,
            calibration_file,
        )
        self.worker.moveToThread(self.thread)
        self.thread.started.connect(self.worker.run)
        self.thread.finished.connect(self.thread.deleteLater)
        self.worker.newInfo.connect(self.on_new_info)

        if self.show is False:
            self.hide()

        self.thread.start()

    def on_new_info(self, frame, frame_info):
        if self.show:
            self.setImage(frame)
        if frame_info is None:
            return

        self.message_port_pub(pmt.intern('info'), pmt.to_pmt(frame_info))
