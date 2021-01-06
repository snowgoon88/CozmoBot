#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg

import numpy as np

import cv2

from opencv_node import OpenCVNode
from detectors import VerticalLinesDetector

if __name__ == '__main__':
    # need an output window
    _output_win = "Vertical Line Detector"
    cv2.namedWindow( _output_win )

    _processor = VerticalLinesDetector()
    _processor.build_gui( _output_win )

    _node = OpenCVNode( 'cozmo_vert_detector', _processor, _output_win )
    _node.start()
    

