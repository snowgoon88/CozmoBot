#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import rospkg

import numpy as np
#from cv_bridge import CvBridge, CvBridgeError
#from sensor_msgs.msg import Image

import cv2

#from img_processor import IntParameter, ImageProcessor

from opencv_node import OpenCVNode
from detectors import ThresoldDetector

if __name__ == '__main__':
    # need an output window
    _output_win = "Base Output"
    cv2.namedWindow( _output_win )

    _processor = ThresoldDetector()
    _processor.build_gui( _output_win )

    _node = OpenCVNode( 'cozmo_thr_detector', _processor, _output_win )
    _node.start()
    

