#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS node that apply some OpenCV processing to incoming images
#
# Subscribe to /in (likely remapped to /camera)
# Every ROS::Image is transformed to a cv2.image
# then ImageProcess
#
# Trackbars/sliders allow to play with some parameters of processing

import sys
import rospy
import rospkg

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

from img_processor import IntParameter, ImageProcessor
from opencv_node import OpenCVNode

# CV <-> ROS
bridge = CvBridge()



if __name__ == '__main__':
    # location of package, needed to load a "no_image.png" file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cozmo')
    print( "COZMO path="+pkg_path )
    _default_img = cv2.imread( pkg_path+"/Ressources/no_image.png", cv2.IMREAD_COLOR )
    if _default_img is None:
        sys.exit("Could not read "+pkg_path+"/Ressources/no_image.png" )

    _output_win = "Base Output"
    cv2.namedWindow( _output_win )
    
    _processor = ImageProcessor( _default_img )
    _param_thres = IntParameter( id='thres', title="Thres",
                                 init_val=127, max_val=255 )
    _processor.add_param( _param_thres )

    _param_blur = IntParameter( id='blur', title='Blur',
                                init_val=5, max_val=33,
                                update_func= lambda x : (x // 2)*2+1 )
    _processor.add_param( _param_blur )
    _processor.build_gui( _output_win )
    _node = OpenCVNode( 'cozmo_thr_detector', _processor, _output_win )
    cv2.imshow( _output_win, _processor.run(_default_img) )
                        
    _node.start()
    

