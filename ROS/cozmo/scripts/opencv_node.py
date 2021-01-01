#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rospy
import rospkg
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np

# location of package, needed to load a "no_image.png" file
_rospack = rospkg.RosPack()
_pkg_path = _rospack.get_path('cozmo')

def load_ressource_img( rel_pathname ):
    fullpath = _pkg_path+"/Ressources/"+rel_pathname
    print( "Loading "+fullpath )
    img = cv2.imread( fullpath, cv2.IMREAD_COLOR )
    if img is None:
        sys.exit( "Could not read "+fullpath )

default_img = load_ressource_img( "no_image.png" )

class IntParameter(object):
    def __init__(self, id, title, init_val, max_val, update_func=None):
        self.id = id
        self.title = title
        self.val = init_val
        self.max_val = max_val
        self.update_func = update_func
        if self.update_func is None:
            self.update_func = lambda v: v
        
    def _update_cbk(self, val):
        self.val = self.update_func( val )
        
    def create_trackbar(self, window):
        cv2.createTrackbar( self.title,
                            window,
                            self.val,
                            self.max_val,
                            self._update_cbk )

class ImageProcessor(object):
    def __init__(self, example_img):
        self.param = {}

    def add_param(self, param):
        self.param[param.id] = param

    def build_gui(self, named_win):
        for p in self.param:
            self.param[p].create_trackbar( named_win )

    def run(self, img):
        # copy the src img
        return img

class OpenCVNode(object):
    def __init__(self, node_name, processor, output_win):
        self.node_name = node_name
        self.processor = processor
        self.output_win = output_win
        # CV <-> ROS
        self.bridge = CvBridge()
        
    def image_cbk(self, message ):
        src_img = self.bridge.imgmsg_to_cv2( message )
        src_img = cv2.cvtColor( src_img, cv2.COLOR_RGB2BGR)

        res_img = self.processor.run( src_img )

        cv2.imshow( self.output_win, res_img )

    def start(self):
        rospy.init_node( self.node_name )
        rospy.Subscriber( 'in', Image, self.image_cbk )

        # main loop with watchdog
        try:
            while not rospy.is_shutdown():
                now = rospy.get_rostime()
                rospy.sleep( 0.1 )
                cv2.waitKey(1)
        finally:
            print( "This is the end...." )

