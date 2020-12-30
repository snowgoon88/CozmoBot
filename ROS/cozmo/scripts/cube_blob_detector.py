#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS node that tries to detect the cube of the Cozmo robot using
# OpenCV 'SimpleBlobDetector'.
# see for example https://www.learnopencv.com/blob-detection-using-opencv-python-c/
#
# This is a work in progress and this node is more like a debugging tool.
# Cubes are NOT recognized yet. Far from it in fact...

# Subscribe to /in (likely remapped to /camera)
# Every ROS::Image is transformed to a cv2.image
# then SimpleBlobDetector is run on a gray version of the image
# and detected blobs are higlighted in red on the original image
#
# Trackbars/sliders allow to play with some parameters of the blob detector

import sys
import rospy
import rospkg

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import cv2

# CV <-> ROS
bridge = CvBridge()

# OpenCV windows
_src_win = "Cozmo Camera"
_out_win = "Final detection"

_default_img = None
# The Blob detector
_blob = None
_params = cv2.SimpleBlobDetector_Params()
_default_param = {
    'blobColor': 0,
    'minArea': 50,
    'minCircularity': 0.1,
    'minConvexity': 0.9,
    'minInertiaRatio': 0.3
    }

def image_cbk( message ):
    src_img = bridge.imgmsg_to_cv2( message )
    src_img = cv2.cvtColor( src_img, cv2.COLOR_RGB2BGR)
    
    cv2.imshow( _src_win, src_img )

    gray_img = cv2.cvtColor( src_img, cv2.COLOR_RGB2GRAY)
    gray_img = np.uint8( gray_img )
    keypoints = _blob.detect( gray_img )
    blob_img = cv2.drawKeypoints( gray_img,
                                  keypoints,
                                  np.array([]),
                                  (0,0,255),
                                  cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    cv2.imshow( _out_win, blob_img )

def start_node():
    rospy.init_node( 'cube_blob_detector' )
    rospy.Subscriber( 'in', Image, image_cbk )

    # main loop with watchdog
    try:
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            rospy.sleep( 0.1 )
            cv2.waitKey(1)
    finally:
        print( "This is the end...." )

def default_detector():
    global _blob

    # Change thresholds
    _params.minThreshold = 10
    _params.maxThreshold = 200

    # Filter by Color
    _params.filterByColor = True
    _params.blobColor = _default_param['blobColor']

    # Filter by Area.
    _params.filterByArea = True
    _params.minArea = _default_param['minArea']

    # Filter by Circularity
    _params.filterByCircularity = True
    _params.minCircularity = _default_param['minCircularity']

    # Filter by Convexity
    _params.filterByConvexity = True
    _params.minConvexity = _default_param['minConvexity']

    # Filter by Inertia
    _params.filterByInertia = False
    _params.minInertiaRatio = _default_param['minInertiaRatio']

    
    _blob = cv2.SimpleBlobDetector_create( _params )

def update_color_cbk( val ):
    global _blob
    _params.blobColor = val

    _blob = cv2.SimpleBlobDetector_create( _params )
    
def update_area_cbk( val ):
    global _blob
    _params.minArea = val
    _params.filterByArea = (val > 0)

    _blob = cv2.SimpleBlobDetector_create( _params )

def update_circularity_cbk( val ):
    global _blob
    _params.minCircularity = float(val) / 100.0
    _params.filterByCircularity = (val > 0)

    _blob = cv2.SimpleBlobDetector_create( _params )

def update_convexity_cbk( val ):
    global _blob
    _params.minConvexity = float(val) / 100.0
    _params.filterByConvexity = (val > 0)

    _blob = cv2.SimpleBlobDetector_create( _params )

def update_inertia_cbk( val ):
    global _blob
    _params.minInertiaRatio = float(val) / 100.0
    _params.filterByInertia = (val > 0)

    _blob = cv2.SimpleBlobDetector_create( _params )


def build_GUI( win ):
    ## Color
    cv2.createTrackbar( 'Color', win,
                        int(_params.blobColor), 255,
                        update_color_cbk )
    ## Area
    cv2.createTrackbar( 'Area', win, int(_params.minArea), 200, update_area_cbk )
    ## Circularity
    cv2.createTrackbar( 'Circle', win,
                        int(_params.minCircularity*100), 100,
                        update_circularity_cbk )
    ## Convexity
    cv2.createTrackbar( 'Convex', win,
                        int(_params.minConvexity*100), 100,
                        update_convexity_cbk )
    ## Inertia
    cv2.createTrackbar( 'Inertia', win,
                        int(_params.minInertiaRatio*100), 100,
                        update_inertia_cbk )
        
if __name__ == '__main__':
    # location of package, needed to load a "no_image.png" file
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('cozmo')
    print( "COZMO path="+pkg_path )
    
    _default_img = cv2.imread( pkg_path+"/Ressources/no_image.png", cv2.IMREAD_COLOR )
    if _default_img is None:
        sys.exit("Could not read "+pkg_path+"/Ressources/no_image.png" )
    cv2.namedWindow( _src_win )
    cv2.imshow( _src_win, _default_img )

    default_detector()
    cv2.namedWindow( _out_win )
    build_GUI( _out_win )
    cv2.imshow( _src_win, _default_img )
                        
    start_node()
    

