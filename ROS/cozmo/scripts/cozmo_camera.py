#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rospy
from cozmo.msg import TrackCmd
import numpy as np

import pycozmo

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

def global_camera(cli, image):
    print( "$$ GLOBAL camera" )

# *******************************************************************CozmoDriver
class CozmoCamera(object):
    """Main class for the Cozmo ROS Camera.

    Heavily depends on the pycozmo librairy
    ()

    Nothing is done until the run method is called.
    
    """

    # --------------------------------------------------------------------  init
    def __init__(self):
        # Connect to the Cozmo
        self.cli = pycozmo.Client()

        # subscriber to be created
        self.camera_pub = None

    # --------------------------------------------------------------- camera_cbk
    def camera_cbk(self, cli, image):
        """Callback for the camera complete 
        
        Convert pycozmo PIL image to openv (np.array) then
        to a ROS sensors_msg.Image
        """
        #print( "__camera_cbk" )
        np_img = np.array( image )

        img_msg = self.bridge.cv2_to_imgmsg( np_img, encoding="rgb8" )
        self.camera_pub.publish( img_msg )
        
    # ---------------------------------------------------------------------- run
    def run(self):
        """Initialize publisher and cv_bridge
        """
        # Connect toCozmo
        self.cli.start()
        self.cli.connect()
        self.cli.wait_for_robot()
        # Raise head
        angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) * 0.1
        self.cli.set_head_angle(angle)
        time.sleep(0.5)
        
        # create publisher
        self.camera_pub = rospy.Publisher("camera", Image )

        # add camera handler
        self.cli.enable_camera(enable=True, color=True)
        self.cli.add_handler( pycozmo.event.EvtNewRawCameraImage,
                              self.camera_cbk, one_shot=False)
        # self.cli.add_handler( pycozmo.event.EvtNewRawCameraImage,
        #                       global_camera, one_shot=False)
        
        # create bridge
        self.bridge = CvBridge()

        # main loop with watchdog
        try:
            while not rospy.is_shutdown():
                rospy.sleep( 0.05 )
        finally:
            print( "This is the end...." )
            self.cli.disconnect()
            self.cli.stop()
    

# ************************************************************************* main
def main():
    """Start node."""
    # initialize ROS node
    rospy.init_node('cozmo_camera')

    # create node object
    cn = CozmoCamera()
    cn.run()

# ********************************************************************** if_main
if __name__ == '__main__':
    main()
    

