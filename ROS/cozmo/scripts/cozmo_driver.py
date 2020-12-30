#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS driver for the Cozmo robot, relies on pycozmo.
#
# see CozmoDriver for more info


# TODO : camera on/off
# TODO : camera gray/color
# TODO : vel / tank

import time
import rospy
from cozmo.msg import TrackCmd
from geometry_msgs.msg import Twist

import pycozmo

import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

SPD_MAX = pycozmo.MAX_WHEEL_SPEED.mmps
COZMO_DIAM = 47 # mm
ROT_MAX = 2.0 * SPD_MAX / COZMO_DIAM

# *******************************************************************CozmoDriver
class CozmoDriver(object):
    """Main class for the Cozmo ROS driver.

    TODO: have eye animation while moving

    Heavily depends on the pycozmo librairy
    (https://github.com/zayfod/pycozmo/)

    Topics:
    - /camera: publishes RGB uint8 320x240 images from the camera, ~14 fps
    - /vel_cmd: subscribes to Twist messages where
       linear.x: forward speed (backward if negative)
       angular.z: rotation speed around the vertical axis
       angular.x: rotation speed for head
       angular.y: rotation speed for lift
    TODO: set linear.x and angular.z to m/s and rad/s
    - /track_cmd: subscribes to TrackCmd messages where
       vel_left: forward speed for the left track
       vel_right: forward speed for the right trac
    TODO: set vel_left and vel_right to m/s

    Nothing is done until the run method is called.

    Watchdog : if too long since last time the robot moved
                  (now - last_time > 0.95 * intervale)
               then stop it
                  (send_velocity_command( 0, 0 )
    """

    # --------------------------------------------------------------------  init
    def __init__(self):
        # Connect to the Cozmo
        self.cli = pycozmo.Client()
        self.speed_left = 0.0
        self.speed_right = 0.0

        # subscriber to be created
        self.trackcmd_sub = None
        self.cmdvel_sub = None
        
        # publisher to be created
        self.camera_pub = None

        # watchdog state
        self.watchdog_interval = 1.0
        self.last_moved = None
        self.last_time = None

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
        
    # ------------------------------------------------------------- trackcmd_cbk
    def trackcmd_cbk(self, message):
        """Callback for the 'trackcmd' message.
        
        Receive Cozmo TrackCmd and send drive_wheels.
        """
        spd_left = message.vel_left
        spd_right = message.vel_right

        # print( "** TODO ** transform {}, {} to drive_wheels".format(
        #     spd_left, spd_right ))
        lw = int(spd_left * pycozmo.MAX_WHEEL_SPEED.mmps)
        rw = int(spd_right * pycozmo.MAX_WHEEL_SPEED.mmps)
        self.send_velocity_command( lw, rw )

    # --------------------------------------------------------------- cmdvel_cbk
    def cmdvel_cbk(self, message):
        """
        Receives ROS geometry_msgs/Twist message, compute vleft and vright
        linear.x : linear speed forward
        angular.z : rotation speed around vertical axes
        angular.y : angular speed for forklift rotation
        angular.x : angular speed for head rotation

        spd = (vl + vr) / 2 ; rot = (vr - vl) / d
        so
        vr = (2*spd + d*rot)/2 ; vl = (2*spd - d*rot)/2
        """
        spd = message.linear.x * SPD_MAX
        rot = message.angular.z * ROT_MAX

        vr = (2.0 * spd + COZMO_DIAM * rot)/2.0
        vl = (2.0 * spd - COZMO_DIAM * rot)/2.0
        vr = np.clip( vr, -pycozmo.MAX_WHEEL_SPEED.mmps,
                      pycozmo.MAX_WHEEL_SPEED.mmps )
        vl = np.clip( vl, -pycozmo.MAX_WHEEL_SPEED.mmps,
                      pycozmo.MAX_WHEEL_SPEED.mmps )
        self.send_velocity_command( vl, vr )

        lift_rot = message.angular.y
        self.cli.move_lift( lift_rot )

        head_rot = message.angular.x
        self.cli.move_head( head_rot )

    # ---------------------------------------------------- send_velocity_command
    def send_velocity_command(self, left_spd, right_speed):
        self.cli.drive_wheels( lwheel_speed=left_spd, rwheel_speed=right_speed)

    # ---------------------------------------------------------------------- run
    def run(self):
        """Initialize subscribers and implement watchdog.
        """
        # Connect toCozmo
        self.cli.start()
        self.cli.connect()
        self.cli.wait_for_robot()
        # Raise head
        angle = (pycozmo.robot.MAX_HEAD_ANGLE.radians - pycozmo.robot.MIN_HEAD_ANGLE.radians) * 0.1
        self.cli.set_head_angle(angle)
        time.sleep(0.5)
        
        # init watchdog state
        self.last_time = rospy.get_rostime()
        self.last_moved = False
        
        # create subscribers
        self.trackcmd_sub = rospy.Subscriber( 'track_cmd', TrackCmd,
                                              self.trackcmd_cbk )
        self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist,
                                           self.cmdvel_cbk )

        # create publisher
        self.camera_pub = rospy.Publisher("camera", Image, queue_size=10 )

        # create bridge Opencv <-> ROS
        self.bridge = CvBridge()
        
        # add camera handler
        self.cli.enable_camera(enable=True, color=True)
        self.cli.add_handler( pycozmo.event.EvtNewRawCameraImage,
                              self.camera_cbk, one_shot=False)
        
        # main loop with watchdog
        try:
            while not rospy.is_shutdown():
                now = rospy.get_rostime()
                if (self.last_moved and
                    ((now - self.last_time).to_sec() >
                     0.995 * self.watchdog_interval)):
                    rospy.loginfo('too long since last command: stopping '
                                  'robot')
                    self.send_velocity_command(0., 0.)
                rospy.sleep(self.watchdog_interval/100.)
        finally:
            if self.last_moved:
                rospy.loginfo('stopping robot before quitting')
                self.send_velocity_command(0., 0.)   
            print( "This is the end...." )
            self.cli.stop_all_motors()
            self.cli.disconnect()
            self.cli.stop()
    

# ************************************************************************* main
def main():
    """Start node."""
    # initialize ROS node
    rospy.init_node('cozmo_driver')

    # create node object
    cn = CozmoDriver()
    cn.run()

# ********************************************************************** if_main
if __name__ == '__main__':
    main()
    

