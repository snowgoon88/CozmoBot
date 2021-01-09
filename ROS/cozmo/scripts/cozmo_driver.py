#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS driver for the Cozmo robot, relies on pycozmo.
#
# see CozmoDriver for more info


# TODO : camera on/off
# TODO : camera gray/color

import time
import pycozmo

import rospy
from cozmo.msg import TrackCmd
from std_msgs.msg import Header
from geometry_msgs.msg import (Twist,
                               Quaternion,
                               Pose,
                               PoseWithCovariance)
from sensor_msgs.msg import (BatteryState, Imu, Image, JointState)
from nav_msgs.msg import Odometry

import numpy as np
from cv_bridge import CvBridge, CvBridgeError


SPD_MAX = pycozmo.MAX_WHEEL_SPEED.mmps
COZMO_DIAM = 47 # mm
ROT_MAX = 2.0 * SPD_MAX / COZMO_DIAM

# ************************************************************************ utils
def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

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
    - /odom: publishes Odometry
       header.frame_id: '/map'
       child_frame_id: '/base_link'
       pose.pose.position: (pose.position.x, pose.position.y, pose.position.z)
       pose.pose.orientation: quaternion( 0, pose_pitch.radians, pose.rotation.angle_z.radians)
    TODO: covariance ?? 
    - /imu: publishes Imu
        orientation = quaternion( 0, 0, pose.rotation.angle_z.radians )
        linear_acceleraton: (accel.x, accel.y, accel.z)
        angular_velocity: (gyro.x, gyro.y, gyro.z)
    - /battery: publishes BatteryState
       present:  True
       voltage:  battery_voltage
    - /joints: publishes JointState
       name:     ['head','lift']
       position: [head_angle.radians, lift.position.height]
       velocity: [0.0, 0.0]
       effort:   [0.0, 0.0]

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

        # no subscriber, no publishing
        if self.camera_pub.get_num_connections() == 0:
            return
                
        #print( "__camera_cbk" )
        np_img = np.array( image )

        img_msg = self.bridge.cv2_to_imgmsg( np_img, encoding="rgb8" )
        img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = "cozmo_camera"
        self.camera_pub.publish( img_msg )

    # ----------------------------------------------------------- robot_state_cb
    def robot_state_cb(self, cli):
        """Callback for the EvtRobotStateUpdated"""
        #
        #   timestamp, pose_frame_id, pose_origin_id,
        #   pose_x, pose_y, pose_z, pose_angle_rad, pose_pitch_rad,
        #   lwheel_speed_mmps, rwheel_speed_mmps, head_angle_rad, lift_height_mm
        #   accel_x, accel_y, accel_z, gyro_x, gyro_y,
        #   gyro_z, battery_voltage, status,
        #   cliff_data_raw, backpack_touch_sensor_raw, curr_path_segment

        now = rospy.Time.now()
        self._publish_battery( now )
        self._publish_imu( now )
        self._publish_joint_state( now )
        self._publish_odom( now )

    def _publish_battery(self, now):
        """
        Publish battery as BatteryState message.

        """
        # Publish only if there are subscribers
        if self.battery_pub.get_num_connections() == 0:
            return

        self.battery_msg.header.stamp = now
        self.battery_msg.voltage      = self.cli.battery_voltage
        self.battery_pub.publish(self.battery_msg)

    def _publish_imu(self, now_time ):
        """IMU data as Imu"""

        #print( "__[CD] pub_imu with {}".format( self.imu_pub.get_num_connections()))
        # print( "  next {}".format( self.imu_pub.get_num_connections()))
        if self.imu_pub.get_num_connections() == 0:
            return
        
        #print( "  prepare msg ")
        self.imu_msg.header.stamp          = now_time
        #print( "  after time {}".format( self.imu_msg.header ))
        #print( "  pose_angle={}".format( self.cli.pose.rotation.angle_z.radians ))
        # NOT WORKING self.imu_msg.orientation = euler_to_quaternion(0.0, 0.0, self.cli.pose_angle)
        # WARN: pose is handled differently on z and y in client
        self.imu_msg.orientation           = euler_to_quaternion(0.0, 0.0, self.cli.pose.rotation.angle_z.radians )
        #print( "  after orient " )
        self.imu_msg.linear_acceleration.x = self.cli.accel.x * 0.001 # Units?
        self.imu_msg.linear_acceleration.y = self.cli.accel.y * 0.001 # Units?
        self.imu_msg.linear_acceleration.z = self.cli.accel.z * 0.001 # Units?
        #print( "  after accel" )
        self.imu_msg.angular_velocity.x    = self.cli.gyro.x # Units?
        self.imu_msg.angular_velocity.y    = self.cli.gyro.y # Units?
        self.imu_msg.angular_velocity.z    = self.cli.gyro.z # Units?
        #print( "  after gyro" )
                
        #print( "  will {}".format( self.imu_msg ))
        self.imu_pub.publish( self.imu_msg )
        #print( "  publish {}".format( self.imu_msg ))

    def _publish_joint_state(self, now):
        """
        Publish joint states as JointStates.

        """
        #print( "__PJS ")
        # Publish only if there are subscribers
        if self._joint_state_pub.get_subscription_count() == 0:
            return

        # TODO better transform of the Lift
        self.js_msg.header.stamp = now
        #print( "  head={}".format( self.cli.head_angle.radians ))
        #print( "  lift={}".format( self.cli.lift_position ))
        self.js_msg.position     = [self.cli.head_angle.radians,
                                     self.cli.lift_position.height.mm * 0.001]
        self.joint_state_pub.publish(self.js_msg)
        
    def _publish_odom(self, now):
        """
        Publish imu data as Imu.

        """
        #print( "__PO" )
        # Publish only if there are subscribers
        # if self.odom_pub.get_num_connections() == 0:
        #     return
        
        self.odom_msg.header.stamp          = now
        #print( " after time" )
        self.odom_msg.pose.pose.position.x  = self.cli.pose.position.x * 0.001 # Units?
        self.odom_msg.pose.pose.position.y  = self.cli.pose.position.y * 0.001 # Units?
        self.odom_msg.pose.pose.position.z  = self.cli.pose.position.z * 0.001 # Units?
        #print( "  after pose" )
        # WARN: pose is handled differently on z and y in client
        self.odom_msg.pose.pose.orientation = euler_to_quaternion(0.0, self.cli.pose_pitch.radians, self.cli.pose.rotation.angle_z.radians )
        #print( "  msg={}".format( self.odom_msg ))
        # TODO: covariance
        #self._odom_msg.pose.covariance       = np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
        # self._odom_msg.twist.twist.linear.x  = self._lin_vel
        # self._odom_msg.twist.twist.angular.z = self._ang_vel
        #self._odom_msg.twist.covariance      = np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()

        self.odom_pub.publish(self.odom_msg)
        #self._last_pose = deepcopy(self._odom_msg.pose.pose)

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

        # create messages
        self._odom_frame = "/map"
        self._base_frame = "/base_link"
        self.odom_msg = Odometry(header = Header(frame_id = self._odom_frame),
                                                child_frame_id = self._base_frame,
                                                pose = PoseWithCovariance())
        self.imu_msg = Imu( header=Header( frame_id = self._base_frame ))
        self.battery_msg            = BatteryState()
        self.battery_msg.present    = True
        self.js_msg                 = JointState()
        self.js_msg.header.frame_id = self._base_frame
        self.js_msg.name            = ['head', 'lift']
        self.js_msg.velocity        = [0.0, 0.0]
        self.js_msg.effort          = [0.0, 0.0]
        
        # create publisher
        self.camera_pub = rospy.Publisher("camera", Image, queue_size=1 )
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1 )
        self.imu_pub = rospy.Publisher("imu", Imu, queue_size=1 )
        self.battery_pub = rospy.Publisher( "battery", BatteryState, queue_size=1 )
        self.joint_state_pub = rospy.Publisher( "joints", JointState, queue_size=1 )
        # create bridge Opencv <-> ROS
        self.bridge = CvBridge()
        
        # add handlers
        self.cli.enable_camera(enable=True, color=True)
        self.cli.add_handler( pycozmo.event.EvtNewRawCameraImage,
                              self.camera_cbk, one_shot=False)

        self.cli.add_handler( pycozmo.event.EvtRobotStateUpdated,
                              self.robot_state_cb, one_shot=False)
        
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
    

