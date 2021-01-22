#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from cozmo.msg import TrackCmd
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

## Joystick mapping for HAMA-X-style
# axes: [
#0   -1:droiteJL,1:gaucheJL,
#1   -1:arrièreJL,1:avantJL,
#2   -1:droiteJR,1:gaucheJR,
#3   -1:arrièreJR,1:avantJR,
#4   -1:RT,1LT
#5   -1:droitePL,1:gauchePL,
#6   -1:arrièrePL,1:avantPL]
#
# buttons: [1V,2R,3B,4J,LB,RB,LT,RT,9,10,JL,JR]

hama_mapping = {
    'axis': { 'leftW':  1, 'rightW': 3,
              'forward': 1, 'rotation': 0, 'lift_fork': 3, 'head': 2},
    'btn':  { 'deadman': 4, 'eat': 1, 'drink': 2, 'none': 0,
              'reset': 8, 'play':9,
              'switch_mode': 8 }
}
logitec_710 = {
    'axis': { 'leftW':  1, 'rightW': 4,
              'forward': 1, 'rotation': 0, 'lift_fork': 3, 'head': 4},
    'btn':  { 'deadman': 4, 'eat': 1, 'drink': 2, 'none': 0,
    'switch_mode': 6 }
}
switch_pro = {
    'axis': { 'leftW': 1, 'rightW': 3,
              'forward': 1, 'rotation': 0, 'lift_fork': 3, 'head': 2},
    # deadman is L, switch_mode is little square btn
    'btn': {'deadman': 5, 'switch_mode': 4}
}

ANTI_RBD_TIME = 0.2 # in seconds
MAX_LIFT_SPD = 3.14 # rad/s
MAX_HEAD_SPD = 3.14 # rad/s

# *************************************************************** CozmoJoyTeleop
class CozmoJoyTeleop( object ):
    """Joystick to Cozmo ROS commands.

    Rosparam:
    - joymap [string]: which mapping (F710,Hama,Switch)

    """

    def __init__(self):
        """Initialize joystick."""
        self.vel_mode = True
        # publishers
        self.track_cmd_pub = rospy.Publisher( '/track_cmd', TrackCmd,
                                              queue_size=10 )
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Mapping joystick and action
        self.joystick_mapping = rospy.get_param( "~joymap", "Hama" )
        rospy.loginfo( "CozmoJoy mapping={}".format( self.joystick_mapping ))
        if self.joystick_mapping == "Hama" :
            self.axis_map = hama_mapping['axis']
            self.btn_map = hama_mapping['btn']
        elif self.joystick_mapping == "F710" :
            self.axis_map = logitec_710['axis']
            self.btn_map = logitec_710['btn']
        elif self.joystick_mapping == "Switch" :
            self.axis_map = switch_pro['axis']
            self.btn_map = switch_pro['btn']

        # internal publication flag to tell what we must send a last wheel_cmd
        self.last_pub = False

        # internal time of last btn command
        self.last_btn_time = rospy.get_time()
        
    def joy_cbk(self, msg):
        """Use joy axis/button to publish to `/track_cmd`.
        Only if the `deadman` button is active.
        """
        axis_map = self.axis_map
        btn_map = self.btn_map
        if msg.buttons[btn_map['deadman']]:
            if self.vel_mode:
                cmd_vel = Twist()
                cmd_vel.linear.x = msg.axes[axis_map['forward']]
                cmd_vel.angular.z = msg.axes[axis_map['rotation']]
                cmd_vel.angular.y = msg.axes[axis_map['lift_fork']] * MAX_LIFT_SPD
                cmd_vel.angular.x = msg.axes[axis_map['head']] * MAX_HEAD_SPD
                self.last_pub = True
                self.cmd_vel_pub.publish( cmd_vel )
            else:
                track_cmd = TrackCmd()
                track_cmd.header.stamp = rospy.Time.now()
                track_cmd.vel_left = msg.axes[axis_map['leftW']]
                track_cmd.vel_right = msg.axes[axis_map['rightW']]
                self.last_pub = True
                self.track_cmd_pub.publish( track_cmd )

        elif self.last_pub:
            if self.vel_mode:
                cmd_vel = Twist() # 0 by default
                self.cmd_vel_pub.publish( cmd_vel )
            else:
                # by default, cmd is 0,0 so publish
                track_cmd = TrackCmd()
                track_cmd.header.stamp = rospy.Time.now()
                self.track_cmd_pub.publish( track_cmd )
            self.last_pub = False

        # Btns
        if rospy.get_time() - self.last_btn_time > ANTI_RBD_TIME:
            if msg.buttons[btn_map['switch_mode']]:
                self.last_btn_time = rospy.get_time()
                self.vel_mode = not self.vel_mode
                if self.vel_mode:
                    print( "__CozmoJoy: switch to VEL_MODE" )
                else:
                    print( "__CozmoJoy: switch to TRACK_MODE" )
                
            #     elif msg.buttons[btn_map['play']]:
            #         self.server_cmd_pub.publish( "play" )
            #         return
            #     elif msg.buttons[btn_map['eat']]:
            #         consume_cmd.header.stamp = rospy.Time.now()
            #         consume_cmd.consume = 'eat'
            #         self.last_time = rospy.get_time()
            #         self.cons_cmd_pub.publish( consume_cmd )
            #     elif msg.buttons[btn_map['drink']]:
            #         consume_cmd.header.stamp = rospy.Time.now()
            #         consume_cmd.consume = 'drink'
            #         self.last_time = rospy.get_time()
            #         self.cons_cmd_pub.publish( consume_cmd )
            #     elif msg.buttons[btn_map['none']]:
            #         consume_cmd.header.stamp = rospy.Time.now()
            #         consume_cmd.consume = 'none'
            #         self.last_time = rospy.get_time()
            #         self.cons_cmd_pub.publish( consume_cmd )


    def run(self):
        """Initialize subscriber and start node."""
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cbk)
        rospy.spin()

# ************************************************************************* main
def main():
    """Start node."""
    # initialize ROS node
    rospy.init_node('cosmo_joy')
    # create instance and run node
    cjt = CozmoJoyTeleop()
    cjt.run()

# ********************************************************************** if_main
if __name__ == '__main__':
    main()
