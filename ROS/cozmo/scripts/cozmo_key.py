#!/usr/bin/env python3
# -*- coding: utf-8 -*-

""" Keyboard control for Cozmo

Publish TrackCmd on /track_cmd
Subscribes to /battery, /joints

QZSD for movements, using TrackCmd
X    for stop

"""

import curses
import math

import rospy
from cozmo.msg import TrackCmd
from sensor_msgs.msg import (BatteryState, JointState)

# ******************************************************************************
# ******************************************************************* TextWindow
# ******************************************************************************
class TextWindow():
    """ mid level API to facilitate using curses
    """
    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError('lineno out of bounds')
        height, width = self._screen.getmaxyx()
        y = int((height / self._num_lines) * lineno)
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()

# ******************************************************************************
# *************************************************************** CozmoKeyTeleop
# ******************************************************************************
class CozmoKeyTeleop():
    # ----------------------------------------------------------------- __init__
    def __init__(self, interface):
        self._interface = interface
        # DEL self._pub_cmd = rospy.Publisher('key_vel', Twist)
        # subcribers
        self.battery_sub = rospy.Subscriber( 'battery', BatteryState,
                                             self._battery_cb )
        self.jointstate_sub = rospy.Subscriber( 'joints', JointState,
                                                self._joints_cb )
        # publishers
        self.track_cmd_pub = rospy.Publisher( '/track_cmd', TrackCmd,
                                              queue_size=10 )
        self._hz = rospy.get_param('~hz', 10)
        self._last_pressed = {}

        self.vel_cmd = False

        self.cozmo_battery_volt = 0.0
        self.cozmo_head_angle = 0.0
        self.cozmo_lift_heigh = 0.0
        
        #self._forward_rate = rospy.get_param('~forward_rate', 0.8)

        self._vel_rate = 0.1
        self._vel_left = 0
        self._vel_right = 0

    movement_bindings = {
        curses.KEY_UP:    ( 1,  1, '-'),
        122: ( 1,  1, '-'), # z
        90: ( 1,  1, '-'),  # Z
        curses.KEY_DOWN:  (-1,  -1, '-'),
        115:     ( -1, -1, '-'),     # s
        83:     ( -1, -1, '-'),      # S
        curses.KEY_LEFT:  ( -0.5,  0.5, '-'),
        113: ( -0.5,  0.5, '-'),     # q
        81:  ( -0.5,  0.5, '-'),     # Q
        curses.KEY_RIGHT: ( 0.5, -0.5, '-'),
        100: ( 0.5, -0.5, '-'),      # d
        68: ( 0.5, -0.5, '-'),       # D
        120: ( 0.0, 0.0, '-'),      # x
        88: ( 0.0, 0.0, '-'),       # X
    }
    # x/X (120/88) : STOP

    # ---------------------------------------------------------------------- run
    def run(self):
        rospy.loginfo( "__CozmoKey running........" )
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                #print( "CURSE keycode=",keycode )
                rospy.logdebug( "CURSE key={}".format( keycode ))
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_cmd()
            self._publish()
            rate.sleep()

    # ----------------------------------------------------------------- clam_vel
    def clamp_vel(self):
        if self._vel_left > 1.0:
            self._vel_left = 1.0
        if self._vel_left < -1.0:
            self._vel_left = -1.0            
        if self._vel_right > 1.0:
            self._vel_right = 1.0
        if self._vel_right < -1.0:
            self._vel_right = -1.0

    # ---------------------------------------------------------------- _callback
    def _battery_cb(self, BatteryState_msg):
        self.cozmo_battery_volt = BatteryState_msg.voltage
    def _joints_cb(self, JointState_msg):
        self.cozmo_head_angle = JointState_msg.position[0]
        self.cozmo_lift_heigh = JointState_msg.position[1]

    # ----------------------------------------------------------------- _set_cmd
    def _set_cmd(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.05:
                keys.append(a)

        rospy.logdebug( "__[KC] _set_cmd key={}".format( keys ))
        leftw = 0.0
        rightw = 0.0
        for k in keys:
            if k == ord('x') or k == ord('X'): # STOP
                leftw = 0
                rightw = 0
                self._vel_left = 0
                self._vel_right = 0
                self.vel_cmd = True
                rospy.logdebug( "__[CK] STOP" )
                break

            rospy.logdebug( "__[CK] movement_key" )
            cmd_l, cmd_r, cmd_str = self.movement_bindings[k]
            leftw += cmd_l
            rightw += cmd_r

        self._vel_left += leftw * self._vel_rate
        self._vel_right += rightw * self._vel_rate
        self.clamp_vel()
        self.vel_cmd = True

    # ------------------------------------------------------------- _key_pressed
    def _key_pressed(self, keycode):
        rospy.logdebug( "__[KC] key_pressed kc={}".format( keycode ))
        if keycode == 27: # ESC
            rospy.logdebug( "__[CK] ESC => quitting" )
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    # ----------------------------------------------------------------- _publish
    def _publish(self):
        self._interface.clear()
        self._interface.write_line(1, "COZMO battery: {:3.1f} V".format( self.cozmo_battery_volt ))
        self._interface.write_line(2, "      head={}\tlift={}".format( self.cozmo_head_angle, self.cozmo_lift_heigh ))
        self._interface.write_line(3, "LEFT_vel: {:3.2f}\tRIGHT_vel: {:3.2f}".
                                   format( self._vel_left, self._vel_right ))
        # self._interface.write_line(3, "last consume: {}".format( self._consume ))

        # self._interface.write_line(1, "Server: p/P PLAY/PAUSE; r/R RESET" )
        self._interface.write_line(5, "Use Z/S Q/D to move, x/X:STOP,   ESC to exit." )
        self._interface.refresh()

        if self.vel_cmd:
            track_cmd = TrackCmd()
            track_cmd.header.stamp = rospy.Time.now()
            track_cmd.vel_left = self._vel_left
            track_cmd.vel_right = self._vel_right
            self.track_cmd_pub.publish( track_cmd )
            self.vel_cmd = False


# ************************************************************************* main
def main(stdscr):
    rospy.init_node('cozmo_key')
    app = CozmoKeyTeleop(TextWindow(stdscr))
    app.run()

# *********************************************************************** ifmain
if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        print( "******* ROSInterruptException ***** in cozmo_key" )
