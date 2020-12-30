#!/bin/sh

# Shell commands follow to lauchn using the python with ROS_PYTHON_VERSION
# Next line is bilingual: it starts a comment in Python, and is a no-op in shell
""":"

# Find a suitable python interpreter (adapt for your specific needs)
cmd=/usr/bin/python${ROS_PYTHON_VERSION}
command -v > /dev/null $cmd && exec $cmd $0 "$@"

echo "OMG Python${ROS_PYTHON_VERSION} not found, exiting!!!!!11!!eleven" >2

exit 2

":"""
# Previous line is bilingual: it ends a comment in Python, and is a no-op in shell
# Shell commands end here

import curses
import math

import rospy
from animat.msg import WheelCmd
from animat.msg import ConsumeCmd
from std_msgs.msg import String

class TextWindow():

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


class AnimatKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        # DEL self._pub_cmd = rospy.Publisher('key_vel', Twist)
        # publishers
        self.wheel_cmd_pub = rospy.Publisher( '/animat/wheel_cmd', WheelCmd,
                                              queue_size=10 )
        self.cons_cmd_pub = rospy.Publisher( '/animat/consume_cmd', ConsumeCmd,
                                             queue_size=10 )
        self.server_cmd_pub = rospy.Publisher( '/animat/server_cmd', String, queue_size=10)
        self._hz = rospy.get_param('~hz', 10)
        self._last_pressed = {}

        self.vel_cmd = False
        self.consume_cmd = False
        self.server_cmd = False
        
        #self._forward_rate = rospy.get_param('~forward_rate', 0.8)

        self._vel_rate = 0.1
        self._vel_left = 0
        self._vel_right = 0
        self._consume = 'none'
        self._server_msg = 'none'

    movement_bindings = {
        curses.KEY_UP:    ( 1,  1, '-'),
        curses.KEY_DOWN:  (-1,  -1, '-'),
        curses.KEY_LEFT:  ( -0.5,  0.5, '-'),
        curses.KEY_RIGHT: ( 0.5, -0.5, '-'),
        101:     ( 0, 0, 'eat'),   # e
        69:     ( 0, 0, 'eat'),    # E
        100:     ( 0, 0, 'drink'), # d
        68:     ( 0, 0, 'drink'),  # D
        115:     ( 0, 0, '-'),     # s
        83:     ( 0, 0, '-'),      # S
        112:    ( 0, 0, 'play' ), #p
        80:    ( 0, 0, 'play' ),  #P
        111:    ( 0, 0, 'pause' ), #o
        79:    ( 0, 0, 'pause' ),  #O
        114:    ( 0, 0, 'reset' ),      #r
        82:    ( 0, 0, 'reset' ),       #R
        
    }
    # s/S (115/83) : STOP

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                ##print( "CURSE keycode=",keycode )
                ##rospy.loginfo( "CURSE key={}".format( keycode ))
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._set_cmd()
            self._publish()
            rate.sleep()

    def clamp_vel(self):
        if self._vel_left > 1.0:
            self._vel_left = 1.0
        if self._vel_left < -1.0:
            self._vel_left = -1.0            
        if self._vel_right > 1.0:
            self._vel_right = 1.0
        if self._vel_right < -1.0:
            self._vel_right = -1.0            
        

    def _set_cmd(self):
        now = rospy.get_time()
        keys = []
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.2:
                keys.append(a)
        leftw = 0.0
        rightw = 0.0
        for k in keys:
            if k == 115 or k == 83: # STOP
                leftw = 0
                rightw = 0
                self._vel_left = 0
                self._vel_right = 0
                self.vel_cmd = True
                break
            elif k == 112 or k == 80: # p/P play
                self._server_msg = "play"
                self.play = True
                self.server_cmd = True
                break
            elif k == 111 or k == 79: # o/O pause
                self._server_msg = "pause"
                self.play = True
                self.server_cmd = True
                break            
            elif k == 114 or k == 82: # r/R Reset
                self._server_msg = "reset"
                self.server_cmd = True
                break
            cmd_l, cmd_r, cmd_consume = self.movement_bindings[k]
            leftw += cmd_l
            rightw += cmd_r
            if cmd_consume != '-':
                self._consume = cmd_consume
                self.consume_cmd = True

        self._vel_left += leftw * self._vel_rate
        self._vel_right += rightw * self._vel_rate
        self.clamp_vel()
        self.vel_cmd = True
            
    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, "LEFT_vel: {:3.2f}\tRIGHT_vel: {:3.2f}".
                                   format( self._vel_left, self._vel_right ))
        self._interface.write_line(3, "last consume: {}".format( self._consume ))

        self._interface.write_line(1, "Server: p/P PLAY/PAUSE; r/R RESET" )
        self._interface.write_line(5, "Use arrow keys to move, s/S:STOP, e/E:eat, d/D:drink,    q to exit." )
        self._interface.refresh()

        if self.vel_cmd:
            wheel_cmd = WheelCmd()
            wheel_cmd.header.stamp = rospy.Time.now()
            wheel_cmd.vel_left = self._vel_left
            wheel_cmd.vel_right = self._vel_right
            self.wheel_cmd_pub.publish( wheel_cmd )
            self.vel_cmd = False

            
        if self.consume_cmd:
            consume_cmd = ConsumeCmd()
            consume_cmd.header.stamp = rospy.Time.now()
            consume_cmd.consume = self._consume
            self.cons_cmd_pub.publish( consume_cmd )
            self._consume = '-'
            self.consume_cmd = False

        if self.server_cmd:
            self.server_cmd_pub.publish( self._server_msg )
            self.server_cmd = False
        

def main(stdscr):
    rospy.init_node('animat_key')
    app = AnimatKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
