# ROS nodes for the Cozmo Robot

Package that depends of [pycozmo](https://github.com/zayfod/pycozmo/) module (https://github.com/zayfod/pycozmo/) and at least on [ROS noetic](https://www.ros.org) (as it uses python3, needed by pycozmo).

- `cozmo_driver.py`: main node, publish camera and subscribes to cmd 
- `cozmo_blob_detector.py`: draft/work in progress to try to visualy detect cozmo cubes => NO cubes are detected.
- `cozmo_orb_detector.py`: draft/work in progress to try to visualy detect cozmo cubes => NO cubes are detected.
- `cozmo_joy.py`: teleoperation using some joystick

## install

- `pycozm` must be reachable
- make a link to `cozmo` in your `$ROS_WORKSPACE/src` dir
- build (I am fond of `catkin build`)
- source the `$ROS_WORKSPACE/devel/setup.[sh|zsh]` according to your shell
- run...

## run

- connect to the cozmo wifi
- `roslaunch cozmo cozmo_server.launch use_joystick:=true` => this will run the cozmo_driver node and the two cube detector nodes to play with
-- use_keyboard:=true|false (false)=> NOT implemented yet
-- use_joystick:=true|false (false)
-- joystick_dev:=... (/dev/input/by-id/usb-ACRUX_HAMA_X-Style_Pad-joystick) => my default joystick, do not buy one, it is no good.
-- joystick_map:=Hama|F710|Switch (Hama) => mapping for axis and buttons
