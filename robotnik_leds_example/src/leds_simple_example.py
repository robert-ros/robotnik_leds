#!/usr/bin/env python

import rospy
from robotnik_leds_msgs.srv import SetLeds, SetLedsRequest

command = SetLedsRequest()
rospy.wait_for_service('led_command_interface/command')
leds_driver_client = rospy.ServiceProxy('led_command_interface/command', SetLeds)

command.state = "EMERGENCY"
command.enable = True
leds_driver_client(command)

