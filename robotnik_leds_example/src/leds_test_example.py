#!/usr/bin/env python

import rospy
from robotnik_leds_msgs.srv import SetLeds, SetLedsRequest


class LedsControl:

	def __init__(self):

		# Init node
		rospy.init_node('leds_test_example_node')

		rospy.loginfo("Starting example...")		

        # Check if ALS driver is already
		rospy.wait_for_service('led_command_interface/command')

		rospy.loginfo("Running example!")


	def setEffect(self):

		command = SetLedsRequest()
		rospy.wait_for_service('led_command_interface/command')
		leds_driver_client = rospy.ServiceProxy('led_command_interface/command', SetLeds)

		command.state = "STATE_1"
		command.enable = True
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_2"
		command.enable = True
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_3"
		command.enable = True
		leds_driver_client(command)

		rospy.sleep(1)	
		command.state = "STATE_4"
		command.enable = True
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_5"
		command.enable = True
		leds_driver_client(command)

	def clearEffect(self):

		command = SetLedsRequest()
		rospy.wait_for_service('led_command_interface/command')
		leds_driver_client = rospy.ServiceProxy('led_command_interface/command', SetLeds)

		command.state = "STATE_3"
		command.enable = False
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_5"
		command.enable = False
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_1"
		command.enable = False
		leds_driver_client(command)

		rospy.sleep(1)	
		command.state = "STATE_4"
		command.enable = False
		leds_driver_client(command)

		rospy.sleep(1)
		command.state = "STATE_2"
		command.enable = False
		leds_driver_client(command)
		

def main():

	leds_control = LedsControl()

	while not rospy.is_shutdown():

		leds_control.setEffect()
		rospy.sleep(5)
		leds_control.clearEffect()
		rospy.sleep(1)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
