#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from robotnik_leds_sdk.srv import SetLeds, SetLedsRequest


class LedsControl:

	def __init__(self):

		# Init node
		rospy.init_node('leds_test_example_node')

		rospy.loginfo("Starting example...")		

        # Check if ALS module is already
		rospy.wait_for_service('leds_driver/command')

		rospy.loginfo("Running example!")


	def sendCommands(self):
		
		rospy.sleep(1)

		command = SetLedsRequest()
		rospy.wait_for_service('leds_driver/command')
		leds_driver_client = rospy.ServiceProxy('leds_driver/command', SetLeds)

		command.state = "STATE_1"
		command.enable = True
		leds_driver_client(command)
		

def main():

	leds_control = LedsControl()

	leds_control.sendCommands()


	while not rospy.is_shutdown():

		dummy = 1        



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
