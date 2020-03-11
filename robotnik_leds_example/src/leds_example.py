#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from robotnik_leds_sdk.srv import SetLeds, SetLedsRequest


class LedsControl:

	def __init__(self):

		# Init node
		rospy.init_node('leds_example_node')

		# Subscribe to cmd_vel robot 
		rospy.Subscriber("/robot/robotnik_base_control/cmd_vel", Twist, self.cmd_vel_callaback)
        

		rospy.loginfo("Waiting Arduino Lighting Signaling (ALS module) ...")		

        # Check if ALS module is already
		rospy.wait_for_service('/leds_driver/command')

		rospy.loginfo("ALS module activated, node already! ")


	def cmd_vel_callaback(self, msg):

		command = SetLedsRequest()
		rospy.wait_for_service('/leds_driver/command')
		leds_driver_client = rospy.ServiceProxy('/leds_driver/command', SetLeds)

		vel_x = msg.linear.x
		vel_y = msg.linear.y
		vel_z = msg.angular.z


		if vel_x == 0 and vel_y == 0 and vel_z == 0:

			#Important: First disable effects, then activate effects
			
			print("Robot stops!")

			
			command.state = "TURN_LEFT"
			command.enable = False
			leds_driver_client(command)

			command.state = "TURN_RIGHT"
			command.enable = False
			leds_driver_client(command)

			command.state = "STOP"
			command.enable = True
			leds_driver_client(command)


		else:

			print("Robot moves!")

			command.state = "STOP"
			command.enable = False
			leds_driver_client(command)


			command.state = "TURN_LEFT"
			command.enable = True
			leds_driver_client(command)

			command.state = "TURN_RIGHT"
			command.enable = True
			leds_driver_client(command)


		


def main():

	leds_control = LedsControl()


	

	while not rospy.is_shutdown():

		dummy = 1        




if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass