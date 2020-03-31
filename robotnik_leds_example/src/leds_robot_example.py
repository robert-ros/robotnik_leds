#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from robotnik_leds_sdk.srv import SetLeds, SetLedsRequest


class LedsControl:

	def __init__(self):

		# Init node
		rospy.init_node('leds_robot_example_node')

		#Get name of this node
		self.node_name = rospy.get_name()

		#Get cmd_vel topic
		cmd_vel_topic_name = rospy.get_param(self.node_name + "/cmd_vel")

		# Subscribe to cmd_vel robot 
		rospy.Subscriber(cmd_vel_topic_name, Twist, self.cmd_vel_callaback, queue_size=1)

		rospy.loginfo("Starting example...")		

        # Check if ALS module is already
		rospy.wait_for_service('leds_driver/command')

		rospy.loginfo("Running example!")


	def cmd_vel_callaback(self, msg):

		command = SetLedsRequest()
		rospy.wait_for_service('leds_driver/command')
		leds_driver_client = rospy.ServiceProxy('leds_driver/command', SetLeds)

		vel_x = msg.linear.x
		vel_y = msg.linear.y
		vel_z = msg.angular.z


		if vel_x <= 0.01 and vel_x >= -0.01 and vel_y <= 0.01 and vel_y >= -0.01 and vel_z <= 0.01 and vel_z >= -0.01:

			#Important: First disable effects, then activate effects
			
			print("Robot stops!")

			command.state = "MOVING"
			command.enable = False
			leds_driver_client(command)

			command.state = "STOP"
			command.enable = True
			leds_driver_client(command)


		else:

			#Important: First disable effects, then activate effects
			
			print("Robot moves!")

			command.state = "STOP"
			command.enable = False
			leds_driver_client(command)

			command.state = "MOVING"
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
