#!/usr/bin/env python


import rospy
from std_msgs.msg import String



def shift_mode_led_driver(name, leds_zone, channel, type, color, direction, speed, sleep, enable):

    rospy.loginfo(name)


def blink_mode_led_driver(name, leds_zone, channel, type, color, ms_on, ms_off, enable):

    rospy.loginfo(name)


def paint_mode_led_driver(name, leds_zone, channel, type, color, enable):

    rospy.loginfo(name)





def main():

    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz


    while not rospy.is_shutdown():
        
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        #pub.publish(hello_str)
    
        #shift_mode_led_driver("left_led", [1,5], 0, "RGB", [0,0,20], "right", 1000, 1000, True)


        name_req = "led_front_right"
        state_req = "EMERGENCY"


        # Get led_config_list from rosparam
        list = rospy.get_param("/robotnik_leds/led_config_list")



        # Search the name of the required LEDs from led_config_list list

        _nameFound = False

        for i in range(0, len(list)):

            _name = list[i].get("name")
            
            if _name == name_req:

                _nameFound = True
                _leds_zone = list[i].get("leds_zone")
                _channel = list[i].get("channel")
                _type = list[i].get("type")

                rospy.loginfo("Name '" + _name + "' found:\n"+ 
                "led_zone: " + str(_leds_zone) +  "\n" + 
                "channel: "  + str(_channel)   +  "\n" +
                "type: "     + str(_type)      +  "\n" +
                "---------")


        if _nameFound == False:

            rospy.logerr("Name '" + name_req + "' not found in rosparam server. Check that the name exists in the led_config.yaml")



        #list = rospy.get_param("/robotnik_leds/led_state_list")
        #print(list)

         
           
         



        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass