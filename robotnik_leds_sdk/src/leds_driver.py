#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from robotnik_leds_sdk.srv import SetLeds, SetLedsResponse
from robotnik_leds_sdk.srv import LedEffects, LedEffectsRequest, LedEffectsResponse


class LedsDriver:


    def __init__(self):


        # Init node
        rospy.init_node('leds_driver_node')

        # Wait for the service offered by the Arduino hardware
        rospy.wait_for_service('/arduino_led_signaling/ack')
        rospy.wait_for_service('/arduino_led_signaling/set_led_properties')


        # Connect to the service offered by the Arduino hardware
        self.leds_driver_ack_service   = rospy.ServiceProxy('/arduino_led_signaling/ack', Trigger)
        self.leds_driver_effect_service = rospy.ServiceProxy('/arduino_led_signaling/set_led_properties', LedEffects)

        # Init service server
        leds_service = rospy.Service('/leds_driver/command', SetLeds, self.leds_service_callback)

        #Get name of this node
        self.node_name = rospy.get_name()

        #Get current time
        self.start_time = rospy.get_rostime().secs

        

    def effect_arduino_signaling_led(self, led_config, state_config, enable):

        led_effect_config = LedEffectsRequest()
        response = LedEffectsResponse()

        # Basic parameters

        led_effect_config.id        =  state_config.get("state")
        led_effect_config.mode      =  state_config.get("mode")
        led_effect_config.channel   =  led_config.get("channel")
        led_effect_config.type      =  led_config.get("type")
        led_effect_config.color_R   =  state_config.get("color")[0]
        led_effect_config.color_G   =  state_config.get("color")[1]
        led_effect_config.color_B   =  state_config.get("color")[2]
        led_effect_config.color_W   =  state_config.get("color")[3]
        led_effect_config.start_led =  led_config.get("leds_zone")[0]
        led_effect_config.end_led   =  led_config.get("leds_zone")[1]

        led_effect_config.enabled   =  enable

        # Optional parameters

        if state_config.get("ms_on") is not None:

            led_effect_config.ms_on = state_config.get("ms_on")


        if state_config.get("ms_off") is not None:

            led_effect_config.ms_off = state_config.get("ms_off")


        if state_config.get("direction") is not None:  

            led_effect_config.direction = state_config.get("direction")


        if state_config.get("speed") is not None: 

            led_effect_config.speed = state_config.get("speed")


        if state_config.get("sleep")is not None:

            led_effect_config.sleep = state_config.get("sleep")




        response = self.leds_driver_effect_service (led_effect_config)

        return response




    def get_config_params(self, led_name_req):

        # Get led_config_list from rosparam
        list = rospy.get_param(self.node_name + "/led_config_list")

        # Search the name of the required LEDs from led_config_list list

        _nameFound = False

        for i in range(0, len(list)):

            _led_name = list[i].get("led_name")
            
            if _led_name == led_name_req:

                _nameFound = True
                _led_config = list[i]

                #_leds_zone = list[i].get("leds_zone")
                #_channel = list[i].get("channel")
                #_type = list[i].get("type")

                #rospy.loginfo("Name '" + _led_name + "' found:\n"+ 
                #"led_zone: " + str(_leds_zone) +  "\n" + 
                #"channel: "  + str(_channel)   +  "\n" +
                #"type: "     + str(_type)      +  "\n" +
                #"---------")


        if _nameFound == False:

            rospy.logerr("Led name '" + led_name_req + "' not found in rosparam server. Check that the name exists in the led_config.yaml")
            _led_config = {}


        return _led_config




    def get_state_params(self, led_state_req):

        # Get led_state_list from rosparam
        list = rospy.get_param(self.node_name + "/led_state_list")

        # Search the name of the required state from led_state_list list

        _stateFound = False

        for i in range(0, len(list)):

            _state_name = list[i].get("state")
            
            if _state_name == led_state_req:

                _stateFound = True
                _state_config = list[i]

                # _state_config[0] = _stateFound
                #rospy.loginfo("State '" + _state_name + "' found")

        if _stateFound == False:

            rospy.logerr("State '" + led_state_req + "' not found in rosparam server. Check that the name exists in the led_state.yaml")
            _state_config = {}


        return _state_config         




    def leds_service_callback(self, req):

        res = SetLedsResponse()

        led_state_req = req.state
        led_enable_req = req.enable

        #If name_req or state_req does not exist return a empty list
        state_config = self.get_state_params(led_state_req)
        led_name = state_config.get("led_name")
        led_config = self.get_config_params(led_name)


        #Check if dictionary is not empty
        if bool(led_config) and bool (state_config):

            #self.set_mode_arduino_signaling_led(led_config,state_config, led_enable_req )      
            self.effect_arduino_signaling_led(led_config, state_config, led_enable_req)
            res.success = True
            res.message = str(led_name) + " has been set to " + str(led_state_req) + " mode and is " + str (led_enable_req)

        else:
            res.success = True
            res.message = "Error: led_name or state_name does not exist"


        return res


    def hold_connection(self):

        current_time = rospy.get_rostime().secs
        
        if (current_time -self.start_time) >= 2:

            self.start_time = current_time
            self.leds_driver_ack_service();




def main():

    leds_driver = LedsDriver()

    while not rospy.is_shutdown():

        leds_driver.hold_connection()


       
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass