#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from robotnik_leds_sdk.srv import SetLeds, SetLedsResponse
from robotnik_leds_sdk.srv import LedsPaint, LedsPaintRequest, LedsPaintResponse
from robotnik_leds_sdk.srv import LedsBlink, LedsBlinkRequest, LedsBlinkResponse
from robotnik_leds_sdk.srv import LedsShift, LedsShiftRequest, LedsShiftResponse


class LedsDriver:
    def __init__(self):

        # Init node
        rospy.init_node('leds_driver_node')


        # Wait for the service offered by the Arduino hardware
        rospy.wait_for_service('/arduino_signaling_led/set_leds/paint_mode')
        rospy.wait_for_service('/arduino_signaling_led/set_leds/blink_mode')
        rospy.wait_for_service('/arduino_signaling_led/set_leds/shift_mode')


        # Connect to the service offered by the Arduino hardware
        self.leds_driver_paint_service = rospy.ServiceProxy('/arduino_signaling_led/set_leds/paint_mode', LedsPaint)
        self.leds_driver_blink_service = rospy.ServiceProxy('/arduino_signaling_led/set_leds/blink_mode', LedsBlink)
        self.leds_driver_shift_service = rospy.ServiceProxy('/arduino_signaling_led/set_leds/shift_mode', LedsShift)

        # Init service
        leds_service = rospy.Service('/leds_driver/command', SetLeds, self.leds_service_callback)

        #Get name of this node
        self.node_name = rospy.get_name()


    def set_mode_arduino_signaling_led(self, led_config, state_config, enable):

        #Enable assignation
        _enable = enable
        
        #Led config assignation
        _name      = led_config.get("name")
        _leds_zone = led_config.get("leds_zone")
        _channel   = led_config.get("channel")
        _type      = led_config.get("type")

        #State config assignation
        _mode = state_config.get("mode")


        #Send the configuration to led device
        if _mode == "paint":

            _color = state_config.get("color")

            self.paint_mode_arduino_signaling_led(_name, _leds_zone, _channel, _type, _color, _enable)


        if _mode == "blink":

            _color = state_config.get("color")
            _ms_on = state_config.get("ms_on")
            _ms_off = state_config.get("ms_off")

            self.blink_mode_arduino_signaling_led(_name, _leds_zone, _channel, _type, _color, _ms_on, _ms_off, _enable)


        if _mode == "shift":

            _color     =  state_config.get("color")
            _direction =  state_config.get("direction")
            _speed     =  state_config.get("speed")
            _sleep     =  state_config.get("sleep")

            self.shift_mode_arduino_signaling_led(_name, _leds_zone, _channel, _type, _color, _direction, _speed, _sleep, _enable)



    def shift_mode_arduino_signaling_led(self, name, leds_zone, channel, type, color, direction, speed, sleep, enable):

        print("==============")
        rospy.loginfo("Shift mode!")
        print(name)
        print(leds_zone)
        print(channel)
        print(type)
        print("------")
        print(color)
        print(direction)
        print(speed)
        print(sleep)
        print(enable)
        print("==============")

        leds_shift_config = LedsShiftRequest()
        response = LedsShiftResponse()

        leds_shift_config.shift_id = name
        leds_shift_config.color_R = color[0]
        leds_shift_config.color_G = color[1]
        leds_shift_config.color_B = color[2]
        leds_shift_config.start_led = leds_zone[0]
        leds_shift_config.end_led = leds_zone[1]
        leds_shift_config.direction = direction
        leds_shift_config.speed = speed
        leds_shift_config.sleep = sleep
        leds_shift_config.enabled = enable

        response = self.leds_driver_shift_service (leds_shift_config)

        return response



    def blink_mode_arduino_signaling_led(self, name, leds_zone, channel, type, color, ms_on, ms_off, enable):

        print("==============")
        rospy.loginfo("Blink mode!")
        print(name)
        print(leds_zone)
        print(channel)
        print(type)
        print("------")
        print(color)
        print(ms_on)
        print(ms_off)
        print(enable)
        print("==============")

        leds_blink_config = LedsBlinkRequest()
        response = LedsBlinkResponse()

        leds_blink_config.blink_id = name
        leds_blink_config.color_R = color[0]
        leds_blink_config.color_G = color[1]
        leds_blink_config.color_B = color[2]
        leds_blink_config.start_led = leds_zone[0]
        leds_blink_config.end_led = leds_zone[1]
        leds_blink_config.ms_on = ms_on
        leds_blink_config.ms_off = ms_off
        leds_blink_config.enabled = enable

        response = self.leds_driver_blink_service (leds_blink_config)

        return response


    def paint_mode_arduino_signaling_led(self, name, leds_zone, channel, type, color, enable):

        print("==============")
        rospy.loginfo("Paint mode!")
        print(name)
        print(leds_zone)
        print(channel)
        print(type)
        print("------")
        print(color)
        print(enable)
        print("==============")

        leds_paint_config = LedsPaintRequest()
        response = LedsPaintResponse()

        leds_paint_config.paint_id = name
        leds_paint_config.color_R = color[0]
        leds_paint_config.color_G = color[1]
        leds_paint_config.color_B = color[2]
        leds_paint_config.start_led = leds_zone[0]
        leds_paint_config.end_led = leds_zone[1]
        leds_paint_config.enabled = enable

        response = self.leds_driver_paint_service (leds_paint_config)

        return response



    def get_config_params(self, led_name_req):

        # Get led_config_list from rosparam
        list = rospy.get_param(self.node_name + "/led_config_list")

        # Search the name of the required LEDs from led_config_list list

        _nameFound = False

        for i in range(0, len(list)):

            _led_name = list[i].get("name")
            
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

            rospy.logerr("Name '" + led_name_req + "' not found in rosparam server. Check that the name exists in the led_config.yaml")
            _led_config = {}


        return _led_config




    def get_state_params(self, led_state_req):

        # Get led_state_list from rosparam
        list = rospy.get_param(self.node_name + "/led_state_list")

        # Search the name of the required state from led_state_list list

        _stateFound = False

        for i in range(0, len(list)):

            _state_name = list[i].get("name")
            
            if _state_name == led_state_req:

                _stateFound = True
                _state_config = list[i]

                # _state_config[0] = _stateFound
                #rospy.loginfo("State '" + _state_name + "' found")

        if _stateFound == False:

            rospy.logerr("Name '" + led_state_req + "' not found in rosparam server. Check that the name exists in the led_state.yaml")
            _state_config = {}


        return _state_config         




    def leds_service_callback(self, req):


        res = SetLedsResponse()

        led_name_req = req.leds_name
        led_state_req = req.state
        led_enable_req = req.enable

        #If name_req or state_req does not exist return a empty list
        led_config = self.get_config_params(led_name_req)
        state_config = self.get_state_params(led_state_req)

        #Check if dictionary is not empyy
        if bool(led_config) and bool (state_config):

            self.set_mode_arduino_signaling_led(led_config,state_config, led_enable_req )
            res.success = True
            res.message = str(led_name_req) + " has been set to " + str(led_state_req) + " mode and is " + str (led_enable_req)

        else:
            res.success = True
            res.message = "Error: led_name or state_name does not exist"


        return res








def main():

    leds_driver = LedsDriver();

    # Init node
    #rospy.init_node('leds_driver_server')

    # Wait for the service client 
    #rospy.wait_for_service('/robotnik_leds/robotnik_leds/set_leds/blink_mode')

    # Connect to the service
    #leds_driver_blink_service = rospy.ServiceProxy('/robotnik_leds/robotnik_leds/set_leds/blink_mode', LedsBlink)


    # Init service
    #leds_service = rospy.Service('/leds_service', SetLeds, leds_service_callback)




    
    dummy = 1



    while not rospy.is_shutdown():



        
        dummy = 1

       

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass