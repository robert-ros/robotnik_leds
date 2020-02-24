#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from robotnik_leds_sdk.srv import SetLeds, SetLedsResponse

def set_mode_led_driver(led_config, state_config, enable):

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

        paint_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _enable)


    if _mode == "blink":

        _color = state_config.get("color")
        _ms_on = state_config.get("ms_on")
        _ms_off = state_config.get("ms_off")

        blink_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _ms_on, _ms_off, _enable)


    if _mode == "shift":

        _color     =  state_config.get("color")
        _direction =  state_config.get("direction")
        _speed     =  state_config.get("speed")
        _sleep     =  state_config.get("sleep")

        shift_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _direction, _speed, _sleep, _enable)



def shift_mode_led_driver(name, leds_zone, channel, type, color, direction, speed, sleep, enable):

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


def blink_mode_led_driver(name, leds_zone, channel, type, color, ms_on, ms_off, enable):

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


def paint_mode_led_driver(name, leds_zone, channel, type, color, enable):

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


def get_config_params(led_name_req):

    # Get led_config_list from rosparam
    list = rospy.get_param("/robotnik_leds/led_config_list")

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




def get_state_params(led_state_req):

    # Get led_state_list from rosparam
    list = rospy.get_param("/robotnik_leds/led_state_list")

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




def leds_service_callback(req):


    res = SetLedsResponse()

    led_name_req = req.leds_name
    led_state_req = req.state
    led_enable_req = req.enable

    #If name_req or state_req does not exist return a empty list
    led_config = get_config_params(led_name_req)
    state_config = get_state_params(led_state_req)

    #Check if dictionary is not empyy
    if bool(led_config) and bool (state_config):

        set_mode_led_driver(led_config,state_config, led_enable_req )
        res.success = True
        res.message = str(led_name_req) + " has been set to " + str(led_state_req) + " mode and is " + str (led_enable_req)

    else:
        res.success = True
        res.message = "Error: led_name or state_name does not exist"


    return res



def main():

    # Init node
    rospy.init_node('leds_driver_server')

    # Init service
    leds_service = rospy.Service('/leds_service', SetLeds, leds_service_callback)


    while not rospy.is_shutdown():
        
        dummy = 1

       

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass