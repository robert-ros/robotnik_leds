#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from robotnik_leds_sdk.srv import SetLeds, SetLedsResponse

def set_led_driver(led_config, state_config, enable):

    #Enable assignation
    _enable = enable
    
    #Led config assignation
    _name = led_config[0]
    _leds_zone = led_config[1]
    _channel = led_config[2]
    _type = led_config[3]

    #State config assignation
    _mode = state_config[0]

    if _mode == "paint":

        _color = state_config[1]
        paint_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _enable)


    if _mode == "blink":

        _color  =  state_config[1]
        _ms_on  =  state_config[2]
        _ms_off =  state_config[3]
        blink_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _ms_on, _ms_off, _enable)


    if _mode == "shift":

        _color     =  state_config[1]
        _direction =  state_config[2]
        _speed     =  state_config[3]
        _sleep     =  state_config[3]

        shift_mode_led_driver(_name, _leds_zone, _channel, _type, _color, _direction, _speed, _sleep, _enable)



def shift_mode_led_driver(name, leds_zone, channel, type, color, direction, speed, sleep, enable):

    rospy.loginfo("Shift mode!")


def blink_mode_led_driver(name, leds_zone, channel, type, color, ms_on, ms_off, enable):

    rospy.loginfo("Blink mode!")


def paint_mode_led_driver(name, leds_zone, channel, type, color, enable):

    rospy.loginfo("Paint mode!")




def get_led_config(led_name_req):

    # Get led_config_list from rosparam
    list = rospy.get_param("/robotnik_leds/led_config_list")

    # Search the name of the required LEDs from led_config_list list

    _nameFound = False

    for i in range(0, len(list)):

        _led_name = list[i].get("name")
        
        if _led_name == led_name_req:

            _nameFound = True
            _leds_zone = list[i].get("leds_zone")
            _channel = list[i].get("channel")
            _type = list[i].get("type")



            rospy.loginfo("Name '" + _led_name + "' found:\n"+ 
            "led_zone: " + str(_leds_zone) +  "\n" + 
            "channel: "  + str(_channel)   +  "\n" +
            "type: "     + str(_type)      +  "\n" +
            "---------")


    if _nameFound == False:

        rospy.logerr("Name '" + led_name_req + "' not found in rosparam server. Check that the name exists in the led_config.yaml")
        _led_name = None
        _leds_zone = None
        _channel = None
        _type = None


    return [_nameFound, _led_name, _leds_zone, _channel, _type]




def get_led_state(led_state_req):


    # Get led_state_list from rosparam
    list = rospy.get_param("/robotnik_leds/led_state_list")

    # Search the name of the required state from led_state_list list

    _stateFound = False
    _state_config = [0]*10

    for i in range(0, len(list)):

        _state_name = list[i].get("name")
        
        if _state_name == led_state_req:

            _stateFound = True
            _state_config[0] = _stateFound

            rospy.loginfo("State '" + _state_name + "' found")

            _mode = list[i].get("mode")

            if _mode == "paint":

                _state_config[1] = list[i].get("mode") 
                _state_config[2] = list[i].get("color")          
            

            if _mode == "blink":

                _state_config[1] = list[i].get("mode")
                _state_config[2] = list[i].get("color") 
                _state_config[3] = list[i].get("ms_on")
                _state_config[4] = list[i].get("ms_off")  


            if _mode == "shift":

                _state_config[1] = list[i].get("mode")
                _state_config[2] = list[i].get("color") 
                _state_config[3] = list[i].get("direction")
                _state_config[4] = list[i].get("speed")
                _state_config[5] = list[i].get("sleep")

    if _stateFound == False:

        rospy.logerr("Name '" + led_state_req + "' not found in rosparam server. Check that the name exists in the led_state.yaml")
        _state_config[0] = _stateFound


    return _state_config         



def leds_service_callback(req):


    res = SetLedsResponse()

    led_name_req = req.leds_name
    led_state_req = req.state
    led_enable_req = req.enable


    led_config = get_led_config(led_name_req)
    state_config = get_led_state(led_state_req)

    if led_config[0] and state_config[0] :

        set_led_driver(led_config,state_config, led_enable_req )
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
        

        #shift_mode_led_driver("left_led", [1,5], 0, "RGB", [0,0,20], "right", 1000, 1000, True)

        dummy = 1
        #led_name_req = "led_front_right"
        #led_state_req = "EMERGENCY"
        #led_enabled = True;


        #led_config = get_led_config(led_name_req)
        #state_config = get_led_state(led_state_req)
        #set_led_driver(led_config,state_config, led_enabled )

       


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass