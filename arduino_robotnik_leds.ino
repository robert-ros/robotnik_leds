/*
 * rosserial Service Server
 * Soporta Arduino UNO, Arduino Leonardo, Arduino Nano, Arduino Micro y Teeny 3.2
 */




#include <Adafruit_NeoPixel.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <robotnik_leds/LedsNormal.h>
#include <robotnik_leds/LedsBlink.h>
#include <robotnik_leds/LedsShift.h>

ros::NodeHandle  nh;
using robotnik_leds::LedsNormal;
using robotnik_leds::LedsBlink;
using robotnik_leds::LedsShift;


#define PIN        6
#define NUMPIXELS  5

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int i;

elapsedMillis timeout_system;
elapsedMillis timeout_blink_on;
elapsedMillis timeout_blink_off;





bool flag_blink = false;
bool flag_blink_mode = false;

void callback_normal(const LedsNormal::Request & req, LedsNormal::Response & res){
  
  if((i++)%2){
    res.state = true;
    pixels.fill(pixels.Color(req.color_R, req.color_G, req.color_B), req.start_led, req.end_led);
    pixels.show();
  }
  else{
    res.state = false;
    pixels.clear();
    pixels.show();
  }
}


void callback_blink(const LedsBlink::Request & req, LedsBlink::Response & res){

  if((i++)%2)
      flag_blink_mode = true;
  else
      flag_blink_mode = false;
      pixels.fill(pixels.Color(0, 0, 0), 1, 4);
      pixels.show();
  res.state = true;
}

void callback_shift(const LedsShift::Request & req, LedsShift::Response & res){
  res.state = false;
}


ros::ServiceServer<LedsNormal::Request, LedsNormal::Response> server_normal_mode("robotnik_leds/set_leds/normal_mode",&callback_normal);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("robotnik_leds/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("robotnik_leds/set_leds/shift_mode",&callback_shift);



uint8_t blink_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                uint16_t start_led, uint16_t end_led,
                uint16_t ms_on, uint16_t ms_off,
                bool enabled) {

    static bool isOn = true;
    static uint8_t state = 0;
    static elapsedMillis blink_time;

    if(enabled){

       Serial.println(blink_time);
        if(blink_time > ms_off && !isOn){
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led, end_led);
            pixels.show();
            blink_time = 0;
            isOn = true;           
        }

        if(blink_time > ms_on && isOn){
            pixels.clear();
            pixels.show();
            blink_time = 0;
            isOn = false;
        }

    }

  return state;

}


void setup()
{

  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   

  nh.initNode();
  nh.advertiseService(server_normal_mode);
  nh.advertiseService(server_blink_mode);
  nh.advertiseService(server_shift_mode);
  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  Serial.begin(9600);

}

void loop()
{
  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }

  blink_mode(0,255,0, 2,2, 100, 100, true);

  
/*
  if(flag_blink_mode){
    
      if(timeout_blink_on > 100 && flag_blink == false){
          pixels.fill(pixels.Color(0, 0, 255), 1, 4);
          pixels.show();
          flag_blink = true;
          timeout_blink_on = 0;
          timeout_blink_off = 0;
      }
      
      if(timeout_blink_off > 100 && flag_blink == true){
          pixels.fill(pixels.Color(0, 0, 0), 1, 4);
          pixels.show();
          flag_blink = false;  
          timeout_blink_on = 0;
          timeout_blink_off = 0;
      }
  
  }
*/
}
