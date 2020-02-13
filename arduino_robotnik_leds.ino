/*
 * rosserial Service Server
 * Soporta Arduino UNO, Arduino Leonardo, Arduino Nano, Arduino Micro y Teeny 3.2
 */




#include <Adafruit_NeoPixel.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <rosserial_arduino/Test.h>
//#include <my_custom_srv_msg_pkg/MyCustomServiceMessage.h>
#include <robotnik_leds/LedsNormal.h>
#include <robotnik_leds/LedsBlink.h>
#include <robotnik_leds/LedsShift.h>

ros::NodeHandle  nh;
using rosserial_arduino::Test;
//using my_custom_srv_msg_pkg::MyCustomServiceMessage;
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


std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

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
  nh.advertise(chatter);
  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

}

void loop()
{
  if(timeout_system > 10){
      timeout_system = 0;
      str_msg.data = hello;
      chatter.publish( &str_msg );
      nh.spinOnce();
  }

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

}
