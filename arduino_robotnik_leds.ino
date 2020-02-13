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



/* Variables globales para el modo OnOff, se llamará PAINT (provisional, implementar un HandleMode)*/
uint8_t  onoff_color_R = 0;
uint8_t  onoff_color_G = 0;
uint8_t  onoff_color_B = 0;
uint16_t onoff_start_led = 0;
uint16_t onoff_end_led = 0;
bool     onoff_enabled = false;


/* Variables globales para el modo Blink (provisional, implementar un HandleMode) */
uint8_t  blink_color_R = 0;
uint8_t  blink_color_G = 0;
uint8_t  blink_color_B = 0;
uint16_t blink_start_led = 0;
uint16_t blink_end_led = 0;
uint16_t blink_ms_on = 0;
uint16_t blink_ms_off = 0;
bool     blink_enabled = false;



/* Tira led */
#define PIN        6
#define NUMPIXELS  5
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);



int i;

elapsedMillis timeout_system;







void callback_onoff(const LedsNormal::Request & req, LedsNormal::Response & res){

  onoff_color_R = req.color_R;
  onoff_color_G = req.color_G;
  onoff_color_B = req.color_B;
  onoff_start_led = req.start_led;
  onoff_end_led = req.end_led;
  onoff_enabled = req.enabled;
  
}


void callback_blink(const LedsBlink::Request & req, LedsBlink::Response & res){

  blink_color_R = req.color_R;
  blink_color_G = req.color_G;
  blink_color_B = req.color_B;
  blink_start_led = req.start_led;
  blink_end_led = req.end_led;
  blink_ms_on = req.ms_on;
  blink_ms_off = req.ms_off; 
  blink_enabled = req.enabled;
  
}


void callback_shift(const LedsShift::Request & req, LedsShift::Response & res){
  res.state = false;
}


ros::ServiceServer<LedsNormal::Request, LedsNormal::Response> server_onoff_mode("robotnik_leds/set_leds/onoff_mode",&callback_onoff);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("robotnik_leds/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("robotnik_leds/set_leds/shift_mode",&callback_shift);



// ============================== ON & OFF   M O D E ================================= //


uint8_t onoff_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led, 
                   bool enabled){

    static uint8_t onoff_state = 0;
    static bool isUpdated = false;
    

    /*  Detecta si ha actualizado algun parámetro de la tira. Esto permite que el microcontrolador
        no actualice constantemente la tira, ya que el valor es siempre el mismo sino han habido cambios
     */
 
    static uint8_t last_color_R = 0, last_color_G = 0, last_color_B = 0;
    static uint16_t last_start_led = 0, last_end_led = 0;

    if(last_color_R != color_R || last_color_G != color_G || last_color_B != color_B ||
       last_start_led != start_led || last_end_led != end_led){

        //Algun parametro se ha actualizado
        isUpdated = true;
        last_color_R = color_R;
        last_color_G = color_G;
        last_color_B = color_B;
        last_start_led = start_led;
        last_end_led = end_led;
    }


    if(enabled){

        if(isUpdated){
            
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led, end_led);
            pixels.show();
            isUpdated = false; // Se ha actualizado la tira led
        }
    }


  return onoff_state;
}





// ============================== B L I N K   M O D E ================================= //


uint8_t blink_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led,
                   uint16_t ms_on, uint16_t ms_off,
                   bool enabled) {

    static bool isOn = false;         //Indica si el el led debe estar encendido o apagado
    static uint8_t blink_state = 0;   //Estado del modo blink
    static elapsedMillis blink_time;  //Tiempo transcurrido entre un intervalo
    static bool isClear = true;       //Indica si hay que limpiar (apagar los leds) la zona donde ha trabajado el modo blink cuando ha terminado

    if(enabled){

        isClear = false;

        if(blink_time > ms_off && !isOn){
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led, end_led);
            pixels.show();
            blink_time = 0;
            isOn = true;           
        }

        if(blink_time > ms_on && isOn){
            pixels.fill(pixels.Color(0, 0, 0), start_led, end_led);
            pixels.show();
            blink_time = 0;
            isOn = false;
        }

    }

    else if (!isClear){
        pixels.fill(pixels.Color(0, 0, 0), start_led, end_led);
        pixels.show();
        isClear = true;
    }
    

  return blink_state;

}


void setup()
{

  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   

  nh.initNode();
  nh.advertiseService(server_onoff_mode);
  nh.advertiseService(server_blink_mode);
  nh.advertiseService(server_shift_mode);
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
      nh.spinOnce();
  }

  
  blink_mode(blink_color_R,blink_color_G,blink_color_B, blink_start_led, blink_end_led, blink_ms_on, blink_ms_off, blink_enabled);
  onoff_mode(onoff_color_R,onoff_color_G,onoff_color_B, onoff_start_led, onoff_end_led, onoff_enabled);
  

}
