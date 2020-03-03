

  #ifndef _BLINK_EFFECT_H_
  #define _BLINK_EFFECT_H_
  
  #include <Arduino.h>
  #include <Adafruit_NeoPixel.h>
  #include "CommonEffect.h"
  
  
  class BlinkEffect: public CommonEffect{
    
    private:
    
      //Adafruit_NeoPixel *pixels;

      bool isOn = false;         //Indica si el el led debe estar encendido o apagado
      uint16_t blink_pixels; // Numero de pixeles involucrados en realizar el efecto blink
      uint8_t blink_state = 0;   //Estado del modo blink
      elapsedMillis blink_time;  //Tiempo transcurrido entre un intervalo
      bool isClear = true;       //Indica si hay que limpiar (apagar los leds) la zona donde ha trabajado el modo blink cuando ha terminado
      uint16_t last_start_led = 0, last_end_led, last_blink_pixels = 0;

      String   id_assigned = 0;
      uint8_t  color_R = 0;
      uint8_t  color_G = 0;
      uint8_t  color_B = 0;
      uint16_t start_led = 0;
      uint16_t end_led = 0;
      uint16_t ms_on = 0;
      uint16_t ms_off = 0;
      bool     enabled = false;

    public:
    
      //BlinkEffect(Adafruit_NeoPixel &pixels);
       //BlinkEffect(int num_pixels, byte pin, byte model);
       
      BlinkEffect(Adafruit_NeoPixel &pixels);

      struct leds_blink{
  
          leds_blink(): 
              id(""),
              color_R(0),
              color_G(0),
              color_B(0),
              start_led(0),
              end_led(0),
              ms_on (500),
              ms_off (500),
              enabled(false) {}
          
          String   id;
          uint8_t  color_R;
          uint8_t  color_G;
          uint8_t  color_B;
          uint16_t start_led;
          uint16_t end_led;
          uint16_t ms_on = 0;
          uint16_t ms_off = 0;
          bool     enabled;
      
      } blink_config;


      uint8_t blink_mode( struct leds_blink blink_config);

      void assign_id(String id_assigned);
 
  };


  
  #endif
