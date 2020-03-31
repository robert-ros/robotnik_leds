

  #ifndef _SHIFT_EFFECT_H_
  #define _SHIFT_EFFECT_H_
  #include <Arduino.h>
  #include "CommonEffect.h"
  
  
  class ShiftEffect: public CommonEffect{
    
    private:

      /* Auxiliar global variables */
      elapsedMicros shift_time;
      int count_pixel = 0; 
      uint8_t shift_state = 0;
      uint16_t shift_pixels;
      float shift_time_ms = 0;
      float speed_per_pixel = 0;
      float shift_time_compensation=0;

      int led_counter = 0;   
      enum state_machine {WAIT_FINAL_LED_ON,  WAIT_FINAL_LED_OFF};
      int state = WAIT_FINAL_LED_ON;
      int aux_counter = 0;
      
    public:
    
      ShiftEffect(Adafruit_NeoPixel &pixels);

      uint8_t shift_mode( struct leds_shift shift_config);
      void run(void);
 
  };


  
  #endif
