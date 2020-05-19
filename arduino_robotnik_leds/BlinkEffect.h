

  #ifndef _BLINK_EFFECT_H_
  #define _BLINK_EFFECT_H_
  
  #include <Arduino.h>
  #include "CommonEffect.h"
  
  
  class BlinkEffect: public CommonEffect{
    
    private:

      /* Auxiliar global variables */
      bool isOn = false;         //Indica si el el led debe estar encendido o apagado
      //elapsedMillis blink_time;  //Tiempo transcurrido entre un intervalo
      float blink_time = 0;
      enum blink_resources {FADE_IN, LED_ON, FADE_OUT, LED_OFF, HOLD_ON, HOLD_OFF};
      //                      0          1      2          3        4        5
      int state = FADE_IN;
      float fade_R = 0, fade_G = 0, fade_B = 0, fade_W = 0;
      int fade_in_tics = 0, fade_out_tics = 0;
      int fade_in_timeout = 0, fade_out_timeout = 0;
      float increment_fade_in_R, increment_fade_in_G, increment_fade_in_B, increment_fade_in_W;
      float increment_fade_out_R, increment_fade_out_G, increment_fade_out_B, increment_fade_out_W;
      
    public:
    
      BlinkEffect(WS2812Serial &pixels);  
      void run(void);
      
  };


  
  #endif
