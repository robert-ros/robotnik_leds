

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
      
    public:
    
      BlinkEffect(WS2812Serial &pixels);  
      void run(void);
      
  };


  
  #endif
