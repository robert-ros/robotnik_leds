

  #ifndef _PAINT_EFFECT_H_
  #define _PAINT_EFFECT_H_
  
  #include <Arduino.h>
  #include "CommonEffect.h"

  
  class PaintEffect: public CommonEffect{
   
    public:

      PaintEffect(Adafruit_NeoPixel &pixels);
      void run(void);
 
  };


  
  #endif
