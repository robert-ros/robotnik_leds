


  #include "PaintEffect.h"


  PaintEffect::PaintEffect(WS2812Serial &pixels):
        CommonEffect(pixels){}


  void PaintEffect::run(void){
    
    if(checkUpdates()){
      
        /* Auxiliar local variables */
        int color_R = this -> effect_config.color_R;    
        int color_G = this -> effect_config.color_G;    
        int color_B = this -> effect_config.color_B;
        int color_W = this -> effect_config.color_W;
        int start_led = this -> effect_config.start_led;
        int end_led = this -> effect_config.end_led;
        int enabled = this -> effect_config.enabled;
    
        
        if(enabled)
            showFillPixels(color_R, color_G, color_B, color_W, start_led, end_led);  
        
        else
            showFillPixels(0,0,0,0, start_led, end_led);
            
    }
    
  }
