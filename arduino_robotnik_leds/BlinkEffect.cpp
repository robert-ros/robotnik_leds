
  #include "BlinkEffect.h"


  BlinkEffect::BlinkEffect(WS2812Serial &pixels):
        CommonEffect(pixels){}
        


  void BlinkEffect::run(void){


      /* Auxiliar local variables */
      uint8_t  color_R = this -> effect_config.color_R;
      uint8_t  color_G = this -> effect_config.color_G;
      uint8_t  color_B = this -> effect_config.color_B;
      uint8_t  color_W = this -> effect_config.color_W;
      uint16_t start_led = this -> effect_config.start_led;
      uint16_t end_led = this -> effect_config.end_led;
      uint16_t ms_on = this -> effect_config.ms_on;
      uint16_t ms_off = this -> effect_config.ms_off;
      bool     enabled = this -> effect_config.enabled;


      if(enabled){

          ms_on = roundToRefreshTime(ms_on);
          ms_off = roundToRefreshTime(ms_off);

           
          if(firstTime()){
            
              blink_time = 0;        
              isOn = true;
          }
  
          if(blink_time < ms_off && isOn){

              showFillPixels(color_R,color_G,color_B, color_W, start_led, end_led);
              Serial.println("on");
              isOn = false;
           }
    
  
           else if (blink_time >= ms_on && blink_time <= ms_on+ms_off && !isOn){
    
              showFillPixels(0,0,0,0,  start_led, end_led);
              Serial.println("off");
              isOn = true;
            }


            else if (blink_time >= ms_on+ms_off){
  
              blink_time = 0;
            } 

          
          blink_time = blink_time + (COMMON_EFFECT_REFRESH_TIME);
      }
    
  }
