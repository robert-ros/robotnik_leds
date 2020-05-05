
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
      uint16_t fade_in = this -> effect_config.fade_in;
      uint16_t fade_out = this -> effect_config.fade_out;
      uint8_t  background_R = this -> effect_config.background_R;
      uint8_t  background_G = this -> effect_config.background_G;
      uint8_t  background_B = this -> effect_config.background_B;
      uint8_t  background_W = this -> effect_config.background_W;  
 
      bool     enabled = this -> effect_config.enabled;

      
      if(enabled){
/*
                    Serial.println(fade_R);
              Serial.println(fade_G);
                Serial.println(fade_B);
                  Serial.println(fade_W);
                    Serial.println("------");
*/
          ms_on = roundToRefreshTime(ms_on);
          ms_off = roundToRefreshTime(ms_off);

      
          if(fade_in > 0){
            
              fade_in_timeout = roundToRefreshTime(fade_in);
              fade_in_tics = fade_in/COMMON_EFFECT_REFRESH_TIME;
  
              increment_fade_in_R = float(color_R-background_R)/float(fade_in_tics);
              increment_fade_in_G = float(color_G-background_G)/float(fade_in_tics);
              increment_fade_in_B = float(color_B-background_B)/float(fade_in_tics);
              increment_fade_in_W = float(color_W-background_W)/float(fade_in_tics);
          }
          

          if(fade_out > 0){
            
              fade_out_timeout = roundToRefreshTime(fade_out); 
              fade_out_tics = fade_out/COMMON_EFFECT_REFRESH_TIME;
             
              increment_fade_out_R = float(color_R-background_R)/float(fade_out_tics);
              increment_fade_out_G = float(color_G-background_G)/float(fade_out_tics);
              increment_fade_out_B = float(color_B-background_B)/float(fade_out_tics);
              increment_fade_out_W = float(color_W-background_W)/float(fade_out_tics);
          }

          
          if(firstTime()){
            
              blink_time = 0;  
              fade_R = background_R;
              fade_G = background_G;
              fade_B = background_B;
              fade_W = background_W;      
              
          }

        if(fade_in == 0 && state == FADE_IN){
            state = LED_ON;
            blink_time = 0;
            fade_R = color_R;
            fade_G = color_G;
            fade_B = color_B;
            fade_W = color_W;
        }
        
        if(fade_out == 0 && state == FADE_OUT){
            state = LED_OFF;
            blink_time = 0;
            fade_R = background_R;
            fade_G = background_G;
            fade_B = background_B;
            fade_W = background_W;
        }
          

          switch(state){

              case FADE_IN:
                    
                    if(blink_time <= fade_in_timeout){
                     
                      fade_R+= increment_fade_in_R;
                      fade_G+= increment_fade_in_G;
                      fade_B+= increment_fade_in_B;
                      fade_W+= increment_fade_in_W;
                      Serial.println(fade_R);
                      Serial.println(fade_G);
                      Serial.println("-----");

                      // Conversion to an integer greater than zero
                      showFillPixels(max(int(fade_R),0),max(int(fade_G),0),max(int(fade_B),0),max(int(fade_W),0), start_led, end_led);
                      
                    }

                    else{

                      state = LED_ON;
                      blink_time = 0; 
                    }

                    break;
            
              case LED_ON:
                  
                    showFillPixels(color_R,color_G,color_B, color_W, start_led, end_led);
                    //showFillPixels(0,50,0, 0, start_led, end_led);
                    state = HOLD_ON;
                     

                    break;
                    
              case HOLD_ON:

                    if(blink_time > ms_on){
                        state = FADE_OUT;
                        blink_time = 0;  
                    }

                    break;

              case FADE_OUT:

                    if(blink_time <= fade_out_timeout){
                     
                      fade_R-= increment_fade_out_R;
                      fade_G-= increment_fade_out_G;
                      fade_B-= increment_fade_out_B;
                      fade_W-= increment_fade_out_W;
        
                       showFillPixels(max(int(fade_R),0),max(int(fade_G),0),max(int(fade_B),0),max(int(fade_W),0), start_led, end_led);
                      //Serial.println(fade_R);
                      //Serial.println(fade_G);
                      //Serial.println("-----");
                    }

                    else{
                      state = LED_OFF; 
                      blink_time = 0; 
                    }

                    break;

              case LED_OFF:

                    showFillPixels(background_R,background_G,background_B,background_W,  start_led, end_led);
                    state = HOLD_OFF;
                    
                    break;

              case HOLD_OFF:

                    if(blink_time > ms_off){
                        state = FADE_IN;
                        blink_time = 0;  
                    }

                    break;
            
          }

          blink_time = blink_time + (COMMON_EFFECT_REFRESH_TIME);
      
          /*
           
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
          */
      }
    
  }
