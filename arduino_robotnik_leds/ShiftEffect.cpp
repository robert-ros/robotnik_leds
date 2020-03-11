
  #include "ShiftEffect.h"


  ShiftEffect::ShiftEffect(Adafruit_NeoPixel &pixels):
        CommonEffect(pixels){}


  void ShiftEffect::run(void){


      /* Auxiliar local variables */
      int color_R = this -> effect_config.color_R;    
      int color_G = this -> effect_config.color_G;    
      int color_B = this -> effect_config.color_B;
      int color_W = this -> effect_config.color_W;
      int start_led = this -> effect_config.start_led;
      int end_led = this -> effect_config.end_led;
      int speed = this -> effect_config.speed;
      String direction = this -> effect_config.direction;
      //int sleep = this -> effect_config.sleep;    //Not used
      int enabled = this -> effect_config.enabled;

      if(checkLedZoneUpdates()){

         count_pixel = start_led;
         showFillPixels(0,0,0,0, start_led, end_led);
      }

      if(enabled){

          shift_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados
          shift_time_ms = shift_time/1000.0;
          speed_per_pixel = float(speed)/float(shift_pixels+1);
          shift_time_compensation=0;
          
          //Realiza el efecto shift
          if(shift_time_ms >= (speed_per_pixel+shift_time_compensation)){
  
            shift_time = 0;
  
            shift_time_compensation = float(speed_per_pixel) - shift_time_ms + shift_time_compensation;
  
            
              //Si se ha alcanzado el led final, se limpia la zona de trabajo y se reinicia el contador
              //En caso contrario, realiza el shift
              if(count_pixel > end_led){
                  
                  fillPixels(0,0,0,0, start_led-1, shift_pixels);
                  showPixels();
                  count_pixel = start_led; 
                  
              }
              else {
                
                  if(direction.equals("right"))
                      
                      setPixelsColor(count_pixel-1, color_R, color_G, color_B, color_W);
                  
                  else if (direction.equals("left"))
                      
                      setPixelsColor( (shift_pixels+start_led) - 1 - (count_pixel-start_led) - 1 , color_R, color_G, color_B, color_W);
                  
                  showPixels();
                  count_pixel++;         
              }
          }
      }
    
  }

  
