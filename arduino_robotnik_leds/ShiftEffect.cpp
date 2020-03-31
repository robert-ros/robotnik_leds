
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
      int led_increment = this -> effect_config.led_increment;
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
                
                  //Serial1.println("---");
                  fillPixels(0,0,0,0, start_led-1, shift_pixels);
                  showPixels();
                  count_pixel = start_led; 
                  
                  
              }
              else {
                    
                 //Serial1.print(count_pixel);
                 // Print on the led strip every X increments (defined in led_increment)
                 if(led_counter < led_increment){
                   
                    if(direction.equals("right"))
                        
                        setOnePixelColor(count_pixel-1, color_R, color_G, color_B, color_W);
                    
                    else if (direction.equals("left"))
                        
                        setOnePixelColor( (shift_pixels+start_led) - 1 - (count_pixel-start_led) - 1 , color_R, color_G, color_B, color_W);

                    led_counter++;
                    
                 }
                 
                 if (led_counter >= led_increment){
                    Serial1.print(" show");
                    
                    led_counter = 0;
                    showPixels();
                 }

                 if (count_pixel >= end_led){

/*
                     const int WAIT_ON_FINAL_LEDS=1, ON_FINAL_LEDS=2, WAIT_OFF_FINAL_LEDS=1,  OFF_FINAL_LEDS=3;

                     led_counter = 0;

                     switch(state){

                      //Wait the difference 
                      case WAIT_ONF_FINAL_LEDS:

                           if (aux_counter <  (led_increment - (end_led % led_increment))){
                              aux_counter++;
                            }    
                            else{
                              aux_counter = 0; 
                              state = ON_FINAL_LEDS;                    
                            }
                            
                            count_pixel--; // Hold the last led

                            break;

                      case ON_FINAL_LEDS:

                            showPixels(); 
                            
                            count_pixel--; // Hold the last led 
                
                            break;

                      case OFF_FINAL_LEDS:

                            break;
                      
                      
                      }
                      */
                     
                     led_counter = 0;

                       if(end_led % led_increment > 0){

                             if (state == WAIT_FINAL_LED_ON){
                            
                                 if (aux_counter <  (led_increment - (end_led % led_increment))){
                                      aux_counter++; 
                                      count_pixel--;
                                      
                                      
                                 }
                                 else{
                                    aux_counter = 0;                    
                                    showPixels();
                                    state = WAIT_FINAL_LED_OFF;
                                 }
                                 

    
                             }
    
                             if(state == WAIT_FINAL_LED_OFF){
                              
                                 if (aux_counter <  led_increment-1){
                                      aux_counter++;
                                      count_pixel--;  
                                 }
                                 else{
                                    aux_counter = 0;                    
                                    
                                    state = WAIT_FINAL_LED_ON;
                                 
                              
                             }
  
                         }
                                        
                     /*
                     if (count_pixel >= end_led){
                        Serial1.print(" end");
                        
                        showPixels();
                        led_counter = 0;  
                     }
                     */
                     }


                     else{
                             
                             if (aux_counter <  led_increment-1){
                                  aux_counter++;
                                  count_pixel--;  
                             }
                             else{
                                aux_counter = 0;                    
                             }
                             
                     }
                 }
                 //Serial1.println();
                 count_pixel++;         
              }
          }
      }
    
  }

  
