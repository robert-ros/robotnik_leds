
  #include "ShiftEffect.h"


  ShiftEffect::ShiftEffect(WS2812Serial &pixels):
        CommonEffect(pixels){}

  
  
  int ShiftEffect::get_next_led(int init_led, int end_led, int led_increment){
 

  
     if(led_counter == end_led || led_counter == 0){
  
         led_counter = init_led-1;
     }
      
  
     led_counter+=led_increment;
  
  
     if(led_counter > end_led){
      
        led_counter = end_led; 
     }
  
     return led_counter;


  }




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
      int sleep = this -> effect_config.sleep;    //Not used
      int led_increment = this -> effect_config.led_increment;
      uint8_t  background_R = this -> effect_config.background_R;
      uint8_t  background_G = this -> effect_config.background_G;
      uint8_t  background_B = this -> effect_config.background_B;
      uint8_t  background_W = this -> effect_config.background_W; 
      int enabled = this -> effect_config.enabled;

      

      if(checkLedZoneUpdates()){

         //Clean the area that will NOT be used
         showFillPixels(0,0,0,0, start_led, end_led);
      }


      if(enabled){
       
        if(firstTime()){
          
          last_led = start_led;
          shift_time = 0;

          //Paint background
          fillPixels(background_R, background_G, background_B, background_W , start_led , end_led);

          // This fixes bug zone, to do: find the real cause
          if (direction.equals("right"))
            led_counter = start_led;
          else
            led_counter = end_led;
          
          //Serial2.println(led_counter);
        }

       

        speed = roundToRefreshTime(speed);
        int shift_leds =  end_led - start_led + 1; // For example: start: 4, end: 2 Hay 4-2+1 = 3 involved leds
        shift_leds = ceil(float(shift_leds)/float(led_increment));
       
        float speed_per_led_aux = float(speed)/float(shift_leds);
        int speed_per_led = roundToRefreshTime(speed_per_led_aux);


         if(state == WAIT_NEXT_LEDS){

              if(current_led >= end_led){
                    
                     state = RESET_LEDS;    
                     shift_time = 0;  // Reset time                 
                     //tic = millis();
                     //Serial.println(toc-tic);
                     //toc = millis();
              }


              else if(shift_time >= speed_per_led){

                      shift_time = 0;  // Reset time  
                    
                      state = PRINT_LEDS;
              }     
   
          }



        if(state == RESET_LEDS){
           
            fillPixels(background_R, background_G, background_B, background_W , start_led , end_led);
            current_led = start_led;
            last_led = start_led;          
            state = WAIT_LAST_LED; 
             
        }


        if(state == WAIT_LAST_LED){  // It is considered the last led when all the leds are off
          
           if(shift_time >= speed_per_led){
               state = WAIT_SLEEP;  
               shift_time = 0; 
            }
        }


        if(state == WAIT_SLEEP){  
          
           if(shift_time >= sleep){
               state = PRINT_LEDS;  
               shift_time = 0; 
            }
        }

        

        
        if(state == PRINT_LEDS){

              current_led = get_next_led(start_led, end_led, led_increment); 
              
              if (direction.equals("right")){
                  //Serial.print("right: ");
                  //Serial.println(current_led);
                  fillPixels(color_R, color_G, color_B, color_W, last_led , current_led);
              }
              
              else{

                 int a = end_led - current_led + start_led;
                 
                 int new_end_led = a + (led_increment-1);
                 int new_start_led = a;

                 //Serial.print("left: ");
                 //Serial.println(new_start_led);
                 
                 fillPixels(color_R, color_G, color_B, color_W, new_start_led, new_end_led); 
               }
               
               last_led = current_led + 1;
               state = WAIT_NEXT_LEDS;

         }
         

          shift_time = shift_time + COMMON_EFFECT_REFRESH_TIME;

 
      }  // End enabled


  } // End run()

  
