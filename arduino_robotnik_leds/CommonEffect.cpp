  
 
  #include "CommonEffect.h"

  
  CommonEffect::CommonEffect(Adafruit_NeoPixel &pixels) {

     this->pixels = &pixels;
      
  }



  void CommonEffect::fillPixels(int color_R, int color_G, int color_B, int color_W, int start_led, int count_led ) {

        pixels->fill(pixels->Color(color_G, color_R, color_B,  color_W), start_led, count_led);
            
  }


  void CommonEffect::showPixels(void) {

        pixels->show();
    
  }



  void CommonEffect::setPixelsColor(int led, int color_R, int color_G, int color_B, int color_W){
    
      pixels->setPixelColor(led, pixels->Color(color_G, color_R, color_B, color_W));  
  
  }



   // Fill and show a led strip from start_led to end_led, both included. First led of the strip starts in 1.

  void CommonEffect::showFillPixels(int color_R, int color_G, int color_B, int color_W, int start_led, int end_led){
    
       //Example: start_led = 2, end_led =  4  There are 4-2+1 = 3 pixels involved
      int count_led =  end_led - start_led + 1; 
      
      pixels->fill(pixels->Color(color_G, color_R, color_B, color_W), start_led - 1, count_led);
      pixels->show();
      
  }




  void CommonEffect::assign_id(String id_assigned){
    
      this->id_assigned = id_assigned; 
  }


  
  void CommonEffect::updateLedZone(struct LedProperties effect_config){

      /* Auxiliar local variables */    
      int old_start_led = this -> effect_config.start_led;
      int old_end_led = this -> effect_config.end_led;
      int new_start_led = effect_config.start_led;
      int new_end_led = effect_config.end_led;
      
      int start_difference = new_start_led - old_start_led;
      int end_difference =   old_end_led - new_end_led;
  
  
      // If the distance beetween new start led and old start led is positive, clean the difference
      if (start_difference > 0){

          updateLedZoneFlag = true;
          fillPixels(0,0,0,0, old_start_led-1, start_difference);
          showPixels();
      }
  
      // Same logic with the end leds
      if (end_difference  > 0){

          updateLedZoneFlag = true;
          fillPixels(0,0,0,0, new_end_led , end_difference );
          showPixels();
      } 
    
  }


  void CommonEffect::updateEffectConfig(struct LedProperties effect_config){

          this -> effect_config = effect_config;

  }


  void CommonEffect::resetEffectConfig(void){

        struct LedProperties default_effect_config;
        
        //Reset variables to default values
        this -> effect_config = default_effect_config;

        firstTimeFlag = true;
    
  }

  void CommonEffect::cleanCurrentLedZone(void){

        int start_led = this -> effect_config.start_led;
        int end_led = this -> effect_config.end_led;

        // Clean led zone
        showFillPixels(0,0,0,0, start_led, end_led);
 
  }

  void CommonEffect::removeEffect(void){

        cleanCurrentLedZone();
        resetEffectConfig();
  }


  uint8_t CommonEffect::update(struct LedProperties effect_config){

          updateFlag = true;
  
          // Check if the ID corresponds to the ID assigned to an instance of this class
          if(id_assigned.equals(effect_config.id)){

              if(effect_config.enabled) {
                                  
                  // Check if the led zone has changed
                  updateLedZone(effect_config);
                  
                  // Update values in the object
                  updateEffectConfig(effect_config);
              }
              
              else{
                  // Clean values of the object, restored default values
                  removeEffect();
              }
                
          }
        
      return -1;
  }

  bool CommonEffect::checkUpdates(void){

      bool isUpdated;

      if(updateFlag)
          isUpdated = true;
      else
          isUpdated = false;

      //Clear updateFlag
      updateFlag = false;
          
      return isUpdated;
  }


  bool CommonEffect::checkLedZoneUpdates(void){

      bool isUpdated;

      if(this -> updateLedZoneFlag)
          isUpdated = true;
      else
          isUpdated = false;

      //Clear updateFlag
      this -> updateLedZoneFlag = false;
          
      return isUpdated;
    
    
    }


  bool CommonEffect::firstTime(void){

      // firtTimeFlag is set false in firstTime() function and is set true in resetEffectConfig function
      bool result;

      if(this -> firstTimeFlag){
          
          result = true;
          this -> firstTimeFlag = false;
      }

      else{
          result = false;
                 
      }
      
   
    return result;
   }

  
