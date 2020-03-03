  
 
  #include "CommonEffect.h"

  
  CommonEffect::CommonEffect(Adafruit_NeoPixel &pixels) {

     this->pixels = &pixels;
      
  }



  void CommonEffect::fillPixels(int color_R, int color_G, int color_B, int start_led, int count_led ) {

        pixels->fill(pixels->Color(color_R, color_G, color_B), start_led, count_led);
            
  }


  void CommonEffect::showPixels(void) {

        pixels->show();
    
  }


  void CommonEffect::beginPixels(void){
    
        pixels->begin();
        pixels->clear();
        pixels->show();
  
  }

  void CommonEffect::setPixelsColor(int led, int color_R, int color_G, int color_B){
    
      pixels->setPixelColor(led, pixels->Color(color_R, color_G, color_B));  
  
  }
  
