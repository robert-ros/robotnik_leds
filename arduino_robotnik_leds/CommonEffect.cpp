  
 
  #include "CommonEffect.h"

  
  CommonEffect::CommonEffect(int num_pixels, byte pin, byte mode) {

      pixels = new Adafruit_NeoPixel(num_pixels, pin, mode);
      
  }

  void CommonEffect::setLedStrip(Adafruit_NeoPixel &pixels){

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
  
