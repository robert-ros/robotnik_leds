
  #include "BlinkEffect.h"



  BlinkEffect::BlinkEffect(Adafruit_NeoPixel &pixels) {

        this->pixels = &pixels;
      
  }



  uint8_t BlinkEffect::blink_mode(struct leds_blink blink_config) {

      uint8_t  id = blink_config.id;
      uint8_t  color_R = blink_config.color_R;
      uint8_t  color_G = blink_config.color_G;
      uint8_t  color_B = blink_config.color_B;
      uint16_t start_led = blink_config.start_led;
      uint16_t end_led = blink_config.end_led;
      uint16_t ms_on = blink_config.ms_on;
      uint16_t ms_off = blink_config.ms_off;
      bool     enabled = blink_config.enabled;
  

      blink_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados
  
      //Nota: start_led cuenta los leds desde 1 mientras que fill() lo hace desde 0 
  
      if(enabled){
  
          isClear = false;
  
          if(blink_time > ms_off && !isOn){
              pixels->fill(pixels->Color(color_R, color_G, color_B), start_led-1, blink_pixels);
            pixels->show();
            blink_time = 0;
            isOn = true;          
        }

        if(blink_time > ms_on && isOn){
            pixels->fill(pixels->Color(0, 0, 0), start_led-1, blink_pixels);
            pixels->show();
            blink_time = 0;
            isOn = false;
        }

    }

    else if (!isClear){
        pixels->fill(pixels->Color(0, 0, 0), start_led-1, blink_pixels);
        pixels->show();
        isClear = true;
    }
    

  return blink_state;

}
