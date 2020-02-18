

  #ifndef _PAINT_EFFECT_H_
  #define _PAINT_EFFECT_H_
  
  #include <Arduino.h>
  #include <Adafruit_NeoPixel.h>
  
  
  class PaintEffect{
    
    private:
    
      Adafruit_NeoPixel *pixels;

      uint8_t paint_state = 0;
      bool isUpdated = false;  
      uint16_t paint_pixels = 0;  // Numero de pixeles involucrados en realizar el efecto blink
      uint8_t last_color_R = 0, last_color_G = 0, last_color_B = 0;
      uint16_t last_start_led = 0, last_end_led = 0;
      bool isClear = true;

      uint8_t  id_assigned = 0;
      uint8_t  color_R = 0;
      uint8_t  color_G = 0;
      uint8_t  color_B = 0;
      uint16_t start_led = 0;
      uint16_t end_led = 0;
      bool     enabled = false;

    public:
    
      PaintEffect(Adafruit_NeoPixel &pixels);

      struct leds_paint{
  
          leds_paint(): 
              id(0),
              color_R(0),
              color_G(0),
              color_B(0),
              start_led(0),
              end_led(0),
              enabled(false) {}
          
          uint8_t  id;
          uint8_t  color_R;
          uint8_t  color_G;
          uint8_t  color_B;
          uint16_t start_led;
          uint16_t end_led;
          bool     enabled;
      
      } paint_config;


      uint8_t paint_mode( struct leds_paint paint_config);
      void assign_id(uint8_t id_assigned);
 
  };


  
  #endif
