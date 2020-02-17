

  #ifndef _SHIFT_EFFECT_H_
  #define _SHIFT_EFFECT_H_
  #include <Arduino.h>
  #include <Adafruit_NeoPixel.h>
  
  
  class ShiftEffect{
    
    private:
    
      Adafruit_NeoPixel *pixels;
      
      elapsedMicros shift_time;
      double shift_tic = 0, shift_toc = 0;
      int count_pixel = 0; 
      uint8_t shift_state = 0;
      uint16_t shift_pixels; // Number of pixels involved in performing the shift effect
      bool isClear = true;   // Indicates if the area needs to be cleaned (turn off the LEDs) where the shift mode worked when finished
      uint16_t last_start_led = 0, last_end_led, last_shift_pixels = 0;

      
    public:
    
      ShiftEffect(Adafruit_NeoPixel &pixels);

      struct leds_shift{
  
          leds_shift(): 
              id(0),
              color_R(0),
              color_G(0),
              color_B(0),
              start_led(0),
              end_led(0),
              direction("right"),
              speed(0),
              sleep(0),
              enabled(false) {}
          
          uint8_t  id;
          uint8_t  color_R;
          uint8_t  color_G;
          uint8_t  color_B;
          uint16_t start_led;
          uint16_t end_led;
          String   direction;
          uint16_t speed;
          uint16_t sleep;
          bool     enabled;
      
      } shift_config;


      uint8_t shift_mode( struct leds_shift shift_config);
 
  };


  
  #endif
