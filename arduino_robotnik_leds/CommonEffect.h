
  #ifndef _COMMON_EFFECT_H_
  #define _COMMON_EFFECT_H_
  
  #include <Arduino.h>
  //#include <Adafruit_NeoPixel.h>
  #include <WS2812Serial.h>

  // General properties used by all led effects

  #define COMMON_EFFECT_REFRESH_TIME 20 // 20 ms

  struct LedProperties{

  LedProperties(): 
      id(""),
      color_R(0),
      color_G(0),
      color_B(0),
      color_W(0),
      start_led(0),
      end_led(0),
      ms_on(0),
      ms_off(0),
      direction(""),
      speed(0),
      sleep(0),
      enabled(false) {}
  
  String   id;
  String   mode;
  uint8_t  channel;
  String   type;
  uint8_t  color_R;
  uint8_t  color_G;
  uint8_t  color_B;
  uint8_t  color_W;
  uint16_t start_led;
  uint16_t end_led;
  uint16_t ms_on = 0;
  uint16_t ms_off = 0;
  String   direction = "right";
  uint16_t speed = 0;
  uint16_t sleep = 0;
  uint16_t led_increment=0;
  bool     enabled;
  
  };

  

  class CommonEffect {
    
    
    private:

        bool updateFlag = false;
        bool updateLedZoneFlag = false;
        bool firstTimeFlag = true;
        
    public:

        WS2812Serial *pixels;


        CommonEffect(WS2812Serial &pixels);

  
        struct LedProperties effect_config;


        /* Low level functions */
        void fillPixels(int color_R, int color_G, int color_B,  int color_W, int start_led, int count_led );
        void setOnePixelColor(int led, int color_R, int color_G, int color_B, int color_W);
        void showPixels(void);
        void showFillPixels(int color_R, int color_G, int color_B, int color_W, int start_led, int end_led);


        /* Effects management functions */
        String id_assigned = "";
        void assign_id(String id_assigned);
        uint8_t update(struct LedProperties effect_config);
        bool checkUpdates(void);
        bool checkLedZoneUpdates(void);
        void updateLedZone(struct LedProperties effect_config);
        void updateEffectConfig(struct LedProperties effect_config);
        void resetEffectConfig (void);
        void cleanCurrentLedZone(void);
        void removeEffect(void);

        /* Functions utils for effects */
        bool firstTime(void);
        int roundToRefreshTime(float speed_decimal);
  };

 #endif
