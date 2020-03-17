

  #ifndef _LED_EFFECTS_H_
  #define _LED_EFFECTS_H_

  #define NUM_EFFECTS 5
  
  #include <Arduino.h>

  #include "IdHandler.h"
  #include "ShiftEffect.h"
  #include "BlinkEffect.h"
  #include "PaintEffect.h"
  #include "CommonEffect.h"

  
  class LedEffects{

    private:

      IdHandler database;

      PaintEffect *paint_effect[NUM_EFFECTS];
      BlinkEffect *blink_effect[NUM_EFFECTS];
      ShiftEffect *shift_effect[NUM_EFFECTS];

      enum resources {CREATE, EDIT, REMOVE, BOOTING, READY, RUNNING, FAULT, EXIT, WAITING_ROS, WAITING_FIRST_COMMAND};

      int checkEffect(struct LedProperties effect_config); 
      void createEffect(struct LedProperties effect_config); 
      void editEffect(struct LedProperties effect_config); 
      void removeEffect(struct LedProperties effect_config); 
      void bootingMode(void);
      void readyMode(bool enabled);
      void runningMode(void);
      void exitMode(void);
      bool checkConnectionROS(void);

      int state = BOOTING;


   
    public:

      LedEffects(Adafruit_NeoPixel &pixels);
      
      void updateEffects(struct LedProperties effect_config);
      void runEffects(void);
      void clearEffects(void);
      String listID(void);
      void startSequence(void);
      void faultSequence(bool fault);

      // This varaible is set to true in the callback service
      bool firstCommandFlag = false;
      bool firstACKFlag = false;
      elapsedMillis timeoutACK;

  };


  
  #endif
