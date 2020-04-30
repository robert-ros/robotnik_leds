

  #ifndef _LED_EFFECTS_H_
  #define _LED_EFFECTS_H_

  #define NUM_EFFECTS 10 // Number of effects for each mode
  #define TYPE_EFFECTS 3 // Number of effects modes (paint, blink, shift)
  #define TOTAL_EFFECTS NUM_EFFECTS*TYPE_EFFECTS // Total number of effects
  
  #include <Arduino.h>

  #include "IdHandler.h"
  #include "ShiftEffect.h"
  #include "BlinkEffect.h"
  #include "PaintEffect.h"
  #include "CommonEffect.h"

  #include <EEPROM.h>

  
  class LedEffects{

    private:

      const float ROS_TIMEOUT = 1500; //Timeout that checks the connection between this device and ROS

      IdHandler database;

      PaintEffect *paint_effect[NUM_EFFECTS];
      BlinkEffect *blink_effect[NUM_EFFECTS];
      ShiftEffect *shift_effect[NUM_EFFECTS];

      struct LedProperties buffer_effects[TOTAL_EFFECTS];
      int index_buffer_effects = 0;
      bool isUpdateEffectsEnabled = false;

      enum resources {CREATE, EDIT, REMOVE, BOOTING, READY, RUNNING, FAULT, EXIT, WAITING_ROS, WAITING_FIRST_COMMAND};

      void updateEffects(struct LedProperties effect_config);
      int  checkEffect(struct LedProperties effect_config); 
      void createEffect(struct LedProperties effect_config); 
      void editEffect(struct LedProperties effect_config); 
      void removeEffect(struct LedProperties effect_config); 
      void bootingMode(void);
      void readyMode(bool enabled);
      void runningMode(void);
      void exitMode(void);
      bool checkConnectionROS(void);
      bool saveStateConfig(struct LedProperties device_config, String state);
      LedProperties readStateConfig(String state);
      int state = BOOTING;


   
    public:

      LedEffects(WS2812Serial &pixels);

      void saveBufferEffects(struct LedProperties effect_config);
      void enableUpdateEffects();
      void updatePendingEffects();
      void runEffects(void);
      void clearEffects(void);
      String listID(void);
      String resetDevice(String password);
      
      void startSequence(void);
      void faultSequence(bool fault);

      String configDevice(String password, struct LedProperties device_config, String state);

      // This varaible is set to true in the callback service
      bool firstCommandFlag = false;
      bool firstACKFlag = false;
      elapsedMillis timeoutACK;

  };


  
  #endif
