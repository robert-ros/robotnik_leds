



  #include "LedEffects.h"

  
  
  LedEffects::LedEffects(WS2812Serial &pixels) {


     for(int i = 0; i < NUM_EFFECTS; i++){ 
     
        paint_effect[i] = new PaintEffect(pixels);
        blink_effect[i] = new BlinkEffect(pixels);
        shift_effect[i] = new ShiftEffect(pixels);
     
     }   
          
  }


  int LedEffects::checkEffect(struct LedProperties effect_config){
      
      int action = -1;
      
      //Check if the effect is enabled
      if(effect_config.enabled){

           //Check if it does NOT exist in the database.
           if(!database.exist_id(effect_config.id)){
  
              action = CREATE;
           } 
           
           else{
           
              action = EDIT;
           }
      }
    
      else{

           //Check if it exists in the database.
           if(database.exist_id(effect_config.id)){
        
              action = REMOVE;
           }
      }

      return action;
  }


  void LedEffects::createEffect(struct LedProperties effect_config){

      String mode;
      int task_id;

      //Save the new effect in the data base
      database.save_id(effect_config.id);

       
      task_id = database.get_serial_id(effect_config.id);   
      mode = effect_config.mode;
       
      if(mode == "paint"){
     
        paint_effect[task_id] -> assign_id(effect_config.id);
        paint_effect[task_id] -> update(effect_config);   
        
      }


      if(mode == "blink"){
     
        blink_effect[task_id] -> assign_id(effect_config.id);
        blink_effect[task_id] -> update(effect_config);  
         
      }


      if(mode == "shift"){
     
        shift_effect[task_id] -> assign_id(effect_config.id);
        shift_effect[task_id] -> update(effect_config);   

      }

        
  }
  
  
  void LedEffects::editEffect(struct LedProperties effect_config){

    
      String mode;
      int task_id;
     
      task_id = database.get_serial_id(effect_config.id);   
      mode = effect_config.mode;
       
      if(mode == "paint"){
     
        paint_effect[task_id] -> update(effect_config);   
      
      }


      if(mode == "blink"){
     
        blink_effect[task_id] -> update(effect_config);   
    
      }


      if(mode == "shift"){
     
        shift_effect[task_id] -> update(effect_config);   
          
      }


  }

  
  void LedEffects::removeEffect(struct LedProperties effect_config){


      String mode;
      int task_id;

      mode = effect_config.mode;
      task_id = database.get_serial_id(effect_config.id);
      
      //Delete effect of the database
      database.delete_id(effect_config.id);
      

      if(mode == "paint"){
        
        paint_effect[task_id] -> update(effect_config);
        paint_effect[task_id] -> assign_id("");  

      }
  
      if(mode == "blink"){
        
        blink_effect[task_id] -> update(effect_config);
        blink_effect[task_id] -> assign_id("");  
                    
      }

      if(mode == "shift"){
        
        shift_effect[task_id] -> update(effect_config);
        shift_effect[task_id] -> assign_id("");  
      
      }

 
  }
  
  void LedEffects::saveBufferEffects(struct LedProperties effect_config){

   

    if(index_buffer_effects < TOTAL_EFFECTS){
      
        buffer_effects[index_buffer_effects] = effect_config;
        index_buffer_effects++;
        
    }
    
  }

  void LedEffects::enableUpdateEffects(){
    
      isUpdateEffectsEnabled = true;
   }


  void LedEffects::updatePendingEffects(){

    if(isUpdateEffectsEnabled){

        for(int i = 0; i < index_buffer_effects; i++){
          
            updateEffects(buffer_effects[i]);
        
        }
    
        index_buffer_effects = 0;
        isUpdateEffectsEnabled = false;
    }
    
  }
  

  void LedEffects::updateEffects(struct LedProperties effect_config){

      //Check the state of a effect. Returns if a state is going to create, edit or remove
      int action = checkEffect(effect_config);   
      switch(action){
        
        case CREATE:  
                createEffect(effect_config);
                break;
                
        case EDIT: 
                editEffect(effect_config);
                break; 
                      
        case REMOVE: 
                removeEffect(effect_config);
                break;
    
      } 

  }



  void LedEffects::bootingMode(void){

      clearEffects();

      struct LedProperties effect_config;
      effect_config = readStateConfig("BOOTING");
/*    
      effect_config.id = "BOOTING";
      effect_config.mode = "blink";
      effect_config.color_R = 20;
      effect_config.color_G = 20;
      effect_config.color_B = 20;
      effect_config.color_W = 0;
      effect_config.start_led = 1;
      effect_config.end_led = 400;
      effect_config.ms_on = 500;
      effect_config.ms_off = 500;
      effect_config.enabled = true; 
*/

      //Update the effect
      updateEffects(effect_config);
    
    
  }


  void LedEffects::readyMode(bool enabled){

      struct LedProperties effect_config;

      effect_config = readStateConfig("READY");

      if(enabled){
        
            clearEffects();
      
            /*
            effect_config.id = "READY";
            effect_config.mode = "paint";
            effect_config.color_R = 0;
            effect_config.color_G = 255;
            effect_config.color_B = 0;
            effect_config.color_W = 0;
            effect_config.start_led = 1;
            effect_config.end_led = 400;
            effect_config.enabled = enabled; 
            */
            
            //Update the effect
            updateEffects(effect_config);
      }

    else {
              
       effect_config.id = "READY";
       effect_config.mode = effect_config.mode;
       effect_config.enabled = false; 
       updateEffects(effect_config); 
    }
  }


  void LedEffects::exitMode(void){

      clearEffects();

      struct LedProperties effect_config;

      effect_config = readStateConfig("EXIT");
/*      
      effect_config.id = "EXIT";
      effect_config.mode = "paint";
      effect_config.color_R = 0;
      effect_config.color_G = 0;
      effect_config.color_B = 20;
      effect_config.color_W = 0;
      effect_config.start_led = 1;
      effect_config.end_led = 400;
      effect_config.enabled = true; 
*/
      //Update the effect
       updateEffects(effect_config);
    
  }

  void LedEffects::runningMode(void){    

    
     for(int i = 0; i < NUM_EFFECTS; i++){ 
      
          paint_effect[i] -> run();
          blink_effect[i] -> run();
          shift_effect[i] -> run();
      }
    
    
  }

  bool LedEffects::checkConnectionROS(void){

      bool shutdownROS = false;

        //Check connection
        if(timeoutACK > ROS_TIMEOUT)
            shutdownROS = true;

      return shutdownROS;
    
  }


  

  void LedEffects::runEffects(void){

    /* Añadido para debug */
    state = 777;

    switch(state){
      
      case BOOTING:
    
            // Booting mode, ROS is starting and Teensy is already
            bootingMode();
  
            state = WAITING_ROS;
        
            break;

      case WAITING_ROS:

             if(SerialUSB && firstACKFlag)
                 state = READY;
             break;
          
      case READY:

            // Ready mode, ROS is ready to send commands to Teensy
            
            readyMode(true);
  
            state = WAITING_FIRST_COMMAND;
          
            break;
          
      case WAITING_FIRST_COMMAND:

            if(firstCommandFlag){
              
                readyMode(false);
                
                //Reset timeout
                timeoutACK = 0;
  
                state = RUNNING;
            }     
  
      
            //Check connection
            if(checkConnectionROS())
                state = EXIT;
          
  
            break;
      
      case RUNNING:
               
            //Running mode, ROS sends commands to Teensy
  
            //Check connection
            if(checkConnectionROS())
                state = EXIT;
          
            break;
      
      case EXIT:
      
            // Exit mode, ROS is shutdown, Teensy set the leds in exit mode
            exitMode();
  
            firstCommandFlag = false;
            firstACKFlag = false;
  
            state = WAITING_ROS;
            
            break;
    }


    // Execute effects set by updateEffects
    runningMode();

          
  }

  void LedEffects::clearEffects(void){


  while(database.number_of_ids() > 0){
    
        String id;
        int task_id;


        id = database.get_first_id();
        task_id = database.get_serial_id(id);
        database.delete_id(id);
        
        struct LedProperties my_config;
        my_config.id = id;
        my_config.enabled = false;
      
        paint_effect[task_id] -> update(my_config);
        paint_effect[task_id] -> run();
        
        blink_effect[task_id] -> update(my_config);
        blink_effect[task_id] -> run();
        
        shift_effect[task_id] -> update(my_config);
        shift_effect[task_id] -> run();

        //Free the task of the assigned id
        paint_effect[task_id] -> assign_id("");
        blink_effect[task_id] -> assign_id("");
        shift_effect[task_id] -> assign_id("");
  

  
/*
     String id;  
     
     // Clear until the database is empty
      while(database.number_of_ids() > 0){

        id = database.get_first_id();
        
        struct LedProperties remove_config;
        remove_config.id = id;
        remove_config.enabled = false;

        removeEffect(remove_config);


        remove_config.mode = "blink";
        updateEffects(remove_config);


        remove_config.mode = "shift";
        updateEffects(remove_config);     

        
        remove_config.mode = "paint";
        updateEffects(remove_config);
*/

      }
    
  }


  

  
  String LedEffects::listID(void){


    return database.list_id();
    
  }



  String LedEffects::configDevice(String password, struct LedProperties device_config, String state){

      String message;
    
      if(password.equals("R0b0tn1K")){

  
          if(state.equals("BOOTING")){

              if (saveStateConfig(device_config, "BOOTING"))
                  message = "BOOTING state has been configured correctly";
              else
                  message = "Incorrect configuration";
          }

          else if(state.equals("READY")){

              if (saveStateConfig(device_config, "READY"))
                  message = "READY state has been configured correctly";
              else
                  message = "Incorrect configuration";
          }

          else if(state.equals("EXIT")){

              if(saveStateConfig(device_config, "EXIT"))
                  message = "EXIT state has been configured correctly";
              else
                  message = "Incorrect configuration";
          }

          else {
              message = "This state does not exist";
          }
        
      }
      else{
        
        message = "Incorrect password";
      }


     return message;
  }


  bool LedEffects::saveStateConfig(struct LedProperties device_config, String state){

    // Active wait to avoid writing to memory multiple times
    delay(2000);

    bool status = true; 

    byte state_addr = 0;
    byte mode_id = 0;
    byte color_R = 0, color_G = 0, color_B = 0, color_W = 0;
    byte start_led_MSB = 0, start_led_LSB = 0;
    byte end_led_MSB = 0, end_led_LSB = 0; 
    byte ms_off_MSB = 0, ms_off_LSB = 0;
    byte ms_on_MSB = 0, ms_on_LSB = 0;
    byte direction = 0;
    byte speed_MSB = 0, speed_LSB = 0;

    if(state.equals("BOOTING"))
        state_addr = 20;
    if(state.equals("READY"))
        state_addr = 40;
    if(state.equals("EXIT"))
        state_addr = 60;


    if(device_config.mode.equals("paint"))
        mode_id = 1;
    if(device_config.mode.equals("blink"))
        mode_id = 2;
    if(device_config.mode.equals("shift"))
        mode_id = 3;

    if (state_addr != 0 && mode_id != 0) {
      
       color_R = device_config.color_R;
       color_G = device_config.color_G;
       color_B = device_config.color_B;
       color_W = device_config.color_W;
    
       start_led_MSB = device_config.start_led >> 8;
       start_led_LSB = device_config.start_led & 255;
    
       end_led_MSB = device_config.end_led >> 8;
       end_led_LSB = device_config.end_led & 255;
    
       ms_off_MSB = device_config.ms_off >> 8;
       ms_off_LSB = device_config.ms_off & 255;
    
       ms_on_MSB = device_config.ms_on >> 8;
       ms_on_LSB = device_config.ms_on & 255;
    
       if(device_config.direction.equals("right"))
          direction = 0;
       else
          direction = 1;
          
       speed_MSB = device_config.speed >> 8;
       speed_LSB = device_config.speed & 255;
    
       // Parameters: - address, - value to be saved 
       EEPROM.write(state_addr + 1,  mode_id);
       EEPROM.write(state_addr + 2,  color_R);
       EEPROM.write(state_addr + 3,  color_G);
       EEPROM.write(state_addr + 4,  color_B);
       EEPROM.write(state_addr + 5,  color_W);
       EEPROM.write(state_addr + 6,  start_led_MSB);
       EEPROM.write(state_addr + 7,  start_led_LSB);
       EEPROM.write(state_addr + 8,  end_led_MSB);
       EEPROM.write(state_addr + 9,  end_led_LSB);
       EEPROM.write(state_addr + 10, ms_off_MSB);
       EEPROM.write(state_addr + 11, ms_off_LSB);
       EEPROM.write(state_addr + 12, ms_on_MSB);
       EEPROM.write(state_addr + 13, ms_on_LSB);
       EEPROM.write(state_addr + 14, direction);
       EEPROM.write(state_addr + 15, speed_MSB);
       EEPROM.write(state_addr + 16, speed_LSB);

    }

    else {
        status = false;
    }

    return status;
  }


  LedProperties LedEffects::readStateConfig(String state){
    
       byte state_addr = 0;
       byte mode_id = 0;
       byte color_R = 0, color_G = 0, color_B = 0, color_W = 0;
       byte start_led_MSB = 0, start_led_LSB = 0;
       byte end_led_MSB = 0, end_led_LSB = 0; 
       byte ms_off_MSB = 0, ms_off_LSB = 0;
       byte ms_on_MSB = 0, ms_on_LSB = 0;
       byte direction = 0;
       byte speed_MSB = 0, speed_LSB = 0;
    

       struct LedProperties device_config;
        
       if(state.equals("BOOTING"))
            state_addr = 20;
       if(state.equals("READY"))
            state_addr = 40;
       if(state.equals("EXIT"))
            state_addr = 60;
            
       device_config.id = state;  
       device_config.enabled = true;   
    
       mode_id       = EEPROM.read(state_addr + 1);
       color_R       = EEPROM.read(state_addr + 2);
       color_G       = EEPROM.read(state_addr + 3);
       color_B       = EEPROM.read(state_addr + 4);
       color_W       = EEPROM.read(state_addr + 5);
       start_led_MSB = EEPROM.read(state_addr + 6);
       start_led_LSB = EEPROM.read(state_addr + 7);
       end_led_MSB   = EEPROM.read(state_addr + 8);
       end_led_LSB   = EEPROM.read(state_addr + 9);
       
       ms_off_MSB    = EEPROM.read(state_addr + 10);
       ms_off_LSB    = EEPROM.read(state_addr + 11);
       ms_on_MSB     = EEPROM.read(state_addr + 12);
       ms_on_LSB     = EEPROM.read(state_addr + 13);
       direction     = EEPROM.read(state_addr + 14);
       speed_MSB     = EEPROM.read(state_addr + 15);
       speed_LSB     = EEPROM.read(state_addr + 16);
    
    
       if(mode_id == 1)
          device_config.mode = "paint";
       if(mode_id == 2)
          device_config.mode = "blink";
       if(mode_id == 3)
          device_config.mode = "shift"; 
    
       device_config.color_R = color_R;
       device_config.color_G = color_G;
       device_config.color_B = color_B;
       device_config.color_W = color_W;
    
       device_config.start_led = ((start_led_MSB << 8 ) | start_led_LSB);
       device_config.end_led = ((end_led_MSB << 8 ) | end_led_LSB);
    
       device_config.ms_off = ((ms_off_MSB << 8 ) | ms_off_LSB);
       device_config.ms_on = ((ms_on_MSB << 8 ) | ms_on_LSB);  
    
       if(direction == 0)
          device_config.direction = "right";
       else
          device_config.direction = "left";
    
       device_config.speed = ((speed_MSB << 8 ) | speed_LSB);

     
     return device_config;
    
  }


  String LedEffects::resetDevice(String password){

      String message;
      
      if(password.equals("R0b0tn1K")){
      
            /* Waring zone */
      
            // Active wait
            delay(5000);
      
            // Reset the hardware
            _reboot_Teensyduino_();

            // If the hardware is reset, it should never reach this line of code
            message = "ERROR: If you see this message, the device has not been reset";
            
      }

      else {

         message = "Incorrect password";
        
      }

      return message;
    
  }
  
  void LedEffects::startSequence(void){


  /* 
   * SEQUENCE:
   * 1. When Teensy is on, paint white in the led strip 
   * 2. Teensy waits for ros node (check if SerialUSB is online)
   * 3. When ros node starts, Teensy paint green in the led strip
   * 4. When the first led effect command is sent, Teensy remove green and set the effect received 
   */
/*
1. Robot apagado
2. Tira led con tensión. arduino apagado y PC apagado
  * los leds estan apagados porque no han recibido nada del arduino.

3. Tira led con tension, arduino encendido y PC apagado  (por ahora no es posible porque es el PC el que ofrece la tensión)
4. Tira led con tension, arduino encendido y PC encendido pero sin nodo corriendo.
  * en este caso, arduino envía el codigo de colores "booting/me estoy encendiendo".

5. Tira led con tension, arduino encendido y PC encendido y con nodo corriendo.
  * en este caso, el nodo envía el estado del robot y arduino lo traduce al código de colores necesario.

Desde el 5, se puede pasar a:
6. Tira led con tension, arduino encendido y PC encendido y nodo apagado.
6.a Tira led con tension, arduino encendido y PC apagado.


7. Tira led con tension, arduino apagado y PC apagado.
  * los leds se quedan con lo último que tenía (sin efectos, ejemplo: si está parpadeando, se quedará o encendido o apagado dependiendo de cuando ocurra)
*/

  struct LedProperties effect_config;

  
  while(!SerialUSB){

      // Booting mode, Teensy is starting
      effect_config.mode = "paint";
      effect_config.color_R = 20;
      effect_config.color_G = 20;
      effect_config.color_B = 20;
      effect_config.color_W = 0;
      effect_config.start_led = 1;
      effect_config.end_led = 400;
      effect_config.enabled = true;
      
      paint_effect[0] -> update(effect_config); 
      paint_effect[0] -> run();
      
  }
  
    effect_config.mode = "paint";
    effect_config.color_R = 0;
    effect_config.color_G = 0;
    effect_config.color_B = 0;
    effect_config.color_W = 0;
    effect_config.start_led = 1;
    effect_config.end_led = 400;
    effect_config.enabled = true;
    
    paint_effect[0] -> update(effect_config); 
    paint_effect[0] -> run();  

    
    paint_effect[0] -> resetEffectConfig();
        
  
  }

  void LedEffects::faultSequence(bool fault){

      struct LedProperties effect_config;
    
      if(fault){
        
        effect_config.mode = "blink";
        effect_config.color_R = 20;
        effect_config.color_G = 0;
        effect_config.color_B = 0;
        effect_config.color_W = 0;
        effect_config.start_led = 1;
        effect_config.end_led = 400;
        effect_config.ms_on = 100;
        effect_config.ms_off = 100;
        effect_config.enabled = true;
        
        blink_effect[0] -> update(effect_config); 
        blink_effect[0] -> run();
        
      }

      else{

         effect_config.enabled = false;
         blink_effect[0] -> update(effect_config); 
         blink_effect[0] -> run();
         blink_effect[0] -> resetEffectConfig();
        
      }
  
  
  }
  
  
