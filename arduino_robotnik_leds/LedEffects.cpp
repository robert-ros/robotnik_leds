



  #include "LedEffects.h"

  
  LedEffects::LedEffects(Adafruit_NeoPixel &pixels) {


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

  void LedEffects::runEffects(void){

      for(int i = 0; i < NUM_EFFECTS; i++){ 
          
          paint_effect[i] -> run();
          blink_effect[i] -> run();
          shift_effect[i] -> run();
        
      }    
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
  
  
