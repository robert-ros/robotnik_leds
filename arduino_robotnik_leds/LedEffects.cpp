



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
