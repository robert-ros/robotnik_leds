
  #include "IdHandler.h"



  IdHandler::IdHandler() {

      //Initialize data base with null values
      for(int i=0; i< M_DB; i++){
        
        id_data_base[i] = "";
          
      }    
  }

  
  void IdHandler::save_id(String id){

      int index = find_available_place();

      if (index >= 0){    
          //Save the data
          id_data_base[index] = id;
      }

  }

  void IdHandler::delete_id(String id){

      int serial_id = get_serial_id(id);

      if (serial_id >= 0){
          //Delete the data
          id_data_base[serial_id] = "";
      
      }
  }



  int IdHandler::get_serial_id (String id){
 
    //Find the position where "String id" is stored in the database
    return find_first_string(id);
    
  }

  int IdHandler::find_available_place(){
    
    //Find the first available place in data base
    return find_first_string("");  
  
  }

  


  bool IdHandler::exist_id (String id){

    bool exist;
    int index = find_first_string(id);

    if(index >= 0)
        exist = true;
    else
        exist = false;
    
    return exist;
  }

 
  int IdHandler::number_of_ids(){

      int number_of_ids = 0;
    
      for (int i= 0; i < M_DB; i++){
  
          if(id_data_base[i] != ""){
            
              number_of_ids++;    
          }  
      }

    return number_of_ids;
  }


  String IdHandler::list_id(){

      String list_id = "";
      
      for (int i= 0; i < M_DB ; i++){
    
        if(id_data_base[i] != ""){
              
          list_id = list_id + ", " + id_data_base[i];   
        }  
      }

      //Delete first ", "
      list_id.remove(0,2);
      
    return list_id;
  }


  void IdHandler::print_id_data_base(){


      for(int i=0; i< M_DB; i++){
        
          Serial.print(id_data_base[i]);
          Serial.print(", ");
      }

      Serial.println("");
      delay(200);
  
  }

  
  int IdHandler::find_first_string(String value){

    int index = -1;

    //Exit when it finds first the String value in data base
    for(int i=0; i< M_DB; i++){
      
      if(id_data_base[i] == value){
          
            index = i;
            break;
      }  
    }
    return index;     
  }




  
