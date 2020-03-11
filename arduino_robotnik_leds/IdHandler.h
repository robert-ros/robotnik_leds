/*
 * IdHandle id(uint32_t size) // Crea un vector de Strings para almacenar los identificadores. Se inicializa en ""
 *                            // Un espacio está disponible si == "" Un espacio esta ocupado si == "cadena de texto identificador"
 * push_id(String id)         // Guarda el id indicado en el primer espacio diponible del vector
 * pop_id(String id)          // Elimina el id. El espacio anteriormente ocupado pasará a valer "". NO se desplazan el resto de elementos del vector
 * available_id(String id)    // Sino hay argumentos devuelve el nuḿero de identificadores activos que hay en el vector (que no sean "") 
 *                               Si se pasa como argumento el id, indica si existe ese identificador en la base de datos
 * get_serial_id              // A cada id en formato de texto se le asigna un id numérico para que pueda ser interpretado por el microcontrolador
 *                            // Para ello devuleve la posición del vector en el que está almacenado el id en formato de texto
 * 
 * get_list_id()              // Devuelve en un string todos los ID existentes de la base de datos
 * 
 * IdHandle id
 * 
 * En callback_mode:
 * 
 * if enabled == true
 *    id.push_id(req.mode_id)                            // Almacena en la base de datos un nuevo id en formato string
 *    task_id = id.get_serial_id(req.mode_id)            // Se guarda el id nuḿerico que representa el id en formato string
 *    mode_effect_task[task_id].assign_id(req.mode_id)   // Se configura la tarea con el id asignado
 * 
 * if enabled == false
 *    task_id = id.get_serial_id(req.mode_id)            // Se guarda el id nuḿerico que representa el id en formato string
 *    id.pop_id(req.mode_id)                             // Elimina de la base de datos el id indicado
 *    mode_effect_task[task_id].assign_id("")            // Libera la tarea del id asignado
 *   
 *   
 */



  #ifndef _ID_HANDLER_H_
  #define _ID_HANDLER_H_
  
  #include <Arduino.h>
  
  
  
  class IdHandler{
    
    private:

      const int M_DB = 30;       // Max memory of data base
      String id_data_base[30];   // Data base initialized with a memory of 10 elements

      int find_first_string(String value);
      int find_available_place(void);
  
    public:
  
      IdHandler();
      void save_id (String id);
      void delete_id (String id);
      int  get_serial_id (String id);

      bool exist_id (String id);
      int  number_of_ids(void);
      String list_id(void);
      
      String get_first_id(void);
      

      
      
      
      void print_id_data_base();
      
 
  };


  
  #endif
