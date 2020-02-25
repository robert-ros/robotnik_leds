/*
 * rosserial Service Server
 * Soporta Arduino UNO, Arduino Leonardo, Arduino Nano, Arduino Micro y Teeny 3.2
 */


/*  TO DO
 *   
 *  IMPORTANTE: pueden haber fallos de sincronizacion ya que las tiras no hacen 1 segundo perfecto 
 *              esto es debido a esperas activas de la librería Adafruit o de ROS. La solucion es
 *              compensar la espera en los millis();
 *              
 *  CORREGIR BUG: a veces cuando Blink debe apagarse, no lo hace
 *              
 *  Ver rendimiento uC, con isUpdate y isClear
 *  Cuando se envie un led 0 a las funciones, gestionarlo para que de un error diciendo que los leds van de 1 a N
 *  Añadir todas las funciones a shift y:
 *        - Añadir un tiempo que permanece apagado de una secuencia a otra
 *  Añadur nuevo modo: cyclon_mode (led de un punto a otro)
 *  
 *  // Tres tipos de sincronizacion para ROS para varias zonas de trabajo usando un mismo efecto (cuando se envia desde ROS)
 *      - Sincronizacion automática: involucra tiempos y posicion de leds, seguramente no sea exacta y tenga desincronizacion
 *      - Tiempo de cortesia: cuando se envia una nueva zona con un efecto, se espera un tiempo antes de comenzar
 *                            para ver si llegan otras zonas con el mismo efecto. De este modo se ponen en marcha
 *                            todas las zonas a la vez.
 *      - Flag de inicio: desde ROS se envian las zonas que emplean el mismo efecto pero con enabled = false, esto hace
 *                        que las zonas no se pongan a trabajar hasta que enabled = true en cada una de las zonas. Se 
 *                        emplea un flag desde ROS en el que enviando por rosservice el id=256 ponga todas las zonas a
 *                        true y por tanto las zonas se inicen a la misma vez
 *                        
 *                        
 *                        
 *  //Sincronización en el programa (opcional pero recomendable)
 *      - Para un mismo efecto, por ejemplo shift, cuando la zona que tenga menos leds se reinicia esta se esperará
 *        apagada hasta que lo agan el resto de zonas. Cuando la ultima zona se apague se reinician todas las cuentas
 *        y se ponen en marcha a la vez todas las zonas. Esto se repetirá cada vez que la zona con menos leds se reinicie
 *       
 *
 *  //Actualización de zonas de trabajo que emplean un mismo modo
 *     - En el robot hay dos zonas de trabajo que emplean el mismo modo. Ahora se quiere añadir una tercera zona de 
 *       trabajo que emplea el mismo modo. Sin embargo hay que sincronizar la nueva zona con el resto. La alternativa
 *       propuesta es que cuando se añada la nueva zona, el resto de zonas activas se reseteen. Esto provoca que los 
 *       leds que usen el mismo modo se apaguen para volver a su zona de inicio y esteticamente no puede quedar bien.
 *
 * Tener funciones comunes para los tres tipos de efectos, estas se heradaran de LedEffect, actualizar, borrar, etc
 * Mejorar el mecanismo de borrado de una zona, cuando se actualiza con una nueva zona de trabajo (sobretodo en paint)
 * BUG/MEJORA: en blink, hasta que no cambie de estado no se actualiza la tira ya que no entra a la funcion para ahorrar recursos. Hay que entrar cada cierto tiempo
 *             para poder actualizar la tira en vez de tener que esperar al encendido o apagado. En blinks muy grandes se nota que tarda en actualizar
 * Implementar servicio para borrar efectos
 * Vector dinamico en función del id máximo asignado o en función del numero de IDs existentes

 */

#include <Adafruit_NeoPixel.h>
#include "ros.h"

#include "IdHandler.h"
#include "ShiftEffect.h"
#include "BlinkEffect.h"
#include "PaintEffect.h"

#include <robotnik_leds_sdk/LedsPaint.h>
#include <robotnik_leds_sdk/LedsBlink.h>
#include <robotnik_leds_sdk/LedsShift.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle  nh;
using robotnik_leds_sdk::LedsPaint;
using robotnik_leds_sdk::LedsBlink;
using robotnik_leds_sdk::LedsShift;
using std_srvs::Trigger;

/* Variables globales para el modo paint */
struct paint_leds{

    paint_leds(): 
        id(""),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        enabled(false) {}
    
    String   id;
    uint8_t  color_R;
    uint8_t  color_G;
    uint8_t  color_B;
    uint16_t start_led;
    uint16_t end_led;
    bool     enabled;
    
    } paint_config;


/* Variables globales para el modo Blink */
struct blink_leds{

    blink_leds(): 
        id(""),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        ms_on (0),
        ms_off (0),
        enabled(false) {}
    
    String   id;
    uint8_t  color_R;
    uint8_t  color_G;
    uint8_t  color_B;
    uint16_t start_led;
    uint16_t end_led;
    uint16_t ms_on;
    uint16_t ms_off;
    bool     enabled;
    
    } blink_config;


/* Variables globales para el modo Shift */
struct shift_leds{

    shift_leds(): 
        id(""),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        direction("right"),
        speed (0),
        sleep (0),
        enabled(false) {}
    
    String   id;
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




/* Tira led */
#define PIN        6
#define NUMPIXELS  130
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

IdHandler id_handler;

#define NUM_EFFECTS 5
ShiftEffect shift_effect[NUM_EFFECTS](pixels);
BlinkEffect blink_effect[NUM_EFFECTS](pixels);
PaintEffect paint_effect[NUM_EFFECTS](pixels);


elapsedMillis timeout_system;


void callback_paint(const LedsPaint::Request & req, LedsPaint::Response & res){

  int task_id;
  
  paint_config.id = req.paint_id;
  paint_config.color_R = req.color_R;
  paint_config.color_G = req.color_G;
  paint_config.color_B = req.color_B;
  paint_config.start_led = req.start_led;
  paint_config.end_led = req.end_led;
  paint_config.enabled = req.enabled;

  if(req.enabled){

      //Check if id exists
      if(!id_handler.exist_id(req.paint_id)){

          //If id not exists, then create it
          id_handler.save_id(req.paint_id); 
          task_id = id_handler.get_serial_id(req.paint_id);
          paint_effect[task_id].assign_id(req.paint_id);
     }
     else{
          //If id exists, edit it
          task_id = id_handler.get_serial_id(req.paint_id);
          paint_effect[task_id].assign_id(req.paint_id);
     }
    
  }
  else {

    //Check if id exists
    if(id_handler.exist_id(req.paint_id)){

        // If id exists, then delete it
        
        task_id = id_handler.get_serial_id(req.paint_id);
        id_handler.delete_id(req.paint_id);
    
        //Disable effect (enabled = false)
        memcpy(&paint_effect[task_id].paint_config , &paint_config, sizeof(paint_effect[task_id].paint_config));  
        paint_effect[task_id].paint_mode(paint_effect[task_id].paint_config);
    
        //Free the task of the assigned id
        paint_effect[task_id].assign_id("");
    }
  }
  
}


void callback_blink(const LedsBlink::Request & req, LedsBlink::Response & res){

  blink_config.id = req.blink_id;
  blink_config.color_R = req.color_R;
  blink_config.color_G = req.color_G;
  blink_config.color_B = req.color_B;
  blink_config.start_led = req.start_led;
  blink_config.end_led = req.end_led;
  blink_config.ms_on = req.ms_on;
  blink_config.ms_off = req.ms_off; 
  blink_config.enabled = req.enabled;

  int task_id;

  if(req.enabled){

      //Check if id exists
      if(!id_handler.exist_id(req.blink_id)){

          //If id not exists, then create it
          id_handler.save_id(req.blink_id); 
          task_id = id_handler.get_serial_id(req.blink_id);
          blink_effect[task_id].assign_id(req.blink_id);
     }
     else{
          //If id exists, edit it
          task_id = id_handler.get_serial_id(req.blink_id);
          blink_effect[task_id].assign_id(req.blink_id);
     }
    
  }
  else {

    //Check if id exists
    if(id_handler.exist_id(req.blink_id)){

        // If id exists, then delete it
        
        task_id = id_handler.get_serial_id(req.blink_id);
        id_handler.delete_id(req.blink_id);
    
        //Disable effect (enabled = false)
        memcpy(&blink_effect[task_id].blink_config , &blink_config, sizeof(blink_effect[task_id].blink_config));  
        blink_effect[task_id].blink_mode(blink_effect[task_id].blink_config);
    
        //Free the task of the assigned id
        blink_effect[task_id].assign_id("");
    }
  }
  
  
}


void callback_shift(const LedsShift::Request & req, LedsShift::Response & res){

  int task_id;

  shift_config.id = req.shift_id;
  shift_config.color_R = req.color_R;
  shift_config.color_G = req.color_G;
  shift_config.color_B = req.color_B;
  shift_config.start_led = req.start_led;
  shift_config.end_led = req.end_led;
  shift_config.direction = req.direction;
  shift_config.speed = req.speed;
  shift_config.sleep = req.sleep;
  shift_config.enabled = req.enabled;


  if(req.enabled){

      //Check if id exists
      if(!id_handler.exist_id(req.shift_id)){

          //If id not exists, then create it
          id_handler.save_id(req.shift_id); 
          task_id = id_handler.get_serial_id(req.shift_id);
          shift_effect[task_id].assign_id(req.shift_id);
     }
     else{
          //If id exists, edit it
          task_id = id_handler.get_serial_id(req.shift_id);
          shift_effect[task_id].assign_id(req.shift_id);
     }
    
  }
  else {

    //Check if id exists
    if(id_handler.exist_id(req.shift_id)){

        // If id exists, then delete it
        
        task_id = id_handler.get_serial_id(req.shift_id);
        id_handler.delete_id(req.shift_id);
    
        //Disable effect (enabled = false)
        memcpy(&shift_effect[task_id].shift_config , &shift_config, sizeof(shift_effect[task_id].shift_config));  
        shift_effect[task_id].shift_mode(shift_effect[task_id].shift_config);
    
        //Free the task of the assigned id
        shift_effect[task_id].assign_id("");
    }
  }

}


void callback_clear(const Trigger::Request & req, Trigger::Response & res){

   //Redefinir clear para que borre los id asignados en formato String ya que hay que tener en cuenta la nueva implementación 
  /*
   for (int i= 0; i < NUM_EFFECTS; i++){
    
      paint_config.id = String(i);
      paint_config.enabled = false;
      memcpy(&paint_effect[i].paint_config , &paint_config, sizeof(paint_effect[i].paint_config));  
      paint_effect[i].paint_mode(paint_effect[i].paint_config);

      blink_config.id = String(i);
      blink_config.enabled = false;
      memcpy(&blink_effect[i].blink_config , &blink_config, sizeof(blink_effect[i].blink_config));  
      blink_effect[i].blink_mode(blink_effect[i].blink_config);


      shift_config.id = String(i);
      shift_config.enabled = false;
      memcpy(&shift_effect[i].shift_config , &shift_config, sizeof(shift_effect[i].shift_config));  
      shift_effect[i].shift_mode(shift_effect[i].shift_config);

   }
  */
   
   //uint8_t test=3;
   // char cadena[16];
   //sprintf(cadena, "%d", test);
   //res.message = cadena;
   
   res.success = true;
   res.message = "All led effects have been disabled";
  
}


void callback_list_id(const Trigger::Request & req, Trigger::Response & res){


  char list_id[300];
  
  id_handler.list_id().toCharArray(list_id, 300);

  res.success = true;
  res.message = list_id;
  
}


// signaling_led_device/set_effect
ros::ServiceServer<LedsPaint::Request, LedsPaint::Response> server_paint_mode("arduino_signaling_led/set_leds/paint_mode",&callback_paint);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("arduino_signaling_led/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("arduino_signaling_led/set_leds/shift_mode",&callback_shift);

ros::ServiceServer<Trigger::Request, Trigger::Response> server_clear_leds("arduino_signaling_led/clear_effects",&callback_clear);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_list_id("arduino_signaling_led/list_id",&callback_list_id);



void setup()
{

  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   


  nh.initNode();
  nh.advertiseService(server_paint_mode);
  nh.advertiseService(server_blink_mode);
  nh.advertiseService(server_shift_mode);
  nh.advertiseService(server_clear_leds);
  nh.advertiseService(server_list_id);
  
 
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);


  Serial.begin(2000000);

/*
  while(!Serial){;}
*/

  for(int i = 0; i < NUM_EFFECTS; i++){

    
      paint_effect[i].assign_id("");
      blink_effect[i].assign_id("");
      shift_effect[i].assign_id("");
      
  }

/*
    id_handler.print_id_data_base();
    
    id_handler.save_id("led_front_left");
    id_handler.save_id("led_front_right");
    id_handler.save_id("led_rear_left");
    id_handler.save_id("led_rear_right");
    
    id_handler.print_id_data_base();
    
    id_handler.delete_id ("led_rear_left");
    id_handler.print_id_data_base();
    
    Serial.println(id_handler.number_of_ids());
    
    id_handler.save_id ("patata");
    id_handler.print_id_data_base();
    
    id_handler.delete_id("patata");
    id_handler.delete_id("led_front_right");

    id_handler.print_id_data_base();
    
    Serial.println(id_handler.number_of_ids());    
    Serial.println(id_handler.exist_id("patata"));
    Serial.println(id_handler.list_id());   
*/

}

void loop()
{


  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }


  for(int i=0; i < NUM_EFFECTS; i++){

    memcpy(&paint_effect[i].paint_config , &paint_config, sizeof(paint_effect[i].paint_config));  
    paint_effect[i].paint_mode(paint_effect[i].paint_config);
    
    memcpy(&blink_effect[i].blink_config , &blink_config, sizeof(blink_effect[i].blink_config));  
    blink_effect[i].blink_mode(blink_effect[i].blink_config);

    memcpy(&shift_effect[i].shift_config , &shift_config, sizeof(shift_effect[i].shift_config));  
    shift_effect[i].shift_mode(shift_effect[i].shift_config);
  
  }
  
  
}
