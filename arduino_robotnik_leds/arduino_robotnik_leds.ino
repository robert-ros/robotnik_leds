/*
 * rosserial Service Server
 * Soporta Arduino UNO, Arduino Leonardo, Arduino Nano, Arduino Micro y Teeny 3.2
 */


/*  TO DO
 *   
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
 * 
 * # TO DO
# Implementar secuencia de incio
# led_state debe aceptar multiples nombres de leds


# Plantear la gestion de colisiones entre efectos por capas
# Revisar los paths globales en arduino
# Proponer en led_command_interface el parametro reservado CLEAR para borrar todo
# Crear servicio config para modifcar parametros internos de Teensy
#       -Funcion de reset para Teensy
#       -Funcion para cambiar el nombre del servicio (util para namspace)
#       -Debe poseer una clave de acceso: p.e: robot
#       -Funcion para cambiar el comportamiento del inicio de los leds


 */


// #include <Adafruit_NeoPixel.h>
#include <WS2812Serial.h>
#include "ros.h"

#include "IdHandler.h"
#include "ShiftEffect.h"
#include "BlinkEffect.h"
#include "PaintEffect.h"
#include "CommonEffect.h"
#include "LedEffects.h"

#include <robotnik_leds_msgs/LedEffects.h>
#include <robotnik_leds_msgs/LedConfig.h>
#include <robotnik_leds_msgs/LedReset.h>
#include <std_srvs/Trigger.h>

ros::NodeHandle  nh;
//using robotnik_leds_sdk::LedEffects;
using robotnik_leds_msgs::LedConfig;
using robotnik_leds_msgs::LedReset;
using std_srvs::Trigger;



/* Tira led */
#define PIN        1
#define NUMPIXELS  450

byte drawingMemory[NUMPIXELS*4];         //  4 bytes per LED for RGBW
DMAMEM byte displayMemory[NUMPIXELS*16]; // 16 bytes per LED for RGBW
WS2812Serial pixels(NUMPIXELS, displayMemory, drawingMemory, PIN, WS2812_RGBW);


IdHandler id_handler;

elapsedMillis timeout_ack;

#define NUM_EFFECTS 10

LedEffects  led_effects = LedEffects(pixels);

elapsedMillis timeout_system;
elapsedMillis timeout_leds;

elapsedMillis timeout_service;


void clear_led_effects(){

  timeout_ack = 0;  
  led_effects.timeoutACK = 0;
  
  led_effects.clearEffects();

}


void callback_set_led_effects(const robotnik_leds_msgs::LedEffects::Request & req, robotnik_leds_msgs::LedEffects::Response & res){

  led_effects.timeoutACK = 0;
   
  led_effects.firstCommandFlag = true;
  
  struct LedProperties effect_config;

  effect_config.id = req.id;
  effect_config.mode = req.mode;
  effect_config.channel = req.channel;
  effect_config.type = req.type;
  effect_config.color_R = req.color_R;
  effect_config.color_G = req.color_G;
  effect_config.color_B = req.color_B;
  effect_config.color_W = req.color_W;
  effect_config.start_led = req.start_led;
  effect_config.end_led = req.end_led;
  effect_config.ms_on = req.ms_on;
  effect_config.ms_off = req.ms_off;
  effect_config.fade_in = req.fade_in;
  effect_config.fade_out = req.fade_out;
  effect_config.background_R = req.background_R;
  effect_config.background_G = req.background_G;
  effect_config.background_B = req.background_B;
  effect_config.background_W = req.background_W;
  effect_config.direction = req.direction;
  effect_config.speed = req.speed;
  effect_config.sleep = req.sleep;
  effect_config.led_increment = req.led_increment;
  effect_config.enabled = req.enabled;


  //Save updates in the buffer
  led_effects.saveBufferEffects(effect_config);
   

}


void callback_update_led_effects(const Trigger::Request & req, Trigger::Response & res){

   led_effects.enableUpdateEffects();
   res.success = true;
   res.message = "All effects stored in the buffer have been updated";
 

}




void callback_clear(const Trigger::Request & req, Trigger::Response & res){

   timeout_ack = 0;
   led_effects.timeoutACK = 0;

   clear_led_effects();
   res.success = true;
   res.message = "All led effects have been disabled";
  
}


void callback_list_id(const Trigger::Request & req, Trigger::Response & res){

  timeout_ack = 0;
  led_effects.timeoutACK = 0;
  
  char list_id[300];
  
  led_effects.listID().toCharArray(list_id, 300);

  res.success = true;
  res.message = list_id;
  
}


void callback_ack(const Trigger::Request & req, Trigger::Response & res){
  
  timeout_ack = 0;
  led_effects.firstACKFlag = true;
  led_effects.timeoutACK = 0;
  
  //digitalWrite(13,LOW);
  res.success = true;
  res.message = "OK";
  
}

void callback_config(const LedConfig::Request & req, LedConfig::Response & res){

    struct LedProperties device_config;
    String password; 
    String state;
    char   message[300];

    state = req.state;
    password = req.password;

    device_config.mode = req.mode;
    device_config.color_R = req.color_R;
    device_config.color_G = req.color_G;
    device_config.color_B = req.color_B;
    device_config.color_W = req.color_W;
    device_config.start_led = req.start_led;
    device_config.end_led = req.end_led;
    device_config.ms_on = req.ms_on;
    device_config.ms_off = req.ms_off;
    device_config.direction = req.direction;
    device_config.speed = req.speed;
    device_config.led_increment = 1;
    
    led_effects.configDevice(password, device_config, state).toCharArray(message, 300);


    
    res.message = message;
  
}


void callback_reset(const LedReset::Request & req, LedReset::Response & res){
  
   char message[300]; 
   String password;

   password = req.password;
    
   led_effects.resetDevice(password).toCharArray(message, 300);
  
   res.message = message;
   
  }




ros::ServiceServer<robotnik_leds_msgs::LedEffects::Request, robotnik_leds_msgs::LedEffects::Response> server_set_led_effects("arduino_led_signaling/set_led_properties",&callback_set_led_effects);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_update_led_effects("arduino_led_signaling/update_led_properties",&callback_update_led_effects);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_clear_leds("arduino_led_signaling/clear_effects",&callback_clear);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_list_id("arduino_led_signaling/list_id",&callback_list_id);
ros::ServiceServer<LedConfig::Request, LedConfig::Response> server_config("arduino_led_signaling/config/default_states",&callback_config);
ros::ServiceServer<Trigger::Request, Trigger::Response> server_ack("arduino_led_signaling/ack",&callback_ack);
ros::ServiceServer<LedReset::Request, LedReset::Response> server_reset("arduino_led_signaling/config/reset_device",&callback_reset);



void setup()
{

/*
  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   
*/

  nh.initNode();
  nh.advertiseService(server_set_led_effects);
  nh.advertiseService(server_update_led_effects);
  nh.advertiseService(server_clear_leds);
  nh.advertiseService(server_list_id);
  nh.advertiseService(server_ack);
  nh.advertiseService(server_config);
  nh.advertiseService(server_reset);
 
 
  pixels.begin();
  pixels.clear();
  pixels.show();
  
  pinMode(13,OUTPUT); 
  pinMode(23,OUTPUT);
  digitalWrite(13,LOW);
  Serial.begin(2000000);


  //while(!Serial){;}
  Serial2.begin(2000000);
  
}

bool flag = true;
float tic=0, toc = 0;
float aux = 0;
int count = 0;

int ledState = LOW;

void toggleLed(int led){

  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);

}


void loop()
{

  
  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }

  // Refresh time is 20 ms by default (min value)
  if(timeout_leds > COMMON_EFFECT_REFRESH_TIME){ 
    
    timeout_leds = 0;
    led_effects.runEffects();
    pixels.show();
    led_effects.updatePendingEffects();

   }







/*
  if(flag) {

    flag = false;
   
    struct LedProperties effect_config;

    effect_config.id = "blink_test";
    effect_config.mode = "blink";
    effect_config.color_R = 0;
    effect_config.color_G = 150;
    effect_config.color_B = 0;
    effect_config.color_W = 0;
    effect_config.background_R = 0;
    effect_config.start_led = 1; 
    effect_config.end_led = 100; 
    effect_config.ms_on = 1000;
    effect_config.ms_off = 1000;
    effect_config.fade_in = 500;
    effect_config.fade_out = 500;
    effect_config.enabled = true;

    led_effects.saveBufferEffects(effect_config);
    led_effects.enableUpdateEffects();

  }
 
  else{

   // Refresh time is 20 ms by default (min value)
   if(timeout_leds >= COMMON_EFFECT_REFRESH_TIME){ 

     timeout_leds = 0;
     //tic = micros();
     led_effects.runEffects();
     //toggleLed(23);
     //digitalWrite(23,HIGH);
     pixels.show();
     //digitalWrite(23,LOW);
     led_effects.updatePendingEffects();
     //toc = micros();
     //Serial.println(toc-tic);
     
     //Serial.print("-------------------------");
     //Serial.println(count++);
     
   }   
 }
*/


  
}
