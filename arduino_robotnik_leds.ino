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
 * Terminar de poner el vector al resto de efectos
 * id debe ser string en mensaje
 * Vector dinamico en función del id máximo asignado o en función del numero de IDs existentes
 * Servicio para apagar todos los efectos
 */

#include <Adafruit_NeoPixel.h>
#include "ros.h"
#include <std_msgs/String.h>
#include "ShiftEffect.h"
#include "BlinkEffect.h"
#include "PaintEffect.h"

#include <robotnik_leds/LedsPaint.h>
#include <robotnik_leds/LedsBlink.h>
#include <robotnik_leds/LedsShift.h>

ros::NodeHandle  nh;
using robotnik_leds::LedsPaint;
using robotnik_leds::LedsBlink;
using robotnik_leds::LedsShift;


/* Variables globales para el modo paint, se llamará PAINT
(provisional, implementar un HandleMode)*/
/*
uint8_t  paint_id = 0;
uint8_t  paint_color_R = 0;
uint8_t  paint_color_G = 0;
uint8_t  paint_color_B = 0;
uint16_t paint_start_led = 0;
uint16_t paint_end_led = 0;
bool     paint_enabled = false;
*/

struct paint_leds{

    paint_leds(): 
        id(0),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        enabled(false) {}
    
    uint8_t  id;
    uint8_t  color_R;
    uint8_t  color_G;
    uint8_t  color_B;
    uint16_t start_led;
    uint16_t end_led;
    bool     enabled;
    
    } paint_config;


/* Variables globales para el modo Blink (provisional, implementar un HandleMode) */
struct blink_leds{

    blink_leds(): 
        id(0),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        ms_on (0),
        ms_off (0),
        enabled(false) {}
    
    uint8_t  id;
    uint8_t  color_R;
    uint8_t  color_G;
    uint8_t  color_B;
    uint16_t start_led;
    uint16_t end_led;
    uint16_t ms_on;
    uint16_t ms_off;
    bool     enabled;
    
    } blink_config;


/* Variables globales para el modo Shift (provisional, implementar un HandleMode) */
/*
uint8_t  shift_id = 0;
uint8_t  shift_color_R = 0;
uint8_t  shift_color_G = 0;
uint8_t  shift_color_B = 0;
uint16_t shift_start_led = 0;
uint16_t shift_end_led = 0;
String   shift_direction = "right";
uint16_t shift_speed = 0;
uint16_t shift_sleep = 0;
bool     shift_enabled = false;
*/

struct shift_leds{

    shift_leds(): 
        id(0),
        color_R(0),
        color_G(0),
        color_B(0),
        start_led(0),
        end_led(0),
        direction("right"),
        speed (0),
        sleep (0),
        enabled(false) {}
    
    uint8_t  id;
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


ShiftEffect shift_effect[5](pixels);

BlinkEffect blink_effect[5](pixels);

PaintEffect effect_5(pixels);
PaintEffect effect_6(pixels);

PaintEffect paint_effect[5](pixels);





elapsedMillis timeout_system;




void callback_paint(const LedsPaint::Request & req, LedsPaint::Response & res){

  paint_config.id = req.paint_id;
  paint_config.color_R = req.color_R;
  paint_config.color_G = req.color_G;
  paint_config.color_B = req.color_B;
  paint_config.start_led = req.start_led;
  paint_config.end_led = req.end_led;
  paint_config.enabled = req.enabled;
  
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
  
}


void callback_shift(const LedsShift::Request & req, LedsShift::Response & res){

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

}


ros::ServiceServer<LedsPaint::Request, LedsPaint::Response> server_paint_mode("robotnik_leds/set_leds/paint_mode",&callback_paint);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("robotnik_leds/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("robotnik_leds/set_leds/shift_mode",&callback_shift);




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
  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  Serial.begin(2000000);

  for(int i = 0; i<5; i++){

      paint_effect[i].assign_id(i);
      blink_effect[i].assign_id(i);
      shift_effect[i].assign_id(i);
      
  }
  
}

void loop()
{

  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }

  

  /*
  effect_3.blink_config.id = blink_id;
  effect_3.blink_config.color_R = blink_color_R;
  effect_3.blink_config.color_G = blink_color_G;
  effect_3.blink_config.color_B = blink_color_B;
  effect_3.blink_config.start_led = blink_start_led;
  effect_3.blink_config.end_led = blink_end_led;
  effect_3.blink_config.ms_on = blink_ms_on;
  effect_3.blink_config.ms_off = blink_ms_off;
  effect_3.blink_config.enabled = blink_enabled;
  */

 
  for(int i=0; i < 5; i++){

    memcpy(&paint_effect[i].paint_config , &paint_config, sizeof(paint_effect[i].paint_config));  
    paint_effect[i].paint_mode(paint_effect[i].paint_config);
    
    memcpy(&blink_effect[i].blink_config , &blink_config, sizeof(blink_effect[i].blink_config));  
    blink_effect[i].blink_mode(blink_effect[i].blink_config);

    memcpy(&shift_effect[i].shift_config , &shift_config, sizeof(shift_effect[i].shift_config));  
    shift_effect[i].shift_mode(shift_effect[i].shift_config);
  
  }

  
/*
    memcpy(&blink_effect[1].blink_config , &blink_config, sizeof(blink_effect[1].blink_config));  
    blink_effect[1].blink_mode(blink_effect[1].blink_config);

    memcpy(&blink_effect[2].blink_config , &blink_config, sizeof(blink_effect[2].blink_config));  
    blink_effect[2].blink_mode(blink_effect[2].blink_config);
  */

/*
    memcpy(&shift_effect[1].shift_config , &shift_config, sizeof(shift_effect[1].shift_config));  
    shift_effect[1].shift_mode(shift_effect[1].shift_config);
*/
  
/*
  effect_1.shift_config.id = 0;
  effect_1.shift_config.color_R = 0;
  effect_1.shift_config.color_G = 20;
  effect_1.shift_config.color_B = 0;
  effect_1.shift_config.start_led = 16;
  effect_1.shift_config.end_led = 30;
  effect_1.shift_config.direction = "left";
  effect_1.shift_config.speed = 1000;
  effect_1.shift_config.sleep = 1000;
  effect_1.shift_config.enabled = true;
  
  effect_1.shift_mode(effect_1.shift_config);



  effect_2.shift_config.id = 0;
  effect_2.shift_config.color_R = 10;
  effect_2.shift_config.color_G = 0;
  effect_2.shift_config.color_B = 0;
  effect_2.shift_config.start_led = 1;
  effect_2.shift_config.end_led = 15;
  effect_2.shift_config.direction = "right";
  effect_2.shift_config.speed = 1000;
  effect_2.shift_config.sleep = 1000;
  effect_2.shift_config.enabled = true;
   
  effect_2.shift_mode( effect_2.shift_config);


  effect_3.blink_config.id = 0;
  effect_3.blink_config.color_R = 0;
  effect_3.blink_config.color_G = 0;
  effect_3.blink_config.color_B = 20;
  effect_3.blink_config.start_led = 31;
  effect_3.blink_config.end_led = 45;
  effect_3.blink_config.ms_on = 750;
  effect_3.blink_config.ms_off = 750;
  effect_3.blink_config.enabled = true;
   
  effect_3.blink_mode(effect_3.blink_config);


  
  effect_4.blink_config.id = 0;
  effect_4.blink_config.color_R = 0;
  effect_4.blink_config.color_G = 10;
  effect_4.blink_config.color_B = 10;
  effect_4.blink_config.start_led = 46;
  effect_4.blink_config.end_led = 60;
  effect_4.blink_config.ms_on = 100;
  effect_4.blink_config.ms_off = 100;
  effect_4.blink_config.enabled = true;
   
  effect_4.blink_mode(effect_4.blink_config);



  
  effect_5.paint_config.id = 0;
  effect_5.paint_config.color_R = 10;
  effect_5.paint_config.color_G = 10;
  effect_5.paint_config.color_B = 10;
  effect_5.paint_config.start_led = 61;
  effect_5.paint_config.end_led = 70;
  effect_5.paint_config.enabled = true;
   
  effect_5.paint_mode(effect_5.paint_config);



  effect_6.paint_config.id = 0;
  effect_6.paint_config.color_R = 10;
  effect_6.paint_config.color_G = 20;
  effect_6.paint_config.color_B = 0;
  effect_6.paint_config.start_led = 71;
  effect_6.paint_config.end_led = 80;
  effect_6.paint_config.enabled = true;
   
  effect_6.paint_mode(effect_6.paint_config);

*/





/*
  blink_effect[1].blink_config.id = 1;
  blink_effect[1].blink_config.color_R = 0;
  blink_effect[1].blink_config.color_G = 0;
  blink_effect[1].blink_config.color_B = 21;
  blink_effect[1].blink_config.start_led = 1;
  blink_effect[1].blink_config.end_led = 15;
  blink_effect[1].blink_config.ms_on = 150;
  blink_effect[1].blink_config.ms_off = 150;
  blink_effect[1].blink_config.enabled = true;
   
  blink_effect[1].blink_mode( blink_effect[1].blink_config);


  blink_effect[2].blink_config.id = 2;
  blink_effect[2].blink_config.color_R = 0;
  blink_effect[2].blink_config.color_G = 21;
  blink_effect[2].blink_config.color_B = 0;
  blink_effect[2].blink_config.start_led = 16;
  blink_effect[2].blink_config.end_led = 30;
  blink_effect[2].blink_config.ms_on = 150;
  blink_effect[2].blink_config.ms_off = 150;
  blink_effect[2].blink_config.enabled = true;
   
  blink_effect[2].blink_mode( blink_effect[2].blink_config);
*/



/*
  shift_effect[1].shift_config.id = 1;
  shift_effect[1].shift_config.color_R = 0;
  shift_effect[1].shift_config.color_G = 0;
  shift_effect[1].shift_config.color_B = 21;
  shift_effect[1].shift_config.start_led = 11;
  shift_effect[1].shift_config.end_led = 20;
  shift_effect[1].shift_config.direction = "left";
  shift_effect[1].shift_config.speed = 500;
  shift_effect[1].shift_config.sleep = 0;
  shift_effect[1].shift_config.enabled = true;
   
  shift_effect[1].shift_mode( shift_effect[1].shift_config);
  */ 
}
