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
 */

#include <Adafruit_NeoPixel.h>
#include "ros.h"
#include <std_msgs/String.h>
#include "LedsEffect.h"
#include "ShiftEffect.h"
#include "BlinkEffect.h"
#include <robotnik_leds/LedsPaint.h>
#include <robotnik_leds/LedsBlink.h>
#include <robotnik_leds/LedsShift.h>

ros::NodeHandle  nh;
using robotnik_leds::LedsPaint;
using robotnik_leds::LedsBlink;
using robotnik_leds::LedsShift;


/* Variables globales para el modo paint, se llamará PAINT
(provisional, implementar un HandleMode)*/
uint8_t  paint_id = 0;
uint8_t  paint_color_R = 0;
uint8_t  paint_color_G = 0;
uint8_t  paint_color_B = 0;
uint16_t paint_start_led = 0;
uint16_t paint_end_led = 0;
bool     paint_enabled = false;


/* Variables globales para el modo Blink (provisional, implementar un HandleMode) */
uint8_t  blink_id = 0;
uint8_t  blink_color_R = 0;
uint8_t  blink_color_G = 0;
uint8_t  blink_color_B = 0;
uint16_t blink_start_led = 0;
uint16_t blink_end_led = 0;
uint16_t blink_ms_on = 0;
uint16_t blink_ms_off = 0;
bool     blink_enabled = false;


/* Variables globales para el modo Shift (provisional, implementar un HandleMode) */
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




/* Tira led */
#define PIN        6
#define NUMPIXELS  130
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

ShiftEffect effect_1(pixels);
ShiftEffect effect_2(pixels);

BlinkEffect effect_3(pixels);
BlinkEffect effect_4(pixels);

int i;

elapsedMillis timeout_system;




void callback_paint(const LedsPaint::Request & req, LedsPaint::Response & res){

  paint_id = req.paint_id;
  paint_color_R = req.color_R;
  paint_color_G = req.color_G;
  paint_color_B = req.color_B;
  paint_start_led = req.start_led;
  paint_end_led = req.end_led;
  paint_enabled = req.enabled;
  
}


void callback_blink(const LedsBlink::Request & req, LedsBlink::Response & res){

  blink_id = req.blink_id;
  blink_color_R = req.color_R;
  blink_color_G = req.color_G;
  blink_color_B = req.color_B;
  blink_start_led = req.start_led;
  blink_end_led = req.end_led;
  blink_ms_on = req.ms_on;
  blink_ms_off = req.ms_off; 
  blink_enabled = req.enabled;
  
}


void callback_shift(const LedsShift::Request & req, LedsShift::Response & res){

  shift_id = req.shift_id;
  shift_color_R = req.color_R;
  shift_color_G = req.color_G;
  shift_color_B = req.color_B;
  shift_start_led = req.start_led;
  shift_end_led = req.end_led;
  shift_direction = req.direction;
  shift_speed = req.speed;
  shift_sleep = req.sleep;
  shift_enabled = req.enabled;

}


ros::ServiceServer<LedsPaint::Request, LedsPaint::Response> server_paint_mode("robotnik_leds/set_leds/paint_mode",&callback_paint);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("robotnik_leds/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("robotnik_leds/set_leds/shift_mode",&callback_shift);



// ============================== P A I N T    M O D E ================================= //


uint8_t paint_mode(uint8_t paint_id,
                   uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led, 
                   bool enabled){

    static uint8_t paint_state = 0;
    static bool isUpdated = false;  
    static uint16_t paint_pixels = 0;  // Numero de pixeles involucrados en realizar el efecto blink


    // Detecta si ha actualizado algun parámetro de la tira. Esto permite que el microcontrolador
    // no actualice constantemente la tira, ya que el valor es siempre el mismo sino han habido cambios
 
    static uint8_t last_color_R = 0, last_color_G = 0, last_color_B = 0;
    static uint16_t last_start_led = 0, last_end_led = 0;

    if(last_color_R != color_R || last_color_G != color_G || last_color_B != color_B ||
       last_start_led != start_led || last_end_led != end_led){

        //Algun parametro se ha actualizado
        isUpdated = true;
        last_color_R = color_R;
        last_color_G = color_G;
        last_color_B = color_B;
        last_start_led = start_led;
        last_end_led = end_led;
    }

    paint_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados


    if(enabled){

        if(isUpdated){
          
            //Nota: start_led cuenta los leds desde 1 mientras que fill() lo hace desde 0   
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led-1 , paint_pixels);
            pixels.show();
            isUpdated = false; // Se ha actualizado la tira led
        }
    }


  return paint_state;
}



// ============================== B L I N K   M O D E ================================= //


uint8_t blink_mode(uint8_t blink_id,
                   uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led,
                   uint16_t ms_on, uint16_t ms_off,
                   bool enabled) {

    static bool isOn = false;         //Indica si el el led debe estar encendido o apagado
    static uint16_t blink_pixels; // Numero de pixeles involucrados en realizar el efecto blink
    static uint8_t blink_state = 0;   //Estado del modo blink
    static elapsedMillis blink_time;  //Tiempo transcurrido entre un intervalo
    static bool isClear = true;       //Indica si hay que limpiar (apagar los leds) la zona donde ha trabajado el modo blink cuando ha terminado

    blink_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados

    //Nota: start_led cuenta los leds desde 1 mientras que fill() lo hace desde 0 

    if(enabled){

        isClear = false;

        if(blink_time > ms_off && !isOn){
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led-1, blink_pixels);
            pixels.show();
            blink_time = 0;
            isOn = true;          
        }

        if(blink_time > ms_on && isOn){
            pixels.fill(pixels.Color(0, 0, 0), start_led-1, blink_pixels);
            pixels.show();
            blink_time = 0;
            isOn = false;
        }

    }

    else if (!isClear){
        pixels.fill(pixels.Color(0, 0, 0), start_led-1, blink_pixels);
        pixels.show();
        isClear = true;
    }
    

  return blink_state;

}




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
}

void loop()
{

  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }
  
  //blink_mode(blink_id, blink_color_R,blink_color_G,blink_color_B, blink_start_led, blink_end_led, blink_ms_on, blink_ms_off, blink_enabled);
  //blink_mode(1, 20,0,0, 2, 4, 500, 500, true);
  //paint_mode(paint_id, paint_color_R,paint_color_G,paint_color_B, paint_start_led, paint_end_led, paint_enabled);
  //paint_mode(1, 20,0,0, 1, 5, true);
  //shift_mode(shift_id, shift_color_R,shift_color_G, shift_color_B, shift_start_led, shift_end_led, shift_direction, shift_speed, shift_sleep, shift_enabled); 
  
  //shift_mode   (1, 0,20,0, 11, 19, "left", 1000, 1000, true);
  //shift_mode_2 (1, 0,0,20, 1, 4, "right", 1000, 1000, true);


  
  
  //shift_mode   (1, 0,0,10, 1, 15, "right", 1000, 1000, true);
  //shift_mode_2   (1, 0,10,0, 16, 30, "left", 1000, 1000, true);
  //shift_mode_2 (1, 0,10,0, 7, 57, "left", 1000, 1000, true);

   /*
    led1.on();
    delay(1000);
    led1.off()
    delay(1000);
*/
   // effect_1.shift_mode(pixels, 1, 0,0,10, 1, 15, "right", 1000, 1000, true);
   // effect_2.shift_mode(pixels, 1, 0,10,0, 16, 30, "left", 1000, 1000, true);
   // effect_3.shift_mode(pixels, 1, 10,0,0, 31, 45, "right", 1000, 1000, true);



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




   
}
