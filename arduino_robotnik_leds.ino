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
 *  Cambiar onoff_mode por paint_mode
 *  Añadir "id" a todos los modos
 *  Añadir todas las funciones a shift y:
 *        - Polarity se elimina  
 *        - Añadir un tiempo que permanece apagado de una secuencia a otra
 *         - shift_mode tendra dos tipos  
 *         - direction sera un string con "left" "right"
 *  Las funciones paint_mode y blink_mode deben actuar sobre led(2,5) en leds 2,3,4 y 5
 *  
 *  // cyclon_mode (led de un punto a otro)
 *
 */

#include <Adafruit_NeoPixel.h>
#include "ros.h"
#include <std_msgs/String.h>
#include <robotnik_leds/LedsNormal.h>
#include <robotnik_leds/LedsBlink.h>
#include <robotnik_leds/LedsShift.h>

ros::NodeHandle  nh;
using robotnik_leds::LedsNormal;
using robotnik_leds::LedsBlink;
using robotnik_leds::LedsShift;



/* Variables globales para el modo OnOff, se llamará PAINT
(provisional, implementar un HandleMode)*/
uint8_t  onoff_color_R = 0;
uint8_t  onoff_color_G = 0;
uint8_t  onoff_color_B = 0;
uint16_t onoff_start_led = 0;
uint16_t onoff_end_led = 0;
bool     onoff_enabled = false;


/* Variables globales para el modo Blink (provisional, implementar un HandleMode) */
uint8_t  blink_color_R = 0;
uint8_t  blink_color_G = 0;
uint8_t  blink_color_B = 0;
uint16_t blink_start_led = 0;
uint16_t blink_end_led = 0;
uint16_t blink_ms_on = 0;
uint16_t blink_ms_off = 0;
bool     blink_enabled = false;


/* Variables globales para el modo Shift (provisional, implementar un HandleMode) */
uint8_t  shift_color_R = 0;
uint8_t  shift_color_G = 0;
uint8_t  shift_color_B = 0;
uint16_t shift_start_led = 0;
uint16_t shift_end_led = 0;
bool     shift_direction = false;
bool     shift_polarity = false;
uint16_t shift_speed = 0;
bool     shift_enabled = false;




/* Tira led */
#define PIN        6
#define NUMPIXELS  130
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);



int i;

elapsedMillis timeout_system;




void callback_onoff(const LedsNormal::Request & req, LedsNormal::Response & res){

  onoff_color_R = req.color_R;
  onoff_color_G = req.color_G;
  onoff_color_B = req.color_B;
  onoff_start_led = req.start_led;
  onoff_end_led = req.end_led;
  onoff_enabled = req.enabled;
  
}


void callback_blink(const LedsBlink::Request & req, LedsBlink::Response & res){

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

  shift_color_R = req.color_R;
  shift_color_G = req.color_G;
  shift_color_B = req.color_B;
  shift_start_led = req.start_led;
  shift_end_led = req.end_led;
  shift_direction = req.direction;
  shift_polarity = req.polarity;
  shift_speed = req.speed;
  shift_enabled = req.enabled;

}


ros::ServiceServer<LedsNormal::Request, LedsNormal::Response> server_onoff_mode("robotnik_leds/set_leds/onoff_mode",&callback_onoff);
ros::ServiceServer<LedsBlink::Request, LedsBlink::Response> server_blink_mode("robotnik_leds/set_leds/blink_mode",&callback_blink);
ros::ServiceServer<LedsShift::Request, LedsShift::Response> server_shift_mode("robotnik_leds/set_leds/shift_mode",&callback_shift);



// ============================== ON & OFF   M O D E ================================= //


uint8_t onoff_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led, 
                   bool enabled){

    static uint8_t onoff_state = 0;
    static bool isUpdated = false;
    

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


    if(enabled){

        if(isUpdated){
            
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led, end_led);
            pixels.show();
            isUpdated = false; // Se ha actualizado la tira led
        }
    }


  return onoff_state;
}



// ============================== B L I N K   M O D E ================================= //


uint8_t blink_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led,
                   uint16_t ms_on, uint16_t ms_off,
                   bool enabled) {

    static bool isOn = false;         //Indica si el el led debe estar encendido o apagado
    static uint8_t blink_state = 0;   //Estado del modo blink
    static elapsedMillis blink_time;  //Tiempo transcurrido entre un intervalo
    static bool isClear = true;       //Indica si hay que limpiar (apagar los leds) la zona donde ha trabajado el modo blink cuando ha terminado

    if(enabled){

        isClear = false;

        if(blink_time > ms_off && !isOn){
            pixels.fill(pixels.Color(color_R, color_G, color_B), start_led, end_led);
            pixels.show();
            blink_time = 0;
            isOn = true;          
        }

        if(blink_time > ms_on && isOn){
            pixels.fill(pixels.Color(0, 0, 0), start_led, end_led);
            pixels.show();
            blink_time = 0;
            isOn = false;
        }

    }

    else if (!isClear){
        pixels.fill(pixels.Color(0, 0, 0), start_led, end_led);
        pixels.show();
        isClear = true;
    }
    

  return blink_state;

}

// ============================== S H I F T    M O D E ================================= //


uint8_t shift_mode(uint8_t color_R, uint8_t color_G, uint8_t color_B,
                   uint16_t start_led, uint16_t end_led,
                   bool direction, bool polarity,
                   uint16_t speed,
                   bool enabled) {

    static elapsedMillis shift_time;
    static int count_pixel = start_led; //Momentaneo.
    static uint8_t shift_state = 0;
    static uint16_t shift_pixels; // Numero de pixeles involucrados en realizar el efecto shift
    static bool isClear = true;   //Indica si hay que limpiar la zona (apagar los leds) donde ha trabajado el modo shift cuando ha terminado
    static uint16_t last_start_led = 0, last_end_led, last_shift_pixels = 0;

    shift_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados


    // Detecta si ha actualizado el led de inicio o el led de fin
    
    if(last_start_led != start_led || last_end_led != end_led){

        //Limpia la zona de trabajo antigua para poder funcionar con la nueva zona
        pixels.fill(pixels.Color(0, 0, 0), last_start_led-1, last_shift_pixels);
        pixels.show();

        last_start_led = start_led;
        last_end_led = end_led;
        last_shift_pixels = shift_pixels;
        
    }


    if(enabled){

        isClear = false;

        //Realiza el efecto shift
        if(shift_time > speed/shift_pixels){
          
            //Si se ha alcanzado el led final, se limpia la zona de trabajo y se reinicia el contador
            //En caso contrario, realiza el shift
            if(count_pixel > end_led){
        
                //Nota: start_led cuenta los leds desde 1 mientras que setPixelColor lo hace desde 0 
                pixels.fill(pixels.Color(0, 0, 0), start_led-1, shift_pixels);  
                pixels.show();
                count_pixel = start_led; 
                shift_time = 0;

            }
            else {
              
                //Nota: count_pixel cuenta los leds de 1 a N_leds mientras que setPixelColor lo hace de 0 a N_leds-1 
                if(direction)
                    pixels.setPixelColor(count_pixel-1, pixels.Color(color_R, color_G, color_B));
                else
                    pixels.setPixelColor( (shift_pixels+start_led) - 1 - (count_pixel-start_led) - 1 , pixels.Color(color_R, color_G, color_B));
                
                pixels.show();
                count_pixel++;
                shift_time = 0;
            }
        }
    }

    else if (!isClear){
        pixels.fill(pixels.Color(0, 0, 0), start_led-1, shift_pixels);
        pixels.show();
        isClear = true;
    }



  return shift_state;               
}



void setup()
{

  #if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  // Arduino Leonardo/Micro, Teensy 3.2
    nh.getHardware()->setBaud(2000000); 
 
  #elif defined(__AVR_ATmega328P__)  // Arduino UNO/Nano
    nh.getHardware()->setBaud(57600);
  #endif   

  nh.initNode();
  nh.advertiseService(server_onoff_mode);
  nh.advertiseService(server_blink_mode);
  nh.advertiseService(server_shift_mode);
  pixels.begin();
  pixels.clear();
  pixels.show();

  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);

  //Serial.println(2000000);
}

void loop()
{

  if(timeout_system > 10){
      timeout_system = 0;
      nh.spinOnce();
  }

  //El primer led será el led 1 (no el led 0). Los leds con incluyentes, si se indica led (2,4) serán
  // los leds 2, 3 y 4
  
  //blink_mode(blink_color_R,blink_color_G,blink_color_B, blink_start_led, blink_end_led, blink_ms_on, blink_ms_off, blink_enabled);
  //blink_mode(20,0,0, 1, 20, 1000, 1000, true);
  //onoff_mode(onoff_color_R,onoff_color_G,onoff_color_B, onoff_start_led, onoff_end_led, onoff_enabled);
  //shift_mode(shift_color_R,shift_color_G, shift_color_B, shift_start_led, shift_end_led, shift_direction, shift_polarity, shift_speed, shift_enabled); 
  //shift_mode (20,0,0, 1, 80, false, 0, 1000, true);



}
