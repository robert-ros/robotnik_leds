
  #include "ShiftEffect.h"



  ShiftEffect::ShiftEffect(Adafruit_NeoPixel &pixels) {

        this->pixels = &pixels;
      
  }


  uint8_t ShiftEffect::shift_mode(struct leds_shift shift_config) {


      uint8_t  id = shift_config.id;
      uint8_t  color_R = shift_config.color_R;
      uint8_t  color_G = shift_config.color_G;
      uint8_t  color_B = shift_config.color_B;
      uint16_t start_led = shift_config.start_led;
      uint16_t end_led = shift_config.end_led;
      String   direction = shift_config.direction;
      uint16_t speed = shift_config.speed;
      uint16_t sleep = shift_config.sleep;
      bool     enabled = shift_config.enabled;

  
      shift_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados
  
  
      // Detecta si ha actualizado el led de inicio o el led de fin
      
      if(last_start_led != start_led || last_end_led != end_led){
  
          // Actualiza el led desde donde comenzará a contar count_pixel
          count_pixel = start_led;
  
          //Limpia la zona de trabajo antigua para poder funcionar con la nueva zona
          pixels->fill(pixels->Color(0, 0, 0), last_start_led-1, last_shift_pixels);
          pixels->show();
  
          last_start_led = start_led;
          last_end_led = end_led;
          last_shift_pixels = shift_pixels;
          
      }
  
  
      if(enabled){
  
          isClear = false;
  
          float shift_time_ms = shift_time/1000.0;
          float speed_per_pixel = float(speed)/float(shift_pixels+1);
          static float shift_time_compensation=0;
          
          //Realiza el efecto shift
          if(shift_time_ms >= (speed_per_pixel+shift_time_compensation)){
  
            shift_time = 0;
  
            shift_time_compensation = float(speed_per_pixel) - shift_time_ms + shift_time_compensation;
  
  
                  //Serial.println(speed_per_pixel);
                  //shift_toc = micros()/1000.0;
                  //Serial.println(shift_toc - shift_tic);
                  //shift_tic = micros()/1000.0;
            
              //Si se ha alcanzado el led final, se limpia la zona de trabajo y se reinicia el contador
              //En caso contrario, realiza el shift
              if(count_pixel > end_led){
                  
                  //Nota: start_led cuenta los leds desde 1 mientras que setPixelColor lo hace desde 0 
                  pixels->fill(pixels->Color(0, 0, 0), start_led-1, shift_pixels);  
                  pixels->show();
                  count_pixel = start_led; 
  
                  //shift_toc = micros()/1000.0;
                  //Serial.print("Resutado_2: ");
                  //Serial.println(shift_toc - shift_tic);
                  //shift_tic = micros()/1000.0;
                  
              }
              else {
                
                  //Nota: count_pixel cuenta los leds de 1 a N_leds mientras que setPixelColor lo hace de 0 a N_leds-1 
                  
                  if(direction.equals("right"))
                      
                      pixels->setPixelColor(count_pixel-1, pixels->Color(color_R, color_G, color_B));
                  
                  else if (direction.equals("left"))
                      
                      pixels->setPixelColor( (shift_pixels+start_led) - 1 - (count_pixel-start_led) - 1 , pixels->Color(color_R, color_G, color_B));
  
                  
                  pixels->show();
                  count_pixel++;         
              }
          }
      }
  
      else if (!isClear){
          pixels->fill(pixels->Color(0, 0, 0), start_led-1, shift_pixels);
          pixels->show();
          isClear = true;
      }
  
  
  
    return shift_state;               
  }

  
