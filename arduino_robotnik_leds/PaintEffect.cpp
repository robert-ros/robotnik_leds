


  #include "PaintEffect.h"


  PaintEffect::PaintEffect(Adafruit_NeoPixel &pixels):
        CommonEffect(pixels){}



  void PaintEffect::assign_id(String id_assigned){
    
      this->id_assigned = id_assigned; 
  }




uint8_t PaintEffect::paint_mode(struct leds_paint paint_config){

      

      // Updated internal values 
      if(id_assigned.equals(paint_config.id)){

          enabled = paint_config.enabled;

          if(enabled){
            color_R = paint_config.color_R;
            color_G = paint_config.color_G;
            color_B = paint_config.color_B;
            start_led = paint_config.start_led;
            end_led = paint_config.end_led;
          }
            
      }

    
    

    // Detecta si ha actualizado algun parámetro de la tira. Esto permite que el microcontrolador
    // no actualice constantemente la tira, ya que el valor es siempre el mismo sino han habido cambios

    //Detecta si algun parametro se ha actualizado
    if(last_color_R != color_R || last_color_G != color_G || last_color_B != color_B ||
       last_start_led != start_led || last_end_led != end_led || last_enabled != enabled){


        //Si se detecta que la zona de trabajo ha cambiado, se borra la antigua zona
        if(last_start_led != start_led || last_end_led != end_led){
          
          //pixels->fill(pixels->Color(0, 0, 0), last_start_led-1, paint_pixels);
          //pixels->show();
          fillPixels(0,0,0, last_start_led-1, paint_pixels);
          showPixels();
          
        }

        //Updated values
        isUpdated = true;
        last_color_R = color_R;
        last_color_G = color_G;
        last_color_B = color_B;
        last_start_led = start_led;
        last_end_led = end_led;
        last_enabled = enabled;
    }


    paint_pixels = end_led - start_led + 1; // Por ejemplo start: 4, end: 2 Hay 4-2+1 = 3 pixeles involucrados
    
    if(enabled){

        if(isUpdated){

            isClear = false;
            //Nota: start_led cuenta los leds desde 1 mientras que fill() lo hace desde 0   
            //pixels->fill(pixels->Color(color_R, color_G, color_B), start_led-1 , paint_pixels);
            //pixels->show();
            fillPixels(color_R,color_G, color_B,  start_led-1, paint_pixels);
            showPixels();
            isUpdated = false; // Se ha actualizado la tira led
        }
    }
    
    else if (!isClear){
        //pixels->fill(pixels->Color(0, 0, 0), start_led-1, paint_pixels);
        //pixels->show();
        fillPixels(0,0,0,   start_led-1, paint_pixels);
        showPixels();
        isClear = true;
    }




  return paint_state;
}
