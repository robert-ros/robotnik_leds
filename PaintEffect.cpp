


  #include "PaintEffect.h"



  PaintEffect::PaintEffect(Adafruit_NeoPixel &pixels) {

        this->pixels = &pixels;
      
  }

  void PaintEffect::assign_id(uint8_t id_assigned){
    
      this->id_assigned = id_assigned; 
  }


uint8_t PaintEffect::paint_mode(struct leds_paint paint_config){

     
      // Updated internal values 
      if(paint_config.id == id_assigned){

          color_R = paint_config.color_R;
          color_G = paint_config.color_G;
          color_B = paint_config.color_B;
          start_led = paint_config.start_led;
          end_led = paint_config.end_led;
          enabled = paint_config.enabled;
            
      }


    // Detecta si ha actualizado algun parámetro de la tira. Esto permite que el microcontrolador
    // no actualice constantemente la tira, ya que el valor es siempre el mismo sino han habido cambios

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

            isClear = false;
          
            //Nota: start_led cuenta los leds desde 1 mientras que fill() lo hace desde 0   
            pixels->fill(pixels->Color(color_R, color_G, color_B), start_led-1 , paint_pixels);
            pixels->show();
            isUpdated = false; // Se ha actualizado la tira led
        }
    }
    
    else if (!isClear){
        pixels->fill(pixels->Color(0, 0, 0), start_led-1, paint_pixels);
        pixels->show();
        isClear = true;
    }


  return paint_state;
}
