#ifndef _ROS_H_
#define _ROS_H_


#include <ArduinoHardware.h>
#include <ros/node_handle.h>

namespace ros
{
  //  default is 25, 25, 512, 512  
  //  Publishers, Subscribers, bytes for input buffer, bytes for output buffer. 

  //Definicion del los nodos que dispondrá el programa. Si se emplea Teensy 3.2 se puede
  //reservar mucha mayor memoria. En el resto de placas, se limita
  #if defined(__MK20DX256__)
      typedef NodeHandle_<ArduinoHardware, 25, 25, 512, 512> NodeHandle;
  #else
      typedef NodeHandle_<ArduinoHardware, 5, 5 , 127, 127> NodeHandle;
  #endif 

}

/*
*   Arduino Leonardo/Micro y Teensy 3.2 tienen USB nativo, por lo que 
*   se ha de espeficiar El resto de placas no es necesario
*       - Con USB nativo se alcanzan velocidades teóricas de hasta 12 Mbit/s (modo full speed)
*       - Con USB por serie (mediante el chip FTDI232 o 16u2) solo se alcanzan velocidades de hasta 3 Mbit/s 
*/
#if defined(__AVR_ATmega32U4__) or defined(__MK20DX256__)  
  #define USE_USBCON 
#endif



#endif
