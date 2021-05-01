
#include <SoftwareSerial.h>

#include "RoombaSerialController.hpp"

#ifdef ARDUINO_M5Stack_Core_ESP32
  SoftwareSerial device( 2, 5);//RX ,TX. M5Stack.
#else
  SoftwareSerial device(10, 11);//RX ,TX.
#endif


RoombaSerialController roomba;

void setup(){
  device.begin(115200);

  roomba.setSerialInterface( &device );
  roomba.Start();
  roomba.DriveMotors( 64, -64);

}

void loop(){}