
#include <SoftwareSerial.h>

#include "RoombaSerialController.hpp"

//SoftwareSerial device(10, 11);//RX ,TX.
SoftwareSerial device( 2, 5);//RX ,TX. M5Stack.


RoombaSerialController roomba;

void setup(){
  device.begin(115200);

  roomba.setSerialInterface( &device );
  roomba.Start();
  roomba.DriveMotors( 64, -64);

}

void loop(){}