#include <SoftwareSerial.h>

#include "RoombaSerialController.hpp"

SoftwareSerial device(10, 11);//RX ,TX.

RoombaSerialController roomba;

void setup(){
  device.begin(115200);

  roomba.setSerialInterface( &device );
  roomba.Start();
  roomba.DriveMotors( 64, -64);

}

void loop(){}