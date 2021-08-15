#include <SoftwareSerial.h>;
SoftwareSerial device(10, 11);

void setup()
{
  device.begin(115200);
  Serial.begin(115200);

  // led(true);  
  byte buffer[] = {
    byte(128), // Start
    byte(135)  // Clean
  };
  device.write(buffer, 2);  
}

void loop()
{
}
