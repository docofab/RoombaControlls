/*
 * 参考ページ：
 * https://qiita.com/legokichi/items/36a13e68722c51c72927
 * https://www.media.lab.uec.ac.jp/?page_id=697
 */

#include <SoftwareSerial.h>;
SoftwareSerial device(10, 11);

void setup()
{
  device.begin(115200);
  Serial.begin(115200);

  led(true);  
}

void loop()
{
  delay(100);
  sensorButton();
  sensorBumper();
  
  /*
   * Query List is Not Working???
   * 
  delay(100);
  byte buffer[] = {  
    byte(149),
    byte(132), // FULL    
    byte(1),     
    byte(7)
  };
  device.write(buffer, sizeof(buffer));
  */
  
  // motor(64, -64);
  // delay(250);
  // motor(-64, 64);
  // delay(250);
}

void sensorButton()
{
  /*
   *  Sensors: Packet [142] [Packet ID]
   */
  // ボタンの値を受信
  device.write(byte(142));
  device.write(18);
  
  int buttonId = device.read();
  Serial.println(buttonId);

  if (buttonId == 2)
  {
    led(false);
  }
  if (buttonId == 4)
  {
    led(true);
  }
}

void sensorBumper()
{
  // バンパーの値を受信
  device.write(byte(142));
  device.write(7);
  int bumperId = device.read();
  Serial.println(bumperId);
}

void led(bool isOn)
{
  int bits = isOn ? 7 : 0;
  int color = 0;  // 0: green / 255: red
  int intensity = isOn ? 255 : 0;

  byte buffer[] = {  
    byte(128),
    byte(132), // FULL
    byte(139),
    byte(bits),
    byte(color),
    byte(intensity)        
  };
  device.write(buffer, sizeof(buffer));
}

void motor(int l, int r){
  byte buffer[] = {
    byte(128), // Start
    byte(132), // FULL
    byte(146), // Drive PWM
    byte(r>>8),
    byte(r),
    byte(l>>8),
    byte(l)
  };
  device.write(buffer, 7);
}
