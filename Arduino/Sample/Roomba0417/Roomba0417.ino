/*
 * 参考ページ：
 * https://qiita.com/legokichi/items/36a13e68722c51c72927
 * https://www.media.lab.uec.ac.jp/?page_id=697
 */

#include <SoftwareSerial.h>;
SoftwareSerial device(10, 11);

void setup(){
  device.begin(115200);
  Serial.begin(115200);

  byte buffer[] = {
    byte(128), // Start
    byte(135)  // Clean
  };
  device.write(buffer, 2);
}

void loop(){
  delay(250);
  
  device.write(byte(142));
  device.write(43);

  //encoderの値を受信
  int enco_left_h = device.read();//high
  int enco_left_l = device.read();//low 
  // enco_left = hex_convert_to16(enco_left_h,enco_left_l); 
  Serial.println(enco_left_h);
  Serial.println(enco_left_l);
  
  // motor(64, -64);
  // delay(250);
  // motor(-64, 64);
  // delay(250);
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
