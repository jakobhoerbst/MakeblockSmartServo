/*
 * 
 * 
 * INFORMATION
 * 
 * install esp32
 * https://randomnerdtutorials.com/installing-the-esp32-board-in-arduino-ide-windows-instructions/
 * DOIT ESP32 DEVKIT V1
 * 
 * //wdt.h reset() noch ausgeklammert
 * 
 * Serial2 für ESP32 verwendet. wir im header festgelegt
 * 
 * List with useful Smart Servo functions in header file
 * 
 * 
  
*/

#include "MakeblockSmartServo.h"

MakeblockSmartServo smartservo;

void setup() {
  Serial.begin(115200);

  smartservo.beginserial();
  delay(5);
  smartservo.assignDevIdRequest();
  delay(50);
  
}

void loop() {

  smartservo.setRGBLed(1, 255, 255, 255);
  if(Serial.available()){
    Serial.println(Serial.read());
    
    }

  smartservo.setPwmMove(1, 100);

  delay(1000);

    smartservo.setPwmMove(1, 0);

    delay(1000);

}
