#include <Arduino.h>
#include "IMUOrientation.h"

IMUOrientation imu;

void setup(){
  Serial.begin(115200);
  while(!Serial){}
  Serial.println("Starting IMU orientation demo...");
  if(!imu.begin()){
    Serial.println("MPU6050 not found!");
    while(true){ delay(1000); }
  }
  imu.calibrate();
  Serial.println("Calibration done.");
}

void loop(){
  imu.update();
  static unsigned long last=0; unsigned long now=millis();
  if(now - last >= 500){
    last = now;
    Serial.print("Roll="); Serial.print(imu.roll(),2);
    Serial.print(" Pitch="); Serial.print(imu.pitch(),2);
    Serial.print(" Yaw="); Serial.println(imu.yaw(),2);
  }
}
