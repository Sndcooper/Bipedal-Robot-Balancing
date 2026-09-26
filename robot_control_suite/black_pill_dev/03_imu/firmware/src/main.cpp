#include <Arduino.h>
#include <Wire.h>
extern HardwareSerial Serial1;
constexpr uint8_t MPU=0x68;
int16_t read16(){return (Wire.read()<<8)|Wire.read();}
void setup(){Serial1.begin(115200);Wire.begin();Wire.setClock(400000);Wire.beginTransmission(MPU);Wire.write(0x6B);Wire.write(0);Serial1.printf("IMU:WAKE=%d\n",Wire.endTransmission());}
void loop(){static uint32_t last=0;if(millis()-last<20)return;last=millis();Wire.beginTransmission(MPU);Wire.write(0x3B);Wire.endTransmission(false);if(Wire.requestFrom(MPU,(uint8_t)14)!=14){Serial1.println("IMU:READ_FAIL");return;}int16_t ax=read16(),ay=read16(),az=read16();read16();read16();int16_t gy=read16();float pitch=atan2f((float)ay,sqrtf((float)ax*ax+(float)az*az))*57.2958f;Serial1.printf("IMU:%.2f,%.2f\n",pitch,gy/131.0f);}
