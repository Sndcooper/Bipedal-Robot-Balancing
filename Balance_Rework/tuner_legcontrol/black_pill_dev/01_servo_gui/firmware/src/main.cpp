#include <Arduino.h>
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

static void drain() { while (Serial2.available()) Serial2.read(); }
static void write8(uint8_t id, uint8_t address, uint8_t value) {
  uint8_t p[] = {0xFF,0xFF,id,4,3,address,value,(uint8_t)~(id+4+3+address+value)};
  Serial2.write(p, sizeof(p)); Serial2.flush(); drain();
}
static void write16(uint8_t id, uint8_t address, uint16_t value) {
  uint8_t lo=value, hi=value>>8;
  uint8_t p[] = {0xFF,0xFF,id,5,3,address,lo,hi,(uint8_t)~(id+5+3+address+lo+hi)};
  Serial2.write(p, sizeof(p)); Serial2.flush(); drain();
}
static void configure(uint8_t id, bool on) {
  write16(id,34,1023); write8(id,26,1); write8(id,27,1);
  write8(id,28,4); write8(id,29,4); write8(id,24,on ? 1 : 0);
}
void setup() { Serial1.begin(115200); Serial2.begin(1000000); Serial1.println("SERVO:READY"); }
void loop() {
  static String line;
  while (Serial1.available()) {
    char c=Serial1.read(); if (c=='\r') continue;
    if (c!='\n') { line+=c; continue; }
    int id=0, value=0;
    if (sscanf(line.c_str(), "TQ %d %d", &id,&value)==2) { configure(id,value!=0); Serial1.println("ACK:TQ"); }
    else if (sscanf(line.c_str(), "P %d %d", &id,&value)==2 && value>=0 && value<=1023) { write16(id,30,value); Serial1.println("ACK:P"); }
    else if (sscanf(line.c_str(), "ALL %d", &value)==1 && value>=0 && value<=1023) { for(uint8_t id: {0,1,6,14}) write16(id,30,value); Serial1.println("ACK:ALL"); }
    else Serial1.println("ERR: use TQ <id> <0|1>, P <id> <0..1023>, ALL <0..1023>");
    line="";
  }
}
