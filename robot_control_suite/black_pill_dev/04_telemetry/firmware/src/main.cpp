#include <Arduino.h>
extern HardwareSerial Serial1; extern HardwareSerial Serial6;
void both(const char* s){Serial1.print(s);Serial6.print(s);}
void setup(){Serial1.begin(115200);Serial6.begin(115200);both("TEL:READY\n");}
void poll(HardwareSerial &p){static String line;while(p.available()){char c=p.read();if(c!='\n'){line+=c;continue;}if(line=="PING") both("PONG\n");else both("ERR:use PING\n");line="";}}
void loop(){poll(Serial1);poll(Serial6);static uint32_t last=0;if(millis()-last>=100){last=millis();char b[48];snprintf(b,sizeof(b),"TEL:ms=%lu,free=%ld\n",(unsigned long)last,(long)(10000));both(b);}}
