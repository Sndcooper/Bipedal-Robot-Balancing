#include <Arduino.h>
extern HardwareSerial Serial1;
constexpr uint32_t ENA=PA1, IN1=PB14, IN2=PB15, ENB=PA0, IN3=PB12, IN4=PB13;
constexpr uint32_t LA=PA6, LB=PA7, RA=PB0, RB=PB1;
volatile long leftCount=0,rightCount=0;
void leftISR(){ leftCount += digitalRead(LB) ? -1 : 1; }
void rightISR(){ rightCount += digitalRead(RB) ? -1 : 1; }
void drive(uint32_t en,uint32_t a,uint32_t b,int v){
  v=constrain(v,-255,255); digitalWrite(a,v>=0); digitalWrite(b,v<0); analogWrite(en,abs(v));
}
void setup(){
  Serial1.begin(115200); analogWriteResolution(8);
  for(uint32_t p:{ENA,IN1,IN2,ENB,IN3,IN4}) pinMode(p,OUTPUT);
  pinMode(LA,INPUT_PULLUP);pinMode(LB,INPUT_PULLUP);pinMode(RA,INPUT_PULLUP);pinMode(RB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LA),leftISR,RISING); attachInterrupt(digitalPinToInterrupt(RA),rightISR,RISING);
  drive(ENA,IN1,IN2,0);drive(ENB,IN3,IN4,0); Serial1.println("MOTOR:READY; M <left> <right>, STOP, ZERO");
}
void loop(){
 static uint32_t last=0; static String s;
 while(Serial1.available()){char c=Serial1.read();if(c!='\n'){s+=c;continue;}int l,r;
  if(sscanf(s.c_str(),"M %d %d",&l,&r)==2){drive(ENA,IN1,IN2,l);drive(ENB,IN3,IN4,r);}
  else if(s=="STOP"){drive(ENA,IN1,IN2,0);drive(ENB,IN3,IN4,0);} else if(s=="ZERO"){noInterrupts();leftCount=rightCount=0;interrupts();} s="";}
 if(millis()-last>=100){last=millis();noInterrupts();long l=leftCount,r=rightCount;interrupts();Serial1.printf("ENC:%ld,%ld\n",l,r);}
}
