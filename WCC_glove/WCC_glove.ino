#include <SoftwareSerial.h>

#define fin0 0
#define fin1 1
#define fin2 2
#define fin3 3
#define fin4 4      //flex sensor (analog)

int val[5];  //save flex sesor val

void setup() {
  Serial.begin(4800);   //lillypad Serial communication
  delay(2000);
}

void loop() {
  readFlexVal();
  checkBent();
  delay(250);
}


void readFlexVal(){
  val[0] = analogRead(fin0);
  val[1] = analogRead(fin1);
  val[2] = analogRead(fin2);
  val[3] = analogRead(fin3);
  val[4] = analogRead(fin4);
}

void checkBent(){
  for(int i = 0; i < 5; i++){
    if(val[i] > 760){ 
      Serial.println(i);  //0,1,2,3,4 == bent
    }else{
      Serial.println(i+5);  //5,6,7,8,9 == not bent
    }
  }
}


