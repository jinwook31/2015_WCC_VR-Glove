const int analogInPin = A0;  // analog input
const int pin_A = 2; // adress pins for MUX
const int pin_B = 3;
const int pin_C = 4;

int sens0 = 0; 
int sens2 = 0;
int sens3 = 0;
int sens5 = 0;
int mid = 0;
      
void setup() {
  //Serial.begin(9600);
  Serial.begin(4800);
  pinMode(pin_A, OUTPUT);
  pinMode(pin_B, OUTPUT);
  pinMode(pin_C, OUTPUT);
}

void loop() {
  
  digitalWrite(pin_A, LOW); digitalWrite(pin_B, LOW); digitalWrite(pin_C, LOW);
  sens0 = analogRead(analogInPin);
  //0
  
  digitalWrite(pin_A, LOW); digitalWrite(pin_B, HIGH); digitalWrite(pin_C, LOW);
  sens2 = analogRead(analogInPin);
  //2
    
  digitalWrite(pin_A, HIGH); digitalWrite(pin_B, HIGH); digitalWrite(pin_C, LOW);
  sens3 = analogRead(analogInPin);
  //3
    
  digitalWrite(pin_A, HIGH); digitalWrite(pin_B, LOW); digitalWrite(pin_C, HIGH);
  sens5 = analogRead(analogInPin);
  //5
 
  Serial.print("   1 : " );                       
  Serial.print(sens3);
  
  Serial.print("   2 : " );                       
  Serial.print(sens0); 
  
  Serial.print("   3 : " );
  int mid = analogRead(A1);  
  Serial.print(mid);
  
  Serial.print("   4 : " );                       
  Serial.print(sens2);
  
  Serial.print("   5 : " );                       
  Serial.println(sens5); 
  
  delay(300);
}

