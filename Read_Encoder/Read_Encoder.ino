#define ENCA1 2
#define ENCB1 3

int pulse = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1),readEncoder, RISING);
}

void loop() {
  Serial.print("PWM = ");
  Serial.println(pulse);
}
void readEncoder(){
  if (digitalRead(ENCA1) == digitalRead(ENCB1)){
    pulse++;
  } else {
    pulse--; 
  }
//  if (digitalRead(ENCA1) != digitalRead(ENCB1)){
//    pulse--;
//  }
}
