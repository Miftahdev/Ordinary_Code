// UJI RPM MOTOR DENGAN SINYAL STEP
int PWM;
#define RPWM1 5
#define LPWM1 6
#define ENCA1 2
#define ENCB1 3
long encoderPosition = 0;

float rpm;
unsigned long currentMillis, previousMillis;

float old_rot_speed = 0;
float new_rot_speed = 0;
volatile long previousEncoder = 0;
volatile long currentEncoder;
int pwm1 = 50;
int data = 0;

const int ppr = 7; //define number of pulses in one round of encoder
const float GR = 1 / 19.2; //gear ratio
const float R = 0.538048496; //load (omniwheel) ratio

float rot_speed;           //rotating speed in rad/s
int interval = 60; //choose interval is 1 second (1000 milliseconds)

void setup() {
  pinMode (ENCA1, INPUT);
  pinMode (ENCB1, INPUT);

  pinMode(RPWM1, OUTPUT);
  pinMode(LPWM1, OUTPUT);

  digitalWrite(ENCA1, HIGH);
  digitalWrite(ENCB1, HIGH);


  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(ENCB1), readEncoder, CHANGE);
  Serial.begin(9600);
}

void loop() {
  while (data < 200) {
    currentMillis = millis();
    currentEncoder = encoderPosition;
    if (currentMillis - previousMillis > interval) {
      rot_speed = (float)((((currentEncoder - previousEncoder) * 60) / ppr) * GR) ;
      previousMillis = currentMillis;
      previousEncoder = currentEncoder;
      new_rot_speed = rot_speed;
      old_rot_speed = new_rot_speed;
      Serial.print(new_rot_speed);
      Serial.print(",");
      pwm1 += 5;
      analogWrite(RPWM1, LOW);
      analogWrite(LPWM1, pwm1);
      Serial.println(pwm1);
      data++;
    }
  }
  analogWrite(RPWM1, LOW);
  analogWrite(LPWM1, LOW);
}


void readEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    encoderPosition--;
  } else {
    encoderPosition++;
  }
}
