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
  readSpeed();
  PWMmotor(50);
}

void readEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    encoderPosition--;
  } else {
    encoderPosition++;
  }
}

void readSpeed() {
  //read velocity of selected motor
  //return velocity in rad/s
  const int ppr = 7; //define number of pulses in one round of encoder
  const float GR = 1 / 19.2; //gear ratio
  const float R = 0.538048496; //load (omniwheel) ratio
  currentEncoder = encoderPosition;

  float rot_speed;           //rotating speed in rad/s
  const int interval = 1000; //choose interval is 1 second (1000 milliseconds)
  currentMillis = millis();

  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    // rot_speed = (float)(((((currentEncoder - previousEncoder) * 60) / ppr) * GR) * R);
    rot_speed = (float)((((currentEncoder - previousEncoder) * 60) / ppr) * GR) ;
    //rot_speed = (float)((currentEncoder - previousEncoder) / (ppr));
    //rot_speed = rot_speed * 9.54929658551; // konversi; rpm 1 rad/s = 9.54929658551 rpm
    previousEncoder = currentEncoder;
    new_rot_speed = rot_speed;
    //if (new_rot_speed != old_rot_speed) {
    //Serial.print("Speed = ");
    Serial.print(new_rot_speed);
    Serial.println();
    old_rot_speed = new_rot_speed;
    //}
  }
}

void PWMmotor (int pwm1) {
  analogWrite(RPWM1, LOW);
  analogWrite(LPWM1, pwm1);
}
