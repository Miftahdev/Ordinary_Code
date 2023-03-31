// UJI RPM MOTOR DENGAN SINYAL STEP
int PWM;
#define RPWM1 6
#define LPWM1 5
#define ENCA1 2
#define ENCB1 3

// SET POINT
int set_point = 175;

// RPM
float rpm;
unsigned long currentMillis, previousMillis;

//PWM
int pwm1;

//ENCODER
long encoderPosition = 0;
float old_rot_speed = 0;
float new_rot_speed = 0;
volatile long previousEncoder = 0;
volatile long currentEncoder;

// MOTOR
const int ppr = 7; //define number of pulses in one round of encoder
const float GR = 1 / 19.2; //gear ratio
const float R = 0.538048496; //load (omniwheel) ratio
float rot_speed;           //rotating speed in rad/s
const int interval = 1000; //choose interval is 1 second (1000 milliseconds)

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
  pwm1 =  ((0.2677 * set_point) + 5.4683);
  PWMmotor(pwm1);
}

void readEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    encoderPosition--;
  } else {
    encoderPosition++;
  }
}

void readSpeed() {
  currentEncoder = encoderPosition;
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    rot_speed = (float)((((currentEncoder - previousEncoder) * 60) / ppr) * GR) ;
    previousEncoder = currentEncoder;
    new_rot_speed = rot_speed;
    Serial.print(new_rot_speed);
    Serial.print(",");
    Serial.print(pwm1);
    Serial.println();
    old_rot_speed = new_rot_speed;
  }
}

void PWMmotor (int pwm1) {
  analogWrite(RPWM1, pwm1);
  analogWrite(LPWM1, LOW);
}
