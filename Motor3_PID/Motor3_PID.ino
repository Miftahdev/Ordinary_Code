// Linear Velocity
int PWM;
#define RPWM1 5
#define LPWM1 6
#define ENCA1 2
#define ENCB1 3

// SET POINT
float set_point =  0.09344 ; //dalam m/s

// Error
float error;

//PID
float kp =  0.4137 ;
float ki = kp * 0.9 ;
float kd = kp * 0.00001 ;
float dt = 1;
float prevError;
float integral;
float derivative;
float PID;

// RPM
float rpm;
unsigned long currentMillis, previousMillis;

//PWM
float pwm1;

//ENCODER
long encoderPosition = 0;
volatile long previousEncoder = 0;
volatile long currentEncoder;

// MOTOR
const int ppr = 7; //define number of pulses in one round of encoder
const float GR = 1 / 19.2; //gear ratio
const float R = 0.538048496; //load (omniwheel) ratio
//RPM
float RPM;           //rotating speed in rad/s
float old_RPM = 0;
float new_RPM = 0;
//rad/s
float RPS;
float old_RPS = 0 ;
float new_RPS = 0;
//m/s
float Vel;
float old_Vel = 0;
float new_Vel = 0;
const int interval = 100; //choose interval is 0.1 second (100 milliseconds)

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
    RPM = (float)((((currentEncoder - previousEncoder) * 600) / ppr) * GR) ;
    RPS = (float)((RPM * 2 * 3.14) / 600); // konversi Rotasi per menit ke Radian per sekon dikali keliling roda dan dibagi waktu
    Vel = (float)(RPS * 0.05); // kecepatan linear adalah kecepatan sudut dikali dengan jari jari roda
    previousEncoder = currentEncoder;
    new_RPM = RPM;
    new_RPS = RPS;
    new_Vel = Vel;
    error = set_point - new_Vel;
    integral += error * dt;
    derivative = ((error - prevError)  / dt );
    PID = (kp * error) + (ki * integral) + (kd * derivative);
    prevError = error;
    PID = min(max(PID, 0), 255);
    pwm1 =  ((499.44 * PID) + 5.221);

    Serial.print(RPM);
    Serial.print(",");
    Serial.print(RPS);
    Serial.print(",");
    Serial.print(Vel);
    Serial.print(",");
    Serial.print(pwm1);
    Serial.print(",");
    Serial.print(error);
    Serial.println();
    old_RPM = new_RPM;
    old_RPS = new_RPS;
    old_Vel = new_Vel;
  }
}

void PWMmotor (int pwm1) {
  analogWrite(RPWM1, LOW);
  analogWrite(LPWM1, pwm1);
}
