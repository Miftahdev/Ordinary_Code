// ROS

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>

//Encoder
#define ENCA1 2
#define ENCB1 3

ros::NodeHandle nh;
std_msgs::Int64 encoder;
ros::Publisher encoderValue("encoderValue", &encoder);


int pulse = 0;

void setup() {
  //  Serial.begin(9600);
  nh.initNode();
  nh.advertise(encoderValue);
  pinMode(ENCA1, INPUT_PULLUP);
  pinMode(ENCB1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder, RISING);
}

void loop() {
  //  Serial.print("PWM = ");
  //  Serial.println(pulse);
  nh.loginfo("Encoder Value");
  encoder.data = pulse;
  encoderValue.publish(&encoder);

  nh.spinOnce();
  delay(100);

}
void readEncoder() {
  if (digitalRead(ENCA1) == digitalRead(ENCB1)) {
    pulse++;
  } else {
    pulse--;
  }
  //  if (digitalRead(ENCA1) != digitalRead(ENCB1)){
  //    pulse--;
  //  }
}
