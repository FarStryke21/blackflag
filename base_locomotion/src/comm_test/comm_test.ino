#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

void stringCallback(const std_msgs::Float32& msg)
{
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}
ros::Subscriber<std_msgs::Float32> sub("cmd_input", stringCallback);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  // put your main code here, to run repeatedly:
  nh.spinOnce();
}
