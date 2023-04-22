/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

bool flag = 0;
char code[] = "00000000";

void messageCb( const std_msgs::String& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
  if(flag == 0)
  {
    flag = 1;
    strcpy(code,toggle_msg.data);
    Serial.println(code);
  }
}

ros::Subscriber<std_msgs::String> sub("mission_code", &messageCb );
std_msgs::String str_msg;
ros::Publisher pub("data", &str_msg);
void setup()
{ 
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);
}

void loop()
{  
  str_msg.data = code;
  pub.publish(&str_msg);
  delay(1000);
  nh.spinOnce();
}
