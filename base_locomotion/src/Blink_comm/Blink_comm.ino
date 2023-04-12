/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

ros::NodeHandle  nh;

bool flag = 0;
bool next = 0;
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

void complete( const std_msgs::Bool& toggle_msg){
  next = 1;
}

ros::Subscriber<std_msgs::Bool> sub1("/completition_status", &complete );

ros::Subscriber<std_msgs::String> sub2("/mission_code", &messageCb );


std_msgs::Bool reached;
ros::Publisher pub("data", &reached);


void setup()
{ 
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pub);
}

void loop()
{  
  reached.data = code;
  pub.publish(&reached);
  delay(1000);
  nh.spinOnce();
}
