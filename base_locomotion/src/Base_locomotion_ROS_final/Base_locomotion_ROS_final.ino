#include <util/atomic.h>
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>

// ROS Flags
char code[8];
bool flag = 0;
int next = 0;
///int command='correct';

ros::NodeHandle nh;

void messageCb( const std_msgs::String& toggle_msg){
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
ros::Subscriber<std_msgs::Bool> sub1("/completion_status", &complete );
ros::Subscriber<std_msgs::String> sub2("/mission_code", &messageCb );
std_msgs::Int32 reached;
ros::Publisher pub("/reached_status", &reached);
std_msgs::Int32 counter;
ros::Publisher pub1("/counter", &counter);
// A class to compute the control signal
class SimplePID{
  public:
    float kp, kd, ki, umax; // Parameters
    float eprev, eintegral; // Storage

  public:
  // Constructor
  SimplePID() : kp(1), kd(0), ki(0), umax(255), eprev(0.0), eintegral(0.0){}

  // A function to set the parameters
  void setParams(float kpIn, float kdIn, float kiIn, float umaxIn){
    kp = kpIn; kd = kdIn; ki = kiIn; umax = umaxIn;
  }

  // A function to compute the control signal
  void evalu(int value, int target, float deltaT, int &pwr, int &dir){
    // error
    int e = target - value;
    // derivative
    float dedt = (e-eprev)/(deltaT);
    // integral
    eintegral = eintegral + e*deltaT;
    // control signal
    float u = kp*e + kd*dedt + ki*eintegral;
    // motor power
    pwr = (int) fabs(u);
    if( pwr > umax ){
      pwr = umax;
    }
    // motor direction
    dir = 1;
    if(u<0){
      dir = -1;
    }
    // store previous error
    eprev = e;
  }
};

// How many motors
#define NMOTORS 4

// Pins
const int enca[] = {19,20,3,2};
const int encb[] = {18,21,52,7};
const int pwm[] = {8,44,46,4};
const int in2[] = {9,40,48,5};
const int in1[] = {10,42,50,6};
int bumper=0;
int counter_zero=0;
int instructions[]={1,0,1,0,0,0,1,1};

// Globals
long prevT = 0;
volatile int posi[] = {0,0,0,0};
int i=0;
int reach=0;
int allowed_err=100;
int stuck=0;
int gonext=0;

// PID class instances
SimplePID pid[NMOTORS];
int target[NMOTORS];
int save = 0;//To save the rostopic


void setup() {
  Serial.begin(57600);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.advertise(pub); 
  nh.advertise(pub1);   

  for(int k = 0; k < NMOTORS; k++){
    pinMode(enca[k],INPUT);
    pinMode(encb[k],INPUT);
    pinMode(pwm[k],OUTPUT);
    pinMode(in1[k],OUTPUT);
    pinMode(in2[k],OUTPUT);

    pid[k].setParams(0.45,0.1,0,200);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]),readEncoder<3>,RISING);
}



void loop() 
{
    // while(Serial.available()==0){
    // }
    counter.data = i+1;
    pub1.publish(&counter);
    
    if(flag == 1 && save == 0)
    {
      for(int k= 0; k<8; k++)
      {
        instructions[k]=code[k] - '0';
      }
      save = 1;
    } 
    if(flag == 1)
  {  
  

    
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;
  
    // Read the position in an atomic block to avoid a potential misread
    int pos[NMOTORS];
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      for(int k = 0; k < NMOTORS; k++){
        pos[k] = posi[k];

      }
    }

    // adjusting the position after reaching the panel
    Serial.print("i :");
    Serial.println(i);  
  if(stuck==1 && next==1) {
    stuck=0;
    next=0;
    // i=i+1;
    // Serial.println("Inside Stuck");
    // gonext=1;
  }


    if(gonext==1 || stuck!=1 && pid[0].eprev<allowed_err && pid[1].eprev<allowed_err && pid[2].eprev<allowed_err && pid[3].eprev<allowed_err && pid[0].eprev>-allowed_err && pid[1].eprev>-allowed_err && pid[2].eprev>-allowed_err && pid[3].eprev>-allowed_err && i<=7){

    {
    gonext=0;
    if(bumper==1){
      bump(500);
      }
    else{
      waitormove(pos);
      }
    }
    }


    // loop through the motors
    for(int k = 0; k < NMOTORS; k++)
    {
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      
      // signal the motor
      if (i==8 || stuck==1)
      {
          digitalWrite(in1[0],LOW);
          digitalWrite(in2[0],LOW);
          digitalWrite(in1[1],LOW);
          digitalWrite(in2[1],LOW);
          digitalWrite(in1[2],LOW);
          digitalWrite(in2[2],LOW);
          digitalWrite(in1[3],LOW);
          digitalWrite(in2[3],LOW);
      } 
      else
      {
        setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
      }
  
    }
  
  }
   nh.spinOnce();


}

void waitormove(int pos[]){
  if (i!=0){
  i=i+1;
  }
  if(counter_zero==1  && i==0){
    i=i+1;
  }
  if(i==0){
    counter_zero=1;
  }
  
  if(instructions[i]==1)
        {  
          if(i<4){
            setTarget(1650,-1650,1650,-1650, pos);
            if(i==0){
              stuck=1;
              reach=1;
              // Serial.println("Reached");
              reached.data=reach;
              pub.publish(&reached);
            }
          }
          else if(i==4 ){
            setTarget(0,0,0,0, pos);
            
          }
          else if(i==5){
//            /setTarget(-600,-600,-600,-600, pos);
          setTarget(0,0,0,0, pos);
          } 
          else if(i==6){
            setTarget(-600,-600,-600,-600, pos);
          }
          else if(i==7) {
            setTarget(-1250,-1250,-1250,-1250,pos);
          } 
          
          
          }
        

        else{
          digitalWrite(in1[0],LOW);
          digitalWrite(in2[0],LOW);
          digitalWrite(in1[1],LOW);
          digitalWrite(in2[1],LOW);
          digitalWrite(in1[2],LOW);
          digitalWrite(in2[2],LOW);
          digitalWrite(in1[3],LOW);
          digitalWrite(in2[3],LOW);
          delay(2000);        
          if(i<4){
            setTarget(1650,-1650,1650,-1650,pos);
          }
          else if(i==4){
            setTarget(0,0,0,0,pos);
          }
          else if(i==5){
//            /setTarget(-600,-600,-600,-600, pos);
              setTarget(0,0,0,0, pos);
          } 
          else if(i==6) {
            setTarget(-600,-600,-600,-600, pos);
          }
          else if(i==7) {
            setTarget(-1250,-1250,-1250,-1250,pos);
          }   
       
        }
      if(instructions[i+1]==1){
          bumper=1;
        }

      }


void setTarget(int m0,int m1, int m2, int m3, int pos[]){
      target[0]=pos[0]+m0;
      target[1]=pos[1]+m1;
      target[2]=pos[2]+m2;
      target[3]=pos[3]+m3;
}
void bump(int time){
  if (i<4){
  for(int k=0;k<NMOTORS;k++){
    pid[k].setParams(0,0,0,255);
    }
    for(int k=0;k<NMOTORS;k++){
    setMotor(1,255,pwm[k],in1[k],in2[k]);
    }
  }
  else
  {
    for(int k=0;k<NMOTORS;k++){
    pid[k].setParams(0,0,0,255);
    }
    for(int k=0;k<NMOTORS;k++){
    setMotor(0,0,pwm[k],in1[k],in2[k]);
    }
    }
  // wait(time);
  delay(time);
  for(int k=0;k<NMOTORS;k++){
    pid[k].setParams(0.45,0.05,0,200);
    }
  reach=1;
  stuck=1;
  bumper=0;
  reached.data=reach;
  pub.publish(&reached);

}
void wait(int time){
  int counter=0;
  while(counter<time){
    counter++;
    // Serial.println(counter);
  }
}
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);

  }
  else if(dir == -1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);

  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}
template <int j>
void readEncoder(){
  int b = digitalRead(encb[j]);
  if(b > 0){
    posi[j]++;
  }
  else{
    posi[j]--;
  }
}
