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

//void precise( const std_msgs::Int32& toggle_msg){
//  command=toggle_msg.data
//}

ros::Subscriber<std_msgs::Bool> sub1("/completion_status", &complete );
ros::Subscriber<std_msgs::String> sub2("/mission_code", &messageCb );
//ros::Subscriber<std_msgs::Int32> sub2("/base_precision", &precise );


std_msgs::Int32 reached;

ros::Publisher pub("/data", &reached);
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
const int in1[] = {9,40,48,5};
const int in2[] = {10,42,50,6};

// char code[] = "11111111";
// char code[] = "00000100";
// char code[] = "01010101";
// char code[] = "00100100";
// char code[] = "11111101";

int instructions[]={0,0,0,0,0,0,0,0};

// Globals
long prevT = 0;
volatile int posi[] = {0,0,0,0};
int i=0;
int allowed_err=1000;

// PID class instances
SimplePID pid[NMOTORS];
int target[NMOTORS];


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

    pid[k].setParams(0.45,0.05,0,150);
  }

  attachInterrupt(digitalPinToInterrupt(enca[0]),readEncoder<0>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[1]),readEncoder<1>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[2]),readEncoder<2>,RISING);
  attachInterrupt(digitalPinToInterrupt(enca[3]),readEncoder<3>,RISING);
}


int save = 0;   //To save the rostopic
void loop() 
{
    counter.data = i;
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
        pos[3] = pos[1];
      }
    }
    // set target position
    
    if(i==0){
      setTarget(0,0,0,0,pos);
      waitormove(0,pos);
    }
    
    // adjusting the position after reaching the panel
//    if(reached.data==1){
//      switch(command){
//        case right:
//        setTarget(10,-10,10,-10);
//        reached.data=0;
//        break;
//        
//        case left:
//        setTarget(-10,10,-10,10);
//        reached.data=0;
//        break;
//        case correct:
//        // setTarget(10,-10,10,-10);
//        reached.data=0;
//        break;
//        default:
//
//      }
//    }
//  pid[0].eprev!=0 && pid[1].eprev!=0 && pid[2].eprev!=0 && pid[3].eprev!=0 &&
    if( pid[0].eprev!=0 && pid[1].eprev!=0 && pid[2].eprev!=0 && pid[3].eprev!=0 && pid[0].eprev<allowed_err && pid[1].eprev<allowed_err && pid[2].eprev<allowed_err && pid[3].eprev<allowed_err && pid[0].eprev>-allowed_err && pid[1].eprev>-allowed_err && pid[2].eprev>-allowed_err && pid[3].eprev>-allowed_err && i<=7)
    {
    waitormove(i,pos);
    
    }


    // loop through the motors
    for(int k = 0; k < NMOTORS; k++)
    {
      int pwr, dir;
      // evaluate the control signal
      pid[k].evalu(pos[k],target[k],deltaT,pwr,dir);
      
      // signal the motor
      if (i==8)
      {
          analogWrite(pwm[0],0);
          analogWrite(pwm[1],0);
          analogWrite(pwm[2],0);
          analogWrite(pwm[3],0);
      } 
      else
      {
        setMotor(dir,pwr,pwm[k],in1[k],in2[k]);
      }
  
    }
  }
  nh.spinOnce();
}

void waitormove(int i,int pos[]){
  if(instructions[i]==1)
        {
          reached.data = 1;
          pub.publish(&reached);

          
          if(next == 0)
            {
              digitalWrite(in1[0],LOW);
              digitalWrite(in2[0],LOW);
              digitalWrite(in1[1],LOW);
              digitalWrite(in2[1],LOW);
              digitalWrite(in1[2],LOW);
              digitalWrite(in2[2],LOW);
              digitalWrite(in1[3],LOW);
              digitalWrite(in2[3],LOW);
              delay(100);
            }
          else if (next==1){
          
          if(i<4){
            setTarget(1550,-1550,1550,-1550, pos);
          }
          else if(i==4){
            setTarget(0,0,0,0, pos);
          } 
          else if(i>4 && i<7){
            setTarget(-1550,-1550,-1550,-1550, pos);
          }
          
          i=i+1;
          next=0;
          reached.data=0;
          pub.publish(&reached);
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
          delay(500);         
          if(i<4){
            setTarget(1550,-1550,1550,-1550,pos);
          }
          else if(i==4){
            setTarget(0,0,0,0,pos);
          } 
          else{
            setTarget(-1550,-1550,-1550,-1550,pos);
          }
          i=i+1;
        }

}
void setTarget(int m0,int m1, int m2, int m3, int pos[]){
      target[0]=pos[0]+m0;
      target[1]=pos[1]+m1;
      target[2]=pos[2]+m2;
      target[3]=pos[3]+m3;
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
