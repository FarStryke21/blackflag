// encoder pinout
// Red: motor +
// Black: motor -
// Green: encoder -
// Blue: encoder +
// Yellow: encoder A
// White: encoder B
#include <ros.h>
#include <geometry_msgs/Twist.h>

// Encoder Pins for Front Right
#define Encoder_output_FR_A 20 // pin2 of the Arduino
#define Encoder_output_FR_B 21 // pin 3 of the Arduino

// Encoder Pins for Front Left
#define Encoder_output_FL_A 19 // pin2 of the Arduino
#define Encoder_output_FL_B 18 // pin 3 of the Arduino

// Encoder Pins for Rear Right
#define Encoder_output_RR_A 3 // pin2 of the Arduino
#define Encoder_output_RR_B 52 // pin 3 of the Arduino

// Encoder Pins for Rear Left
#define Encoder_output_RL_A 2 // pin2 of the Arduino
#define Encoder_output_RL_B 7 // pin 3 of the Arduino

ros::NodeHandle nh;
float linear_velocity = 0.0;
int menuChoice = 0;

int Count_pulses = 0;

// Motor FR connections - Motor Driver 1
int enFR = 44;
int inFR_1 = 40;
int inFR_2 = 42;


// Motor FL connections - Motor Driver 1
int enFL = 8;
int inFL_1 = 9;
int inFL_2 = 10;

// Motor RR connections - Motor Driver 2
int enRR = 46;
int inRR_1 = 48;
int inRR_2 = 50;


// Motor RL connections - Motor Driver 2
int enRL = 4;
int inRL_1 = 5;
int inRL_2 = 6;

float x = 0.0;
float z = 0.0;


void cmdVelCallback(const geometry_msgs::Twist& twist_msg) {
  x = twist_msg.linear.x;
  z = twist_msg.angular.z;

  if(x == 2.0)
  {
    menuChoice = 2;
  }
  else if (x == -2.0)
  {
    menuChoice = 1;
  }
  else if (z == 2.0)
  {
    menuChoice = 4;
  }
  else if (z == -2.0)
  {
    menuChoice = 3;
  }
}

ros::Subscriber<geometry_msgs::Twist> sub("/turtle1/cmd_vel", cmdVelCallback);

void setup() 
{
  Serial.begin(9600);
  pinMode(Encoder_output_FR_A,INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_FR_B,INPUT); // sets the Encoder_output_B pin as the input
  pinMode(Encoder_output_FL_A,INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_FL_B,INPUT); // sets the Encoder_output_B pin as the input
  pinMode(Encoder_output_RR_A,INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_RR_B,INPUT); // sets the Encoder_output_B pin as the input
  pinMode(Encoder_output_RL_A,INPUT); // sets the Encoder_output_A pin as the input
  pinMode(Encoder_output_RL_B,INPUT); // sets the Encoder_output_B pin as the input

  attachInterrupt(digitalPinToInterrupt(Encoder_output_FR_A),DC_Motor_Encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_FL_A),DC_Motor_Encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_RR_A),DC_Motor_Encoder,RISING);
  attachInterrupt(digitalPinToInterrupt(Encoder_output_RL_A),DC_Motor_Encoder,RISING);
  
  // Set all the motor control pins to outputs
  pinMode(enFR, OUTPUT);
  pinMode(enFL, OUTPUT);
  pinMode(inFR_1, OUTPUT);
  pinMode(inFR_2, OUTPUT);
  pinMode(inFL_1, OUTPUT);
  pinMode(inFL_2, OUTPUT);

  // Set all the motor control pins to outputs
  pinMode(enRR, OUTPUT);
  pinMode(enRL, OUTPUT);
  pinMode(inRR_1, OUTPUT);
  pinMode(inRR_2, OUTPUT);
  pinMode(inRL_1, OUTPUT);
  pinMode(inRL_2, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(inFR_1, LOW);
  digitalWrite(inFR_2, LOW);
  digitalWrite(inFL_1, LOW);
  digitalWrite(inFL_2, LOW);

  // Turn off motors - Initial state
  digitalWrite(inRR_1, LOW);
  digitalWrite(inRR_2, LOW);
  digitalWrite(inRL_1, LOW);
  digitalWrite(inRL_2, LOW);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  if (Serial.available() > 0) {
    char key = Serial.read();
    if (key == 'q') {
      exit(0);
    }
  }
  
  switch (menuChoice) 
  {
    case 1:
        while (Serial.available() == 0)
        {
//          Reverse
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, HIGH);
          digitalWrite(inFR_2, LOW);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, HIGH);
          digitalWrite(inFL_2, LOW);

          analogWrite(enRR,255);
          digitalWrite(inRR_1,HIGH);
          digitalWrite(inRR_2,LOW);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,HIGH);
          digitalWrite(inRL_2,LOW);

          //Serial.println(Count_pulses);
        }

    case 2:
        while (Serial.available() == 0)
        {
//          Front
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, LOW);
          digitalWrite(inFR_2, HIGH);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, LOW);
          digitalWrite(inFL_2, HIGH);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,LOW);
          digitalWrite(inRR_2,HIGH);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,LOW);
          digitalWrite(inRL_2,HIGH);
          
          //Serial.println(Count_pulses); 
        }

    case 3:
    while (Serial.available() == 0)
    {
//      Right    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, HIGH);
          digitalWrite(inFR_2, LOW);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, LOW);
          digitalWrite(inFL_2, HIGH);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,LOW);
          digitalWrite(inRR_2,HIGH);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,HIGH);
          digitalWrite(inRL_2,LOW);
          
          //Serial.println(Count_pulses); 
    
    }
    case 4:
      while (Serial.available() == 0)
    {
//      Left    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, LOW);
          digitalWrite(inFR_2, HIGH);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, HIGH);
          digitalWrite(inFL_2, LOW);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,HIGH);
          digitalWrite(inRR_2,LOW);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,LOW);
          digitalWrite(inRL_2,HIGH);
          
          //Serial.println(Count_pulses); 
  }
    case 5:
      while (Serial.available() == 0)
    {
//      Rear right    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, HIGH);
          digitalWrite(inFR_2, LOW);
          
          analogWrite(enFL,0);
          
          analogWrite(enRR,0);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,HIGH);
          digitalWrite(inRL_2,LOW);
          
          //Serial.println(Count_pulses); 
  } 

    case 6:
      while (Serial.available() == 0)
    {
//      Rear left 

          analogWrite(enFR,0);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, HIGH);
          digitalWrite(inFL_2, LOW);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,HIGH);
          digitalWrite(inRR_2,LOW);
          
          analogWrite(enRL,0);
         // Serial.println(Count_pulses); 
  }    

      case 7:
      while (Serial.available() == 0)
    {
//      Front right 

          analogWrite(enFR,0);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, LOW);
          digitalWrite(inFL_2, HIGH);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,LOW);
          digitalWrite(inRR_2,HIGH);
          
          analogWrite(enRL,0);
          //Serial.println(Count_pulses); 
  }  

      case 8:
      while (Serial.available() == 0)
    {
//      Front left 
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, LOW);
          digitalWrite(inFR_2, HIGH);
          
          analogWrite(enFL,0);
          
          analogWrite(enRR,0);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,LOW);
          digitalWrite(inRL_2,HIGH);
          
          //Serial.println(Count_pulses); 
 
  }
      case 9:
      while (Serial.available() == 0)
    {
//      clokwise    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, HIGH);
          digitalWrite(inFR_2, LOW);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, LOW);
          digitalWrite(inFL_2, HIGH);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,HIGH);
          digitalWrite(inRR_2,LOW);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,LOW);
          digitalWrite(inRL_2,HIGH);
          
          //Serial.println(Count_pulses); 
  }
      case 10:
      while (Serial.available() == 0)
    {
//      counterclockwise    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, LOW);
          digitalWrite(inFR_2, HIGH);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, HIGH);
          digitalWrite(inFL_2, LOW);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,LOW);
          digitalWrite(inRR_2,HIGH);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,HIGH);
          digitalWrite(inRL_2,LOW);
          
          //Serial.println(Count_pulses); 
  }  
  
  case 0:
      while (Serial.available() == 0)
    {
//      CLockwise    
          analogWrite(enFR, 255);
          digitalWrite(inFR_1, LOW);
          digitalWrite(inFR_2, LOW);
          
          analogWrite(enFL, 255);
          digitalWrite(inFL_1, LOW);
          digitalWrite(inFL_2, LOW);
          
          analogWrite(enRR,255);
          digitalWrite(inRR_1,LOW);
          digitalWrite(inRR_2,LOW);
          
          analogWrite(enRL,255);
          digitalWrite(inRL_1,LOW);
          digitalWrite(inRL_2,LOW);
          
          //Serial.println(Count_pulses); 
  }  
   
}
  nh.spinOnce();
}

void DC_Motor_Encoder()
{
  int b = digitalRead(Encoder_output_FR_A);
  if(b > 0)
  {
    Count_pulses++;
  }
  else
  {
    Count_pulses--;
  }
}
