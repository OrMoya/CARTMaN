j//differential drive motor control for rosserial
#include <Encoder.h>
#include <ArduinoHardware.h>
#include <PID_v1.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/time.h>


#define ENC_CPR 4741 //Motor Encoder's counts per revoluation, as given by the manufacturer
#define EN_L 5
#define IN1_L 7
#define IN2_L 6
 
#define EN_R 10
#define IN1_R 9
#define IN2_R 8

// --- Robot Constants ---
const double wheelRadius = .06; //Wheel Radius in m, 120 mm diameter
const double wheelSeparation = .140; //Wheel Distance from center of motors

//encoder pins, CARTMAN uses an arduino uno, which only has two interrupt pins (2&3)
Encoder encL(3,12);
Encoder encR(2,11);

//PID Motor Control
//motor L
double pkL = 1;
double ikL = 0;
double dkL = 0.02;

double setpointL, inputL, outputL, outputBL;
PID PIDL(&inputL, &outputL, &setpointL, pkL, ikL, dkL, DIRECT);
//motor R
double pkR = 1;
double ikR = 0;
double dkR = 0.02;

double setpointR, inputR, outputR, outputBR;
PID PIDR(&inputR, &outputR, &setpointR, pkR, ikR, dkR, DIRECT);

float demandL;
float demandR;
float x;
float z;

double velocityL;
double velocityR;

unsigned long currentMillis;
unsigned long previousMillis;

float encDiffL;
float encDiffR;

float encErrL;
float encErrR;

float encPosL;
float encPosR;

float encPreL = 0;
float encPreR = 0;

ros::NodeHandle nh;

void messageCb( const geometry_msgs::Twist& msg) 
{
  z = msg.angular.z;
  x = msg.linear.x;
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("Speed", &speed_msg);
void setup() {

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  
  pinMode(IN1_L, OUTPUT); //setup pins
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  
  Serial.begin(9600);
  Serial.println("Encoder Test:");

  PIDL.SetMode(AUTOMATIC);
  PIDL.SetOutputLimits(-255, 255);
  PIDL.SetSampleTime(10);

  PIDR.SetMode(AUTOMATIC);
  PIDR.SetOutputLimits(-255, 255);
  PIDR.SetSampleTime(10);

}

void loop() {
  
  nh.spinOnce();
  
  encPosL = encL.read();
  encPosR = encR.read();
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  currentMillis = millis();

  if(currentMillis - previousMillis >= 10){

    previousMillis = currentMillis;

    demandL =  x - (z*(wheelSeparation/2));
    demandR =  x + (z*(wheelSeparation/2));

    encDiffL = encPosL - encPreL;
    encDiffR = encPosR - encPreR;

    encErrL = (demandL*165) - encDiffL; //118 
    encErrR = (demandR*125) - encDiffR;

    setpointL = demandL * 165; //encoder has 118 counts per 10ms to achieve 1m/s
    inputL = encDiffL;
    PIDL.Compute();

    setpointR = demandR * 125;
    inputR = encDiffR;
    PIDR.Compute();

    //compute actual velocity
    if(abs(encPosL - encPreL) < 5){
      velocityL = 0;
    }else{
      velocityL = (((encPosL - encPreL)/ENC_CPR)*2*PI)*100*wheelRadius;
    }
      if(abs(encPosR - encPreR) < 5){
      velocityR = 0;
    }else{
      velocityR = (((encPosL - encPreL)/ENC_CPR)*2*PI)*100*wheelRadius;
    }

    encPreL = encPosL;
    encPreR = encPosR;
   }

   //drive motor
  if(outputL > 0) 
  {
    outputBL = abs(outputL);
    digitalWrite(IN1_L, HIGH);
    digitalWrite(IN2_L, LOW);
    analogWrite(EN_L, outputBL);
  }
  else if (outputL < 0 )
  {
    outputBL = abs(outputL);
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, HIGH);
    analogWrite(EN_L, outputBL);    
  }
  else
  {
    digitalWrite(IN1_L, LOW);
    digitalWrite(IN2_L, LOW);
    analogWrite(EN_L, 0); 
  }

  if(outputR > 0) 
  {
    outputBR = abs(outputR);
    digitalWrite(IN1_R, HIGH);
    digitalWrite(IN2_R, LOW);
    analogWrite(EN_R, outputBR);
  }
  else if (outputR < 0 )
  {
    outputBR = abs(outputR);
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, HIGH);
    analogWrite(EN_R, outputBR);    
  }
  else
  {
    digitalWrite(IN1_R, LOW);
    digitalWrite(IN2_R, LOW);
    analogWrite(EN_R, 0); 
  }

  publishSpeed(100);
}

void publishSpeed(double time)
{
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = velocityL;
  speed_msg.vector.y = velocityR;
  speed_msg.vector.z = 100;
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
