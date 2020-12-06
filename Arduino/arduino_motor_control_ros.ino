#define LOOPTIME 100 //100 miliseconds
#define EN_L 5
#define IN1_L 7
#define IN2_L 6
 
#define EN_R 10
#define IN1_R 8
#define IN2_R 9

const double ENC_CPR = 4741; //Motor Encoder's counts per revoluation, as given by the manufacturer

double mod = .145; //modifier need to match demand and actual speeds
// --- Robot Constants ---
const double wheelRadius = .06; //Wheel Radius in m, 120 mm diameter
const double wheelSeparation = .280; //Wheel Distance from center of motors

//encoder pins, CARTMAN uses an arduino uno, which only has two interrupt pins (2&3)
Encoder encL(3,19);
Encoder encR(2,18);

//PID Motor Control
//motor L
double pkL = .1;
double ikL = 0;
double dkL = 0.002;

double setpointL, inputL, outputL, outputBL;
PID PIDL(&inputL, &outputL, &setpointL, pkL, ikL, dkL, DIRECT);
//motor R
double pkR = .1;
double ikR = 0;
double dkR = 0.002;

double setpointR, inputR, outputR, outputBR;
PID PIDR(&inputR, &outputR, &setpointR, pkR, ikR, dkR, DIRECT);

float demandL;
float demandR;
float x;
float z;

double speed_act_left = 0;
double speed_act_right = 0;

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

  demandL =  (x - (z*(wheelSeparation/2)))/mod;
  demandR =  (x + (z*(wheelSeparation/2)))/mod;
    
  nh.spinOnce();
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
geometry_msgs::Vector3Stamped speed_msg;
ros::Publisher speed_pub("speed", &speed_msg);
void setup() {
  
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(speed_pub);
  
  pinMode(IN1_L, OUTPUT); //setup pins
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);

  PIDL.SetMode(AUTOMATIC);
  PIDL.SetOutputLimits(-255, 255);
  PIDL.SetSampleTime(LOOPTIME);

  PIDR.SetMode(AUTOMATIC);
  PIDR.SetOutputLimits(-255, 255);
  PIDR.SetSampleTime(LOOPTIME);

}

void loop() {
  
  nh.spinOnce();
  
  currentMillis = millis();

  if(currentMillis - previousMillis >= LOOPTIME){

    encPosL = encL.read();
    encPosR = encR.read();
    
    previousMillis = currentMillis;

    encDiffL = encPosL - encPreL;
    encDiffR = encPosR - encPreR;

    encErrL = (demandL*1256 - encDiffL); //118 
    encErrR = (demandR*1256 - encDiffR);

    setpointL = demandL * 1257;//encoder has 1257 counts per 100ms to achieve 1m/s
    inputL = encDiffL; //encoder counts, encDiff = previous count - current count
    PIDL.Compute(); //compute the needed pwm value to match the demanded speed

    setpointR = demandR * 1256;
    inputR = encDiffR;
    PIDR.Compute();

    //compute actual velocity
    if(abs(encPosL - encPreL) < 5){
      speed_act_left = 0;
    }else{
      speed_act_left = ((encDiffL/ENC_CPR)*2*PI)*(1000/LOOPTIME)*wheelRadius;
    }
      if(abs(encPosR - encPreR) < 5){
      speed_act_right = 0;
    }else{
      speed_act_right = ((encDiffR/ENC_CPR)*2*PI)*(1000/LOOPTIME)*wheelRadius;
    }

    encPreL = encPosL;
    encPreR = encPosR;
   

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

  publishSpeed(LOOPTIME);
  }
}

void publishSpeed(double time)
{
  speed_msg.header.stamp = nh.now();
  speed_msg.vector.x = speed_act_left;
  speed_msg.vector.y = speed_act_right;
  speed_msg.vector.z = time/1000; //looptime in seconds
  speed_pub.publish(&speed_msg);
  nh.spinOnce();
  nh.loginfo("Publishing odometry");
}
