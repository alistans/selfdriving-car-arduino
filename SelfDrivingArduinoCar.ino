#include <IRremote.h>

//wheel pins
const int leftWheel = 5;
const int rightWheel = 6;
const int leftWheelReverse = 3;
const int rightWheelReverse = 9;

//ultrasonic sensor pins
const int triggerLeft = 7;
const int echoLeft = 8;
const int triggerFront = 10;
const int echoFront = 11;
const int triggerRight = 12;
const int echoRight = 13;

//variables
const int maxDistanceFront = 30;
const int maxDistanceSide = 30;
boolean driving = false;
int velocity = 230;
int stuck = 0;
int maxStuck = 5;

//current measure vars
const int currentSensor = A0;
int mVperAmp = 185;
double Voltage = 0;
double VRMS = 0;
double AmpsRMS = 0;
const float maxCurrent = 0.80;

void setup()
{
  Serial.begin(9600);
  pinMode(leftWheel, OUTPUT);
  pinMode(rightWheel, OUTPUT);
  pinMode(echoFront, INPUT);
  pinMode(triggerFront, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(triggerLeft, OUTPUT);
  pinMode(echoRight, INPUT);
  pinMode(triggerRight, OUTPUT);
}

void loop()
{
  drive();
}

void drive(){
    //calculate the current distances
    long distanceFront = calcDistance(echoFront, triggerFront);
    long distanceLeft = calcDistance(echoLeft, triggerLeft);
    long distanceRight = calcDistance(echoRight, triggerRight);
    
    if  (distanceFront < maxDistanceFront || distanceLeft < maxDistanceSide || distanceRight < maxDistanceSide){
      redirect();
    } else {
      goForward();
    }

    //check if the car is stuck by measuring the current
    float currentCurrent = getCurrent();
    if(currentCurrent >= maxCurrent){
        //the current has to be above the maxcurrent 'maxStuck' times to avoid false/irregular measurements
        if(stuck >= maxStuck){
          goReverse();
          stuck = 0;
        } else {
         stuck++;
        } 
    } else {
      stuck = 0;
    }
}

long calcDistance(int echo, int trigger){
    long distance = 5000; //number that's bigger than the distance can be
    
    for (int i = 0; i <= 1; i++) {
      long responseTime, responseDistance;
      
      digitalWrite(trigger, LOW);
      delayMicroseconds(2);
      digitalWrite(trigger, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger, LOW);
 
      //calculate the time it took before the echo received a high pulse
      responseTime = pulseIn(echo, HIGH);
    
      //formula to convert the time to distance in cm
      responseDistance = responseTime / 29 / 2;
      
      if(distance > responseDistance ){
        distance = responseDistance;
      }
    }
    return distance;
}

void redirect(){
  Serial.println("redirecting...");
  float left = calcDistance(echoLeft, triggerLeft);
  float right = calcDistance(echoRight, triggerRight);
  float front = calcDistance(echoFront, triggerFront);
  
  if(front > left && front > right){
    goForward();
  } else if (right >= left){
    goRight();
  } else{
    goLeft();
  }
  return;
}

void goLeft(){
  analogWrite(rightWheel, 255);
  digitalWrite(leftWheel, LOW);
  return;
}

void goRight(){
  analogWrite(leftWheel, 255);
  digitalWrite(rightWheel, LOW);
  return;
}

void goForward(){
  analogWrite(leftWheel, velocity);
  analogWrite(rightWheel, velocity);
  return;
}

void goStop(){
  Serial.println("stop");
  digitalWrite(leftWheel, LOW);
  digitalWrite(rightWheel, LOW);
  return;
}

void goReverse(){
  goStop();
  digitalWrite(leftWheelReverse, HIGH);
  digitalWrite(rightWheelReverse, HIGH);
  delay(1000);
  redirect();
  delay(1000);
  digitalWrite(leftWheelReverse, LOW);
  digitalWrite(rightWheelReverse, LOW);
  return;
}

float getCurrent(){
  //calculate the peak to peak voltage
  VPP = getVPP();

  //calculate the root mean square of the voltage
  VRMS = (VPP/2.0) *0.707; 

  //calculate the root mean square of the current
  AmpsRMS = (VRMS * 1000)/mVperAmp;
  return AmpsRMS;
}

float getVPP(){
  float result;
  int readValue;            
  int maxValue = 0;          
  int minValue = 1024;       
  
  uint32_t startTime = millis();
  
  //sample for 1 Sec
  while((millis()-startTime) < 500){  
      readValue = analogRead(currentSensor);
       // see if you have a new maxValue and/or minValue
       if (readValue > maxValue){
           maxValue = readValue;
       }
       if (readValue < minValue) {
           minValue = readValue;
       }
   }
   
   // Subtract min from max
   result = ((maxValue - minValue) * 5.0)/1024.0;
      
   return result;
}
