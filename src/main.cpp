
#include "Arduino.h"

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#define FWD_PIN (2)
#define TURN_PIN (3)
#define STATUS_PIN (13)
#define WATCHDOG_TIMEOUT (500)

ros::NodeHandle  nh;

Servo steerServo;
Servo throttleServo;

unsigned long watchdogTime;
bool armed;

void throttleCallback( const std_msgs::UInt16& cmd_msg){
    watchdogTime = millis();
  throttleServo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

void steerCallback( const std_msgs::UInt16& cmd_msg){
    watchdogTime = millis();
  steerServo.write(cmd_msg.data); //set servo angle, should be from 0-180  
}

// void heartbeatCallback( const std_msgs::Empty& msg){
//     watchdogTime = millis();
//     digitalWrite(STATUS_PIN, HIGH-digitalRead(STATUS_PIN));  //toggle led  
// }

void armedCallback( const std_msgs::Bool& msg){
    armed = msg.data;
}

void writeToServos(const int steerVal,const int throttleVal)
{
    if ( steerVal > 180 || steerVal < 0 || throttleVal > 180 || throttleVal < 0 || armed == false )
    {
      steerServo.write(0); //set servo angle, should be from 0-180  
      throttleServo.write(0); //set servo angle, should be from 0-180  
    }
    else
    {
      steerServo.write(steerVal); //set servo angle, should be from 0-180  
      throttleServo.write(throttleVal); //set servo angle, should be from 0-180  
    }
}

// std_msgs::String debugStr;
// ros::Publisher debugPub("debug", &debugStr);
// void printDebug(const char* msg)
// {
//     debugStr.data = msg;
//     debugPub.publish(&debugStr);
// }

ros::Subscriber<std_msgs::UInt16> steerSub("servo_steer", steerCallback);
ros::Subscriber<std_msgs::UInt16> throttleSub("servo_throttle", throttleCallback);
// ros::Subscriber<std_msgs::Empty> heartbeatSub("heartbeat", heartbeatCallback);
ros::Subscriber<std_msgs::Bool> armedSub("armed", armedCallback);

void setup(){
  pinMode(STATUS_PIN, OUTPUT);

  steerServo.attach(TURN_PIN); 
  throttleServo.attach(FWD_PIN); 

  armed = false;
  writeToServos(0,0);

  nh.initNode();
  nh.subscribe(steerSub);
  nh.subscribe(throttleSub);
  // nh.subscribe(heartbeatSub);
  nh.subscribe(armedSub);
    // printDebug("Resetting motors");
  
  watchdogTime = millis();
}

void loop(){
    if ( millis() - watchdogTime > WATCHDOG_TIMEOUT )
    {
        // Watch dog timer timed out
        armed = false;
        writeToServos(0,0);
        // printDebug("Timeout");
    }
    else
    {
        digitalWrite(STATUS_PIN, HIGH);  
    }
  nh.spinOnce();
  delay(1);
}
