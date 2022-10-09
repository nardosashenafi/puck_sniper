#include "RMDServoState.h"
#include "mcp_can.h"
#include <SPI.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <math.h>

#define CAN_INTERRUPT_PIN 2
#define CAN_WAIT_TIMEOUT_US 500
#define CAN_SEND_PERIOD_US 10000     // 3333 gives about (reliably) 211 hz output to serial
#define DEBUG_PRINT 0
#define EFFECTIVE_GEAR_RATIO 0.14588859416445624f


long unsigned int rxId;
unsigned char flagRecv = 0;
unsigned char rxLen = 0;
unsigned char rxBuf[8];
unsigned long can_timer, read_timer, sw_timer;
volatile bool motor_enable = true;

float x_desired = 340.0, y_desired = 220.0;  //correspond to the center of the camera view
float pitch=0.0, yaw=0.0;
float theta1=0.0, theta2=0.0;
float omega1=0.0, omega2=0.0;
float dist_z = 300;

MCP_CAN CAN(9); 
RMDServoState servo1(0x141, &CAN, CAN_WAIT_TIMEOUT_US);
RMDServoState servo2(0x142, &CAN, CAN_WAIT_TIMEOUT_US);

ros::NodeHandle node_handle;
float puck_x = 0.0;
float puck_y = 0.0;
int sub_pin  = 11;

void subscriberCallback(const geometry_msgs::Pose& puck_pose_msg){

    puck_x = puck_pose_msg.position.x;
    puck_y = puck_pose_msg.position.y;
    digitalWrite(sub_pin, HIGH-digitalRead(sub_pin));

     if (motor_enable && (micros() - can_timer > CAN_SEND_PERIOD_US)) {
        bool wait = false; 

        inverseKinematics();
        if (!servo1.requestVelocity((long int) (omega1*100.0), wait)) { 
        }
        if (!servo2.requestVelocity((long int) (omega2*100.0), wait)) { 
        }

        can_timer = micros();
    }
}

ros::Subscriber<geometry_msgs::Pose> puck_pose_subscriber("puck_xy_position", &subscriberCallback);

void setup() {

    //RUN INITIAL SETUP
      while (CAN_OK != CAN.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ)) {           
            delay(100);
      }
      CAN.setMode(MCP_NORMAL);
  
      if (
          !servo1.requestClearError(true) ||
          !servo1.requestEncoder(true)
      ) {
      }
      if (
          !servo2.requestClearError(true) ||
          !servo2.requestEncoder(true)
      ) {
    }

      if (!servo1.requestPosition((long int) (0.0*100.0), false)) { 
      }
      if (!servo2.requestPosition((long int) (0.0*100.0), false)) { 
      }
        
        sw_timer = millis();
    
 //       delay(100);
    
    // SUBSCRIBE
    pinMode(sub_pin, OUTPUT);
    node_handle.initNode();
    node_handle.subscribe(puck_pose_subscriber);
    

}


void ISR_motor_enable() {

    if (motor_enable) {
        servo1.requestMotorOff();
        servo2.requestMotorOff();
    }
    motor_enable = !motor_enable;
}


void loop() {

    node_handle.spinOnce();
    delay(1);
}

void inverseKinematics()
{
    yaw                 =  0.0;

    float delta         =  x_desired - puck_x;
    float dot_prod      =  puck_x*x_desired + pow(dist_z,2.0);
    float desired_mag   =  sqrt( pow(x_desired, 2.0) + pow(dist_z, 2.0));
    float current_mag   =  sqrt( pow(puck_x, 2.0) +  pow(dist_z, 2.0));

    float sgn           = (delta>0) - (delta<0);
    pitch               = (sgn)*(acos(dot_prod/(desired_mag*current_mag)) * 180.0/PI);
    theta1              = 0.5/EFFECTIVE_GEAR_RATIO*(pitch+yaw);
    theta2              = 0.5/EFFECTIVE_GEAR_RATIO*(-pitch+yaw);
    omega1              = 15.0*(theta1);
    omega2              = 15.0*(theta2);
}
