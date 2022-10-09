/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;
int mypin = 13;

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(mypin, HIGH-digitalRead(mypin));   // blink the led
}

ros::Subscriber<std_msgs::Empty> sub("toggle_led", &messageCb );

void setup()
{ 
  pinMode(mypin, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

}

void loop()
{  
  nh.spinOnce();
  delay(1);
}
