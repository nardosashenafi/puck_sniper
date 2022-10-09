#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>

ros::NodeHandle node_handle;
float u = 0.0;
float v = 0.0;
int sub_pin = 13;

void subscriberCallback(const geometry_msgs::Pose& puck_pose_msg){

    u = puck_pose_msg.position.x;
    v = puck_pose_msg.position.y;
    digitalWrite(sub_pin, HIGH-digitalRead(sub_pin));

}

ros::Subscriber<geometry_msgs::Pose> puck_pose_subscriber("puck_xy_position", &subscriberCallback);

void setup()
{
    pinMode(sub_pin, OUTPUT);
    node_handle.initNode();
    node_handle.subscribe(puck_pose_subscriber);
//    delay(1000);
}

void loop()
{
//    node_handle.subscribe(puck_pose_subscriber);
//    Serial.print("x puck pose is ");
//    Serial.println(u);
//    Serial.print("y puck pose is ");
//    Serial.println(v);

    node_handle.spinOnce();
    delay(10);

}
