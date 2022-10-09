#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

geometry_msgs::Point puck_pose_msg;

int main(int argc, int **argv)
{
    ros::init(argc, argv, "puck_pos_pubisher_node");
    ros::NodeHandle node_handle;
    ros::Publisher puck_pose_pub = node_handle.advertise<geometry_msgs::Point>("puck_pose", 1000);

    //TODO: receive puck pose through rosserial

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        puck_pose_msg.x = ;
        puck_pose_msg.y = ;
        puck_pose_msg.z = ;

        puck_pose_pub.publish(puck_pose_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}