#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle nn;

  ros::Publisher chatter_pub = nn.advertise<geometry_msgs::Point>("point", 100);

  ros::Rate loop_rate(1);

  int count = 0;

  while (ros::ok())
  {
    geometry_msgs::Point msg;

    msg.x = 0.139;
    msg.y = 0.0;
    msg.z = 0.232;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ROS_INFO("sending..");
    ++count;
  }


  return 0;
}