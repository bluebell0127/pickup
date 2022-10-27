#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <sstream>
#include "gb_visual_detection_3d_msgs/BoundingBox3d.h"
#include "gb_visual_detection_3d_msgs/BoundingBoxes3d.h"


int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle nn;

  ros::Publisher chatter_pub = nn.advertise<gb_visual_detection_3d_msgs::BoundingBoxes3d>("point", 100);

  ros::Rate loop_rate(1);

  int count = 0;

  while (ros::ok())
  {
    gb_visual_detection_3d_msgs::BoundingBoxes3d msg;

    msg.bounding_boxes[0].xmin = 0.119+0.2;
    msg.bounding_boxes[0].xmax = 0.119+0.2;
    
    msg.bounding_boxes[0].ymin = 0.05-0.5;
    msg.bounding_boxes[0].ymax = 0.05-0.5;
    
    msg.bounding_boxes[0].zmin = 0.232;
    msg.bounding_boxes[0].zmax = 0.232;

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    ROS_INFO("sending..");
    ++count;
  }


  return 0;
}