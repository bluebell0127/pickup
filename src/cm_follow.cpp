#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <iostream>
#include "../include/op_follow/main_window.hpp"
#include "../include/op_follow/qnode.hpp"
#include <ros/network.h>
#include "body_tracker_msgs/BodyTracker.h"  // Publish custom message
#include "body_tracker_msgs/Skeleton.h"
#include <visualization_msgs/Marker.h>


std::vector<double> present_kinematic_position_;
open_manipulator_msgs::KinematicsPose kinematics_pose_msg;
double path_time = 2;
int initcount = 1;
float init_xpose, init_ypose, init_zpose;


class ttm
{
private:

  ros::NodeHandle n_;
  ros::ServiceClient move_client_;
  ros::Subscriber sub_;
  ros::Subscriber present_pose_sub; 
  ros::Subscriber hand_xyz_data_sub;
  ros::ServiceClient gripper_client;
  ros::ServiceClient joint_client_;

public:

  ttm()
  { 
    gripper_client = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");
    
    move_client_ = n_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path_position_only");

    joint_client_ = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");

    present_pose_sub = n_.subscribe("open_manipualtor/gripper/kinematics_pose", 10, &ttm::kinematicsPoseCallback, this);
  
    hand_xyz_data_sub = n_.subscribe("body_tracker/skeleton", 10, &ttm::xyzpointCallback, this);

  }

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
  {
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name = joint_name;
    srv.request.joint_position.position = joint_angle;
    srv.request.path_time = path_time;

    ROS_INFO("-- x( %.3lf) y( %.3lf) z( %.3lf)\n", srv.request.joint_position.position[0], srv.request.joint_position.position[1], srv.request.joint_position.position[2]);

    if(joint_client_.call(srv))
    {
      return srv.response.is_planned;
    }
      return false;
  }

  void init()
  {
    std::vector<std::string> joint_name_;
    std::vector<double> joint_angle;

    joint_name_.push_back("joint1");
    joint_name_.push_back("joint2");
    joint_name_.push_back("joint3");
    joint_name_.push_back("joint4");

    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    
    setJointSpacePath(joint_name_, joint_angle, path_time);
    
    ROS_INFO("INIT MODE SET");

  }

  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
  {
    std::vector<double> temp_position;
    
    temp_position.push_back(msg->pose.position.x);
    temp_position.push_back(msg->pose.position.y);
    temp_position.push_back(msg->pose.position.z);

    present_kinematic_position_ = temp_position;

    kinematics_pose_msg.pose = msg->pose;
  }

  void xyzpointCallback(const body_tracker_msgs::Skeleton::ConstPtr &handdata)
  {
    std::vector<double> kinematics_pose;

    int gripper = handdata->gesture;
    
//    if(initcount == 1)
//    {
//      init_xpose = handdata->joint_position_right_hand.x;
//      init_ypose = handdata->joint_position_right_hand.y;
//      init_zpose = handdata->joint_position_right_hand.z;
//      initcount++;
//      return;
//    }

//    else
//    {
      float present_xpose = handdata->joint_position_right_hand.x;
      float present_ypose = handdata->joint_position_right_hand.y;
      float present_zpose = handdata->joint_position_right_hand.z;

      float movex = present_xpose - init_xpose;
      float movey = present_ypose - init_ypose;     
      float movez = present_zpose - init_zpose;

      kinematics_pose.push_back(movex);
      kinematics_pose.push_back(movey);
      kinematics_pose.push_back(movez);

      setTaskSpacePath(kinematics_pose, path_time);

      ROS_INFO("-- x( %.3lf) y( %.3lf) z( %.3lf)\n", movex, movey, movez);

      if(gripper >= 1)
      {
        gripper_close();
      }
      else
      {
        gripper_open();
      }    
      return;
//    }
  }
    
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
  {
    open_manipulator_msgs::SetKinematicsPose srv;

    srv.request.end_effector_name = "gripper";

    if( std::isnan(kinematics_pose.at(0)) || std::isnan(kinematics_pose.at(1)) || std::isnan(kinematics_pose.at(2)) ) 
    
    {
      std::cerr << "Warning Error: NaN value operation.";
      return false;
    }

    srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
    srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
    srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);

    srv.request.kinematics_pose.pose.orientation.w = kinematics_pose_msg.pose.orientation.w;
    srv.request.kinematics_pose.pose.orientation.x = kinematics_pose_msg.pose.orientation.x;
    srv.request.kinematics_pose.pose.orientation.y = kinematics_pose_msg.pose.orientation.y;
    srv.request.kinematics_pose.pose.orientation.z = kinematics_pose_msg.pose.orientation.z;

//    ROS_INFO("-- x( %.3lf, %.3lf) y( %.3lf, %.3lf) z( %.3lf, %.3lf) w( %.3lf)\n", kinematics_pose.at(0), srv.request.kinematics_pose.pose.orientation.x,
//      kinematics_pose.at(1), srv.request.kinematics_pose.pose.orientation.y, kinematics_pose.at(2), srv.request.kinematics_pose.pose.orientation.z, srv.request.kinematics_pose.pose.orientation.w );

    srv.request.path_time = path_time;

   if(move_client_.call(srv))
    {
      ROS_INFO("move robot arm");
      return srv.response.is_planned;
    }
    return false;
  }

  void gripper_open(void)
  {
    std::vector<double> joint_angle;
    joint_angle.push_back(0.01);

    if(setToolControl(joint_angle))
    {
      ROS_INFO("Send gripper open");
      return;
    }
    ROS_INFO("[ERROR] gripper open service");      
  }

  void gripper_close(void)
  {
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);

    if(setToolControl(joint_angle))
    {
      ROS_INFO("Send gripper close");
      return;
    }

    ROS_INFO("[ERROR] gripper close service");
  }
  
  bool setToolControl(std::vector<double> joint_angle)
  {
    open_manipulator_msgs::SetJointPosition srv;
    srv.request.joint_position.joint_name.push_back("gripper");
    srv.request.joint_position.position = joint_angle;

    if(gripper_client.call(srv))
    {
      return srv.response.is_planned;
    }
    return false;
  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow");
  
  ros::start();

  ros::Rate loop_rate(100);

  ttm ttm1;
  ttm1.init();
  
//  ttm1.gripper_open();

  ros::spin();

  return 0;
}
