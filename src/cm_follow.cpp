#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include <std_msgs/String.h>
#include <iostream>
#include "../include/op_follow/main_window.hpp"
#include "../include/op_follow/qnode.hpp"
#include <ros/network.h>


std::vector<double> present_kinematic_position_;
open_manipulator_msgs::KinematicsPose kinematics_pose_msg;
std::vector<double> joint_angle;
 std::vector<std::string> joint_name_;
double path_time = 1;


class ttm
{
private:

  ros::NodeHandle n_;
  ros::ServiceClient move_client_;
  ros::Subscriber sub_;
  ros::Subscriber present_pose_sub; 
  ros::ServiceClient gripper_client;
  ros::ServiceClient joint_space_path_client_;

public:

  ttm()
  { 

    gripper_client = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
    
    move_client_ = n_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_position_only");
  
    present_pose_sub = n_.subscribe("gripper/kinematics_pose", 10, &ttm::kinematicsPoseCallback, this);

    joint_space_path_client_ = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");

    sub_ = n_.subscribe("point", 10, &ttm::pointCallback, this);
 
  }

  void init()
  {

    joint_name_.push_back("joint1");
    joint_name_.push_back("joint2");
    joint_name_.push_back("joint3");
    joint_name_.push_back("joint4");

    joint_angle.push_back( 0.00);
    joint_angle.push_back(-1.05);
    joint_angle.push_back( 0.35);
    joint_angle.push_back( 0.70);
    setJointSpacePath(joint_name_, joint_angle, 2.0);

    std::vector<double> gripper_value;
    gripper_value.push_back(0.0);
    setToolControl(gripper_value);
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

    ROS_INFO("-- x( %.3lf, %.3lf) y( %.3lf, %.3lf) z( %.3lf, %.3lf) w( %.3lf)\n", kinematics_pose.at(0), srv.request.kinematics_pose.pose.orientation.x,
      kinematics_pose.at(1), srv.request.kinematics_pose.pose.orientation.y, kinematics_pose.at(2), srv.request.kinematics_pose.pose.orientation.z, srv.request.kinematics_pose.pose.orientation.w );

    srv.request.path_time = path_time;

   if(move_client_.call(srv))
    {
      ROS_INFO("move robot arm");
      return srv.response.is_planned;
    }
    return false;
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


  void pointCallback(const geometry_msgs::Point& input)
  {    
    std::vector<double> kinematics_pose;

    kinematics_pose.push_back(input.x);
    kinematics_pose.push_back(input.y);
    kinematics_pose.push_back(input.z);

    setTaskSpacePath(kinematics_pose, path_time);

    ROS_INFO("Send task pose");

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
    ROS_INFO("[ERR!!] gripper open service");      
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

    ROS_INFO("[ERR!!] gripper close service");
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

  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
  {
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(joint_space_path_client_.call(srv))
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

  ros::Rate loop_rate(0.5);
  
  ROS_INFO("waiting data..");

  ttm ttm1;

  ttm1.init();

  ros::spin();

  return 0;
}
