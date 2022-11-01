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
double path_time = 1;
int initcount = 0;
float init_xpose, init_ypose, init_zpose;
bool open_manipulator_is_moving_;


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
  ros::Subscriber open_manipulator_states_sub_;

public:

  ttm()
  { 
    gripper_client = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_tool_control");
    
    move_client_ = n_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("/open_manipulator/goal_task_space_path_position_only");

    joint_client_ = n_.serviceClient<open_manipulator_msgs::SetJointPosition>("/open_manipulator/goal_joint_space_path");

    open_manipulator_states_sub_ = n_.subscribe("states", 10, &ttm::manipulatorStatesCallback, this);

    present_pose_sub = n_.subscribe("open_manipulator/gripper/kinematics_pose", 10, &ttm::kinematicsPoseCallback, this);
  
    hand_xyz_data_sub = n_.subscribe("body_tracker/skeleton", 10, &ttm::xyzpointCallback, this);
    
  }

  void manipulatorStatesCallback(const open_manipulator_msgs::OpenManipulatorState::ConstPtr &msg)
  {
    if (msg->open_manipulator_moving_state == msg->IS_MOVING)
      open_manipulator_is_moving_ = true;
    else
      open_manipulator_is_moving_ = false;
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
    
    setJointSpacePath(joint_name_, joint_angle, 2);
    
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

//    ROS_INFO("presentpose x( %.3lf) y( %.3lf) z( %.3lf)\n", present_kinematic_position_[0] , present_kinematic_position_[1], present_kinematic_position_[2]);
  }

  void xyzpointCallback(const body_tracker_msgs::Skeleton::ConstPtr &handdata)
{
    std::vector<double> kinematics_pose;


    int gripper = handdata->gesture;
    float present_xpose = handdata->joint_position_right_hand.x;
    float present_ypose = handdata->joint_position_right_hand.y;
    float present_zpose = handdata->joint_position_right_hand.z;

    float shoulder_xpose = handdata->joint_position_right_shoulder.x;
    float shoulder_ypose = handdata->joint_position_right_shoulder.y;
    float shoulder_zpose = handdata->joint_position_right_shoulder.z;
    
    float movex = (present_xpose-shoulder_xpose)*0.71*(-1)*0.7;
    float movey = (present_ypose-shoulder_ypose)*0.8*(-1)*0.5;     
    float movez = ((present_zpose-shoulder_zpose)+0.4)*0.64;

    kinematics_pose.push_back(movex);
    kinematics_pose.push_back(movey);
    kinematics_pose.push_back(movez);
    initcount++;
    if(initcount < 60 )
    {

      /*if(movex< 0.110)
      {
        movex= 0.110;
      }
      if(0<movey<0.110)
      {
        movey= 0.110;
      }
      if(-0.110<movey<0)
      {
        movey= -0.110;
      }
      if(movez< 0)
      {
        movez= 0;
      }
      if(movez>0.300)
      {          
        movez= 0.300;
      }*/

      return;
    }    
    else
    {
      if (!open_manipulator_is_moving_) 
      {   
          setTaskSpacePath(kinematics_pose, path_time);
          ROS_INFO("movexyz x( %.3lf) y( %.3lf) z( %.3lf)\n", movex, movey, movez);
          initcount = 0;
      }
    }  
  
      if(gripper >= 1)
      {
        gripper_close();
      }
      else
      {
        gripper_open();
      }
      return;
  }
    
  bool setTaskSpacePath(std::vector<double> kinematics_pose, double path_time)
  {
    open_manipulator_msgs::SetKinematicsPose srv;

    srv.request.end_effector_name = "gripper";

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

  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Rate rate(25);

  ttm ttm1;
  ttm1.init();
  
  ros::waitForShutdown();
//  ros::spin();

}
