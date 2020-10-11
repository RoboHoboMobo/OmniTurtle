#pragma once

#include <ros/ros.h>
#include "control_msgs/JointControllerState.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "ros/callback_queue.h"

#include "mecanum_solver.h"

namespace OmniTurtle
{

class OmniTurtleManager
{
public:

  using JointValue = std::map<std::string, double*>;

  OmniTurtleManager();
  void launch(ros::NodeHandle&, const std::string&, const MecanumSolver&);
  bool isRunning() const;

private:
  bool m_isRunning;
  const double m_frequency = 10.0;

  ros::NodeHandle m_node;
  const MecanumSolver* m_mecanumSolver;
  sensor_msgs::JointState m_currentJointState; // joint state callback variable

  JointValue m_jointsAngles;

  // Publishers section
  ros::Publisher m_mecanum_rf_jnt_pub;
  ros::Publisher m_mecanum_lf_jnt_pub;
  ros::Publisher m_mecanum_rb_jnt_pub;
  ros::Publisher m_mecanum_lb_jnt_pub;

  // Subscribers section
  ros::Subscriber m_ctrl_sub;           // Control topic subscriber
  ros::Subscriber m_mecanum_rf_jnt_sub; // Joints current state's subscribers
  ros::Subscriber m_mecanum_lf_jnt_sub;
  ros::Subscriber m_mecanum_rb_jnt_sub;
  ros::Subscriber m_mecanum_lb_jnt_sub;

  void sendMessage(const sensor_msgs::JointState&);

  // Callbacks section
  void controlCallback(const geometry_msgs::Twist&);
  void mecanumRFCallback(const control_msgs::JointControllerState&);
  void mecanumLFCallback(const control_msgs::JointControllerState&);
  void mecanumRBCallback(const control_msgs::JointControllerState&);
  void mecanumLBCallback(const control_msgs::JointControllerState&);

  void moveForward(double);
  void moveBackward(double);
  void moveRight(double);
  void moveLeft(double);
  void moveDiagRightUp(double);
  void moveDiagLeftUp(double);
  void moveDiagRightDown(double);
  void moveDiagLeftDown(double);
  void turnAroundClockwise(double);
  void turnAroundAnticlockwise(double);
  void stop();
};

}
