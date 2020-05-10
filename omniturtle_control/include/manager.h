#ifndef MANAGER_H
#define MANAGER_H

#include"mecanum_solver.h"

#include<ros/ros.h>

#include"control_msgs/JointControllerState.h"
#include"geometry_msgs/Twist.h"
#include"std_msgs/Int32.h"
#include"std_msgs/Float64.h"

namespace OmniTurtle
{
  class OmniTurtleManager
  {
  private:
    ros::NodeHandle* n;
    const MecanumSolver* msp;

    // Subscribers section
    ros::Subscriber ctrl_sub;           // Control topic subscriber
    ros::Subscriber mecanum_rf_jnt_sub; // Joints current state's subscribers
    ros::Subscriber mecanum_lf_jnt_sub;
    ros::Subscriber mecanum_rb_jnt_sub;
    ros::Subscriber mecanum_lb_jnt_sub;

    // Callbacks section
    void controlCallback(const geometry_msgs::Twist& msg);
    void mecanumRFCallback(const control_msgs::JointControllerState& msg);
    void mecanumLFCallback(const control_msgs::JointControllerState& msg);
    void mecanumRBCallback(const control_msgs::JointControllerState& msg);
    void mecanumLBCallback(const control_msgs::JointControllerState& msg);

    sensor_msgs::JointState cur_jnt_state; // joint state callback variable
    size_t find_joint(const sensor_msgs::JointState &jsm, std::string joint_name) const;

    // Publishers section
    ros::Publisher mecanum_rf_jnt_pub;
    ros::Publisher mecanum_lf_jnt_pub;
    ros::Publisher mecanum_rb_jnt_pub;
    ros::Publisher mecanum_lb_jnt_pub;

    void sendMessage(const sensor_msgs::JointState& jsm);

    double delta;
    bool is_running;

  public:
    OmniTurtleManager(ros::NodeHandle& node, const std::string& sub_topic_name,
                        const MecanumSolver& ms);
    virtual void moveForward();
    virtual void moveBackward();
    virtual void moveRight();
    virtual void moveLeft();
    virtual void moveDiagRightUp();
    virtual void moveDiagLeftUp();
    virtual void moveDiagRightDown();
    virtual void moveDiagLeftDown();
    virtual void turnAroundClockwise();
    virtual void turnAroundAnticlockwise();
    virtual void stop();

    bool isRunning() const { return is_running; }

  };

}

#endif // MANAGER_H
