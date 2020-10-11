#include"../include/manager.h"

namespace {

size_t findJoint(const sensor_msgs::JointState &jointState,
                  const std::string& jointName)
{
  auto found = std::find(jointState.name.cbegin(), jointState.name.cend(), jointName);
  if (found != jointState.name.end())
    return static_cast<size_t>(std::distance(jointState.name.cbegin(), found));
  else
    throw std::invalid_argument("OmniTurtleManager: " + jointName + "isn't found");
}

} // namespace

OmniTurtle::OmniTurtleManager::OmniTurtleManager()
  : m_isRunning{false}
  , m_node{}
  , m_mecanumSolver{}
  , m_currentJointState{}
  , m_jointsAngles{}
{
}

void OmniTurtle::OmniTurtleManager::launch(ros::NodeHandle& node,
                                           const std::string& subscriberTopicName,
                                           const MecanumSolver& mecanumSolver)
{
  m_isRunning = true;
  m_node = node;
  m_mecanumSolver = &mecanumSolver;

  m_currentJointState.name = {"mecanum_rf_joint", "mecanum_lf_joint",
                        "mecanum_rb_joint", "mecanum_lb_joint"};

  m_currentJointState.position.resize(m_currentJointState.name.size(), 0.0);

  for (size_t i=0; i < m_currentJointState.name.size(); i++)
    m_jointsAngles[m_currentJointState.name.at(i)] = &m_currentJointState.position.at(i);

  m_mecanum_rf_jnt_pub =
      m_node.advertise<std_msgs::Float64>("/omniturtle/mecanum_rf_joint_position_controller/command", 1);
  m_mecanum_lf_jnt_pub =
      m_node.advertise<std_msgs::Float64>("/omniturtle/mecanum_lf_joint_position_controller/command", 1);
  m_mecanum_rb_jnt_pub =
      m_node.advertise<std_msgs::Float64>("/omniturtle/mecanum_rb_joint_position_controller/command", 1);
  m_mecanum_lb_jnt_pub =
      m_node.advertise<std_msgs::Float64>("/omniturtle/mecanum_lb_joint_position_controller/command", 1);

  m_ctrl_sub = m_node.subscribe(subscriberTopicName, 1,
                                &OmniTurtle::OmniTurtleManager::controlCallback, this);

  m_mecanum_rf_jnt_sub = m_node.subscribe("/omniturtle/mecanum_rf_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumRFCallback, this);

  m_mecanum_lf_jnt_sub = m_node.subscribe("/omniturtle/mecanum_lf_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumLFCallback, this);

  m_mecanum_rb_jnt_sub = m_node.subscribe("/omniturtle/mecanum_rb_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumRBCallback, this);

  m_mecanum_lb_jnt_sub = m_node.subscribe("/omniturtle/mecanum_lb_joint_position_controller/state", 1,
                                    &OmniTurtle::OmniTurtleManager::mecanumLBCallback, this);
}

inline bool OmniTurtle::OmniTurtleManager::isRunning() const
{
  return m_isRunning;
}

void OmniTurtle::OmniTurtleManager::sendMessage(const sensor_msgs::JointState &jointState)
{
  std::map<std::string, ros::Publisher>
      jointNameToPub{{"mecanum_rf_joint", m_mecanum_rf_jnt_pub},
                     {"mecanum_lf_joint", m_mecanum_lf_jnt_pub},
                     {"mecanum_rb_joint", m_mecanum_rb_jnt_pub},
                     {"mecanum_lb_joint", m_mecanum_lb_jnt_pub}};

  for (const auto& i : jointNameToPub) {
    std_msgs::Float64 controlMessage;
    controlMessage.data = jointState.position.at(findJoint(jointState, i.first));
    i.second.publish(controlMessage);
  }
}

void OmniTurtle::OmniTurtleManager::controlCallback(const geometry_msgs::Twist& message)
{
  if(m_currentJointState.name.size() != m_mecanumSolver->JOINT_NUM)
    throw std::runtime_error("OmniTurtleManager::controlCallbackError: invalid message length " +
                                  std::to_string(m_currentJointState.name.size()));

  if (message.linear.x>0 && message.linear.y==0 && message.angular.z==0)
    moveForward(message.linear.x / m_frequency * 1000.0); // velocity [m/s] -> distance [mm]
  else if (message.linear.x<0 && message.linear.y==0 && message.angular.z==0)
    moveBackward(-message.linear.x / m_frequency * 1000.0);
  else if (message.linear.x==0 && message.linear.y>0 && message.angular.z==0)
    moveLeft(message.linear.y / m_frequency * 1000.0);
  else if (message.linear.x==0 && message.linear.y<0 && message.angular.z==0)
    moveRight(-message.linear.y / m_frequency * 1000.0);
  else if (message.linear.x>0 && message.linear.y<0 && message.angular.z==0)
    moveDiagRightUp(message.linear.x / m_frequency * 1000.0);
  else if (message.linear.x>0 && message.linear.y>0 && message.angular.z==0)
    moveDiagLeftUp(message.linear.x / m_frequency * 1000.0);
  else if (message.linear.x<0 && message.linear.y<0 && message.angular.z==0)
    moveDiagRightDown(-message.linear.x / m_frequency * 1000.0);
  else if (message.linear.x<0 && message.linear.y>0 && message.angular.z==0)
    moveDiagLeftDown(-message.linear.x / m_frequency * 1000.0);
  else if (message.linear.x==0 && message.linear.y==0 && message.angular.z>0)
    turnAroundAnticlockwise(message.angular.z / m_frequency * 1000.0);
  else if (message.linear.x==0 && message.linear.y==0 && message.angular.z<0)
    turnAroundClockwise(-message.angular.z / m_frequency * 1000.0);
  else if (message.linear.x==0 && message.linear.y==0 && message.angular.z==0)
    stop();
}

void OmniTurtle::OmniTurtleManager::
mecanumRFCallback(const control_msgs::JointControllerState& message)
{
  *(m_jointsAngles["mecanum_rf_joint"]) = message.process_value;
}
void OmniTurtle::OmniTurtleManager::
mecanumLFCallback(const control_msgs::JointControllerState& message)
{
  *(m_jointsAngles["mecanum_lf_joint"]) = message.process_value;
}
void OmniTurtle::OmniTurtleManager::
mecanumRBCallback(const control_msgs::JointControllerState& message)
{
  *(m_jointsAngles["mecanum_rb_joint"]) = message.process_value;
}
void OmniTurtle::OmniTurtleManager::
mecanumLBCallback(const control_msgs::JointControllerState& message)
{
  *(m_jointsAngles["mecanum_lb_joint"]) = message.process_value;
}

void OmniTurtle::OmniTurtleManager::moveForward(double delta)
{
  sendMessage(m_mecanumSolver->moveForward(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveBackward(double delta)
{
  sendMessage(m_mecanumSolver->moveBackward(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveRight(double delta)
{
  sendMessage(m_mecanumSolver->moveRight(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveLeft(double delta)
{
  sendMessage(m_mecanumSolver->moveLeft(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagRightUp(double delta)
{
  sendMessage(m_mecanumSolver->moveDiagRightUp(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagLeftUp(double delta)
{
  sendMessage(m_mecanumSolver->moveDiagLeftUp(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagRightDown(double delta)
{
  sendMessage(m_mecanumSolver->moveDiagRightDown(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::moveDiagLeftDown(double delta)
{
  sendMessage(m_mecanumSolver->moveDiagLeftDown(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::turnAroundClockwise(double delta)
{
  sendMessage(m_mecanumSolver->turnAroundClockwise(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::turnAroundAnticlockwise(double delta)
{
  sendMessage(m_mecanumSolver->turnAroundAnticlockwise(m_currentJointState, delta));
}

void OmniTurtle::OmniTurtleManager::stop()
{
  sendMessage(m_mecanumSolver->stop(m_currentJointState, 0.0));
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"omniturtle_manager");
  ros::NodeHandle n;

  OmniTurtle::OmniTurtleManager otm;
  otm.launch(n, "/omniturtle_control", OmniTurtle::MecanumSolver());

  ros::spin();

  return 0;
}


