#include"../include/keyboard_controller.h"

OmniTurtle::KeyboardController::KeyboardController(ros::NodeHandle& node,
                                                std::string pub_topic_name) :
                             n(&node), keyboard_pub(), ctrl_msg(), is_running(true)
{
  if(&node == nullptr)
    throw std::invalid_argument("KeyboardControllerError: invalid node");

  keyboard_pub = n->advertise<geometry_msgs::Twist>(pub_topic_name, 1);

  /// NCurses block
  if(initscr() == nullptr) // Init new screen
    throw std::runtime_error("KeyboardControllerError: unable to launch NCurses screen");

  if(clear() == ERR)   // Clear screen
    throw std::runtime_error("KeyboardControllerError: unable to clear NCurses screen");

  noecho();  // Don't display input characters
  raw();     // Raw input without confirmation thru Enter key

}

void OmniTurtle::KeyboardController::getInput()
{
  if(!is_running)
    return;

  // Curses library code
  if(clear() == ERR)
    throw std::runtime_error("KeyboardControllerError: unable to clear NCurses screen");
  printw("***OmniTurtle KEYBOARD CONTROL NODE***\n\n");
  printw("Rollover is not enable!\n");
  printw("Use 'WASD' or 8/4/2/6 to move forward/left/backward/right\n");
  printw("Use Q/E/Z/C or 7/9/1/3 to move diagonally\n");
  printw("Use O/P to turn around clockwise/anticlockwise\n");
  printw("Use SPACE to stop\n");
  printw("\nPress Ctrl+C to quit\n");

  ctrl_msg.linear.x = 0.0;
  ctrl_msg.linear.y = 0.0;
  ctrl_msg.linear.z = 0.0;

  ctrl_msg.angular.x = 0.0;
  ctrl_msg.angular.y = 0.0;
  ctrl_msg.angular.z = 0.0;

  int in_key = wgetch(stdscr);
  if(in_key == ERR)
    throw std::runtime_error("KeyboardControllerError: input error");

  switch (in_key)
  {
    case QL:
    case QU:
    case SEVEN:
      ctrl_msg.linear.x = 1.0;
      ctrl_msg.linear.y = 1.0;
      break;

    case WL:
    case WU:
    case EIGHT:
      ctrl_msg.linear.x = 1.0;
      break;

    case EL:
    case EU:
    case NINE:
      ctrl_msg.linear.x = 1.0;
      ctrl_msg.linear.y = -1.0;
      break;

    case AL:
    case AU:
    case FOUR:
      ctrl_msg.linear.y = 1.0;
      break;

    case SL:
    case SU:
    case TWO:
      ctrl_msg.linear.x = -1.0;
      break;

    case DL:
    case DU:
    case SIX:
     ctrl_msg.linear.y = -1.0;
      break;

    case ZL:
    case ZU:
    case ONE:
      ctrl_msg.linear.x = -1.0;
      ctrl_msg.linear.y = 1.0;
      break;

    case CL:
    case CU:
    case THREE:
      ctrl_msg.linear.x = -1.0;
      ctrl_msg.linear.y = -1.0;
      break;

    case OL:
    case OU:
      ctrl_msg.angular.z = 1.0;
      break;

    case PL:
    case PU:
      ctrl_msg.angular.z = -1.0;
      break;

    case CTRLC:
      is_running = false;
  }
}

void OmniTurtle::KeyboardController::sendMessage() const
{
  keyboard_pub.publish(ctrl_msg);
}

void OmniTurtle::KeyboardController::quit() const
{
  noraw();
  cbreak();
  refresh();

  if(endwin() == ERR) // Quit curses mode
    throw std::runtime_error("KeyboardControllerError: unable to quit NCurses screen");
}

int main(int argc, char** argv)
{
  ros::init(argc,argv,"obot_keyboard_control");
  ros::NodeHandle n;

  OmniTurtle::KeyboardController kc(n, "obot_keyboard_control");
  ros::Rate rate(200); //Hz
  while(ros::ok() && kc.isRunning())
  {
    kc.getInput();
    kc.sendMessage();

    ros::spinOnce();
    rate.sleep();
  }

  kc.quit();

  return 0;
}
