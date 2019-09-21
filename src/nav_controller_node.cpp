#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <pthread.h>
pthread_t command_thread;

using namespace std;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


void doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResult::ConstPtr& result);
void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);


void* CommandParser(void* param);

//typedef enum {cmd_none, cmd_goto, cmd_move, cmd_stop, cmd_batt} t_command;
char  _cmd;
int   _goal;
float _shift;
char  _dir;
bool  _cmd_ready;




/*********************************************************************
 * MAIN
 *********************************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "nav_controller");

  // get parameters from private namespace:
  ros::NodeHandle _nh("~");
//  _nh.param("usb_hwid", usb_hwid, (std::string)"16c0:0483");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("...waiting for the move_base action server to come up");
    if(!ros::ok())
      exit(1);
  }
  ROS_INFO("move_base action server is up");

  //spin a thread to parse console commands
  if (pthread_create(&command_thread, NULL, &CommandParser, (void*)&_nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create CommandParser thread");
    exit(2);
  }

  // publish and subscribe under this namespace:
  ros::NodeHandle nh;


  while(ros::ok())
  {
    if(_cmd_ready)
    {
      switch(_cmd)
      {
        case 'g' : 
                   cout << "{goal accepted}" << endl;
                   break;
        case 'm' : 
                   cout << "{move accepted}" << endl;
                   break;
        case 's' : 
                   cout << "{stop accepted}" << endl;
                   break;
        case 'b' : 
                   cout << "{batt accepted}" << endl;
                   break;
        case 'q' : exit(0);
        default  : break;
      }
      _cmd_ready = false;
    }

    ros::spinOnce();
  }

/*
  move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved 1 meter forward");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
*/

  return 0;
}







/*********************************************************************
 * ActionLib callbacks
 *********************************************************************/
// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
//  ROS_INFO("Answer: %i", result->sequence.back());
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
//  ROS_INFO("Got Feedback of length %lu", feedback->sequence.size());
}





/**************************************************************
 * Thread to parse console command lines
 **************************************************************/
void* CommandParser(void* param)
{
  ROS_DEBUG_STREAM("Command parser thread started");
//  ros::NodeHandle *nh = (ros::NodeHandle*)param;

  _cmd_ready = false;
  cout << endl;

  while(ros::ok())
  {
    if(_cmd_ready)
    {
      ros::Duration(0.1).sleep();
      continue;
    }

    if(!(cin >> _cmd))
    {
      cin.clear();
      cin.ignore();
      cout << " <command error>" << endl;
      _cmd = 0;
    }

    switch(_cmd)
    {
      case 'g' : if(!(cin >> _goal))
                 {
                   cin.clear();
                   cin.ignore();
                   cout << " <goal number error>" << endl;
                   _cmd = 0;
                 }
                 break;
      case 's' : break;
      case 'm' : if(!(cin >> _shift))
                 {
                   cin.clear();
                   cin.ignore();
                   cout << " <move shift error>" << endl;
                   _cmd = 0;
                 }
                 if(!(cin >> _dir))
                 {
                   cin.clear();
                   cin.ignore();
                   cout << " <move axis error>" << endl;
                   _cmd = 0;
                 }
                 break;
      case 'b' : break;
      case 'q' : break;
      case 'h' : cout << " |commands: g - goal [number]" << endl;
                 cout << " |          s - stop" << endl;
                 cout << " |          m - move [meters] [axis (x or y)]" << endl;
                 cout << " |          b - battery" << endl;
                 cout << " |          h - help" << endl;
                 cout << " |          q - quit" << endl;
                 _cmd = 0;
                 break;
      default  : cout << " <unknown command>" << endl;
                 _cmd = 0;
                 break;
    }
    _cmd_ready = true;
  }

}

