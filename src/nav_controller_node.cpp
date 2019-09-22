/***********************************************************************************
 *  Navigation Controller
 *  A ROS node that that controls a robot by sending goals to Ros Navigation Stack
 *  and processing its feedback. Implements ActionLib Client functionality for move_base.
 *
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * ------------------------------
 *  boredman@BoredomProjects.net
 * ------------------------------
 *
 ***********************************************************************************/
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>

#include <sensor_msgs/BatteryState.h>
sensor_msgs::BatteryState _battery_state;

// Callback for receiving BatteryState messages
void batteryStateCb(const sensor_msgs::BatteryState& bat_state)
{
  _battery_state = bat_state;
}

#include <pthread.h>
pthread_t command_thread;

#include "XmlRpcGoal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result);

int _feedback_rate_factor;


void* CommandParser(void* param);

char  _cmd;
int   _goal;
geometry_msgs::Point _move;
bool  _cmd_ready;




/*********************************************************************
 * MAIN
 *********************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_controller");

  // get parameters from private namespace:
  ros::NodeHandle _nh("~");

  XmlRpcGoal goals;

  if (_nh.getParam("goals", goals) == false)
  {
    ROS_ERROR("Unable to get goals parameters");
    exit(1);
  }

  std::string server_name = _nh.param<std::string>("server_name", "move_base");
  ROS_INFO("using action server name: %s", server_name.c_str());

  //no need to spin extra thread, since we will call ros::spinOnce() ourselves
  MoveBaseClient ac(server_name, false);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("...waiting for the action server to come up");
    ros::spinOnce();
    if(!ros::ok())
      exit(2);
  }
  ROS_INFO("action server is up!");

  //spin a thread to parse console commands
  if (pthread_create(&command_thread, NULL, &CommandParser, (void*)&_nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create CommandParser thread");
    exit(3);
  }

  std::string fixed_frame_id = _nh.param<std::string>("fixed_frame_id", "map");
  std::string base_frame_id  = _nh.param<std::string>("base_frame_id", "base_link");
  _feedback_rate_factor = _nh.param<int>("feedback_rate_factor", 10);

  // publish and subscribe under this namespace:
  ros::NodeHandle nh;

  // register callback to receive battery_state messages
  ros::Subscriber sub = nh.subscribe("battery", 10, batteryStateCb);

  move_base_msgs::MoveBaseGoal goal;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
    if(_cmd_ready)
    {
      switch(_cmd)
      {
        case 'g': std::cout << " {goal accepted: " << goals.getName(_goal) << "}" << std::endl;
                  goal.target_pose.header.frame_id = fixed_frame_id;
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose = goals.getPose(_goal);
                  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                  break;
        case 'm': std::cout << " {move accepted}" << std::endl;
                  goal.target_pose.header.frame_id = base_frame_id;
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose.position = _move;
                  goal.target_pose.pose.orientation.x = 0.0;
                  goal.target_pose.pose.orientation.y = 0.0;
                  goal.target_pose.pose.orientation.z = 0.0;
                  goal.target_pose.pose.orientation.w = 1.0;
                  ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                  break;
        case 's': std::cout << " {stop accepted}" << std::endl;
                  ac.cancelAllGoals();
                  break;
        case 'b': std::cout << " {battery is at " << (int)(_battery_state.percentage * 100) << "% ("
                                                  << (int)_battery_state.power_supply_status << ")}" << std::endl;
                  break;
        case 'q': exit(0);
        default : break;
      }
      _cmd_ready = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}




/*********************************************************************
 * ActionLib callbacks
 *********************************************************************/
// Called once when the goal becomes active
void activeCb()
{
  std::cout << " <Goal just went active>" << std::endl;
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback)
{
  static int counter = 0;
  if (counter++ >= _feedback_rate_factor)
  {
    counter = 0;
    std::cout << " <feedback of current position: x=" << feedback->base_position.pose.position.x << 
                                               ", y=" << feedback->base_position.pose.position.y << std::endl;
  }
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  std::cout << " <Finished in state [" << state.toString().c_str() << "]" << std::endl;
  // result seems to be unused in move_base
}




/**************************************************************
 * Thread to parse console command lines
 **************************************************************/
void* CommandParser(void* param)
{
  ROS_DEBUG_STREAM("Command parser thread started");
  ros::NodeHandle *nh = (ros::NodeHandle*)param;

  _cmd_ready = false;
  std::cout << std::endl;

  ros::Rate loop_rate(100);
  while(ros::ok())
  {
    loop_rate.sleep();

    if(_cmd_ready)
      continue;

    if(!(std::cin >> _cmd))
    {
      std::cin.clear();
      std::cin.ignore();
      std::cout << " <command error>" << std::endl;
      continue;
    }

    switch(_cmd)
    {
      case 'g': if(!(std::cin >> _goal))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <goal number error>" << std::endl;
                  break;
                }
                _cmd_ready = true;
                break;

      case 's': _cmd_ready = true;
                break;

      case 'm': float shift;
                if(!(std::cin >> shift))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <move shift error>" << std::endl;
                  break;
                }
                char dir;
                if(!(std::cin >> dir))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <move direction type error>" << std::endl;
                  break;
                }
                if(dir == 'f')
                {
                  _move.x = shift;
                  _move.y = 0.0;
                }
                else if(dir == 'b')
                {
                  _move.x = -shift;
                  _move.y = 0.0;
                }
                else if(dir == 'l')
                {
                  _move.x = 0.0;
                  _move.y = shift;
                }
                else if(dir == 'r')
                {
                  _move.x = 0.0;
                  _move.y = -shift;
                }
                else
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <move direction list error>" << std::endl;
                  break;
                }
                _move.z = 0.0;
                _cmd_ready = true;
                break;

      case 'b': 
                _cmd_ready = true;
                break;

      case 'q': _cmd_ready = true;
                break;

      case 'h': std::cout << " |commands: g - goal [number]" << std::endl;
                std::cout << " |          s - stop" << std::endl;
                std::cout << " |          m - move [meters] [direction (f, b, l or r)]" << std::endl;
                std::cout << " |          b - battery" << std::endl;
                std::cout << " |          h - help" << std::endl;
                std::cout << " |          q - quit" << std::endl;
                break;

      default : std::cout << " <unknown command>" << std::endl;
                break;
    }
  }

}

