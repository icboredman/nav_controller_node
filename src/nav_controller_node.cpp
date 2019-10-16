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

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <pocketsphinx.h>

#include <sensor_msgs/BatteryState.h>
sensor_msgs::BatteryState _battery_state;

// Callback for receiving BatteryState messages
void batteryStateCb(const sensor_msgs::BatteryState& bat_state)
{
  _battery_state = bat_state;
}

#include <pthread.h>
pthread_t console_cmd_thread, voice_cmd_thread;

void* ConsoleCmdParser(void* param);
void* VoiceCmdParser(void* param);


#include "XmlRpcGoal.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result);

int _feedback_rate_factor;


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

//-------------
  //spin a thread to process voice commands
  if (pthread_create(&voice_cmd_thread, NULL, &VoiceCmdParser, (void*)&_nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create ConsoleCmdParser thread");
    exit(3);
  }

//-------------

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
  if (pthread_create(&console_cmd_thread, NULL, &ConsoleCmdParser, (void*)&_nh) != 0)
  {
    ROS_ERROR_STREAM("Couldn't create ConsoleCmdParser thread");
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
void* ConsoleCmdParser(void* param)
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

    // block while waiting for input
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




/**************************************************************
 * Thread to process voice commands
 **************************************************************/
void* VoiceCmdParser(void* param)
{
  ROS_DEBUG_STREAM("Voice processing thread started");
  ros::NodeHandle *nh = (ros::NodeHandle*)param;

  ps_decoder_t *ps = NULL;
  cmd_ln_t *config = NULL;
  ad_rec_t *ad = NULL;
  int16 adbuf[2048];
  uint8 utt_started, in_speech;
  int32 k;
  char const *hyp;

  config = cmd_ln_init(NULL, ps_args(), TRUE,
                       "-hmm",  nh->param<std::string>("model_path", "").c_str(),
                       "-dict", nh->param<std::string>("dictionary_file", "").c_str(),
                       "-logfn", "/dev/null",
                       NULL);

//  config = cmd_ln_parse_r(NULL, cont_args_def, argc, argv, FALSE);
//  cmd_ln_set_str_r(config, "-adcdev", nh->param<std::string>("voice_input_dev", "default:CARD=Device").c_str());
//  cmd_ln_set_str_r(config, "-dict", nh->param<std::string>("dictionary_file", "file.dic").c_str());
//  cmd_ln_set_float_r(config, "-kws_threshold", nh->param<double>("kws_threshold", 1e-50));

  if ((ps = ps_init(config)) == NULL)
  {
    cmd_ln_free_r(config);
    ROS_ERROR_STREAM("ps_init failed");
    cmd_ln_free_r(config);
    return NULL;
  }

  std::string my_name = nh->param<std::string>("kws_name", "robot");
  ps_set_keyphrase(ps, "kws", my_name.c_str());

  ps_set_jsgf_file(ps, "jsgf", nh->param<std::string>("grammar_file", "file.jsgf").c_str());
  ps_set_fsg(ps, "fsg", ps_get_fsg(ps, "jsgf"));  // define grammar

//  ps_set_search(ps, "kws");
  ps_set_search(ps, "fsg");

  if ((ad = ad_open_dev(nh->param<std::string>("voice_input_dev", "sysdefault").c_str(),
                        (int)cmd_ln_float32_r(config,"-samprate"))) == NULL)
    ROS_ERROR_STREAM("Failed to open audio device");
  if (ad_start_rec(ad) < 0)
    ROS_ERROR_STREAM("Failed to start recording");
  if (ps_start_utt(ps) < 0)
    ROS_ERROR_STREAM("Failed to start utterance");

  utt_started = FALSE;
  ROS_INFO_STREAM("Ready....");

  _cmd_ready = false;
  std::cout << std::endl;

  while(ros::ok())
  {
    if(_cmd_ready)
      continue;

    if ((k = ad_read(ad, adbuf, 2048)) < 0)
      ROS_ERROR_STREAM("Failed to read audio");
    ps_process_raw(ps, adbuf, k, FALSE, FALSE);
    in_speech = ps_get_in_speech(ps);
    if (in_speech && !utt_started)
    {
      utt_started = TRUE;
      ROS_INFO_STREAM("Listening...");
    }
    if (!in_speech && utt_started)
    { // speech -> silence transition, time to start new utterance
      ps_end_utt(ps);
      hyp = ps_get_hyp(ps, NULL);
      if (hyp != NULL)
      {
        std::cout << hyp << std::endl;
      }

      if (ps_start_utt(ps) < 0)
        ROS_ERROR_STREAM("Failed to start utterance");
      utt_started = FALSE;
      ROS_INFO_STREAM("Ready....");
    }

    ros::Duration(0.1).sleep();
  }

  ad_close(ad);
  ps_free(ps);
  cmd_ln_free_r(config);
}


/*
static const arg_t cont_args_def[] = {
    POCKETSPHINX_OPTIONS,
    { "-adcdev",
      ARG_STRING,
      "default:CARD=Device",
      "Name of audio device to use for input."},
//    { "-lm",
//      ARG_STRING,
//      "",
//      "Word trigram language model input file"},
    { "-alignctl",	// only here to supress error message
      ARG_STRING,
      NULL,
      "Control file listing transcript files to force-align to utts" },
    CMDLN_EMPTY_OPTION
};

    ps_decoder_t *ps = NULL;
    cmd_ln_t *config = NULL;

    config = cmd_ln_parse_r(NULL, cont_args_def, 0, NULL, TRUE);
//    ROS_INFO("hmm: %s", (char*)(cmd_ln_access_r(config, "-hmm")->ptr));
//    cmd_ln_set_str_r(config, "-adcdev", "default:CARD=Device");
    ps_default_search_args(config);
    err_set_logfp(NULL);
    if ((ps = ps_init(config)) == NULL)
    {
      cmd_ln_free_r(config);
      return 1;
    }
//    ROS_INFO("dev: %s", (char*)cmd_ln_str_r(config, "-adcdev"));

    ps_set_keyphrase(ps, "kws", "heisenberg");
    ps_set_search(ps, "kws");

*/

