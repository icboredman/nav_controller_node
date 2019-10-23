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
#include <iostream>       // std::cout
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread>         // std::thread, std::this_thread::yield
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

#include <sphinxbase/err.h>
#include <sphinxbase/ad.h>
#include <pocketsphinx.h>

#include "XmlRpcGoal.h"

#include <sensor_msgs/BatteryState.h>
sensor_msgs::BatteryState battery_state_;

// Callback for receiving BatteryState messages
void batteryStateCb(const sensor_msgs::BatteryState& bat_state)
{
  battery_state_ = bat_state;
}

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void activeCb();
void feedbackCb(const move_base_msgs::MoveBaseFeedback::ConstPtr& feedback);
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result);

int feedback_rate_factor_;


void ConsoleCmdParser(ros::NodeHandle& nh);
void VoiceCmdParser(ros::NodeHandle& nh);

std::atomic_bool stop_threads_(false);
std::atomic_bool cmd_ready_(false);
std::atomic_char cmd_;
geometry_msgs::Pose target_pose_;
std::string target_name_;




/*********************************************************************
 * MAIN
 *********************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "nav_controller_node");

  // get parameters from private namespace:
  ros::NodeHandle _nh("~");

  std::string server_name = _nh.param<std::string>("actionlib_server_name", "move_base");
  ROS_INFO("using action server name: %s", server_name.c_str());

  std::string fixed_frame_id = _nh.param<std::string>("fixed_frame_id", "map");
  std::string base_frame_id  = _nh.param<std::string>("base_frame_id", "base_link");
  feedback_rate_factor_ = _nh.param<int>("feedback_rate_factor", 10);

  // publish and subscribe under this namespace:
  ros::NodeHandle nh;

  // register callback to receive battery_state messages
  ros::Subscriber sub = nh.subscribe("battery", 10, batteryStateCb);

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
  std::thread console_cmd_thread (ConsoleCmdParser, std::ref(_nh));
  //spin a thread to process voice commands
  std::thread voice_cmd_thread (VoiceCmdParser, std::ref(_nh));

  move_base_msgs::MoveBaseGoal goal;

  ros::Rate loop_rate(10);
  while(ros::ok() && !stop_threads_)
  {
    if(cmd_ready_)
    {
      switch(cmd_)
      {
        case 'g': std::cout << " {goal accepted: " << target_name_ << "}" << std::endl;
                  goal.target_pose.header.frame_id = fixed_frame_id;
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose = target_pose_;
                  //ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                  break;
        case 't':
        case 'm': std::cout << " {move accepted: " << target_name_ << "}" << std::endl;
                  goal.target_pose.header.frame_id = base_frame_id;
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose = target_pose_;
                  //ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
                  break;
        case 's': std::cout << " {stop accepted}" << std::endl;
                  ac.cancelAllGoals();
                  break;
        case 'b': std::cout << " {battery is at " << (int)(battery_state_.percentage * 100) << "% ("
                                                  << (int)battery_state_.power_supply_status << ")}" << std::endl;
                  break;
        case 'q': stop_threads_ = true;
                  console_cmd_thread.join();
                  voice_cmd_thread.join();
                  return 0;
        case '?':
        default : break;
      }
      cmd_ready_ = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  stop_threads_ = true;
  console_cmd_thread.join();
  voice_cmd_thread.join();
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
  if (counter++ >= feedback_rate_factor_)
  {
    counter = 0;
    std::cout << " <feedback of current position: x=" << feedback->base_position.pose.position.x <<
                                               ", y=" << feedback->base_position.pose.position.y <<
                                                  ">" << std::endl;
  }
}

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResult::ConstPtr& result)
{
  std::cout << " <Finished in state [" << state.toString().c_str() << "]>" << std::endl;
  // result seems to be unused in move_base
}




/**************************************************************
 * Thread to parse console command lines
 **************************************************************/
void ConsoleCmdParser(ros::NodeHandle& nh)
{
  ROS_DEBUG_STREAM("Console parser thread started");

  XmlRpcGoal goals, moves, turns, numbers;
  if (nh.getParam("goals", goals) == false)
  {
    ROS_ERROR("Unable to read goals parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("turns", turns) == false)
  {
    ROS_ERROR("Unable to read turns parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("moves", moves) == false)
  {
    ROS_ERROR("Unable to read moves parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("numbers", numbers) == false)
  {
    ROS_ERROR("Unable to read numbers parameters from yaml");
    stop_threads_ = true;
    return;
  }

  std::cout << std::endl;

  ros::Rate loop_rate(100);
  while(ros::ok() && !stop_threads_)
  {
    loop_rate.sleep();

    if(cmd_ready_)
      continue;

//    // check for presence of data in stdin
//    std::cin.seekg(0, std::cin.end);
//    int length = std::cin.tellg();
//    std::cin.seekg(0, std::cin.beg);
//    if(length < 0)
//      continue;

    char cmd;
    // block while waiting for input
    if(!(std::cin >> cmd))
    {
      std::cin.clear();
      std::cin.ignore();
      std::cout << " <command error>" << std::endl;
      continue;
    }

    char p1, p2;
    double shift;
    switch(cmd)
    {
      case 'g': if(!(std::cin >> p1))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <goal number error>" << std::endl;
                  break;
                }
                if (goals.checkPose(p1))
                {
                  target_pose_ = goals.getPose(p1);
                  target_name_ = goals.getName(p1);
                  cmd_ = cmd;
                  cmd_ready_ = true;
                }
                else
                  std::cout << " <goal id '" << p1 << "' doesn't exist>" << std::endl;
                break;

      case 'm': if(!(std::cin >> p1))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <move shift error>" << std::endl;
                  break;
                }
                shift = numbers.getVal(p1);
                if(!(std::cin >> p2))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <move direction error>" << std::endl;
                  break;
                }
                if (moves.checkPose(p2))
                {
                  target_pose_ = moves.getPose(p2);
                  target_name_ = moves.getName(p2);
                  target_pose_.position.x *= shift;
                  target_pose_.position.y *= shift;
                  cmd_ = cmd;
                  cmd_ready_ = true;
                }
                else
                  std::cout << " <move id '" << p2 << "' doesn't exist>" << std::endl;
                break;

      case 't': if(!(std::cin >> p1))
                {
                  std::cin.clear();
                  std::cin.ignore();
                  std::cout << " <turn direction error>" << std::endl;
                  break;
                }
                if (turns.checkPose(p1))
                {
                  target_pose_ = turns.getPose(p1);
                  target_name_ = turns.getName(p1);
                  cmd_ = cmd;
                  cmd_ready_ = true;
                }
                else
                  std::cout << " <turn id '" << p1 << "' doesn't exist>" << std::endl;
                break;

      case 's':
      case 'b':
      case 'q': cmd_ = cmd;
                cmd_ready_ = true;
                break;

      case 'h': std::cout << " |commands: g - goal [number]" << std::endl;
                std::cout << " |          m - move [meters] [direction (f, b, l or r)]" << std::endl;
                std::cout << " |          t - turn [direction (a, l or r)]" << std::endl;
                std::cout << " |          s - stop" << std::endl;
                std::cout << " |          b - battery" << std::endl;
                std::cout << " |          h - help" << std::endl;
                std::cout << " |          q - quit" << std::endl;
                break;

      default : std::cout << " <unknown command>" << std::endl;
                break;
    }
  }
  ROS_DEBUG_STREAM("Console parser thread received command to terminate");
}




/**************************************************************
 * Thread to process voice commands
 **************************************************************/
void VoiceCmdParser(ros::NodeHandle& nh)
{
  ROS_DEBUG_STREAM("Voice processing thread started");

  XmlRpcGoal commands, goals, moves, turns, numbers;
  if (nh.getParam("commands", commands) == false)
  {
    ROS_ERROR("Unable to read commands parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("goals", goals) == false)
  {
    ROS_ERROR("Unable to read goals parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("turns", turns) == false)
  {
    ROS_ERROR("Unable to read turns parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("moves", moves) == false)
  {
    ROS_ERROR("Unable to read moves parameters from yaml");
    stop_threads_ = true;
    return;
  }
  if (nh.getParam("numbers", numbers) == false)
  {
    ROS_ERROR("Unable to read numbers parameters from yaml");
    stop_threads_ = true;
    return;
  }

  ps_decoder_t *ps = NULL;
  cmd_ln_t *config = NULL;
  ad_rec_t *ad = NULL;
  int16 adbuf[2048];
  uint8 utt_started, in_speech;
  int32 k;
  char const *hyp;

  config = cmd_ln_init(NULL, ps_args(), TRUE,
                       "-hmm",  nh.param<std::string>("model_path", "").c_str(),
                       "-dict", nh.param<std::string>("dictionary_file", "").c_str(),
                       "-logfn", "/dev/null",
                       NULL);

  if ((ps = ps_init(config)) == NULL)
  {
    cmd_ln_free_r(config);
    ROS_ERROR_STREAM("ps_init failed");
    cmd_ln_free_r(config);
    stop_threads_ = true;
    return;
  }

  std::string my_name = nh.param<std::string>("kws_name", "robot");
  ps_set_keyphrase(ps, "kws", my_name.c_str());

  ps_set_jsgf_file(ps, "jsgf", nh.param<std::string>("grammar_file", "file.jsgf").c_str());
  ps_set_fsg(ps, "fsg", ps_get_fsg(ps, "jsgf"));  // define grammar

  ps_set_search(ps, "kws");
  bool kws = true;
  //ps_set_search(ps, "fsg");

  if ((ad = ad_open_dev(nh.param<std::string>("voice_input_dev", "sysdefault").c_str(),
                        (int)cmd_ln_float32_r(config,"-samprate"))) == NULL)
  {
    ROS_ERROR_STREAM("Failed to open audio device");
    stop_threads_ = true;
    return;
  }
  if (ad_start_rec(ad) < 0)
  {
    ROS_ERROR_STREAM("Failed to start recording");
    stop_threads_ = true;
    return;
  }
  if (ps_start_utt(ps) < 0)
  {
    ROS_ERROR_STREAM("Failed to start utterance");
    stop_threads_ = true;
    return;
  }

  utt_started = FALSE;
  ROS_INFO_STREAM("Sphinx Ready....");

  std::cout << std::endl;

  int silence_counter = 0;

  while(ros::ok() && !stop_threads_)
  {
    ros::Duration(0.1).sleep();

    if(cmd_ready_)
      continue;

    // read 128 ms of audio at a time
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
      { // we have a hypothesis
        ROS_INFO("Hypothesis: %s", hyp);
        if (kws && strstr(hyp, my_name.c_str()) != NULL)
        {
          ps_set_search(ps, "fsg");
          ROS_DEBUG_STREAM("Switched to FSG mode");
          kws = false;
        }
        else if (!kws)
        { // check each action
          char *hyp_copy = strdup(hyp);   // must use free() at the end
          char *token = strtok(hyp_copy, " ");

          if (commands.getName('g').compare(token) == 0)
          { // found 'GO', now locate last token in hyp_copy -> destination
            char *token_last = NULL;
            while ((token = strtok(NULL, " ")) != NULL)
              token_last = token;
            // find id of that destination
            char p1 = goals.getId(token_last);
            // get pose for this id
            if (goals.checkPose(p1))
            {
              target_pose_ = goals.getPose(p1);
              target_name_ = goals.getName(p1);
              cmd_ = 'g';
              cmd_ready_ = true;
            }
            else
              ROS_ERROR("goal id '%c' doesn't exist", p1);
          }

          else if (commands.getName('m').compare(token) == 0)
          { // found 'MOVE', now find next token -> distance
            token = strtok(NULL, " ");
            char p1 = numbers.getId(token);
            double shift = numbers.getVal(p1);
            // find last token -> direction
            char *token_last = NULL;
            while ((token = strtok(NULL, " ")) != NULL)
              token_last = token;
            char p2 = moves.getId(token_last);
            if (moves.checkPose(p2))
            {
              target_pose_ = moves.getPose(p2);
              target_name_ = moves.getName(p2);
              target_pose_.position.x *= shift;
              target_pose_.position.y *= shift;
              cmd_ = 'm';
              cmd_ready_ = true;
            }
            else
              ROS_ERROR("move id '%c' doesn't exist", p2);
          }

          else if (commands.getName('t').compare(token) == 0)
          { // found 'TURN', now find next (and last) token -> direction
            token = strtok(NULL, " ");
            char p1 = turns.getId(token);
            if (turns.checkPose(p1))
            {
              target_pose_ = turns.getPose(p1);
              target_name_ = turns.getName(p1);
              cmd_ = 't';
              cmd_ready_ = true;
            }
            else
              ROS_ERROR("turn id '%c' doesn't exist", p1);
          }

          else if (commands.getName('s').compare(token) == 0)
          { // found 'STOP'
            cmd_ = 's';
            cmd_ready_ = true;
          }

          else if (commands.getName('b').compare(token) == 0)
          { // found 'BATTERY'
            cmd_ = 'b';
            cmd_ready_ = true;
          }

          else
          { // WTF?
            ROS_DEBUG("unrecognized command: %s", token);
            cmd_ = '?';
            cmd_ready_ = true;
          }

          free(hyp_copy);

          ps_set_search(ps, "kws");
          ROS_DEBUG_STREAM("Switched back to KWS mode");
          kws = true;
        }
      }
      if (ps_start_utt(ps) < 0)
        ROS_ERROR_STREAM("Failed to start utterance");
      utt_started = FALSE;
      ROS_DEBUG_STREAM("Ready....");
      silence_counter = 0;
    }
    if (!in_speech && !utt_started && !kws)
    {
      if (++silence_counter > 30)   // approx. 3 sec
      {
        silence_counter = 0;
        ps_end_utt(ps);
        ps_set_search(ps, "kws");
        ROS_DEBUG_STREAM("Timeout. Switched back to KWS mode");
        kws = true;
        if (ps_start_utt(ps) < 0)
          ROS_ERROR_STREAM("Failed to start utterance");
        utt_started = FALSE;
        ROS_DEBUG_STREAM("Ready....");
      }
    }
  }

  ad_close(ad);
  ps_free(ps);
  cmd_ln_free_r(config);
  ROS_DEBUG_STREAM("Voice processing thread received command to terminate");
}

