/*******************************************************************************
 * Copyright (c) 2016, ROBOTIS CO., LTD.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of ROBOTIS nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*
 * sound_play.cpp
 *
 *  Created on: 2016. 8. 11.
 *      Author: zerom
 */

#include "ros_mpg321_player/ros_mpg321_player.h"

pid_t       g_play_pid = -1;
std::string g_sound_file_path = "";
ros::Publisher g_done_msg_pub;

void play_sound_callback(const std_msgs::String::ConstPtr& msg)
{
  std_msgs::String done_msg;

  if(msg->data == "")
  {
    if(g_play_pid != -1)
      kill(g_play_pid, SIGKILL);

    g_play_pid = -1;
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    return;
  }

  if(g_play_pid != -1)
    kill(g_play_pid, SIGKILL);

  g_play_pid = fork();

  switch(g_play_pid)
  {
  case -1:
    fprintf(stderr, "Fork Failed!! \n");
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    break;
  case 0:
    execl("/usr/bin/mpg321", "mpg321", (g_sound_file_path + msg->data).c_str(), "-q", (char*)0);
    done_msg.data = "play_sound";
    g_done_msg_pub.publish(done_msg);
    break;
  default:
    break;
  }

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sound_play");
  ros::NodeHandle nh;

  g_sound_file_path = nh.param<std::string>("sound_file_path", "");
  if(g_sound_file_path != "" && g_sound_file_path.compare(g_sound_file_path.size()-1, 1, "/") != 0)
    g_sound_file_path += "/";

  ros::Subscriber play_mp3_sub = nh.subscribe("/play_sound_file", 10, &play_sound_callback);
  g_done_msg_pub = nh.advertise<std_msgs::String>("/robotis/movement_done", 5);

  ros::spin();
  return 0;
}



