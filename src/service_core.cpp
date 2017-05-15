#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/PadOrder.h"
#include "turtlebot3_carrier_icra2017/AvailableItemList.h"
#include "turtlebot3_carrier_icra2017/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"

ros::Publisher pub_pad_order_tb3g;
ros::Publisher pub_pad_order_tb3r;
ros::Publisher pub_pad_order_tb3p;
ros::Publisher pub_is_item_available;

ros::Publisher pub_play_sound;

ros::Publisher pub_poseStamped_tb3g;
ros::Publisher pub_poseStamped_tb3r;
ros::Publisher pub_poseStamped_tb3p;

ros::Subscriber sub_pad_order_tb3g;
ros::Subscriber sub_pad_order_tb3r;
ros::Subscriber sub_pad_order_tb3p;

ros::Subscriber sub_arrival_status;


int chosen_item_num[3] = {0, };
bool is_item_available[3] = {true, };

int service_sequence[3] = {0, };

geometry_msgs::PoseStamped poseStamped[2];


void fnPublishVoiceFilePath(const char* file_path)
{
  std_msgs::String str;

  str.data = file_path;

  pub_play_sound.publish(str);
}

void fnPublishGoalPoseTB3G()
{
  poseStamped[0].header.frame_id = "map";
  poseStamped[0].header.stamp = ros::Time::now();
  poseStamped[0].pose.position.x = -0.629751861095;
  poseStamped[0].pose.position.y = 0.014581086114;
  poseStamped[0].pose.position.z = 0.0;

  poseStamped[0].pose.orientation.x = 0.0;
  poseStamped[0].pose.orientation.y = 0.0;
  poseStamped[0].pose.orientation.z = 0.08531826008;
  poseStamped[0].pose.orientation.w = 0.996353749678;


  // counter
  poseStamped[1].header.frame_id = "map";
  poseStamped[1].header.stamp = ros::Time::now();
  poseStamped[1].pose.position.x = -1.94660317898;
  poseStamped[1].pose.position.y = -0.418711960316;
  poseStamped[1].pose.position.z = 0.0;

  poseStamped[1].pose.orientation.x = 0.0;
  poseStamped[1].pose.orientation.y = 0.0;
  poseStamped[1].pose.orientation.z = 0.998491629827;
  poseStamped[1].pose.orientation.w = -0.0549041452392;


  if (service_sequence[0] == 0)
  {
    ROS_INFO("Going to get item");

    pub_poseStamped_tb3g.publish(poseStamped[1]);

    service_sequence[0] = 1;

    fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");
  }
  else if (service_sequence[0] == 1)
  {
    ROS_INFO("Coming back with item");

    pub_poseStamped_tb3g.publish(poseStamped[0]);

    turtlebot3_carrier_icra2017::AvailableItemList availableItemList;

    is_item_available[chosen_item_num[0] - 1] = true;

    availableItemList.item_number = chosen_item_num[0];
    availableItemList.is_item_available = is_item_available[chosen_item_num[0] - 1];

    pub_is_item_available.publish(availableItemList);

    chosen_item_num[0] = 0;

    service_sequence[0] = 2;

    fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-3.mp3");
  }
  else if (service_sequence[0] == 2)
  {
    fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-4.mp3");
  }
}

void cbCheckArrivalStatusTB3G(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
{
  if (rcvMoveBaseActionResult.status.status == 3)
  {
    fnPublishGoalPoseTB3G();
  }
  else
  {
    ROS_INFO("rcvMoveBaseActionResult.status.status : %d", rcvMoveBaseActionResult.status.status);
  }
}

void cbControlServiceStatus(const turtlebot3_carrier_icra2017::PadOrder rcvPadOrder)
{
  if (chosen_item_num[rcvPadOrder.pad_number - 1] != 0)
  {
    ROS_INFO("Your TurtleBot is currently moving");
    return;
  }

  if (!is_item_available[rcvPadOrder.item_number - 1])
  {
    ROS_INFO("Chosen item is currently unavailable");
    return;
  }

  chosen_item_num[rcvPadOrder.pad_number - 1] = rcvPadOrder.item_number;
  is_item_available[rcvPadOrder.item_number - 1] = false;

  ROS_INFO("Orderred by Pad No. %d", rcvPadOrder.pad_number);
  ROS_INFO("Chosen Item No. %d", chosen_item_num[rcvPadOrder.pad_number - 1]);

  if (service_sequence[0] == 2)
    service_sequence[0] = 0;

  fnPublishGoalPoseTB3G();

  turtlebot3_carrier_icra2017::AvailableItemList availableItemList;

  is_item_available[chosen_item_num[rcvPadOrder.pad_number - 1]] = false;

  availableItemList.item_number = chosen_item_num[rcvPadOrder.pad_number- 1];
  availableItemList.is_item_available = is_item_available[chosen_item_num[rcvPadOrder.pad_number - 1] - 1];

  pub_is_item_available.publish(availableItemList);

  fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");

  // chosen_item_num[0] = 0;
  //
  //
  // availableItemList.item_number = rcvPadOrder.item_number;
  // availableItemList.is_item_available = false;
  //
  // pub_is_item_available.publish(availableItemList);


  // ROS_INFO("sound published");

  // ROS_INFO("availableItemList.item_number = %d // availableItemList.is_item_available = %d", availableItemList.item_number, availableItemList.is_item_available);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_core");


  ros::NodeHandle n;

  pub_pad_order_tb3g = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3g/pad_order_echo", 100);
  pub_pad_order_tb3r = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3r/pad_order_echo", 100);
  pub_pad_order_tb3p = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3p/pad_order_echo", 100);

  pub_is_item_available = n.advertise<turtlebot3_carrier_icra2017::AvailableItemList>("/is_item_available", 100);

  pub_play_sound = n.advertise<std_msgs::String>("/play_sound_file", 100);

  pub_poseStamped_tb3g = n.advertise<geometry_msgs::PoseStamped>("/tb3g/move_base_simple/goal", 1000);
  pub_poseStamped_tb3r = n.advertise<geometry_msgs::PoseStamped>("/tb3r/move_base_simple/goal", 1000);
  pub_poseStamped_tb3p = n.advertise<geometry_msgs::PoseStamped>("/tb3p/move_base_simple/goal", 1000);

  sub_pad_order_tb3g = n.subscribe("/tb3g/pad_order", 100, cbControlServiceStatus);
  sub_pad_order_tb3r = n.subscribe("/tb3r/pad_order", 100, cbControlServiceStatus);
  sub_pad_order_tb3p = n.subscribe("/tb3p/pad_order", 100, cbControlServiceStatus);

  sub_arrival_status = n.subscribe("/tb3g/move_base/result", 100, cbCheckArrivalStatusTB3G);

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
