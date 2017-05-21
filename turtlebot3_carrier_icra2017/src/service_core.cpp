#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/PadOrder.h"
#include "turtlebot3_carrier_icra2017/AvailableItemList.h"
#include "turtlebot3_carrier_icra2017/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"

#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/InitTurtlebotPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"

#define MSE_IS_POSE_INITIALIZING_COMPLETED 15000.0

class ServiceCore
{
public:
  ServiceCore()
  {
    nh_.getParam("init_pose/position", init_pose_position);
    nh_.getParam("init_pose/orientation", init_pose_orientation);

    nh_.getParam("table_pose_green/position", table_pose_position);
    nh_.getParam("table_pose_green/orientation", table_pose_orientation);

    pub_initial_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    pub_table_pose = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pub_twist = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    sub_gather_particle = nh_.subscribe("particlecloud", 1, &ServiceCore::cbGatherParticle, this);

    is_pose_initialized = fnSetInitialPose();
  }

  void cbGatherParticle(const geometry_msgs::PoseArray& poseArray)
  {
    if (!is_pose_initialized)
      return;

    int size = poseArray.poses.size();

    float score = 0.0;

    for (int i = 0; i < size; i++)
    {
      for (int j = i + 1; j < size; j++)
        score += sqrt((poseArray.poses[i].position.x - poseArray.poses[j].position.x) * (poseArray.poses[i].position.x - poseArray.poses[j].position.x) +
                      (poseArray.poses[i].position.y - poseArray.poses[j].position.y) * (poseArray.poses[i].position.y - poseArray.poses[j].position.y));
    }

    if (score > MSE_IS_POSE_INITIALIZING_COMPLETED)
    {
      geometry_msgs::Twist twist;
      twist.angular.z = 0.8;

      ROS_INFO("on pose initializing | MSE : %lf > %lf", score, MSE_IS_POSE_INITIALIZING_COMPLETED);

      pub_twist.publish(twist);
    }
    else
    {
      geometry_msgs::Twist twist;
      twist.angular.z = 0.0;

      ROS_INFO("pose initializing completed");

      pub_twist.publish(twist);

      fnTablePose();

      exit(0);
    }
  }

  bool fnSetInitialPose()
  {
    geometry_msgs::PoseWithCovarianceStamped pubPoseWithCovarianceStamped;

    pubPoseWithCovarianceStamped.header.stamp = ros::Time::now();

    pubPoseWithCovarianceStamped.pose.pose.position.x = init_pose_position[0];
    pubPoseWithCovarianceStamped.pose.pose.position.y = init_pose_position[1];
    pubPoseWithCovarianceStamped.pose.pose.position.z = init_pose_position[2];

    pubPoseWithCovarianceStamped.pose.pose.orientation.x = init_pose_orientation[0];
    pubPoseWithCovarianceStamped.pose.pose.orientation.y = init_pose_orientation[1];
    pubPoseWithCovarianceStamped.pose.pose.orientation.z = init_pose_orientation[2];
    pubPoseWithCovarianceStamped.pose.pose.orientation.w = init_pose_orientation[3];

    pubPoseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

    ros::Rate poll_rate(100);
    while(pub_initial_pose.getNumSubscribers() == 0)
      poll_rate.sleep();

    pub_initial_pose.publish(pubPoseWithCovarianceStamped);

    return true;
  }

  void fnTablePose()
  {
    // start
    pubPoseStamped.header.frame_id = "map";
    pubPoseStamped.header.stamp = ros::Time::now();

    pubPoseStamped.pose.position.x = table_pose_position[0];
    pubPoseStamped.pose.position.y = table_pose_position[1];
    pubPoseStamped.pose.position.z = table_pose_position[2];

    pubPoseStamped.pose.orientation.x = table_pose_orientation[0];
    pubPoseStamped.pose.orientation.y = table_pose_orientation[1];
    pubPoseStamped.pose.orientation.z = table_pose_orientation[2];
    pubPoseStamped.pose.orientation.w = table_pose_orientation[3];

    pub_table_pose.publish(pubPoseStamped);
  }

private:
  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pub_initial_pose;
  ros::Publisher pub_twist;
  ros::Publisher pub_table_pose;

  // Subscriber
  ros::Subscriber sub_gather_particle;

  // msgs
  geometry_msgs::PoseStamped pubPoseStamped;

  std::vector<double> init_pose_position;
  std::vector<double> init_pose_orientation;

  std::vector<double> table_pose_position;
  std::vector<double> table_pose_orientation;

  bool is_pose_initialized = false;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pose_initialization");

  //Create an object of class ServiceCore that will take care of everything
  ServiceCore serviceCore;

  ros::spin();

  return 0;
}


//
// ros::Publisher pub_pad_order_tb3g;
// ros::Publisher pub_pad_order_tb3r;
// ros::Publisher pub_pad_order_tb3p;
// ros::Publisher pub_is_item_available;
//
// ros::Publisher pub_play_sound;
//
// ros::Publisher pub_poseStamped_tb3g;
// ros::Publisher pub_poseStamped_tb3r;
// ros::Publisher pub_poseStamped_tb3p;
//
// ros::Subscriber sub_pad_order_tb3g;
// ros::Subscriber sub_pad_order_tb3r;
// ros::Subscriber sub_pad_order_tb3p;
//
// ros::Subscriber sub_arrival_status;
//
//
// int chosen_item_num[3] = {0, };
// bool is_item_available[3] = {true, };
//
// int service_sequence[3] = {0, };
//
// geometry_msgs::PoseStamped poseStamped[2];
//
//
// void fnPublishVoiceFilePath(const char* file_path)
// {
//   std_msgs::String str;
//
//   str.data = file_path;
//
//   pub_play_sound.publish(str);
// }
//
// void fnPublishGoalPoseTB3G()
// {
//   // start
//   poseStamped[0].header.frame_id = "map";
//   poseStamped[0].header.stamp = ros::Time::now();
//   poseStamped[0].pose.position.x = -0.162312030792;
//   poseStamped[0].pose.position.y = 1.78772580624;
//   poseStamped[0].pose.position.z = 0.0;
//
//   poseStamped[0].pose.orientation.x = 0.0;
//   poseStamped[0].pose.orientation.y = 0.0;
//   poseStamped[0].pose.orientation.z = -0.785727710125;
//   poseStamped[0].pose.orientation.w = 0.618572522459;
//
//
//   // counter
//   poseStamped[1].header.frame_id = "map";
//   poseStamped[1].header.stamp = ros::Time::now();
//   poseStamped[1].pose.position.x = 0.979266941547;
//   poseStamped[1].pose.position.y = 1.7585580349;
//   poseStamped[1].pose.position.z = 0.0;
//
//   poseStamped[1].pose.orientation.x = 0.0;
//   poseStamped[1].pose.orientation.y = 0.0;
//   poseStamped[1].pose.orientation.z = -0.178078902255;
//   poseStamped[1].pose.orientation.w = 0.984016211539;
//
//
//
//   if (service_sequence[0] == 0)
//   {
//     ROS_INFO("Going to get item");
//
//     pub_poseStamped_tb3g.publish(poseStamped[1]);
//
//     service_sequence[0] = 1;
//
//     fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");
//   }
//   else if (service_sequence[0] == 1)
//   {
//     ROS_INFO("Coming back with item");
//
//     pub_poseStamped_tb3g.publish(poseStamped[0]);
//
//     turtlebot3_carrier_icra2017::AvailableItemList availableItemList;
//
//     is_item_available[chosen_item_num[0] - 1] = true;
//
//     availableItemList.item_number = chosen_item_num[0];
//     availableItemList.is_item_available = is_item_available[chosen_item_num[0] - 1];
//
//     pub_is_item_available.publish(availableItemList);
//
//     chosen_item_num[0] = 0;
//
//     service_sequence[0] = 2;
//
//     fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-3.mp3");
//   }
//   else if (service_sequence[0] == 2)
//   {
//     fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-4.mp3");
//   }
// }
//
// void cbCheckArrivalStatusTB3G(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
// {
//   if (rcvMoveBaseActionResult.status.status == 3)
//   {
//     fnPublishGoalPoseTB3G();
//   }
//   else
//   {
//     ROS_INFO("rcvMoveBaseActionResult.status.status : %d", rcvMoveBaseActionResult.status.status);
//   }
// }
//
// void cbControlServiceStatus(const turtlebot3_carrier_icra2017::PadOrder rcvPadOrder)
// {
//   if (chosen_item_num[rcvPadOrder.pad_number - 1] != 0)
//   {
//     ROS_INFO("Your TurtleBot is currently moving");
//     return;
//   }
//
//   if (!is_item_available[rcvPadOrder.item_number - 1])
//   {
//     ROS_INFO("Chosen item is currently unavailable");
//     return;
//   }
//
//   chosen_item_num[rcvPadOrder.pad_number - 1] = rcvPadOrder.item_number;
//   is_item_available[rcvPadOrder.item_number - 1] = false;
//
//   ROS_INFO("Orderred by Pad No. %d", rcvPadOrder.pad_number);
//   ROS_INFO("Chosen Item No. %d", chosen_item_num[rcvPadOrder.pad_number - 1]);
//
//   if (service_sequence[0] == 2)
//     service_sequence[0] = 0;
//
//   fnPublishGoalPoseTB3G();
//
//   turtlebot3_carrier_icra2017::AvailableItemList availableItemList;
//
//   is_item_available[chosen_item_num[rcvPadOrder.pad_number - 1]] = false;
//
//   availableItemList.item_number = chosen_item_num[rcvPadOrder.pad_number- 1];
//   availableItemList.is_item_available = is_item_available[chosen_item_num[rcvPadOrder.pad_number - 1] - 1];
//
//   pub_is_item_available.publish(availableItemList);
//
//   fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");
//
//   // chosen_item_num[0] = 0;
//   //
//   //
//   // availableItemList.item_number = rcvPadOrder.item_number;
//   // availableItemList.is_item_available = false;
//   //
//   // pub_is_item_available.publish(availableItemList);
//
//
//   // ROS_INFO("sound published");
//
//   // ROS_INFO("availableItemList.item_number = %d // availableItemList.is_item_available = %d", availableItemList.item_number, availableItemList.is_item_available);
// }
//
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "service_core");
//
//
//   ros::NodeHandle n;
//
//   pub_pad_order_tb3g = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3g/pad_order_echo", 100);
//   pub_pad_order_tb3r = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3r/pad_order_echo", 100);
//   pub_pad_order_tb3p = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3p/pad_order_echo", 100);
//
//   pub_is_item_available = n.advertise<turtlebot3_carrier_icra2017::AvailableItemList>("/is_item_available", 100);
//
//   pub_play_sound = n.advertise<std_msgs::String>("/tb3g/play_sound_file", 100);
//
//   pub_poseStamped_tb3g = n.advertise<geometry_msgs::PoseStamped>("/tb3g/move_base_simple/goal", 1000);
//   pub_poseStamped_tb3r = n.advertise<geometry_msgs::PoseStamped>("/tb3r/move_base_simple/goal", 1000);
//   pub_poseStamped_tb3p = n.advertise<geometry_msgs::PoseStamped>("/tb3p/move_base_simple/goal", 1000);
//
//   sub_pad_order_tb3g = n.subscribe("/tb3g/pad_order", 100, cbControlServiceStatus);
//   sub_pad_order_tb3r = n.subscribe("/tb3r/pad_order", 100, cbControlServiceStatus);
//   sub_pad_order_tb3p = n.subscribe("/tb3p/pad_order", 100, cbControlServiceStatus);
//
//   sub_arrival_status = n.subscribe("/tb3g/move_base/result", 100, cbCheckArrivalStatusTB3G);
//
//   ros::Rate loop_rate(10);
//
//   ros::spin();
//
//   return 0;
// }
