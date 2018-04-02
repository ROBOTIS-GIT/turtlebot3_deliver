#include "ros/ros.h"
// #include "turtlebot3_deliver_service/PadOrder.h"
// #include "turtlebot3_deliver_service/AvailableItemList.h"
// #include "turtlebot3_deliver_service/ServiceStatus.h"
// #include "std_msgs/String.h"
// #include "actionlib_msgs/GoalStatusArray.h"
// #include "move_base_msgs/MoveBaseActionResult.h"
// #include "geometry_msgs/PoseStamped.h"
// #include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "turtlebot3_deliver_service/InitTurtlebotPose.h"

/*
header:
  seq: 1
  stamp:
    secs: 1495278439
    nsecs: 731332939
  frame_id: map
pose:
  pose:
    position:
      x: -0.954769432545
      y: 1.48137366772
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.694655700189
      w: 0.719342378979
  covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

*/

// ros::Publisher pub_initial_pose;

ros::ServiceClient client_initial_pose;

turtlebot3_deliver_service::InitTurtlebotPose initTurtlebotPose;

// geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;


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


// int chosen_item_num[3] = {0, };
// bool is_item_available[3] = {true, };
//
// int service_sequence[3] = {0, };

// geometry_msgs::PoseStamped poseStamped[2];

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
//   poseStamped[1].pose.orientation.x = 0.0;
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
//     turtlebot3_deliver_service::AvailableItemList availableItemList;
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
// void cbControlServiceStatus(const turtlebot3_deliver_service::PadOrder rcvPadOrder)
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
//   turtlebot3_deliver_service::AvailableItemList availableItemList;
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

void fnPublishInitialPose()
{
  // initTurtlebotPose.poseWithCovarianceStamped.header.frame_id = "map";
  // initTurtlebotPose.poseWithCovarianceStamped.header.stamp = ros::Time::now();
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.position.x = -0.954769432545;
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.position.y = 1.48137366772;
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.position.z = 0.0;
  //
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.orientation.z = -0.694655700189;
  // initTurtlebotPose.poseWithCovarianceStamped.pose.pose.orientation.w = 0.719342378979;
  //
  // // double covariance[] = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
  // initTurtlebotPose.poseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

  // pub_initial_pose.publish(poseWithCovarianceStamped);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "call_initial_pose");

  ros::NodeHandle n;

  // ros::Rate loop_rate(10);

  // pub_initial_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/tb3g/initialpose", 1000);

  initTurtlebotPose.request.poseWithCovarianceStamped.header.frame_id = "map";
  initTurtlebotPose.request.poseWithCovarianceStamped.header.stamp = ros::Time::now();
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.position.x = -0.954769432545;
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.position.y = 1.48137366772;
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.position.z = 0.0;

  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.orientation.z = -0.694655700189;
  initTurtlebotPose.request.poseWithCovarianceStamped.pose.pose.orientation.w = 0.719342378979;

  initTurtlebotPose.request.poseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

  // initTurtlebotPose.request.aa = 1;


  client_initial_pose = n.serviceClient<turtlebot3_deliver_service::InitTurtlebotPose>("/tb3g/initial_turtlebot_pose");

  client_initial_pose.call(initTurtlebotPose);


  // while(!ros::isShuttingDown())
  //   ros::Duration(1.0).sleep();

  // while(ros::ok())
  // {
  //   // ROS_INFO("%x", ros::ok());
  //   pub_initial_pose.publish(poseWithCovarianceStamped);
  //   // loop_rate.sleep();
  //
  //   ros::spinOnce();
  // }

  // fnPublishInitialPose();

  // loop_rate.sleep();

  // ros::spinOnce();

  // pub_initial_pose.publish(poseWithCovarianceStamped);
  // loop_rate.sleep();
    /*
  header:
    seq: 1
    stamp:
      secs: 1495278439
      nsecs: 731332939
    frame_id: map
  pose:
    pose:
      position:
        x: -0.954769432545
        y: 1.48137366772
        z: 0.0
      orientation:
        x: 0.0
        y: 0.0
        z: -0.694655700189
        w: 0.719342378979
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]

  */



  // counter
  // poseStamped[1].header.frame_id = "map";
  // poseStamped[1].header.stamp = ros::Time::now();
  // poseStamped[1].pose.position.x = 0.979266941547;
  // poseStamped[1].pose.position.y = 1.7585580349;
  // poseStamped[1].pose.position.z = 0.0;
  //
  // poseStamped[1].pose.orientation.x = 0.0;
  // poseStamped[1].pose.orientation.y = 0.0;
  // poseStamped[1].pose.orientation.z = -0.178078902255;
  // poseStamped[1].pose.orientation.w = 0.984016211539;


  // pub_pad_order_tb3g = n.advertise<turtlebot3_deliver_service::PadOrder>("/tb3g/pad_order_echo", 100);
  // pub_pad_order_tb3r = n.advertise<turtlebot3_deliver_service::PadOrder>("/tb3r/pad_order_echo", 100);
  // pub_pad_order_tb3p = n.advertise<turtlebot3_deliver_service::PadOrder>("/tb3p/pad_order_echo", 100);
  //
  // pub_is_item_available = n.advertise<turtlebot3_deliver_service::AvailableItemList>("/is_item_available", 100);
  //
  // pub_play_sound = n.advertise<std_msgs::String>("/tb3g/play_sound_file", 100);
  //
  // pub_poseStamped_tb3g = n.advertise<geometry_msgs::PoseStamped>("/tb3g/move_base_simple/goal", 1000);
  // pub_poseStamped_tb3r = n.advertise<geometry_msgs::PoseStamped>("/tb3r/move_base_simple/goal", 1000);
  // pub_poseStamped_tb3p = n.advertise<geometry_msgs::PoseStamped>("/tb3p/move_base_simple/goal", 1000);
  //
  // sub_pad_order_tb3g = n.subscribe("/tb3g/pad_order", 100, cbControlServiceStatus);
  // sub_pad_order_tb3r = n.subscribe("/tb3r/pad_order", 100, cbControlServiceStatus);
  // sub_pad_order_tb3p = n.subscribe("/tb3p/pad_order", 100, cbControlServiceStatus);
  //
  // sub_arrival_status = n.subscribe("/tb3g/move_base/result", 100, cbCheckArrivalStatusTB3G);



  // ros::spin();

  return 0;
}
