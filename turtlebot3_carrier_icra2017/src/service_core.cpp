#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/PadOrder.h"
#include "turtlebot3_carrier_icra2017/AvailableItemList.h"
#include "turtlebot3_carrier_icra2017/ServiceStatus.h"
#include "std_msgs/String.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"

#define ROBOT_NUMBER_TB3P 0
#define ROBOT_NUMBER_TB3G 1
#define ROBOT_NUMBER_TB3R 2

class ServiceCore
{
public:
  ServiceCore()
  {
    fnInitParam();

    // pub_initial_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    // pub_table_pose = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    // pub_twist = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    //
    // sub_gather_particle = nh_.subscribe("particlecloud", 1, &ServiceCore::cbGatherParticle, this);
    //




    pubServiceStatusPadtb3p = nh_.advertise<turtlebot3_carrier_icra2017::ServiceStatus>("/tb3p/service_status", 1);
    pubServiceStatusPadtb3g = nh_.advertise<turtlebot3_carrier_icra2017::ServiceStatus>("/tb3g/service_status", 1);
    pubServiceStatusPadtb3r = nh_.advertise<turtlebot3_carrier_icra2017::ServiceStatus>("/tb3r/service_status", 1);

    pub_is_item_available = nh_.advertise<turtlebot3_carrier_icra2017::AvailableItemList>("/is_item_available", 1);

    pub_play_sound = nh_.advertise<std_msgs::String>("/tb3g/play_sound_file", 1);

    pubPoseStampedTb3p = nh_.advertise<geometry_msgs::PoseStamped>("/tb3p/move_base_simple/goal", 1);
    pubPoseStampedTb3g = nh_.advertise<geometry_msgs::PoseStamped>("/tb3g/move_base_simple/goal", 1);
    pubPoseStampedTb3r = nh_.advertise<geometry_msgs::PoseStamped>("/tb3r/move_base_simple/goal", 1);

    sub_pad_order_tb3p = nh_.subscribe("/tb3p/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
    sub_pad_order_tb3g = nh_.subscribe("/tb3g/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);
    sub_pad_order_tb3r = nh_.subscribe("/tb3r/pad_order", 1, &ServiceCore::cbReceivePadOrder, this);

    sub_arrival_status_tb3p = nh_.subscribe("/tb3p/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusTB3P, this);
    sub_arrival_status_tb3g = nh_.subscribe("/tb3g/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusTB3G, this);
    sub_arrival_status_tb3r = nh_.subscribe("/tb3r/move_base/result", 1, &ServiceCore::cbCheckArrivalStatusTB3R, this);

    ros::Rate loop_rate(5);

    while (ros::ok())
    {
      fnPubServiceStatus();

      fnPubPose();
      ros::spinOnce();
      loop_rate.sleep();
    }
    // is_pose_initialized = fnSetInitialPose();
  }

  void fnInitParam()
  {
    nh_.getParam("table_pose_tb3p/position", target_pose_position);
    nh_.getParam("table_pose_tb3p/orientation", target_pose_orientation);

    poseStampedTable[0].header.frame_id = "map";
    poseStampedTable[0].header.stamp = ros::Time::now();

    poseStampedTable[0].pose.position.x = target_pose_position[0];
    poseStampedTable[0].pose.position.y = target_pose_position[1];
    poseStampedTable[0].pose.position.z = target_pose_position[2];

    poseStampedTable[0].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[0].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[0].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[0].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("table_pose_tb3g/position", target_pose_position);
    nh_.getParam("table_pose_tb3g/orientation", target_pose_orientation);

    poseStampedTable[1].header.frame_id = "map";
    poseStampedTable[1].header.stamp = ros::Time::now();

    poseStampedTable[1].pose.position.x = target_pose_position[0];
    poseStampedTable[1].pose.position.y = target_pose_position[1];
    poseStampedTable[1].pose.position.z = target_pose_position[2];

    poseStampedTable[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[1].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("table_pose_tb3r/position", target_pose_position);
    nh_.getParam("table_pose_tb3r/orientation", target_pose_orientation);

    poseStampedTable[2].header.frame_id = "map";
    poseStampedTable[2].header.stamp = ros::Time::now();

    poseStampedTable[2].pose.position.x = target_pose_position[0];
    poseStampedTable[2].pose.position.y = target_pose_position[1];
    poseStampedTable[2].pose.position.z = target_pose_position[2];

    poseStampedTable[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedTable[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedTable[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedTable[2].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_bread/position", target_pose_position);
    nh_.getParam("counter_pose_bread/orientation", target_pose_orientation);

    poseStampedCounter[0].header.frame_id = "map";
    poseStampedCounter[0].header.stamp = ros::Time::now();

    poseStampedCounter[0].pose.position.x = target_pose_position[0];
    poseStampedCounter[0].pose.position.y = target_pose_position[1];
    poseStampedCounter[0].pose.position.z = target_pose_position[2];

    poseStampedCounter[0].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[0].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[0].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[0].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_drink/position", target_pose_position);
    nh_.getParam("counter_pose_drink/orientation", target_pose_orientation);

    poseStampedCounter[1].header.frame_id = "map";
    poseStampedCounter[1].header.stamp = ros::Time::now();

    poseStampedCounter[1].pose.position.x = target_pose_position[0];
    poseStampedCounter[1].pose.position.y = target_pose_position[1];
    poseStampedCounter[1].pose.position.z = target_pose_position[2];

    poseStampedCounter[1].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[1].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[1].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[1].pose.orientation.w = target_pose_orientation[3];


    nh_.getParam("counter_pose_snack/position", target_pose_position);
    nh_.getParam("counter_pose_snack/orientation", target_pose_orientation);

    poseStampedCounter[2].header.frame_id = "map";
    poseStampedCounter[2].header.stamp = ros::Time::now();

    poseStampedCounter[2].pose.position.x = target_pose_position[0];
    poseStampedCounter[2].pose.position.y = target_pose_position[1];
    poseStampedCounter[2].pose.position.z = target_pose_position[2];

    poseStampedCounter[2].pose.orientation.x = target_pose_orientation[0];
    poseStampedCounter[2].pose.orientation.y = target_pose_orientation[1];
    poseStampedCounter[2].pose.orientation.z = target_pose_orientation[2];
    poseStampedCounter[2].pose.orientation.w = target_pose_orientation[3];
  }


  void fnPublishVoiceFilePath(const char* file_path)
  {
    std_msgs::String str;

    str.data = file_path;

    pub_play_sound.publish(str);
  }
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
  //     pubPoseStampedTb3g.publish(poseStamped[1]);
  //
  //     service_sequence[0] = 1;
  //
  //     fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");
  //   }
  //   else if (service_sequence[0] == 1)
  //   {
  //     ROS_INFO("Coming back with item");
  //
  //     pubPoseStampedTb3g.publish(poseStamped[0]);
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

  void cbCheckArrivalStatusTB3P(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[0] = true;
      // if (robot_service_sequence[1] == 1)
      // fnPublishGoalPoseTB3G();
      //
      //
      //
      // is_item_available[item_num_chosen_by_pad[1]] = true;
      //
      // item_num_chosen_by_pad[1] = -1;
      //
      // robot_service_sequence[1] = 0; // init
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusTB3P : %d", rcvMoveBaseActionResult.status.status);
    }
  }

  void cbCheckArrivalStatusTB3G(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[1] = true;
      // if (robot_service_sequence[1] == 1)
      // fnPublishGoalPoseTB3G();
      //
      //
      //
      // is_item_available[item_num_chosen_by_pad[1]] = true;
      //
      // item_num_chosen_by_pad[1] = -1;
      //
      // robot_service_sequence[1] = 0; // init
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusTB3G : %d", rcvMoveBaseActionResult.status.status);
    }
  }

  void cbCheckArrivalStatusTB3R(const move_base_msgs::MoveBaseActionResult rcvMoveBaseActionResult)
  {
    if (rcvMoveBaseActionResult.status.status == 3)
    {
      is_robot_reached_target[2] = true;
      // if (robot_service_sequence[1] == 1)
      // fnPublishGoalPoseTB3G();
      //
      //
      //
      // is_item_available[item_num_chosen_by_pad[1]] = true;
      //
      // item_num_chosen_by_pad[1] = -1;
      //
      // robot_service_sequence[1] = 0; // init
    }
    else
    {
      ROS_INFO("cbCheckArrivalStatusTB3R : %d", rcvMoveBaseActionResult.status.status);
    }
  }

  void fnPubPose()
  {
    // ros::Rate loop_rate(5);
    //
    // while (ros::ok())
    // {
      // ROS_INFO("isrunning1");

      if (is_robot_reached_target[ROBOT_NUMBER_TB3G])
      {
        if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 1)
        {
          ROS_INFO("start_voice");

          fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-1.mp3");

          ros::Duration(5.0).sleep();

          fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-2.mp3");

          // waiting for loading
          ros::Duration(5.0).sleep();

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 2;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 2)
        {
          // go to counter
          pubPoseStampedTb3g.publish(poseStampedCounter[item_num_chosen_by_pad[ROBOT_NUMBER_TB3G]]);

          is_robot_reached_target[ROBOT_NUMBER_TB3G] = false;

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 3;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 3)
        {
          ROS_INFO("middle_voice");

          fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-3.mp3");

          // waiting for loading
          ros::Duration(10.0).sleep();

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 4;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 4)
        {
          // return to table
          pubPoseStampedTb3g.publish(poseStampedTable[ROBOT_NUMBER_TB3G]);

          is_robot_reached_target[ROBOT_NUMBER_TB3G] = false;

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 5;
        }
        else if (robot_service_sequence[ROBOT_NUMBER_TB3G] == 5)
        {
          ROS_INFO("finish_voice");

          fnPublishVoiceFilePath("/home/turtlebot/Desktop/voice/boy1-4.mp3");

          // waiting for loading
          ros::Duration(5.0).sleep();

          robot_service_sequence[ROBOT_NUMBER_TB3G] = 0;

          is_item_available[item_num_chosen_by_pad[ROBOT_NUMBER_TB3G]] = true;

          item_num_chosen_by_pad[ROBOT_NUMBER_TB3G] = -1;

          ROS_INFO("ended");
        }
      }

      // if ()
      //
      // if ()
      // turtlebot3_carrier_icra2017::ServiceStatus serviceStatus;
      //
      // serviceStatus.item_num_chosen_by_pad = item_num_chosen_by_pad;
      // serviceStatus.is_item_available = is_item_available;
      // serviceStatus.robot_service_sequence = robot_service_sequence;
      //
      // pubServiceStatusPadtb3p.publish(serviceStatus);
      // pubServiceStatusPadtb3g.publish(serviceStatus);
      // pubServiceStatusPadtb3r.publish(serviceStatus);

    //
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
  }

  void cbReceivePadOrder(const turtlebot3_carrier_icra2017::PadOrder padOrder)
  {
    if (!is_item_available[padOrder.item_number])
    {
      ROS_INFO("Chosen item is currently unavailable");
      return;
    }

    if (robot_service_sequence[padOrder.pad_number] != 0)
    {
      ROS_INFO("Your TurtleBot is currently on servicing");
      return;
    }

    if (item_num_chosen_by_pad[padOrder.pad_number] != -1)
    {
      ROS_INFO("Your TurtleBot is currently on servicing");
      return;
    }

    item_num_chosen_by_pad[padOrder.pad_number] = padOrder.item_number;

    robot_service_sequence[padOrder.pad_number] = 1; // just left from the table

    is_item_available[padOrder.item_number] = false;
  }



    // void cbInitStatus(const std_msgs::Bool init)
    // {
    //   if (init)
    //     is
    // }

    // chosen_item_num[padOrder.pad_number - 1] = padOrder.item_number;
    // is_item_available[padOrder.item_number - 1] = false;
    //
    // ROS_INFO("Orderred by Pad No. %d", padOrder.pad_number);
    // ROS_INFO("Chosen Item No. %d", chosen_item_num[padOrder.pad_number - 1]);
    //
    // if (service_sequence[0] == 2)
    //   service_sequence[0] = 0;
    //
    // fnPublishGoalPoseTB3G();
    //
    // turtlebot3_carrier_icra2017::AvailableItemList availableItemList;
    //
    // is_item_available[chosen_item_num[padOrder.pad_number - 1]] = false;
    //
    // availableItemList.item_number = chosen_item_num[padOrder.pad_number- 1];
    // availableItemList.is_item_available = is_item_available[chosen_item_num[padOrder.pad_number - 1] - 1];
    //
    // pub_is_item_available.publish(availableItemList);
    //

    // chosen_item_num[0] = 0;
    //
    //
    // availableItemList.item_number = padOrder.item_number;
    // availableItemList.is_item_available = false;
    //
    // pub_is_item_available.publish(availableItemList);
    // ROS_INFO("sound published");
    // ROS_INFO("availableItemList.item_number = %d // availableItemList.is_item_available = %d", availableItemList.item_number, availableItemList.is_item_available);

  void fnPubServiceStatus()
  {
    // ros::Rate loop_rate(5);
    //
    // while (ros::ok())
    // {
      // ROS_INFO("isrunning2");

      turtlebot3_carrier_icra2017::ServiceStatus serviceStatus;

      serviceStatus.item_num_chosen_by_pad = item_num_chosen_by_pad;
      serviceStatus.is_item_available = is_item_available;
      serviceStatus.robot_service_sequence = robot_service_sequence;

      pubServiceStatusPadtb3p.publish(serviceStatus);
      pubServiceStatusPadtb3g.publish(serviceStatus);
      pubServiceStatusPadtb3r.publish(serviceStatus);

      // ROS_INFO("hello");

    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
  }

private:

  // enum ROBOT_NUMBER
  // {
  //   TB3P = 0
  //   , TB3G
  //   , TB3R
  // } robot_number;
  //
  // enum ITEM_NUMBER
  // {
  //   TB3P = 0
  //   , TB3G
  //   , TB3R
  // } item_number;

  ros::NodeHandle nh_;

  // Publisher
  ros::Publisher pubServiceStatusPadtb3p;
  ros::Publisher pubServiceStatusPadtb3g;
  ros::Publisher pubServiceStatusPadtb3r;
  ros::Publisher pub_is_item_available;

  ros::Publisher pub_play_sound;

  ros::Publisher pubPoseStampedTb3p;
  ros::Publisher pubPoseStampedTb3g;
  ros::Publisher pubPoseStampedTb3r;

  // Subscriber
  ros::Subscriber sub_pad_order_tb3g;
  ros::Subscriber sub_pad_order_tb3r;
  ros::Subscriber sub_pad_order_tb3p;

  ros::Subscriber sub_arrival_status_tb3p;
  ros::Subscriber sub_arrival_status_tb3g;
  ros::Subscriber sub_arrival_status_tb3r;

  // msgs
  geometry_msgs::PoseStamped poseStampedTable[3];
  geometry_msgs::PoseStamped poseStampedCounter[3];

  std::vector<double> target_pose_position;
  std::vector<double> target_pose_orientation;

  boost::array<int, 3> item_num_chosen_by_pad = { {-1, -1, -1} };
  boost::array<bool, 3> is_item_available = { {true, true, true} };
  boost::array<int, 3> robot_service_sequence = { {0, 0, 0} };

  bool is_robot_reached_target[3] = {true, true, true};
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "service_core");

  //Create an object of class ServiceCore that will take care of everything
  ServiceCore serviceCore;

  ros::spin();

  return 0;
}
