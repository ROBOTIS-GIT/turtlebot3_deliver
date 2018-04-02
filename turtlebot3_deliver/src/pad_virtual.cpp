#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/PadOrder.h"
#include "turtlebot3_carrier_icra2017/ServiceStatus.h"

class PadVirtual
{
public:
  PadVirtual()
  {
    // fnInitParam();

    ros::param::get("~robot_num", robot_num);

    pub_pad_order = nh_.advertise<turtlebot3_carrier_icra2017::PadOrder>("pad_order", 1);

    subServiceStatus = nh_.subscribe("service_status", 1, &PadVirtual::cbCheckServiceStatus, this);

    // sub_arrival_status = nh_.subscribe("/tb3g/move_base/result", 1, cbCheckArrivalStatusTB3G);

    fnPubPadOrder();

    // is_pose_initialized = fnSetInitialPose();
  }

  void cbCheckServiceStatus(const turtlebot3_carrier_icra2017::ServiceStatus rcvServiceStatus)
  {
    item_num_chosen_by_pad = rcvServiceStatus.item_num_chosen_by_pad;
    is_item_available = rcvServiceStatus.is_item_available;
    robot_service_sequence = rcvServiceStatus.robot_service_sequence;

    // ROS_INFO("aa");

    // ROS_INFO("%d %d %d", is_item_available[item_num_chosen_by_pad[robot_num]], robot_service_sequence[robot_num], item_num_chosen_by_pad[robot_num]);

    // ROS_INFO("%d %x %d", item_num_chosen_by_pad[0], is_item_available[2], robot_service_sequence[0]);
    // is_item_available[rcvServiceStatus.item_number - 1] = rcvServiceStatus.is_item_available;
  }

  void fnPubPadOrder()
  {
    ros::Rate loop_rate(1);

    int selected_item_num = -1;

    while (ros::ok())
    {
      turtlebot3_carrier_icra2017::PadOrder padOrder;

      std::string inputString;
      std::cout << "Which item would you choose? (0. Bread 1. Drink 2. Snack or q. Quit)" << '\n';
      std::getline(std::cin, inputString);

      if (inputString.compare("0") == 0)
      {
        selected_item_num = 0;
        ROS_INFO("Bread was selected");
      }
      else if (inputString.compare("1") == 0)
      {
        selected_item_num = 1;
        ROS_INFO("Juice was selected");
      }
      else if (inputString.compare("2") == 0)
      {
        selected_item_num = 2;
        ROS_INFO("Snack was selected");
      }
      else if (inputString.compare("q") == 0)
      {
        return;
      }
      else
      {
        ROS_INFO("Sorry, selected item is now unavailable. Please choose another item");
        goto spin;
      }



      if (!is_item_available[selected_item_num])
      {
        ROS_INFO("but chosen item is currently unavailable");
        goto spin;
      }

      if (robot_service_sequence[robot_num] != 0)
      {
        ROS_INFO("but your TurtleBot is currently on servicing");
        goto spin;
      }

      if (item_num_chosen_by_pad[robot_num] != -1)
      {
        ROS_INFO("but your TurtleBot is currently on servicing");
        goto spin;
      }

      padOrder.pad_number = robot_num;
      padOrder.item_number = selected_item_num;

      pub_pad_order.publish(padOrder);

      spin:

      ros::spinOnce();
      loop_rate.sleep();
    }
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
  ros::Publisher pub_pad_order;

  // Subscriber
  ros::Subscriber subServiceStatus;

  boost::array<int, 3> item_num_chosen_by_pad = { {-1, -1, -1} };
  boost::array<bool, 3> is_item_available = { {true, true, true} };
  boost::array<int, 3> robot_service_sequence = { {0, 0, 0} };

  int robot_num;
};

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pad_virtual");

  //Create PadVirtual class
  PadVirtual padVirtual;

  ros::spin();

  return 0;
}
