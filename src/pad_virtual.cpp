#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/PadOrder.h"
#include "turtlebot3_carrier_icra2017/AvailableItemList.h"

ros::Publisher pub_pad_order;

ros::Subscriber sub_available_item_list;

bool is_item_available[3] = {true, true, true};

void cbRefreshAvailableItemList(const turtlebot3_carrier_icra2017::AvailableItemList rcvAvailableItemList)
{
  is_item_available[rcvAvailableItemList.item_number - 1] = rcvAvailableItemList.is_item_available;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pad_virtual");

  ros::NodeHandle n;

  pub_pad_order = n.advertise<turtlebot3_carrier_icra2017::PadOrder>("/tb3g/pad_order", 100);

  sub_available_item_list = n.subscribe("/is_item_available", 100, cbRefreshAvailableItemList);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std::string inputString;
    std::cout << "Which item do you choose? (a. Juice b. Snack c. Bread)" << '\n';
    std::getline(std::cin, inputString);

    if (inputString.compare("a") == 0)
    {
      if (is_item_available[0])
      {
        turtlebot3_carrier_icra2017::PadOrder padOrder;

        padOrder.pad_number = 1;
        padOrder.item_number = 1;

        pub_pad_order.publish(padOrder);

        ROS_INFO("Juice was selected");
      }
      else
      {
        ROS_INFO("Sorry, selected item is now unavailable. Please choose another item");
      }
    }
    else if (inputString.compare("b") == 0)
    {
      if (is_item_available[1])
      {
        turtlebot3_carrier_icra2017::PadOrder padOrder;

        padOrder.pad_number = 1;
        padOrder.item_number = 2;

        pub_pad_order.publish(padOrder);

        ROS_INFO("Snack was selected");
      }
      else
      {
        ROS_INFO("Sorry, selected item is now unavailable. Please choose another item");
      }
    }
    else if (inputString.compare("c") == 0)
    {
      if (is_item_available[2])
      {
        turtlebot3_carrier_icra2017::PadOrder padOrder;

        padOrder.pad_number = 1;
        padOrder.item_number = 3;

        pub_pad_order.publish(padOrder);

        ROS_INFO("Bread was selected");
      }
      else
      {
        ROS_INFO("Sorry, selected item is now unavailable. Please choose another item");
      }
    }
    else
    {
      break;
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
