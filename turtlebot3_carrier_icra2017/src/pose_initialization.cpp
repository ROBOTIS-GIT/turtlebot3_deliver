#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/InitTurtlebotPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "tf2_msgs/TFMessage.h"

#define MSE_IS_POSE_INITIALIZING_COMPLETED 15000.0

class PoseInitialization
{
public:
  PoseInitialization()
  {
    std::string machine_name;
    ros::param::get("~machine_name", machine_name);

    nh_.getParam("init_pose/position", init_pose_position);
    nh_.getParam("init_pose/orientation", init_pose_orientation);

    // nh_.getParam("table_pose/position", table_pose_position);
    // nh_.getParam("table_pose/orientation", table_pose_orientation);

    nh_.getParam("table_pose_" + machine_name + "/position", table_pose_position);
    nh_.getParam("table_pose_" + machine_name + "/orientation", table_pose_orientation);

    pub_initial_pose = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);
    pub_table_pose = nh_.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
    pub_twist = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    sub_gather_particle = nh_.subscribe("particlecloud", 1, &PoseInitialization::cbGatherParticle, this);

    is_pose_initialized = fnSetInitialPose();
  }

  void cbGatherParticle(const geometry_msgs::PoseArray& poseArray)
  {
    // if (!is_pose_initialized)
    //   return;
    //
    // int size = poseArray.poses.size();
    //
    // float score = 0.0;
    //
    // for (int i = 0; i < size; i++)
    // {
    //   for (int j = i + 1; j < size; j++)
    //     score += sqrt((poseArray.poses[i].position.x - poseArray.poses[j].position.x) * (poseArray.poses[i].position.x - poseArray.poses[j].position.x) +
    //                   (poseArray.poses[i].position.y - poseArray.poses[j].position.y) * (poseArray.poses[i].position.y - poseArray.poses[j].position.y));
    // }
    //
    // if (score > MSE_IS_POSE_INITIALIZING_COMPLETED)
    // {
    //   geometry_msgs::Twist twist;
    //   twist.angular.z = 0.8;
    //
    //   ROS_INFO("on pose initializing | MSE : %lf > %lf", score, MSE_IS_POSE_INITIALIZING_COMPLETED);
    //
    //   pub_twist.publish(twist);
    // }
    // else
    // {
    //   geometry_msgs::Twist twist;
    //   twist.angular.z = 0.0;
    //
    //   ROS_INFO("pose initializing completed");
    //
    //   pub_twist.publish(twist);
    //
    //   fnTablePose();
    //
    //   exit(0);
    // }
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

  //Create an object of class PoseInitialization that will take care of everything
  PoseInitialization poseInitialization;

  ros::spin();

  return 0;
}
