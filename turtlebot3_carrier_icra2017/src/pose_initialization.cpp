#include "ros/ros.h"
#include "turtlebot3_carrier_icra2017/InitTurtlebotPose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Twist.h"

#define MSE_IS_POSE_INITIALIZING_COMPLETED 15000

ros::Publisher pub_initial_pose;
ros::Publisher pub_twist;

ros::Subscriber sub_particle_cloud;

void fnSetInitialPose()
{
  geometry_msgs::PoseWithCovarianceStamped poseWithCovarianceStamped;

  poseWithCovarianceStamped.header.frame_id = "map";
  poseWithCovarianceStamped.header.stamp = ros::Time::now();
  poseWithCovarianceStamped.pose.pose.position.x = -0.954769432545;
  poseWithCovarianceStamped.pose.pose.position.y = 1.48137366772;
  poseWithCovarianceStamped.pose.pose.position.z = 0.0;

  poseWithCovarianceStamped.pose.pose.orientation.x = 0.0;
  poseWithCovarianceStamped.pose.pose.orientation.y = 0.0;
  poseWithCovarianceStamped.pose.pose.orientation.z = -0.694655700189;
  poseWithCovarianceStamped.pose.pose.orientation.w = 0.719342378979;

  poseWithCovarianceStamped.pose.covariance = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};

  pub_initial_pose.publish(poseWithCovarianceStamped);
}

void cbGatherParticle(const geometry_msgs::PoseArray poseArray)
{
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

    ROS_INFO("on pose initializing");

    pub_twist.publish(twist);
  }
  else
  {
    geometry_msgs::Twist twist;
    twist.angular.z = 0.0;

    ROS_INFO("pose initializing completed");

    pub_twist.publish(twist);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pose_initialization");
  ros::NodeHandle n;

  pub_initial_pose = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/tb3g/initialpose", 100);
  pub_twist = n.advertise<geometry_msgs::Twist>("/tb3g/cmd_vel", 100);
  // pub_gather_particle = n.advertise<geometry_msgs::tf>("/tb3g/tf", 100);

  sub_particle_cloud = n.subscribe("/tb3g/particlecloud", 100, cbGatherParticle);

  ros::Rate poll_rate(100);
  while(pub_initial_pose.getNumSubscribers() == 0)
    poll_rate.sleep();

  fnSetInitialPose();

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
