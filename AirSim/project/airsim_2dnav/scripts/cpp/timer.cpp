#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "nav_msgs/Odometry.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control_timer");
  ros::NodeHandle n;
  double t = 0;
  double poll_period = 0.2;
  ROS_INFO("Checking for odom...");
  boost::shared_ptr<nav_msgs::Odometry const> data;
  data = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(10));
  if (data != NULL){
    ROS_INFO("Initiating control loop with polling period: %.2fs", poll_period);
    ros::Publisher cont_pub = n.advertise<std_msgs::Float64>("/control_timer", 2);
    while (ros::ok()) {
      t += poll_period;
      std_msgs::Float64 tE;
      tE.data = t;
      cont_pub.publish(tE);
      ros::Duration(poll_period).sleep();
    }
  }
  else {
    ROS_INFO("No data!");
  }
  return 0;
}
