#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include <vector>
using std::vector;

class Goal
{
private:
  int goal_cnt;
  int config;
  vector<int> goal_types;
  vector<vector<double>> pose_seq;
  ros::Publisher cont_pub;
  ros::Subscriber sub_odom;
public:
  Goal(ros::NodeHandle *n) {
    goal_cnt = 0;
    config = 10;
    cont_pub = n->advertise<geometry_msgs::Vector3>("/goals", 1);
    sub_odom = n->subscribe("/odom", 5, &Goal::odom, this);
  }

  void callback();
  void odom(const nav_msgs::Odometry::ConstPtr& msg);
};

void Goal::callback() {
  ROS_INFO("Loading goals for configuration: %d", config);
  while (cont_pub.getNumSubscribers() == 0){
    if (ros::ok()) {
      ros::Duration(0.05).sleep();
    }
    else {
      return;
    }
  }
  // ros::Duration(1).sleep();

  switch (config) {
    case 1:
        // # Config 1 - Default (~1540 m, ~167s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-122},{-197,-128},{-179,-128},{-135.5,-128},{-90,-128},{-84,-143},{-84,-153},{-84,-175},{-83,-195},{-79,-203},{-78, -207},{-84, -208}, {-85, -205}, {-85,-195},{-84,-128},{-84,-68},{-84,-58},{-99,-48},{-115,-48},{-135,-48},{-203, -48},{-212, -63},{-212, -78},{-212,-198},{-212, -241},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-80,-256},{-20, -256},{34.5, -256},{44, -241},{44, -220},{44, -198},{44, -138},{29, -128},{1,-128},{7.5, -133},{15, -128},{37, -128},{44, -110}, {44,-95}, {44.5, -65.5}};
        goal_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,1,2,3,4,1,2,3,4,1,1,2,3,4,0,4,1,2,3,4,1,2,7,6,5,1,2,3,7};
        break;
    case 2:
        // # Config 2 - Efficiency (< Distance, < Time), (~1520 m, ~162s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128},{-94,-128},{-84,-143},{-84,-153},{-84,-198},{-84,-246},{-99,-256}, {-114, -256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-120}, {-69,-128}, {-54,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-79},{39,-71},{38, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
        goal_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
        break;
    case 3:
        // # Config 3 - Fastest (> Distance, <<< Time), (~1555 m, ~144s)
        pose_seq = {{-80, 0}, {-135, 0}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128}, {-70, -128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -79}, {43,-75}, {39, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{-70,-256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-198}};
        goal_types = {0,4,1,2,3,4,1,2,3,4,0,4,1,2,3,4,1,5,5,5,3,4,1,2,3,4,0,4,1,2,3,4,1,2,3,4,1,2,3,7};
        break;
    case 4:
        // # Config 4 - Shortest (<<< Distance, >> Time), (~1440 m, ~180s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-163,-48}, {-141,-49},{-133,-51},{-130, -48},{-144, -48}, {-153, -48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-163,-128}, {-141,-129},{-133,-131},{-130, -128},{-143, -128},{-153, -128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-72,-128}, {-55,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-78},{39,-69},{38, -67},{44, -65}, {45, -67}, {44,-85},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
        goal_types = {0,4,1,2,3,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
        break;
    case 5:
        // # Config 5 - Shortest (<<<< Distance, >> Time), (~1398 m, ~215s) (reverse, forward, reverse)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-74,-128}, {-59,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -65},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{0, -256}};
        goal_types = {0,4,1,2,3,1,2,3,4,8,9,10,6,8,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,6,8,9,10,6,7};
        break;
    case 6:
        // # Config 6 - Shortest (<<<< Distance, >>> Time), (~1400 m, ~200s) (reverse, forward, reverse) x2
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-199, -48},{-212, -65},{-212, -78},{-212, -74}, {-212,-116},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-244},{-69,-256},{-54, -256}, {0, -256}, {31, -256}, {44, -241}, {44, -226}, {44, -198}, {44, -138},{29, -128},{1,-128},{34, -128},{44, -110}, {44,-95}, {44.5, -65}, {44.5, -65}};
        goal_types = {0,4,1,2,3,1,2,3,4,6,8,6,6,6,8,6,6,1,2,3,4,1,2,3,4,1,2,3,4,6,8,6,6,6,8,6,6,6,8,6,1,2,3,1,7};
        break;
    case 7:
        // # Config 7 - Start backwards
        pose_seq = {{0,0},{35, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-110},{34,-127},{2,-129},{34,-129},{44,-143},{44,-158},{44,-198},{44, -246},{29, -256},{14,-256},{0, -256},{-74,-256},{-84,-241},{-84,-226},{-84.5,-200},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-143},{-202,-130},{-137,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-137,-48},{-183,-48},{-197,-48},{-210,-37},{-212,-30},{-212,-15},{-205,-1},{-180,0.0},{-135,1},{-135,1}};
        goal_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,8,9,10,8,9,10,6,6,7};
        break;
    case 8:
        // # Config 8 - 7 & end forward
        pose_seq = {{0,0},{32, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-115},{28,-127},{4.5,-128},{34,-129},{44,-143},{44,-158},{44,-198},{44, -241},{27, -256},{12,-256},{0, -256},{-70,-256},{-84,-241},{-84,-226},{-84.5,-199},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-145},{-200,-130},{-142,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-135,-48},{-94,-48},{-84,-33},{-84,-18},{-84,-13},{-103,0},{-114,0},{-135,1}};
        goal_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,1,2,3,1,2,3,7};
        break;
    case 9:
        pose_seq = {{0,0},{24,-0.5},{32, -0.5},{44, -20},{44,-35},{44,-65},{44,-95},{44,-113},{25,-127},{0,-128},{15,-128},{33,-129},{44,-143},{44,-158},{44,-198},{44,-230},{44, -241},{27, -256},{12,-256},{0, -256},{-59,-256},{-73,-256},{-84,-241},{-84,-226},{-84.5,-198},{-85,-235},{-84,-244},{-94,-255},{-135.5,-255},{-178,-253},{-196.5,-242.5},{-207,-218},{-211,-198},{-212,-158},{-212,-145},{-200,-130},{-135.5,-128},{-150,-128},{-190,-128},{-203,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-199,-49},{-135,-48},{-110,-48},{-93,-48},{-85,-33},{-85,-18},{-85,-13},{-103,0},{-114,0},{-135,1}};
        goal_types = {0,6,8,9,10,6,6,9,10,11,1,0,2,3,4,0,1,2,3,4,0,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,0,1,2,3,4,4,2,3,4,0,1,2,3,1,2,3,7};
        break;
    case 10:
        pose_seq = {{0,0},{24,-0.5},{32, -0.5},{44, -20},{44,-35},{44,-65},{44,-95},{44,-113},{25,-127},{0,-128},{15,-128},{33,-129},{44,-143},{44,-158},{44,-198},{44,-230},{44, -241},{27, -256},{12,-256},{0, -256},{-59,-256},{-73,-256},{-84,-241},{-84,-226},{-84.5,-198},{-85,-235},{-91.8,-247.8},{-105,-255.5},{-135.5,-255},{-178,-253},{-196.5,-242.5},{-207,-218},{-211,-198},{-212,-158},{-212,-145},{-200,-130},{-135.5,-128},{-150,-128},{-190,-128},{-203,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-199,-49},{-135,-48},{-110,-48},{-93,-48},{-85,-33},{-85,-18},{-85,-13},{-103,0},{-114,0},{-135,1}};
        goal_types = {0,6,8,9,10,6,6,9,10,11,1,0,2,3,4,0,1,2,3,4,0,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,0,1,2,3,4,4,2,3,4,0,1,2,3,1,2,3,7};
        break;
  }
  geometry_msgs::Vector3 goals;
  goals.x = pose_seq[0][0];
  goals.y = pose_seq[0][1];
  goals.z = goal_types[0];
  ROS_INFO("Sending goals achievements");
  cont_pub.publish(goals);
}

void Goal::odom(const nav_msgs::Odometry::ConstPtr &msg) {
  int rad = 5;

  if (goal_cnt < pose_seq.size()) {
    if (goal_types[goal_cnt] == 5 || goal_types[goal_cnt] == 11)
    {
      rad = 3;
    }
    geometry_msgs::Vector3 goals;
    double diff = sqrt(pow(msg->pose.pose.position.x - pose_seq[goal_cnt][0],2) + pow(msg->pose.pose.position.y - pose_seq[goal_cnt][1],2) + pow(msg->pose.pose.position.z,2));
    if (diff < rad) {
      goal_cnt += 1;
      if (goal_cnt < pose_seq.size()) {
        goals.x = pose_seq[goal_cnt][0];
        goals.y = pose_seq[goal_cnt][1];
        goals.z = goal_types[goal_cnt];
        cont_pub.publish(goals);
      }
    }
  }
  else {
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "goal_post");
  ros::NodeHandle n;
  Goal goal = Goal(&n);
  goal.callback();
  ros::spin();
  // ros::MultiThreadedSpinner spinner(3);
  // spinner.spin();
  return 0;
}
