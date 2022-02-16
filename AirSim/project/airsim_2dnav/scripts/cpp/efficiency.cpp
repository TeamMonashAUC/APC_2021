#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"
#include <vector>
using std::vector;

class Efficiency
{
private:
  // Constants
  const double mass = 2176.46; // mass of the vehicle [kg]
  const double g = 9.81; // gravity constant [m/s^2]
  const double friction = 0.01; // rolling friction
  const double rho = 1.2; // for air at NTP [kg/m^3]
  const double drag_coeff = 0.357; // drag coefficient
  const double area = 3.389; // front area of vehicle [m^2]
  const double f_r = mass*g*friction;
  const double drag = 0.5 * rho * drag_coeff * area;

  // Goal
  vector<vector<double>> goals = {{44.50, -65.75}, {0, -256}, {-84.5,-198.22}, {-135, 1.50}};
  const int last_goal_int = 3; // Change last goal here
  double last_goal[2] = {goals[last_goal_int][0], goals[last_goal_int][1]};

  // Graph
  vector<double> v_array, a_array, e_array, d_array, ma_array, t_array, goaltime;
  int passed;

  // Variables
  double distance, energy, duration, x0, y0, z0, v0, v_x0, v_y0, v_z0, t0;
  bool start, end, showgraph;
  ros::Publisher pub_v, pub_a, pub_e, pub_d, pub_ma, pub_t, pub_gt;
  std_msgs::Float64MultiArray v_dat, a_dat,e_dat, d_dat, ma_dat, t_dat, gt_dat;
  ros::Subscriber sub_odom, sub_score;
public:
  Efficiency(ros::NodeHandle *n) {
    v0 = v_x0 = v_y0 = v_z0 = x0 = y0 = z0 = t0 = distance = duration = energy = 0;
    start = end = false;
    passed = 0;
    showgraph = true;

    pub_v = n->advertise<std_msgs::Float64MultiArray>("/velocity", 1);
    pub_a = n->advertise<std_msgs::Float64MultiArray>("/acceleration", 1);
    pub_e = n->advertise<std_msgs::Float64MultiArray>("/energy", 1);
    pub_d = n->advertise<std_msgs::Float64MultiArray>("/dragF", 1);
    pub_ma = n->advertise<std_msgs::Float64MultiArray>("/inertialF", 1);
    pub_gt = n->advertise<std_msgs::Float64MultiArray>("/goalTime", 1);
    pub_t = n->advertise<std_msgs::Float64MultiArray>("/time", 1);
    sub_score = n->subscribe("/current_score", 1, &Efficiency::getScore, this);
    sub_odom = n->subscribe("/odom", 1000, &Efficiency::odom, this);
  }
  void odom(const nav_msgs::Odometry::ConstPtr& msg);
  void getScore(const std_msgs::Int8::ConstPtr& num);
};

void Efficiency::odom(const nav_msgs::Odometry::ConstPtr &msg) {
  double car_x = msg->pose.pose.position.x;
  double car_y = msg->pose.pose.position.y;
  double car_z = msg->pose.pose.position.z;
  double yaw = msg->pose.pose.orientation.z;
  double v_x = msg->twist.twist.linear.x;
  double v_y = msg->twist.twist.linear.y;
  double v_z = msg->twist.twist.linear.z;

  double diff_radius = sqrt(pow(last_goal[0] - car_x,2) + pow(last_goal[1] - car_y,2) + car_z*car_z);
  double velocity = sqrt(pow(v_x, 2) + pow(v_y, 2) + pow(v_z, 2));

  if (diff_radius < 3 && !end) {
    ros::Duration(1).sleep();
    ROS_INFO("Duration: %f, Distance: %f, Energy: %f",duration, distance, energy);
    end = true;

    if (showgraph) {
      v_dat.data = v_array;
      a_dat.data = a_array;
      e_dat.data = e_array;
      d_dat.data = d_array;
      ma_dat.data = ma_array;
      gt_dat.data = goaltime;
      t_dat.data = t_array;

      while (pub_t.getNumSubscribers() == 0){
        if (ros::ok()) {
          ros::Duration(0.05).sleep();
        }
        else {
          return;
        }
      }
      pub_v.publish(v_dat);
      pub_a.publish(a_dat);
      pub_e.publish(e_dat);
      pub_d.publish(d_dat);
      pub_ma.publish(ma_dat);
      pub_gt.publish(gt_dat);
      ros::Duration(0.5).sleep();
      pub_t.publish(t_dat);
    }
    return;
  }
  else if (end) {
    return;
  }
  else if (start) {
    // double dv = velocity - v0;
    // v0 = velocity;
    double dv = sqrt(pow(v_x - v_x0, 2) + pow(v_y - v_y0, 2) + pow(v_z - v_z0, 2));
    v_x0 = v_x;
    v_y0 = v_y;
    v_z0 = v_z;

    double dd = sqrt(pow(car_x - x0, 2) + pow(car_y - y0, 2) + pow(car_z - z0, 2));
    distance += dd;
    x0 = car_x;
    y0 = car_y;
    z0 = car_z;

    // Calculate dt and save t0
    double t = ros::Time::now().toSec();
    double dt = t - t0;
    duration += dt;
    t0 = t;

    double acceleration = dv/dt;
    double f_d = drag * (velocity * velocity);
    double f_i = mass * acceleration;

    // calculate forces acting against the moving vehicle
    double f_total = f_r + f_d + f_i;
    double e = f_total*dd;
    energy += e;
    // ROS_INFO("duration is %f, dv is %f, dt is %f, dd is %f, energy is %f", duration, dv, dt_s, dd, energy);
    if (showgraph) {
      v_array.push_back(velocity);
      t_array.push_back(duration);
      a_array.push_back(acceleration);
      d_array.push_back(f_d);
      ma_array.push_back(f_i);
      e_array.push_back(e);
    }
  }
  else {
    if (velocity > 0) {
      t0 = ros::Time::now().toSec();
      start = true;
    }
  }
}
void Efficiency::getScore(const std_msgs::Int8::ConstPtr& num) {
  if (showgraph) {
    if (passed != num->data) {
      goaltime.push_back(duration);
      passed = num->data;
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "efficiency_calc_cpp");
  ros::NodeHandle n;
  Efficiency efficiency = Efficiency(&n);
  ros::spin();
  return 0;
}
