#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <string>
#include <vector>
using namespace std;

// control_master_simple.cpp
// Author(s): Jayson Teh, Navaneeth Nair, Zi Yu
// Documented by: Navaneeth Nair R Dhileepan
// School: Monash University Malaysia
// Description: - C++ source code that controls the PhysXCar in AirSim from start to finish
//                by subscribing to the car's odometry data, calculating the required throttle,
//                gear & steering values based on pre-defined goal sequences and goal types,
//                and publish these values to AirSim to move the car at a defined period.
//              - This _simple variant simplifies the throttle to basic increments and decrements for low speeds (<= 5m/s)

class Control { // Control class for modular code
private:
  // Constants
  const double pi = 3.14159265358979323846;
  const double rad90 = 90*pi/180; // 90 degrees in radians
  const double b_wheel_base = 3; // Wheelbase length [m]
  const vector<double> fct = {0.35, 0.005 , 0.02}; // Throttle factors

  // Main parameters
  const int config = 10; // Goal sequence configuration
  const double poll_period = 0.2; // Period of calling callback() function

  // Declare method attributes (variables global to class)
  double car_x, car_y, car_z, yaw, v_x, v_y, v_z, goal_x, goal_y, arrSize, t_poll, t_tot, t0, throttle, steering, prev_gas;
  int goal_type, prev_goal_type, stop_cal_t;
  bool stop, end, start, move;
  vector<int> pose_types; // 1D array of type 'int'
  vector<vector<double>> pose_seq; // 2D array of type 'double'
  string prev_gear = "";

  // Declare publishers, subscribers and messages
  ros::Subscriber sub_odom;
  ros::Publisher pub_throttle, pub_steering, pub_gear;
  std_msgs::Float64 throttle_data, steering_data;
  std_msgs::String gear_data;

public:
  Control(ros::NodeHandle *n) { // Contructor: The deferenced pointer of type ros::NodeHandle is passed, which add these nodelets to this ROS node.
    // Initialize method attributes
    car_x = car_y = car_z = yaw = v_x = v_y = v_z = goal_x = goal_y = arrSize = t_poll = t_tot = t0 = throttle = steering = prev_gas = 0;
    stop_cal_t = 0;
    goal_type = prev_goal_type = -1;
    stop = end = start = move = false;

    // Define publishers and subscribers (nodelets).
    sub_odom = n->subscribe("/odom", 5, &Control::odom, this); // ("/topic name", queue size, class method, class)
    pub_throttle = n->advertise<std_msgs::Float64>("/throttle_command", 1); // <data type>("/topic name", queue size)
    pub_steering = n->advertise<std_msgs::Float64>("/steering_command", 1);
    pub_gear = n->advertise<std_msgs::String>("/gear_command", 1);
  }

  // Declare class methods
  void callback(double *time); // Pointer to time variable is passed
  void odom(const nav_msgs::Odometry::ConstPtr& msg); // Pointer to message is passed
  void getGoals();
};

// Class method that performs the calculation for controlling the car, total time elapsed from controller start is passed
void Control::callback(double *time) {
  string gear = "forward"; // Initialize gear to forward

  if (goal_type == 7 && stop && !end) { // End condition
    ROS_INFO("Controller Time: %.4f", *time); // Display total controller time
    end = true;
    return;
  }
  else if (end) { // Exit control when reached final goal
    return;
  }
  // Calculate velocity as a magnitude of cartesian vectors
  double car_speed = sqrt(pow(v_x,2) + pow(v_y,2) + pow(v_z,2));
  // Calculate offsets to goal using odom data
  double x_diff = goal_x - car_x;
  double y_diff = goal_y - car_y;
  // Calculate radial distance
  double diff_radius = sqrt(pow(x_diff,2) + pow(y_diff,2) + pow(car_z, 2));

  if (goal_type == 6 || goal_type == 8 || goal_type == 9 || goal_type == 10 || goal_type == 11) { // Reverse goal types
    gear = "reverse";
  }

  // Stop calculating steering when it reaches steady state
  if (abs(steering) <= 0.2 && goal_type == prev_goal_type){
    stop_cal_t += 1;
  }
  else {
    stop_cal_t = 0;
    prev_goal_type = goal_type;
  }

  // Steering control
  if (car_speed > 1 && (stop_cal_t <= 10 || (stop_cal_t-10)%10 == 0)) { // Settling time = 10 x poll_period
    if (abs(yaw) > 1.0) { // Truncate out-of-bound values
      yaw = 1;
    }
    // Convert quartenion angle to euler angle
    double theta = 2 * asin(yaw) + rad90;
    // Transform goal coordinates using forward kinematics
    double g_dx = -(cos(theta) * x_diff - sin(theta) * y_diff);
    double g_dy = -(sin(theta) * x_diff + cos(theta) * y_diff);
    double a = atan(g_dy / g_dx); // alpha
    if (g_dx < 0 && gear == "reverse") { // Flip steering sign for reversing
      a = -a;
    }
    double omega = 1.5 * a; // Scalar constant to define angular velocity omega

    if (omega != 0) {
      // Apply Ackermann's steering
      double r = car_speed / -omega;
      steering = atan(b_wheel_base / r);
    }
  }

  if (goal_type == 7 && diff_radius < 3) { // End condition
    stop = true;
  }

  // Throttle controls
  if (car_speed < 0.5) { // Low speed condition
    throttle = 0.5;
  }
  else if (!stop) {  // not at final goal
    move = true; // Flag when car starts moving
    if (goal_type != 5 && goal_type != 11) { // Not at stop & go goal
      if (car_speed < 2) {
        throttle = fct[0]; // Base throttle
      }
      else if (throttle < 0.49) { // Limit max throttle
        throttle += fct[1]; // increment throttle
      }
    }
    else {
      if (diff_radius < 20 && throttle > 0.1) {
        throttle -= fct[2]; // decrement throttle when close to stop & go, min 0.1
      }
    }
  }
  else { // End condition
    throttle = 0;
  }

  if (abs(steering) > 0.6) { // Limit max steering
    steering = 0.6*abs(steering) / steering;
  }

  // Publish data
  if (!move) { // Always publish gear & throttle at the start to prevent synching issues
    gear_data.data = gear;
    pub_gear.publish(gear_data);
    throttle_data.data = throttle;
    pub_throttle.publish(throttle_data);
  }
  else { // Publish gear and throttle only during changes
    if (gear.compare(prev_gear)!= 0){
      throttle = 1; // 'Brake' the car by counter-throttling
      steering = 0; // Reset steering
      prev_gear = gear;
      gear_data.data = gear; // update previous gear
      pub_gear.publish(gear_data);
    }
    if (prev_gas != throttle) {
      throttle_data.data = throttle;
      pub_throttle.publish(throttle_data);
      prev_gas = throttle; // update previous throttle
    }
  }
  if (stop_cal_t <= 10 || (stop_cal_t-10)%10 == 0){ // Stop publishing steering when reached steady state
    steering_data.data = -steering; // Flip sign to satisfy competition environment conditions
    pub_steering.publish(steering_data);
  }
}

// Class method that gets called when odometry message is published to /odom by the AirSim-ROS wrapper, and passed to msg variable
void Control::odom(const nav_msgs::Odometry::ConstPtr &msg) {
  if (!end) { // Perform operations while end condition is not true
    // Get cartesian positions
    car_x = msg->pose.pose.position.x;
    car_y = msg->pose.pose.position.y;
    car_z = msg->pose.pose.position.z;
    // Get z-quartenion orientation
    yaw = msg->pose.pose.orientation.z;
    // Get cartesian velocities
    v_x = msg->twist.twist.linear.x;
    v_y = msg->twist.twist.linear.y;
    v_z = msg->twist.twist.linear.z;

    // Timer
    double t = ros::Time::now().toSec(); // Get current time in seconds
    double dt = 0; // Start condition
    if (!start) {
      start = true;
      ROS_INFO("Starting control loop(cpp) with polling period: %.2fs", poll_period); // Report set period
      dt = 0;
      callback(&t_tot); // Callback at t = 0
    }
    else {
      dt = t - t0; // Calculate time elapsed since last time step
    }

    t_poll += dt; // Total time elapsed from last call to callback function
    t_tot += dt; // Total time elapsed from start
    t0 = t; // Update current time for next step

    // Update goal sequence
    if (arrSize > 0) {
      int rad = 5;
      if (goal_type == 5 || goal_type == 11) // Smaller radius for stop & go, and [0,0] goal
      {
        rad = 3;
      }
      double diff = sqrt(pow(car_x - goal_x,2) + pow(car_y - goal_y,2) + pow(car_z,2)); // Radial distance to goal
      if (diff < rad) { // Car reaches within radius defined above
        // Delete goal and its type
        pose_seq.erase(pose_seq.begin());
        pose_types.erase(pose_types.begin());
        arrSize -= 1;
        if (arrSize > 0) {
          // Update goal coordinates and type
          goal_x = pose_seq[0][0];
          goal_y = pose_seq[0][1];
          goal_type = pose_types[0];
        }
      }
    }
    // Controller callback @ poll_period
    double t_err = abs(t_poll - poll_period)/poll_period; // Calculate error
    if (t_err < 0.05 || t_poll > poll_period) { // callback when within 5% precision or when poll elapsed time exceeds defined period
      t_poll = 0; // Reset callback elapsed time
      callback(&t_tot); // Variable's pointer reference is passed
    }
  }
  else {
    return;
  }
}

// Class method that allocates array of coordinates and goal types
void Control::getGoals() {
  ROS_INFO("Loading goals for configuration: %d", config);

  // Allocate goals array based on defined config at class declaration
  switch (config) {
    case 1:
    // Config 1 - Default (~1540 m, ~167s)
    pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-122},{-197,-128},{-179,-128},{-135.5,-128},{-90,-128},{-84,-143},{-84,-153},{-84,-175},{-83,-195},{-79,-203},{-78, -207},{-84, -208}, {-85, -205}, {-85,-195},{-84,-128},{-84,-68},{-84,-58},{-99,-48},{-115,-48},{-135,-48},{-203, -48},{-212, -63},{-212, -78},{-212,-198},{-212, -241},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-80,-256},{-20, -256},{34.5, -256},{44, -241},{44, -220},{44, -198},{44, -138},{29, -128},{1,-128},{7.5, -133},{15, -128},{37, -128},{44, -110}, {44,-95}, {44.5, -65.5}};
    pose_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,1,2,3,4,1,2,3,4,1,1,2,3,4,0,4,1,2,3,4,1,2,7,6,5,1,2,3,7};
    break;
    case 2:
    // Config 2 - Efficiency (< Distance, < Time), (~1520 m, ~162s)
    pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128},{-94,-128},{-84,-143},{-84,-153},{-84,-198},{-84,-246},{-99,-256}, {-114, -256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-120}, {-69,-128}, {-54,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-79},{39,-71},{38, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
    pose_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
    break;
    case 3:
    // Config 3 - Fastest (> Distance, <<< Time), (~1555 m, ~144s)
    pose_seq = {{-80, 0}, {-135, 0}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128}, {-70, -128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -79}, {43,-75}, {39, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{-70,-256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-198}};
    pose_types = {0,4,1,2,3,4,1,2,3,4,0,4,1,2,3,4,1,5,5,5,3,4,1,2,3,4,0,4,1,2,3,4,1,2,3,4,1,2,3,7};
    break;
    case 4:
    // Config 4 - Shortest (<<< Distance, >> Time), (~1440 m, ~180s)
    pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-163,-48}, {-141,-49},{-133,-51},{-130, -48},{-144, -48}, {-153, -48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-163,-128}, {-141,-129},{-133,-131},{-130, -128},{-143, -128},{-153, -128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-72,-128}, {-55,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-78},{39,-69},{38, -67},{44, -65}, {45, -67}, {44,-85},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
    pose_types = {0,4,1,2,3,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
    break;
    case 5:
    // Config 5 - Shortest (<<<< Distance, >> Time), (~1398 m, ~215s) (reverse, forward, reverse)
    pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-74,-128}, {-59,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -65},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{0, -256}};
    pose_types = {0,4,1,2,3,1,2,3,4,8,9,10,6,8,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,6,8,9,10,6,7};
    break;
    case 6:
    // Config 6 - Shortest (<<<< Distance, >>> Time), (~1400 m, ~200s) (reverse, forward, reverse) x2
    pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-199, -48},{-212, -65},{-212, -78},{-212, -74}, {-212,-116},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-244},{-69,-256},{-54, -256}, {0, -256}, {31, -256}, {44, -241}, {44, -226}, {44, -198}, {44, -138},{29, -128},{1,-128},{34, -128},{44, -110}, {44,-95}, {44.5, -65}, {44.5, -65}};
    pose_types = {0,4,1,2,3,1,2,3,4,6,8,6,6,6,8,6,6,1,2,3,4,1,2,3,4,1,2,3,4,6,8,6,6,6,8,6,6,6,8,6,1,2,3,1,7};
    break;
    case 7:
    // Config 7 - Start backwards
    pose_seq = {{0,0},{35, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-110},{34,-127},{2,-129},{34,-129},{44,-143},{44,-158},{44,-198},{44, -246},{29, -256},{14,-256},{0, -256},{-74,-256},{-84,-241},{-84,-226},{-84.5,-200},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-143},{-202,-130},{-137,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-137,-48},{-183,-48},{-197,-48},{-210,-37},{-212,-30},{-212,-15},{-205,-1},{-180,0.0},{-135,1},{-135,1}};
    pose_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,8,9,10,8,9,10,6,6,7};
    break;
    case 8:
    // Config 8 - 7 & end forward
    pose_seq = {{0,0},{32, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-115},{28,-127},{4.5,-128},{34,-129},{44,-143},{44,-158},{44,-198},{44, -241},{27, -256},{12,-256},{0, -256},{-70,-256},{-84,-241},{-84,-226},{-84.5,-199},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-145},{-200,-130},{-142,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-135,-48},{-94,-48},{-84,-33},{-84,-18},{-84,-13},{-103,0},{-114,0},{-135,1}};
    pose_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,1,2,3,1,2,3,7};
    break;
    case 9:
    // Config 9 - Grass cutting service
    pose_seq = {{0,0},{24,-0.5},{32, -0.5},{44, -20},{44,-35},{44,-65},{44,-95},{44,-113},{25,-127},{0,-128},{15,-128},{33,-129},{44,-143},{44,-158},{44,-198},{44,-230},{44, -241},{27, -256},{12,-256},{0, -256},{-59,-256},{-73,-256},{-84,-241},{-84,-226},{-84.5,-198},{-85,-235},{-84,-244},{-94,-255},{-135.5,-255},{-178,-253},{-196.5,-242.5},{-207,-218},{-211,-198},{-212,-158},{-212,-145},{-200,-130},{-135.5,-128},{-150,-128},{-190,-128},{-203,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-199,-49},{-135,-48},{-110,-48},{-93,-48},{-85,-33},{-85,-18},{-85,-13},{-103,0},{-114,0},{-135,1}};
    pose_types = {0,6,8,9,10,6,6,9,10,11,1,0,2,3,4,0,1,2,3,4,0,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,0,1,2,3,4,4,2,3,4,0,1,2,3,1,2,3,7};
    break;
    case 10:
    // Config 10 - Optimized grass cutting
    pose_seq = {{0,0},{34.5, 0},{44,-30}, {44,-115},{29,-124},{-0.5,-128},{37,-130},{42,-143},{44,-198},{43, -244},{29, -252.5},{0, -256},{-76,-255},{-81.5,-241},{-84.5,-197},{-84,-246},{-99,-255},{-114,-255},{-135.5,-255.5},{-178,-253},{-197,-242.5},{-207,-219},{-211.5,-198},{-210.5,-140},{-197,-133},{-135,-128},{-205,-127},{-210,-115},{-212,-98},{-212,-74},{-211,-60},{-199,-49},{-135,-48},{-92,-47.5},{-86,-30.5},{-86,-19},{-86,-11},{-98,-4},{-135,1}};
    pose_types = {0,8,9,8,9,11,1,2,4,1,2,4,1,2,5,8,9,10,6,8,9,10,6,8,9,11,1,2,3,4,1,2,4,1,2,3,1,2,7};
    break;
    default:
    // Config 10 - Optimized grass cutting
    pose_seq = {{0,0},{34.5, 0},{44,-30}, {44,-115},{29,-124},{-0.5,-128},{37,-130},{42,-143},{44,-198},{43, -244},{29, -252.5},{0, -256},{-76,-255},{-81.5,-241},{-84.5,-197},{-84,-246},{-99,-255},{-114,-255},{-135.5,-255.5},{-178,-253},{-197,-242.5},{-207,-219},{-211.5,-198},{-210.5,-140},{-197,-133},{-135,-128},{-205,-127},{-210,-115},{-212,-98},{-212,-74},{-211,-60},{-199,-49},{-135,-48},{-92,-47.5},{-86,-30.5},{-86,-19},{-86,-11},{-98,-4},{-135,1}};
    pose_types = {0,8,9,8,9,11,1,2,4,1,2,4,1,2,5,8,9,10,6,8,9,10,6,8,9,11,1,2,3,4,1,2,4,1,2,3,1,2,7};
    break;
  }
  // Initialize goal coordinates, array size and goal type
  arrSize = pose_types.size();
  goal_x = pose_seq[0][0];
  goal_y = pose_seq[0][1];
  goal_type = pose_types[0];
}

// Main function
int main(int argc, char **argv)
{
  // Initialize control node
  ros::init(argc, argv, "cmd_car_control");
  ros::NodeHandle n; // ROS object that stores all the nodelets for this node (subscribers, publishers, services, etc.)

  // Initialize nodelets and get goals
  Control control = Control(&n); // Constructor to add subscribers and publishers to node, pointer reference of NodeHandle is passed
  ROS_INFO("Initialized control node");
  control.getGoals();

  // Set AirSim parameters
  const double tstart = ros::Time::now().toSec(); // Current time in seconds
  double tE = 0; // Initialize elapsed time
  bool i = false; // Flag to check if img params is set

  while (tE < 10) { // Check for 10 secs
    tE = ros::Time::now().toSec() - tstart; // Update elapsed time
    // If parameters are available in the parameter server, modify it
    if (ros::param::has("/airsim_node/update_airsim_img_response_every_n_sec")) {
      ros::param::set("/airsim_node/update_airsim_img_response_every_n_sec", 0.5);
      i = true;
    }
    if (ros::param::has("/airsim_node/update_lidar_every_n_sec")) {
      ros::param::set("/airsim_node/update_lidar_every_n_sec", 500);
      if (i) {
        ROS_INFO("Set!");
        break;
      }
    }
  }
  // Wait for odom message to load
  ROS_INFO("Checking for odom...");
  boost::shared_ptr<nav_msgs::Odometry const> data;
  data = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(20)); // Blocks until message is received, 20s timeout

  if (data != NULL){
    ros::spin(); // Block the node from exiting
  }
  else {
    ROS_INFO("No odom data!");
  }
  return 0;
}
