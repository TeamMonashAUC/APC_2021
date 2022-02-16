#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <string>
#include <vector>
using std::vector;
// #include <airsim_car_ros_pkgs/CarCmd.h>

class Control {
    private:
      // Constants
      const double pi = 3.14159265358979323846;
      const double rad90 = 90*pi/180;
      const double b_wheel_base = 3;

      // Control parameters
      const double power = 2;
      const double vmax = 5;
      const double a_vmin = 4.5;
      const double a_fvmax = 0.6;
      const double vslow = 12;
      const double vc_atk = 6;
      const double vc_ext = 6;
      const double v3pd = 4;
      const double v3pr = 10;

      // Main parameters
      const int config = 10;
      const double poll_period = 0.2;

      // Declare variables
      double car_x, car_y, car_z, yaw, v_x, v_y, v_z, goal_x, goal_y, arrSize, t_poll, t_tot, t0;
      int goal_type,prev_goal_type,stop_cal_t,count_debug;
      bool stop, end, start;
      vector<int> pose_types;
      vector<vector<double>> pose_seq;
      std::string prev_gear = "";

      // KH
      double throttle, steering, accel;
      vector<vector<double>> mem;

      // Declare publishers, subscribers and messages
      ros::Subscriber sub_odom;
      // No remap
      // ros::Publisher pub_controls;
      // airsim_car_ros_pkgs::CarCmd controls;

      // Remap
      ros::Publisher pub_throttle, pub_steering, pub_gear, pub_brake;
      std_msgs::Float64 throttle_data, brake_data, steering_data;
      std_msgs::String gear_data;

    public:
      Control(ros::NodeHandle *n) {
        car_x = car_y = car_z = yaw = v_x = v_y = v_z = goal_x = goal_y = arrSize = t_poll = t_tot = t0 = count_debug = 0;
        accel = 0;
        goal_type = -1;
        stop = end = start = false;

        sub_odom = n->subscribe("/odom", 5, &Control::odom, this);
        // No remap
        // pub_controls = n->advertise<airsim_car_ros_pkgs::CarCmd>("/airsim_node/PhysXCar/car_cmd_body_frame", 1);

        // Remap
        pub_throttle = n->advertise<std_msgs::Float64>("/throttle_command", 1);
        pub_steering = n->advertise<std_msgs::Float64>("/steering_command", 1);
        pub_gear = n->advertise<std_msgs::String>("/gear_command", 1);
        pub_brake = n->advertise<std_msgs::Float64>("/brake_command", 1);
      }

      void callback(const double time);
      void odom(const nav_msgs::Odometry::ConstPtr& msg);
      void getGoals();
};

void Control::callback(const double time) {
  // double throttle, brake, steering, target_speed, a, fct_z, g_dx, g_dy, omega;
  // throttle = brake = steering = target_speed = a = fct_z = g_dx = g_dy = omega = 0;
  // KH
  double brake, target_speed, a, fct_z, g_dx, g_dy, omega;
  brake = target_speed = a = fct_z = g_dx = g_dy = omega = 0;
  std::string gear = "forward";

  if (goal_type == 7 && stop && !end) {
    ROS_INFO("Controller Time: %.2f",time);
    end = true;
    return;
  }
  else if (end) {
    return;
  }
  // Calculate velocity and offsets to goal using odom data
  double car_speed = sqrt(pow(v_x,2) + pow(v_y,2) + pow(v_z,2));
  double x_diff = goal_x - car_x;
  double y_diff = goal_y - car_y;
  double diff_radius = sqrt(pow(x_diff,2) + pow(y_diff,2) + pow(car_z, 2));

  // Steering control
  if (abs(steering) <=0.2 && goal_type == prev_goal_type){//stop when steering is stable
    stop_cal_t +=1;
    count_debug +=1;
  }
  else {
    stop_cal_t =0;
    prev_goal_type = goal_type;
  }

  if (car_speed > 1 && (stop_cal_t <= 10 || (stop_cal_t-10)%10 == 0)) {
    if (abs(yaw) > 1.0) { // Truncate out-of-bound values
      yaw = 1;
    }
    // Calculate angle offset using forward kinematics
    double theta = 2 * asin(yaw) + rad90;
    g_dx = -(cos(theta) * x_diff - sin(theta) * y_diff);
    g_dy = -(sin(theta) * x_diff + cos(theta) * y_diff);
    a = atan(g_dy / g_dx);
    double gain = 1;
    omega = 0;
    if ((g_dx < 0) && (goal_type == 6 || goal_type == 8 || goal_type == 9 || goal_type == 10 || goal_type == 11)) { // Flip steering sign for reversing
      a = -a;
    }
    if (goal_type == 3 || goal_type == 10) {
      gain = 0.8 * pow(car_speed / 10, 2) + 0.6;
    }
    else {
      gain = 1;
    }
    fct_z = 1.5;
    omega = fct_z * a;

    if (omega != 0) {
      double r = car_speed / -omega;
      double phi = atan(b_wheel_base / r);
      steering = phi * gain;
    }
  }

  if (goal_type == 6 || goal_type == 8 || goal_type == 9 || goal_type == 10 || goal_type == 11) {
    gear = "reverse";
  }

  if (goal_type == 7 && diff_radius < 3) {
    stop = true;
  }

  // Throttle controls
  if (car_speed < 0.5) {
    throttle = 0.5;
  }

  else if (!stop) {
    const vector<double> fct = {0.007, 0.007 , 0.03};
    const double dt = 1;
    double t = ros::Time::now().toSec();
    if (diff_radius < 10 && (goal_type == 5 || goal_type == 11)) {
      mem.clear();
    }
    mem.push_back({t, car_speed});

    double elapsed = mem.back().at(0) - mem.front().at(0);

    if (elapsed > dt) {
      mem.erase(mem.begin());
    }

    if (elapsed > 0) {
      accel = (mem.back().at(1) - mem.front().at(1))/elapsed;
    }

    double increment = fct[0] * (vmax - car_speed)/10;
    double decrement = fct[2] *(car_speed)/10;

    if (!(goal_type == 5  || goal_type == 11)) {
      if (car_speed > 2) { // Limit proportional gain if velocity hasnt reached steady state
        throttle += increment;
      }
      throttle += fct[1] * -accel;
    }
    else {
      if (diff_radius < 30 && throttle >= 0.2) {
        throttle -= decrement;
      }
    }
  }
  else {
    throttle = 0;
  }

  // Truncate values
  if (throttle < 0) {
    throttle = 0;
  }

  if (throttle > 1) {
    throttle = 1;
  }

  // if (brake < 0) {
  //   brake = 0;
  // }
  //
  // if (brake > 1) {
  //   brake = 1;
  // }

  if (abs(steering) > 0.6) {
    steering = 0.6*abs(steering) / steering;
  }

  // Publish data
  // No remap
  // controls.brake = brake;
  // if (gear == "forward") {
  //   controls.is_manual_gear = false;
  //   controls.gear_immediate = false;
  // }
  // else {
  //   controls.is_manual_gear = true;
  //   controls.manual_gear = -1;
  //   controls.gear_immediate = true;
  // }
  // controls.throttle = throttle;
  // controls.steering = steering;
  // pub_controls.publish(controls);

  // // Remap
  throttle_data.data = throttle;
  steering_data.data = -steering;
  if (gear.compare(prev_gear)!= 0 ){
    gear_data.data = gear;
    pub_gear.publish(gear_data);
    prev_gear = gear;
  }
  if (stop_cal_t <= 10 || (stop_cal_t-10)%10 == 0){
    pub_steering.publish(steering_data);
  }
  pub_throttle.publish(throttle_data);

  brake_data.data = brake;
  pub_brake.publish(brake_data);


  // ROS_INFO("Status: [x (m): %f, y (m): %f, yaw (q): %f]", car_x, car_y, yaw);
  // ROS_INFO("Publishing: [D_state: %d, Throttle:  %f, Brake: %f, target_speed: %f, Speed_cur: %f, angular_z: %f, a: %f, steer: %f, goal_type: %d, g_dx: %f, g_dy: %f, diff_radius: %f]" ,state,throttle, brake,target_speed,car_speed,omega,a,steering,goal_type,g_dx,g_dy,diff_radius);

}

void Control::odom(const nav_msgs::Odometry::ConstPtr &msg) {
  if (!end) {
    car_x = msg->pose.pose.position.x;
    car_y = msg->pose.pose.position.y;
    car_z = msg->pose.pose.position.z;
    yaw = msg->pose.pose.orientation.z;
    v_x = msg->twist.twist.linear.x;
    v_y = msg->twist.twist.linear.y;
    v_z = msg->twist.twist.linear.z;

    // Timer
    double t = ros::Time::now().toSec();
    double dt = 0;
    if (!start) {
      start = true;
      ROS_INFO("Starting control loop(cpp) with polling period: %.2fs", poll_period);
      dt = 0;
      callback(t_tot);
    }
    else {
      dt = t - t0;
    }

    t_poll += dt;
    t_tot += dt;
    t0 = t;

    // Goals
    if (arrSize > 0) {
      int rad = 5;
      if (goal_type == 5 || goal_type == 11)
      {
        rad = 3;
      }
      double diff = sqrt(pow(car_x - goal_x,2) + pow(car_y - goal_y,2) + pow(car_z,2));
      if (diff < rad) {
        pose_seq.erase(pose_seq.begin());
        pose_types.erase(pose_types.begin());
        arrSize -= 1;
        if (arrSize > 0) {
          goal_x = pose_seq[0][0];
          goal_y = pose_seq[0][1];
          goal_type = pose_types[0];
        }
      }
    }
    // Controller callback @ poll_period
    double t_err = abs(t_poll - poll_period)/poll_period;
    if (t_err < 0.05 || t_poll > poll_period) {
      t_poll = 0;
      callback(t_tot);
    }
  }
  else {
    return;
  }
}

void Control::getGoals() {
  ROS_INFO("Loading goals for configuration: %d", config);
  // ros::Duration(1).sleep();

  switch (config) {
    case 1:
        // # Config 1 - Default (~1540 m, ~167s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-122},{-197,-128},{-179,-128},{-135.5,-128},{-90,-128},{-84,-143},{-84,-153},{-84,-175},{-83,-195},{-79,-203},{-78, -207},{-84, -208}, {-85, -205}, {-85,-195},{-84,-128},{-84,-68},{-84,-58},{-99,-48},{-115,-48},{-135,-48},{-203, -48},{-212, -63},{-212, -78},{-212,-198},{-212, -241},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-80,-256},{-20, -256},{34.5, -256},{44, -241},{44, -220},{44, -198},{44, -138},{29, -128},{1,-128},{7.5, -133},{15, -128},{37, -128},{44, -110}, {44,-95}, {44.5, -65.5}};
        pose_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,1,2,3,4,1,2,3,4,1,1,2,3,4,0,4,1,2,3,4,1,2,7,6,5,1,2,3,7};
        break;
    case 2:
        // # Config 2 - Efficiency (< Distance, < Time), (~1520 m, ~162s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128},{-94,-128},{-84,-143},{-84,-153},{-84,-198},{-84,-246},{-99,-256}, {-114, -256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-120}, {-69,-128}, {-54,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-79},{39,-71},{38, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
        pose_types = {0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
        break;
    case 3:
        // # Config 3 - Fastest (> Distance, <<< Time), (~1555 m, ~144s)
        pose_seq = {{-80, 0}, {-135, 0}, {-205, 0}, {-212, -15}, {-212, -30}, {-212, -74}, {-212,-120},{-197,-128},{-179,-128},{-135.5,-128}, {-70, -128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -79}, {43,-75}, {39, -67},{44, -66}, {45, -69}, {45,-77},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{-70,-256}, {-135.5, -256}, {-202, -256}, {-212, -241}, {-212, -226},{-212,-198},{-212, -57},{-197, -48}, {-183,-48}, {-135,-48}, {-94,-48}, {-84,-63}, {-84,-78}, {-84,-198}};
        pose_types = {0,4,1,2,3,4,1,2,3,4,0,4,1,2,3,4,1,5,5,5,3,4,1,2,3,4,0,4,1,2,3,4,1,2,3,4,1,2,3,7};
        break;
    case 4:
        // # Config 4 - Shortest (<<< Distance, >> Time), (~1440 m, ~180s)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-163,-48}, {-141,-49},{-133,-51},{-130, -48},{-144, -48}, {-153, -48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-163,-128}, {-141,-129},{-133,-131},{-130, -128},{-143, -128},{-153, -128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-72,-128}, {-55,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -90}, {43,-78},{39,-69},{38, -67},{44, -65}, {45, -67}, {44,-85},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256}};
        pose_types = {0,4,1,2,3,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7};
        break;
    case 5:
        // # Config 5 - Shortest (<<<< Distance, >> Time), (~1398 m, ~215s) (reverse, forward, reverse)
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-202, -48},{-212, -63},{-212, -78},{-212, -74}, {-212,-120},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-140},{-74,-128}, {-59,-128}, {0,-128},{34, -128},{44, -110}, {44,-95}, {44, -65},{44, -198},{44, -246},{29, -256},{14,-256},{0, -256},{0, -256}};
        pose_types = {0,4,1,2,3,1,2,3,4,8,9,10,6,8,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,6,8,9,10,6,7};
        break;
    case 6:
        // # Config 6 - Shortest (<<<< Distance, >>> Time), (~1400 m, ~200s) (reverse, forward, reverse) x2
        pose_seq = {{-80, 0}, {-135, 0.5}, {-203, 0}, {-212, -15}, {-212, -30},{-212, -38},{-197, -48}, {-183,-48}, {-135,-48},{-199, -48},{-212, -65},{-212, -78},{-212, -74}, {-212,-116},{-197,-128},{-179,-128}, {-135,-128},{-202,-128}, {-212,-143}, {-212, -158}, {-212,-198},{-212, -246},{-197, -256},{-182, -256},{-135.5, -256},{-94,-256},{-84,-241},{-84,-226},{-84.5,-198},{-84,-244},{-69,-256},{-54, -256}, {0, -256}, {31, -256}, {44, -241}, {44, -226}, {44, -198}, {44, -138},{29, -128},{1,-128},{34, -128},{44, -110}, {44,-95}, {44.5, -65}, {44.5, -65}};
        pose_types = {0,4,1,2,3,1,2,3,4,6,8,6,6,6,8,6,6,1,2,3,4,1,2,3,4,1,2,3,4,6,8,6,6,6,8,6,6,6,8,6,1,2,3,1,7};
        break;
    case 7:
        // # Config 7 - Start backwards
        pose_seq = {{0,0},{35, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-110},{34,-127},{2,-129},{34,-129},{44,-143},{44,-158},{44,-198},{44, -246},{29, -256},{14,-256},{0, -256},{-74,-256},{-84,-241},{-84,-226},{-84.5,-200},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-143},{-202,-130},{-137,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-137,-48},{-183,-48},{-197,-48},{-210,-37},{-212,-30},{-212,-15},{-205,-1},{-180,0.0},{-135,1},{-135,1}};
        pose_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,8,9,10,8,9,10,6,6,7};
        break;
    case 8:
        // # Config 8 - 7 & end forward
        pose_seq = {{0,0},{32, 0},{44, -15},{44,-30},{44,-65},{44,-95},{44,-115},{28,-127},{4.5,-128},{34,-129},{44,-143},{44,-158},{44,-198},{44, -241},{27, -256},{12,-256},{0, -256},{-70,-256},{-84,-241},{-84,-226},{-84.5,-199},{-84,-226},{-84,-241},{-94,-256},{-135.5,-256},{-182,-256},{-197,-256},{-210,-246},{-212,-195},{-212,-158},{-212,-145},{-200,-130},{-142,-128},{-202,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-200,-49},{-135,-48},{-94,-48},{-84,-33},{-84,-18},{-84,-13},{-103,0},{-114,0},{-135,1}};
        pose_types = {0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,1,2,3,1,2,3,7};
        break;
    case 9:
        pose_seq = {{0,0},{24,-0.5},{32, -0.5},{44, -20},{44,-35},{44,-65},{44,-95},{44,-113},{25,-127},{0,-128},{15,-128},{33,-129},{44,-143},{44,-158},{44,-198},{44,-230},{44, -241},{27, -256},{12,-256},{0, -256},{-59,-256},{-73,-256},{-84,-241},{-84,-226},{-84.5,-198},{-85,-235},{-84,-244},{-94,-255},{-135.5,-255},{-178,-253},{-196.5,-242.5},{-207,-218},{-211,-198},{-212,-158},{-212,-145},{-200,-130},{-135.5,-128},{-150,-128},{-190,-128},{-203,-128},{-212,-113},{-212,-98},{-212,-74},{-212,-78},{-212,-63},{-199,-49},{-135,-48},{-110,-48},{-93,-48},{-85,-33},{-85,-18},{-85,-13},{-103,0},{-114,0},{-135,1}};
        pose_types = {0,6,8,9,10,6,6,9,10,11,1,0,2,3,4,0,1,2,3,4,0,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,0,1,2,3,4,4,2,3,4,0,1,2,3,1,2,3,7};
        break;
    case 10:
        pose_seq = {{0,0},{34.5, 0},{44,-30}, {44,-115},{29,-124},{-0.5,-128},{37,-130},{42,-143},{44,-198},{43, -244},{29, -252.5},{0, -256},{-76,-255},{-81.5,-241},{-84.5,-197},{-84,-246},{-99,-255},{-114,-255},{-135.5,-255.5},{-178,-253},{-197,-242.5},{-207,-219},{-211.5,-198},{-210.5,-140},{-197,-133},{-135,-128},{-205,-127},{-210,-115},{-212,-98},{-212,-74},{-211,-60},{-199,-49},{-135,-48},{-92,-47.5},{-86,-30.5},{-86,-19},{-86,-11},{-98,-4},{-135,1}};
        pose_types = {0,8,9,8,9,11,1,2,4,1,2,4,1,2,5,8,9,10,6,8,9,10,6,8,9,11,1,2,3,4,1,2,4,1,2,3,1,2,7};
        break;
  }
  arrSize = pose_types.size();
  goal_x = pose_seq[0][0];
  goal_y = pose_seq[0][1];
  goal_type = pose_types[0];
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_car_control");
  ros::NodeHandle n;

  // Initialize nodelets and get goals
  Control control = Control(&n); // Call constructor to add subscribers and publishers to node
  ROS_INFO("Initialized control node");
  control.getGoals();

  // Reset parameters
  const double tstart = ros::Time::now().toSec();
  double tE = 0;

  while (tE < 10) { // Check for 3 secs
    tE = ros::Time::now().toSec() - tstart;
    if (ros::param::has("/airsim_node/update_airsim_control_every_n_sec")) {
      ros::param::set("/airsim_node/update_airsim_control_every_n_sec", 0.2);
    }
    if (ros::param::has("/airsim_node/update_airsim_img_response_every_n_sec")) {
      ros::param::set("/airsim_node/update_airsim_img_response_every_n_sec", 500);
    }
    if (ros::param::has("/airsim_node/update_lidar_every_n_sec")) {
      ros::param::set("/airsim_node/update_lidar_every_n_sec", 500);
      ROS_INFO("Parameters set!");
      break;
    }
  }
  // Wait for odom message to load
  ROS_INFO("Checking for odom...");
  boost::shared_ptr<nav_msgs::Odometry const> data;
  data = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", ros::Duration(20));

  if (data != NULL){
    ros::spin();
  }
  else {
    ROS_INFO("No odom data!");
  }
  // ros::MultiThreadedSpinner spinner(3); // Total no. of threads = Minimum(5) + input number
  // spinner.spin();
  return 0;
}
