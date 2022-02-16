#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <cmath>
#include <string>

class Control {
    private:
      // Declare constants & variables
      const double pi = 3.14159265358979323846;
      const double rad90 = 90*pi/180;
      const double b_wheel_base = 3;
      const double power = 2;
      const double vmax = 16;
      const double a_vmin = 0.9;
      const double a_fvmax = 0.6;
      const double vslow = 12;
      const double vc_atk = 6;
      const double vc_ext = 6;
      const double v3pd = 4;
      const double v3pr = 10;
      double car_x;
      double car_y;
      double yaw;
      double v_x;
      double v_y;
      double v_z;
      double goal_x;
      double goal_y;
      int goal_type;
      bool stop;
      bool end;
      bool fow;
      ros::Publisher pub_throttle;
      // ros::Publisher pub_brake;
      ros::Publisher pub_steering;
      ros::Publisher pub_gear;
      ros::Subscriber sub_odom;
      ros::Subscriber sub_goal;
      ros::Subscriber sub_control;
    public:
      Control(ros::NodeHandle *n) {
        car_x = 0;
        car_y = 0;
        yaw = 0;
        v_x = 0;
        v_y = 0;
        v_z = 0;
        goal_x = 0;
        goal_y = 0;
        goal_type = -1;
        stop = false;
        end = false;
        fow = true;
        pub_throttle = n->advertise<std_msgs::Float64>("/throttle_command", 2);
        // pub_brake = n->advertise<std_msgs::Float64>("/brake_command", 10);
        pub_steering = n->advertise<std_msgs::Float64>("/steering_command", 2);
        pub_gear = n->advertise<std_msgs::String>("/gear_command", 2);
        sub_odom = n->subscribe("/odom", 5, &Control::odom, this);
        sub_goal = n->subscribe("/goals", 1, &Control::goalCall, this);
        sub_control = n->subscribe("/control_timer", 2, &Control::callback, this);
      }

      void callback(const std_msgs::Float64::ConstPtr& time);
      void odom(const nav_msgs::Odometry::ConstPtr& msg);
      void goalCall(const geometry_msgs::Vector3::ConstPtr& goal);
};

void Control::callback(const std_msgs::Float64::ConstPtr &time) {
  double throttle = 0;
  double brake = 0;
  double steering = 0;
  std::string gear = "forward";
  double target_speed = 0;
  double a = 0;
  double fct_z = 0;
  int state = 0;
  double g_dx = 0;
  double g_dy = 0;
  double omega = 0;

  if (goal_type == 7 && stop && !end) {
    ROS_INFO("Time: %.2f",time->data);
    end = true;
    return;
  }
  else if (end) {
    return;
  }
  double car_speed = sqrt(pow(v_x,2) + pow(v_y,2) + pow(v_z,2));
  double x_diff = goal_x - car_x;
  double y_diff = goal_y - car_y;
  double diff_radius = sqrt(pow(x_diff,2) + pow(y_diff,2));

  if (car_speed > 1) {
    if (abs(yaw) > 1.0) {
      yaw = 1;
    }
    double theta = 2 * asin(yaw) + rad90;
    g_dx = -(cos(theta) * x_diff - sin(theta) * y_diff);
    g_dy = -(sin(theta) * x_diff + cos(theta) * y_diff);
    a = atan(g_dy / g_dx);
    double gain = 1;
    omega = 0;
    if ((g_dx < 0) && (goal_type == 6 || goal_type == 8 || goal_type == 9 || goal_type == 10 || goal_type == 11)) {
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

  if (goal_type == 7 && diff_radius < 1) {
    stop = true;
  }

  if (!stop) {
    target_speed = a_fvmax * pow(car_speed, power) + a_vmin;
    if (diff_radius < 96 && (goal_type == 5 || goal_type == 11)) {
      if (car_speed > 8) {
      	throttle = 0.015 * diff_radius + 0.1;
      }
      else {
         throttle = 0.0075 * diff_radius + 0.2;
      }
      if (goal_type == 11) {
      	gear = "reverse";
      }
    }
    else if ((goal_type == 0 || goal_type == 1 || goal_type == 4 || goal_type == 5 || goal_type == 6 || goal_type == 7 || goal_type == 8) || goal_type == 11) { // Straights
      if (goal_type == 0 || goal_type == 4 || goal_type == 5 || goal_type == 6 || goal_type == 11) {
        state = 1;
      	if (target_speed > vmax) {
      	  target_speed = vmax;
      	}
      }
      else {
        state = 2;
        if (target_speed > vslow) {
          target_speed = vslow;
        }
      }

      double t_min = 0.15;
      double diff_speed = target_speed - car_speed;
      if (car_speed < 3) {
      	throttle = 0.4;
      }
      else if (car_speed < target_speed) {
        throttle = diff_speed / target_speed + t_min;
      }
      else {
        throttle = 0;
      }
      if (goal_type == 6 || goal_type == 8 || goal_type == 11) {
        state = -state;
        gear = "reverse";
      }
    }
    else if  (goal_type == 2 || goal_type == 3 || goal_type == 9 || goal_type == 10) {
      if (goal_type == 2 || goal_type == 9) {
        state = 3;
        if (target_speed > vc_atk) {
      	  target_speed = vc_atk;
      	}
      }
      else {
        state = 4;
        if (target_speed > vc_ext) {
      	  target_speed = vc_ext;
      	}
      }
      double diff_speed = target_speed - car_speed;
      if (target_speed >= car_speed) {
        throttle = diff_speed/target_speed + 0.18;
      }
      if (goal_type == 9 || goal_type == 10) {
        state = -state;
        gear = "reverse";
      }
    }
  }
  else {
    state = 7;
    throttle = 0;
  }

  if (!fow && !(goal_type == 6 || goal_type == 8 || goal_type == 9 || goal_type == 10 || goal_type == 11)) {
    throttle = 1;
  }

  if (gear == "reverse") {
    fow = false;
  }
  else {
    fow = true;
  }

  if (throttle < 0) {
    throttle = 0;
  }
  if (brake < 0) {
    brake = 0;
  }

  if (throttle > 1) {
    throttle = 1;
  }

  if (brake > 1) {
    brake = 1;
  }

  if (abs(steering) > 1) {
    steering = abs(steering) / steering;
  }

  // Publish data
  // With remap
  std_msgs::Float64 throttle_data;
  // std_msgs::Float64 brake_data;
  std_msgs::Float64 steering_data;
  std_msgs::String gear_data;
  throttle_data.data = throttle;
  // brake_data.data = brake;
  gear_data.data = gear;
  steering_data.data = -steering;
  pub_throttle.publish(throttle_data);
  // pub_brake.publish(brake_data);
  pub_gear.publish(gear_data);
  pub_steering.publish(steering_data);

  // ROS_INFO("Status: [x (m): %f, y (m): %f, yaw (q): %f]", car_x, car_y, yaw);
  // ROS_INFO("Publishing: [D_state: %d, Throttle:  %f, Brake: %f, target_speed: %f, Speed_cur: %f, angular_z: %f, a: %f, steer: %f, goal_type: %d, g_dx: %f, g_dy: %f, diff_radius: %f]" ,state,throttle, brake,target_speed,car_speed,omega,a,steering,goal_type,g_dx,g_dy,diff_radius);

}

void Control::odom(const nav_msgs::Odometry::ConstPtr &msg) {
  car_x = msg->pose.pose.position.x;
  car_y = msg->pose.pose.position.y;
  yaw = msg->pose.pose.orientation.z;
  v_x = msg->twist.twist.linear.x;
  v_y = msg->twist.twist.linear.y;
  v_z = msg->twist.twist.linear.z;
}

void Control::goalCall(const geometry_msgs::Vector3::ConstPtr &goal) {
  goal_x = goal->x;
  goal_y = goal->y;
  goal_type = goal->z;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_car_control");
  ros::NodeHandle n;
  Control control = Control(&n);
  ros::spin();
  // ros::MultiThreadedSpinner spinner(3); // Total no. of threads = Minimum(5) + input number
  // spinner.spin();
  return 0;
}
