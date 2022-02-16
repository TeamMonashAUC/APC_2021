#!/usr/bin/env python
import rospy
import roslib
import tf
from std_msgs.msg import String, Int8, Float64
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, Vector3
from nav_msgs.msg import Odometry, Path
import sensor_msgs
import sensor_msgs.msg
import airsim_car_ros_pkgs as air
import airsim_car_ros_pkgs.msg
import math




def brake(brake_data):
    brake = brake_data.data
    pub = rospy.Publisher('/airsim_node/PhysXCar/car_cmd_body_frame', air.msg.CarCmd, queue_size=10)
    controls = airsim_car_ros_pkgs.msg.CarCmd()
    controls.brake = brake
    if gear == "forward":
        controls.is_manual_gear = False
        controls.gear_immediate = False
    else:
        controls.is_manual_gear = True
        controls.manual_gear = -1
        controls.gear_immediate = True
    controls.throttle = throttle
    controls.steering = -steering
    rospy.loginfo("Throttle: %f, Brake: %f, Gear: %s", throttle, brake, gear)
    pub.publish(controls)

def gear(gear_data):
    global gear
    gear = gear_data.data


def throttle(throttle_data):
    global throttle
    throttle = throttle_data.data
def steering(steering_data):
    global steering
    steering = steering_data.data


def listener():
    global brake_data, gear_data, throttle_data
    brake_data = Float64
    gear_data = String
    throttle_data = Float64
    rospy.init_node('tester', anonymous=True)
    rospy.Subscriber("/brake_command", Float64, brake)  # Get local planner's global plan
    rospy.Subscriber("/gear_command", String, gear)  # Get local planner's global plan
    rospy.Subscriber("/throttle_command", Float64, throttle)  # Get goal
    rospy.Subscriber("/steering_command", Float64, steering)  # Get goal
    rospy.spin()


if __name__ == '__main__':
    listener()
