#!/usr/bin/env python3

import rospy
# import setup_path 
import airsim

import numpy as np
from sensor_msgs.msg import Imu


def pub_imu(data):
    imu_ros = Imu()
    imu_ros.header.stamp = rospy.Time.now()
    imu_ros.header.frame_id = 'base_link'

    orient = data.orientation
    imu_ros.orientation.x = orient.x_val
    imu_ros.orientation.y = orient.y_val
    imu_ros.orientation.z = orient.z_val
    imu_ros.orientation.w = orient.w_val

    a_vel = data.angular_velocity
    imu_ros.angular_velocity.x = a_vel.x_val
    imu_ros.angular_velocity.y = a_vel.y_val
    imu_ros.angular_velocity.z = a_vel.z_val

    l_vel = data.linear_acceleration
    imu_ros.linear_acceleration.x = l_vel.x_val
    imu_ros.linear_acceleration.y = l_vel.y_val
    imu_ros.linear_acceleration.z = l_vel.z_val
    
    return imu_ros

def main():

    # connect the simulator
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(False)
    client.armDisarm(True)

    imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
    rospy.init_node('car_imu')
    rate = rospy.Rate(50.0)

    while not rospy.is_shutdown():

		# get the imu data
        imu_data = client.getImuData()
        imu_ros = pub_imu(imu_data)
        imu_pub.publish(imu_ros)
        rate.sleep()


if __name__ == "__main__":
	
	main()