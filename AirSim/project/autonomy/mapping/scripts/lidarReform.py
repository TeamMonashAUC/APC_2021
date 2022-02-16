#!/usr/bin/env python3

# from airsim.types import LidarData
import rospy
# import setup_path 
import airsim

import numpy  as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan,PointCloud2,Imu
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2

def pub_pointcloud(points):
	pc = PointCloud2()
	pc.header.stamp = rospy.Time.now()
	pc.header.frame_id = 'Lidar1'
	pc.height = 1
	pc.width = len(points)

	pc.fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1),
			PointField('intensity', 12, PointField.FLOAT32, 1),
			PointField('ring', 16, PointField.UINT16, 1),
			PointField('time', 20, PointField.FLOAT32, 1)]

	pc.is_bigendian = False
	pc.point_step = 24
	pc.row_step = pc.point_step * points.shape[0]
	pc.is_dense = True

	pc.data = np.asarray(points, np.float32).tostring()
	
	return pc

def main():

	# connect the simulator
	client = airsim.CarClient()
	client.confirmConnection()
	client.enableApiControl(False)
	client.armDisarm(True)

	pointcloud_pub = rospy.Publisher('/pointcloud2', PointCloud2, queue_size=10)
	rospy.init_node('car_lidar')
	rate = rospy.Rate(20.0)

	while not rospy.is_shutdown():

		# get the lidar data
		lidarData = client.getLidarData()

		if len(lidarData.point_cloud) >3:

			points = np.array(lidarData.point_cloud,dtype=np.dtype('f4'))
			points = np.reshape(points,(int(points.shape[0]/3),3))
			num_temp = np.shape(points)[0]
			for p in points:
				p[2] = -p[2]
				# p[1] = -p[1]
			
			print(points)
			# print(num_temp)
			numpy_temp = np.zeros(num_temp)
			numpy_temp1 = np.ones(num_temp)
			# print(numpy_temp.shape)
			points = np.c_[points,numpy_temp1,numpy_temp,numpy_temp]

			# print('points:',points)
			pc = pub_pointcloud(points)

			pointcloud_pub.publish(pc)
			rate.sleep()
		else:
			print("No points received from Lidar data")


if __name__ == "__main__":
	
	main()