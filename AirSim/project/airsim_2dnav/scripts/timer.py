# Name: Drayang Chua Kai Yang
# Date: 24 Feb 2021
# School: Monash University Malaysia

# Description: The python file is created to control how fast the car_control.py file publishes the messages to certain 
#              topics in order to control the virtual car. This can be done by defining a specific polling period value 
#              and publishing the message to the ‘/control_timer’ topic with the defined polling rate. In this way, 
#              it allows the callback function in the car_control.py file to be called at a specific rate.

#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry

if __name__ == '__main__':
    rospy.init_node('control_timer')
    t = 0
    poll_period = 0.2 # Polling rate in seconds - changes controller frequency
    rospy.loginfo("Checking for odom...")
    data = rospy.wait_for_message("/odom", Odometry, 20)
    
    # if data from /odom topic is received, we call the following loop
    if data != None:
        rospy.loginfo("Initiating control loop with polling period: %.2fs"%(poll_period))
        # Start controller
        pub = rospy.Publisher("/control_timer", Float64, queue_size = 10)
        while not rospy.is_shutdown():
            tE = Float64()
            tE.data = t
            t += poll_period
            pub.publish(tE)
            rospy.sleep(poll_period)
            
    else:
        rospy.loginfo("No odom data!")
