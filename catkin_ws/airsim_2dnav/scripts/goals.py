#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import tf
import actionlib
from std_msgs.msg import Header, Int8
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Vector3
from nav_msgs.msg import Odometry, OccupancyGrid
from tf.transformations import quaternion_from_euler
import roslaunch
import rospkg


class Goal:

    def __init__(self):
        self.pose_seq = list()
        self.goal_cnt = 0
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    def callback(self):
        pub = rospy.Publisher("/goal_type",Int8,queue_size=10)
        goal_type = Int8()
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(1000.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        mb_goal = MoveBaseGoal()
        rospy.sleep(3) # Delay start
        rospy.loginfo("Starting goals achievements ...")

        # Start to 11th (last) goal - note some goals (4th and 8th) are not located where they're supposed to be as this allows the car to remain fast while having enough time to brake without adding more types of goals
        # U-turn
        points = [[-80, 0], [-135, 0], [-202, 0], [-212, -15], [-212, -30], [-212, -74], [-212,-118],[-197,-128],[-179,-128.5],[-135.5,-128],[-94,-128],[-84,-143],[-84,-153],[-84,-175],[-83,-195],[-79,-203],[-78, -207],[-84, -208], [-85, -205], [-84,-195],[-84,-128],[-84,-68],[-84,-58],[-99,-48],[-115,-48],[-135,-48],[-202, -48],[-212, -63],[-212, -78],[-212,-198],[-212, -241],[-212, -246],[-197, -256],[-182, -256],[-135.5, -256],[-20, -256],[34.5, -256],[44, -241],[44, -220],[44, -198],[44, -138],[29, -128],[1,-128],[7.5, -133],[15, -128],[34, -128],[44, -113], [44,-98], [44, -66]]

        orient_seq = [0, 0, 0,0,0, 0,0,0,0,0,0,0,0]
        goal_type.data = 0

        # List of goal quaternions:
        quat_seq = list()
        for yawangle in orient_seq:
            quat_seq.append(Quaternion(*(quaternion_from_euler(0, 0, yawangle * math.pi / 180, axes='sxyz'))))
        for i in range(len(points)):
            self.pose_seq.append(Pose(Point(points[i][0], points[i][1], 0), quat_seq[1]))
        mb_goal.target_pose.header = Header()
        mb_goal.target_pose.header.stamp = rospy.Time.now()
        mb_goal.target_pose.header.frame_id = "world_enu"
        mb_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
        pub.publish(goal_type)
        self.client.send_goal(mb_goal)
    def position(self, msg):
        # Array of goal types
        # goal_arr = [0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,2,3,4,1,2,3,4,1,2,3,4,4,1,2,3,4,1,2,7,6,5,1,2,3,7] # 1-11
        # # & Fast
        goal_arr = [0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,1,2,3,4,1,2,3,4,1,1,2,3,4,4,1,2,3,4,1,2,7,6,5,1,2,3,7] # 1-11

        if goal_arr[self.goal_cnt] in [5,6,7]: # Smaller radius for 3-point D
            rad = 3
        else:
            rad = 5
        pub = rospy.Publisher("/goal_type",Int8,queue_size=10)
        goal_type = Int8()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.orient = math.sinh(msg.pose.pose.orientation.z)*2
        diff = math.sqrt((self.x - self.pose_seq[self.goal_cnt].position.x)**2 + (self.y - self.pose_seq[self.goal_cnt].position.y)**2)
        if diff < rad:
            self.goal_cnt += 1
            if self.goal_cnt < len(self.pose_seq):
                goal_type.data = goal_arr[self.goal_cnt]
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "world_enu"
                next_goal.target_pose.header.stamp = rospy.Time.now()
                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " + str(self.goal_cnt + 1) + " to Action Server")
                rospy.loginfo("Now heading to goal %d", self.goal_cnt + 1)
                pub.publish(goal_type)
                self.client.send_goal(next_goal)
            else:
                launch.shutdown()
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                return
def listener():
    global msg, costmap, launch
    msg = Odometry
    goal = Goal()
    costmap = OccupancyGrid
    rospack = rospkg.RosPack()
    nav_dir=rospack.get_path('airsim_2dnav')
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [nav_dir+"/launch/move_base.launch"])
    launch.start()
    rospy.init_node("goal_post")
    goal.callback()
    rospy.Subscriber("/odom", Odometry, goal.position)
    rospy.spin()

if __name__ == '__main__':
    listener()
