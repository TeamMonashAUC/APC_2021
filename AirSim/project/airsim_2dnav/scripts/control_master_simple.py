#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
import math

# control_master_simple.py
# Author(s): Jayson Teh, Navaneeth Nair, Zi Yu, Khai Hoe
# Documented by: Navaneeth Nair R Dhileepan
# School: Monash University Malaysia
# Description: - Python script that controls the PhysXCar in AirSim from start to finish
#                by subscribing to the car's odometry data, calculating the required throttle,
#                gear & steering values based on pre-defined goal sequences and goal types,
#                and publish these values to AirSim to move the car at a defined period.
#              - This _simple variant simplifies the throttle to basic increments and decrements for low speeds (<= 5m/s)

class Control: # Control class for modular code
	def __init__(self): # Class constructor
		# Constants
		self.fct = [0.35, 0.005, 0.02] # Throttle factors
		self.rad90 = math.radians(90) # 90 degrees in radians
		self.b_wheel_base = 3 # Wheelbase length [m]

		# Main parameters
		self.poll_period = 0.2 # Period of calling callback() function
		self.config = 10 # Goal sequence configuration

		# Initialize method attributes (variables global to class)
		self.car_x = self.car_y = self.car_z = self.yaw = self.v_x = self.v_y = self.v_z = self.t_poll = self.t_tot = self.t0 = self.throttle = self.steering = self.prev_gas = self.stop_cal_t = 0
		self.stop = self.end = self.start = self.move = False
		self.prev_gear = ""
		self.prev_goal_type = self.goal_type = -1

		# Initialize publishers and messages
		self.pub_gear = rospy.Publisher("/gear_command", String, queue_size = 1)
		self.pub_throttle = rospy.Publisher("/throttle_command", Float64, queue_size = 1)
		self.pub_steering = rospy.Publisher("/steering_command", Float64, queue_size = 1)
		self.throttle_data = self.steering_data = Float64()
		self.gear_data = String()

	# Class method that performs the calculation for controlling the car, total time elapsed from controller start is passed
	def callback(self, time):
		gear = "forward" # Initialize gear to forward
		if self.stop and not self.end: # Exit control when reached final goal
			rospy.loginfo("Controller Time: %.4f" %(time)) # Display total controller time
			self.end = True
			return
		elif self.end: # End condition
			return

		# Calculate velocity as a magnitude of cartesian vectors
		car_speed = math.sqrt(math.pow(self.v_x, 2) + math.pow(self.v_y, 2) + math.pow(self.v_z, 2))

		# Calculate distance from car to goal
		x_diff = self.goal_x - self.car_x
		y_diff = self.goal_y - self.car_y
		# Calculate radial distance
		diff_radius = math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2) + math.pow(self.car_z, 2))
		# Stop calculating steering when it reaches steady state
		if abs(self.steering) <= 0.2 and self.goal_type == self.prev_goal_type:
			self.stop_cal_t += 1
		else:
			self.stop_cal_t = 0
			self.prev_goal_type = self.goal_type
		# Steering control
		if car_speed > 1 and (self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0): # Settling time = 10 x poll_period
			if abs(self.yaw) > 1.0: # Truncate out-of-range math bounds
				self.yaw = 1
			# Convert quartenion angle to euler angle
			theta = 2 * math.asin(self.yaw) + self.rad90
			# Transform goal coordinates using forward kinematics
			g_dx = -(math.cos(theta) * x_diff - math.sin(theta) * y_diff)
			g_dy = -(math.sin(theta) * x_diff + math.cos(theta) * y_diff)
			a = math.atan(g_dy / g_dx)  # alpha
			if g_dx < 0 and self.goal_type in [6,8,9,10,11]: # Flip steering sign for reversing
				a = -a
			omega = 1.5 * a # Scalar constant to define angular velocity omega

			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				self.steering = math.atan(self.b_wheel_base / r)

		if self.goal_type in [6,8,9,10,11]: # Reverse goal types
			gear = "reverse"

		if self.goal_type == 7 and diff_radius < 3: # End condition
			self.stop = True

		# Throttle control
		if car_speed < 0.5 :  # Low speed condition
			self.throttle = 0.5
		elif not self.stop:  # not at final goal
			self.move = True # Flag when car starts moving
			if self.goal_type not in [5, 11]: # Not at stop & go goal
				if car_speed < 2:
					self.throttle = self.fct[0] # Base throttle
				elif self.throttle < 0.49: # Limit max throttle
					self.throttle += self.fct[1] # increment throttle
			else:
				if diff_radius < 20 and self.throttle > 0.1:
					self.throttle -= self.fct[2] # decrement throttle when close to stop & go, min 0.1
		else: # End condition
			self.throttle = 0

		if abs(self.steering) > 0.6: # Limit max steering
			self.steering = 0.6*abs(self.steering) / self.steering

		### Publish controls ###
		if not self.move: # Always publish gear & throttle at the start to prevent synching issues
			self.gear_data.data = gear
			self.pub_gear.publish(self.gear_data)
			self.throttle_data.data = self.throttle
			self.pub_throttle.publish(self.throttle_data)
		else: # Publish gear and throttle only during changes
			if gear != self.prev_gear:
				self.throttle = 1 # 'Brake' the car by counter-throttling
				self.steering = 0 # Reset steering
				self.gear_data.data = gear
				self.pub_gear.publish(self.gear_data)
				self.prev_gear = gear # update previous gear

			if self.prev_gas != self.throttle:
				self.throttle_data.data = self.throttle
				self.pub_throttle.publish(self.throttle_data)
				self.prev_gas = self.throttle # update previous throttle

		if self.stop_cal_t <= 10 or (self.stop_cal_t - 10) % 10 == 0: # Stop publishing steering when reached steady state
			self.steering_data.data = -self.steering # Flip sign to satisfy competition environment conditions
			self.pub_steering.publish(self.steering_data)

		#rospy.loginfo("Publishing: [D_state: %d, Z_state: %s, Throttle:  %f, Brake: %f, target_speed: %d, Speed_cur: %f, angular_z: %f, a: %f, steer: %f, goal_type: %d, diff_radius: %f]" %(case, z_state, self.throttle, self.brake,target_speed,car_speed,omega,a,self.steering,goal_type,diff_radius))

	# Class method that gets called when odometry message is published to /odom by the AirSim-ROS wrapper, and passed to msg variable
	def odom(self, msg):
		if not self.end: # Perform operations while end condition is not true
			# Get cartesian positions
			self.car_x = msg.pose.pose.position.x
			self.car_y = msg.pose.pose.position.y
			self.car_z = msg.pose.pose.position.z
			# Get z-quartenion orientation
			self.yaw = msg.pose.pose.orientation.z
			# Get cartesian velocities
			self.v_x = msg.twist.twist.linear.x
			self.v_y = msg.twist.twist.linear.y
			self.v_z = msg.twist.twist.linear.z

			# timer
			t = rospy.get_time() # Get current time in seconds
			if not self.start: # Start condition
				self.start = True
				rospy.loginfo("Starting control loop(py) with polling period: %.2fs" %(self.poll_period)) # Report set period
				dt = 0
				self.callback(self.t_tot) # Callback at t = 0
			else:
				dt = t - self.t0 # Calculate time elapsed since last time step

			self.t_poll += dt # Total time elapsed from last call to callback function
			self.t_tot += dt # Total time elapsed from start
			self.t0 = t # Update current time for next step

			# Update goal sequence
			if self.arrSize > 0:
				if self.goal_type in [0,5,11]: # Smaller radius for stop & go, and [0,0] goal
				    rad = 3
				else:
				    rad = 5
				diff = math.sqrt((self.car_x - self.goal_x)**2 + (self.car_y - self.goal_y)**2 + self.car_z**2) # Radial distance to goal
				if diff < rad: # Car reaches within radius defined above
					# Delete goal and its type
					self.pose_seq.pop(0)
					self.pose_types.pop(0)
					self.arrSize -= 1
					if self.arrSize > 0:
						# Update goal coordinate and type
						self.goal_x = self.pose_seq[0][0]
						self.goal_y = self.pose_seq[0][1]
						self.goal_type = self.pose_types[0]

			# callback at defined period
			t_err = abs(self.t_poll - self.poll_period)/self.poll_period; # Calculate error
			if (t_err < 0.05 or self.t_poll > self.poll_period): # callback when within 5% precision or when poll elapsed time exceeds defined period
				self.t_poll = 0 # Reset callback elapsed time
				self.callback(self.t_tot)
		else:
			return

	# Class method that allocates array of coordinates and goal types
	def getGoals(self):
		rospy.loginfo("Loading goals for configuration: %d" %(self.config))

		# Allocate goals array based on defined config at class declaration
		if self.config == 1:
		    # Config 1 - Default (~1540 m, ~167s)
		    self.pose_seq = [[-80, 0], [-135, 0.5], [-205, 0], [-212, -15], [-212, -30], [-212, -74], [-212,-122],[-197,-128],[-179,-128],[-135.5,-128],[-90,-128],[-84,-143],[-84,-153],[-84,-175],[-83,-195],[-79,-203],[-78, -207],[-84, -208], [-85, -205], [-85,-195],[-84,-128],[-84,-68],[-84,-58],[-99,-48],[-115,-48],[-135,-48],[-203, -48],[-212, -63],[-212, -78],[-212,-198],[-212, -241],[-212, -246],[-197, -256],[-182, -256],[-135.5, -256],[-80,-256],[-20, -256],[34.5, -256],[44, -241],[44, -220],[44, -198],[44, -138],[29, -128],[1,-128],[7.5, -133],[15, -128],[37, -128],[44, -110], [44,-95], [44.5, -65.5]]
		    self.pose_types = [0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,0,1,1,2,3,4,1,2,3,4,1,1,2,3,4,0,4,1,2,3,4,1,2,7,6,5,1,2,3,7] # 1-11
		elif self.config == 2:
		    # Config 2 - Efficiency (< Distance, < Time), (~1520 m, ~162s)
		    self.pose_seq = [[-80, 0], [-135, 0.5], [-205, 0], [-212, -15], [-212, -30], [-212, -74], [-212,-120],[-197,-128],[-179,-128],[-135.5,-128],[-94,-128],[-84,-143],[-84,-153],[-84,-198],[-84,-246],[-99,-256], [-114, -256], [-135.5, -256], [-202, -256], [-212, -241], [-212, -226],[-212,-198],[-212, -57],[-197, -48], [-183,-48], [-135,-48], [-94,-48], [-84,-63], [-84,-78], [-84,-120], [-69,-128], [-54,-128], [0,-128],[34, -128],[44, -110], [44,-95], [44, -90], [43,-79],[39,-71],[38, -67],[44, -66], [45, -69], [45,-77],[44, -198],[44, -246],[29, -256],[14,-256],[0, -256]]
		    self.pose_types = [0,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7] # 1-11
		elif self.config == 3:
		    # Config 3 - Fastest (> Distance, <<< Time), (~1555 m, ~144s)
		    self.pose_seq = [[-80, 0], [-135, 0], [-205, 0], [-212, -15], [-212, -30], [-212, -74], [-212,-120],[-197,-128],[-179,-128],[-135.5,-128], [-70, -128], [0,-128],[34, -128],[44, -110], [44,-95], [44, -79], [43,-75], [39, -67],[44, -66], [45, -69], [45,-77],[44, -198],[44, -246],[29, -256],[14,-256],[0, -256],[-70,-256], [-135.5, -256], [-202, -256], [-212, -241], [-212, -226],[-212,-198],[-212, -57],[-197, -48], [-183,-48], [-135,-48], [-94,-48], [-84,-63], [-84,-78], [-84,-198]]
		    self.pose_types = [0,4,1,2,3,4,1,2,3,4,0,4,1,2,3,4,1,5,5,5,3,4,1,2,3,4,0,4,1,2,3,4,1,2,3,4,1,2,3,7] # 1-11)
		elif self.config == 4:
		    # Config 4 - Shortest (<<< Distance, >> Time), (~1440 m, ~180s)
		    self.pose_seq = [[-80, 0], [-135, 0.5], [-203, 0], [-212, -15], [-212, -30],[-212, -38],[-197, -48], [-183,-48], [-163,-48], [-141,-49],[-133,-51],[-130, -48],[-144, -48], [-153, -48],[-202, -48],[-212, -63],[-212, -78],[-212, -74], [-212,-120],[-197,-128],[-179,-128], [-163,-128], [-141,-129],[-133,-131],[-130, -128],[-143, -128],[-153, -128],[-202,-128], [-212,-143], [-212, -158], [-212,-198],[-212, -246],[-197, -256],[-182, -256],[-135.5, -256],[-94,-256],[-84,-241],[-84,-226],[-84.5,-198],[-84,-140],[-72,-128], [-55,-128], [0,-128],[34, -128],[44, -110], [44,-95], [44, -90], [43,-78],[39,-69],[38, -67],[44, -65], [45, -67], [44,-85],[44, -198],[44, -246],[29, -256],[14,-256],[0, -256]]
		    self.pose_types = [0,4,1,2,3,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,5,5,5,3,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,5,5,5,5,3,4,1,2,3,7]
		elif self.config == 5:
		    # Config 5 - Shortest (<<<< Distance, >> Time), (~1398 m, ~215s) (reverse, forward, reverse)
		    self.pose_seq = [[-80, 0], [-135, 0.5], [-203, 0], [-212, -15], [-212, -30],[-212, -38],[-197, -48], [-183,-48], [-135,-48],[-202, -48],[-212, -63],[-212, -78],[-212, -74], [-212,-120],[-197,-128],[-179,-128], [-135,-128],[-202,-128], [-212,-143], [-212, -158], [-212,-198],[-212, -246],[-197, -256],[-182, -256],[-135.5, -256],[-94,-256],[-84,-241],[-84,-226],[-84.5,-198],[-84,-140],[-74,-128], [-59,-128], [0,-128],[34, -128],[44, -110], [44,-95], [44, -65],[44, -198],[44, -246],[29, -256],[14,-256],[0, -256],[0, -256]]
		    self.pose_types = [0,4,1,2,3,1,2,3,4,8,9,10,6,8,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,1,2,3,4,6,8,9,10,6,7]
		elif self.config == 6:
		    # Config 6 - Shortest (<<<< Distance, >>> Time), (~1400 m, ~200s) (reverse, forward, reverse) x2
		    self.pose_seq = [[-80, 0], [-135, 0.5], [-203, 0], [-212, -15], [-212, -30],[-212, -38],[-197, -48], [-183,-48], [-135,-48],[-199, -48],[-212, -65],[-212, -78],[-212, -74], [-212,-116],[-197,-128],[-179,-128], [-135,-128],[-202,-128], [-212,-143], [-212, -158], [-212,-198],[-212, -246],[-197, -256],[-182, -256],[-135.5, -256],[-94,-256],[-84,-241],[-84,-226],[-84.5,-198],[-84,-244],[-69,-256],[-54, -256], [0, -256], [31, -256], [44, -241], [44, -226], [44, -198], [44, -138],[29, -128],[1,-128],[34, -128],[44, -110], [44,-95], [44.5, -65], [44.5, -65]]
		    self.pose_types = [0,4,1,2,3,1,2,3,4,6,8,6,6,6,8,6,6,1,2,3,4,1,2,3,4,1,2,3,4,6,8,6,6,6,8,6,6,6,8,6,1,2,3,1,7]
		elif self.config == 7:
		    # Config 7 - Start backwards
		    self.pose_seq = [[0,0],[35, 0],[44, -15],[44,-30],[44,-65],[44,-95],[44,-110],[34,-127],[2,-129],[34,-129],[44,-143],[44,-158],[44,-198],[44, -246],[29, -256],[14,-256],[0, -256],[-74,-256],[-84,-241],[-84,-226],[-84.5,-200],[-84,-226],[-84,-241],[-94,-256],[-135.5,-256],[-182,-256],[-197,-256],[-210,-246],[-212,-195],[-212,-158],[-212,-143],[-202,-130],[-137,-128],[-202,-128],[-212,-113],[-212,-98],[-212,-74],[-212,-78],[-212,-63],[-200,-49],[-137,-48],[-183,-48],[-197,-48],[-210,-37],[-212,-30],[-212,-15],[-205,-1],[-180,0.0],[-135,1],[-135,1]]
		    self.pose_types = [0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,8,9,10,8,9,10,6,6,7]
		elif self.config == 8:
		    # Config 8 - 7 & end forward
		    self.pose_seq = [[0,0],[32, 0],[44, -15],[44,-30],[44,-65],[44,-95],[44,-115],[28,-127],[4.5,-128],[34,-129],[44,-143],[44,-158],[44,-198],[44, -241],[27, -256],[12,-256],[0, -256],[-70,-256],[-84,-241],[-84,-226],[-84.5,-199],[-84,-226],[-84,-241],[-94,-256],[-135.5,-256],[-182,-256],[-197,-256],[-210,-246],[-212,-195],[-212,-158],[-212,-145],[-200,-130],[-142,-128],[-202,-128],[-212,-113],[-212,-98],[-212,-74],[-212,-78],[-212,-63],[-200,-49],[-135,-48],[-94,-48],[-84,-33],[-84,-18],[-84,-13],[-103,0],[-114,0],[-135,1]]
		    self.pose_types = [0,8,9,10,6,6,9,10,6,1,2,3,4,1,2,3,4,1,2,3,4,8,9,10,6,8,9,10,6,8,9,10,6,1,2,3,4,4,2,3,4,1,2,3,1,2,3,7]
		elif self.config == 9:
            # Config 9 - Grass cutting service
			self.pose_seq = [[0,0],[34, 0],[42, -15], [44,-30], [44,-110],[29,-124],[20,-127.5],[3,-128],[36,-130],[42,-143],[44,-158],[44,-198],[43, -240],[29, -252.5],[14,-254.5],[0, -256],[-73,-255],[-81,-241],[-84.5,-226],[-84.5,-199],[-84,-245],[-99,-254],[-114,-255],[-135.5,-255.5],[-178,-253],[-196.5,-242.5],[-207,-219],[-211.5,-198],[-210.5,-145],[-197,-133],[-175,-128.5],[-136.5,-128],[-204,-127],[-210,-115],[-212,-98],[-212,-74],[-211,-62],[-199,-51],[-170,-48],[-135,-48],[-95,-47.5],[-86,-30.5],[-85,-19],[-85,-15],[-98,-4],[-113,0],[-135,1]]
			self.pose_types = [0,8,9,10,8,9,10,11,1,2,3,4,1,2,3,4,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,2,3,4,1,2,3,4,1,2,3,1,2,3,7]
		elif self.config == 10:
		    # Config 10 - Optimized grass cutting
		    self.pose_seq = [[0,0], [34.5, 0],[44,-30], [44,-115],[29,-124],[-0.5,-128],[37,-130],[42,-143],[44,-198],[43, -244],[29, -252.5],[0, -256],[-76,-255],[-81.5,-241],[-84.5,-197],[-84,-246],[-99,-255],[-114,-255],[-135.5,-255.5],[-178,-253],[-196.5,-242.5],[-207,-219],[-211.5,-198],[-210.5,-140],[-197,-133],[-135,-128],[-205,-127],[-210,-115],[-212,-98],[-212,-74],[-211,-60],[-199,-49],[-135,-48],[-92,-47.5],[-86,-30.5],[-86,-19],[-86,-11],[-98,-4],[-135,1]]
		    self.pose_types = [0,8,9,8,9,11,1,2,4,1,2,4,1,2,5,8,9,10,6,8,9,10,6,8,9,11,1,2,3,4,1,2,4,1,2,3,1,2,7]

		# Initialize goal coordinates, array size and goal type
		self.arrSize = len(self.pose_types)
		self.goal_x = self.pose_seq[0][0]
		self.goal_y = self.pose_seq[0][1]
		self.goal_type = self.pose_types[0]

# Main function
def listener():
	# Initialize control node
	rospy.init_node('cmd_car_control')

	# Initialize nodelets and get goals
	control = Control()
	rospy.Subscriber("/odom", Odometry, control.odom)
	rospy.loginfo("Initialized control node")
	control.getGoals()

	# Set AirSim parameters
	tstart = rospy.get_time() # Current time in seconds
	tE = 0 # Initialize elapsed time

	while tE < 10: # Wait for 10 secs
	 	tE = rospy.get_time() - tstart; # Update elapsed time
		# If parameters are available in the parameter server, modify it
	 	if rospy.has_param('/airsim_node/update_airsim_img_response_every_n_sec'):
	 		rospy.set_param('/airsim_node/update_airsim_img_response_every_n_sec', 500)
	 	if rospy.has_param('/airsim_node/update_lidar_every_n_sec'):
	 		rospy.set_param('/airsim_node/update_lidar_every_n_sec', 500)
	 		rospy.loginfo("Parameters set!")
	 		break

	# Wait for odom to be available
	rospy.loginfo("Checking for odom...")
	data = rospy.wait_for_message("/odom", Odometry, 20) # Blocks until message is received, 20s timeout
	if data != None:
		rospy.spin() # Block the node from exiting
	else:
	    rospy.loginfo("No odom data!")

if __name__ == '__main__':
	listener() # Call listener function when script is executed
