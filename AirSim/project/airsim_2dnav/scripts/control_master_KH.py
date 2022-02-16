#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Float64
from nav_msgs.msg import Odometry
#import airsim_car_ros_pkgs as air
#import airsim_car_ros_pkgs.msg
import math

class Control:
	def __init__(self):
		# Control parameters
		self.power = 1.1
		self.vmax = 5 # Top Speed at Straights
		self.a_vmin = 4.5 # Min speed to accelerate to on straights
		self.a_fvmax = 0.9 # Acceleration factor on straights
		self.vslow = 12 # Top speed when slowing down at straights before corners
		self.vc_atk = 6 # Cornering attack velocity
		self.vc_ext = 6 # Cornering exit velocity @ car_speed > self.vc_ext1[1], or if self.vc_ext1[1] == 0 then this is the exit velocity at all speeds
		self.v3pD = 4 # 3p-D velocity
		self.v3pR = 10 # 3p-R velocity
		self.throttle = 0
		self.steering = 0
		# Main parameters
		self.poll_period = 0.2
		self.config = 10

		# Initialize method variables
		self.car_x = self.car_y = self.car_z = self.yaw = self.v_x = self.v_y = self.v_z = self.t_poll = self.t_tot = self.t0 = 0
		self.stop = self.end = self.start = False

		#khai hoe
		self.mem = []
		self.mem2 = []
		self.g_dx = 0
		self.prev_gdx = 0
		self.accel = 0

		# Initialize publishers and class
		# No remap
		#self.pub_controls = rospy.Publisher('/airsim_node/PhysXCar/car_cmd_body_frame', air.msg.CarCmd, queue_size=1)
		#self.controls = air.msg.CarCmd()
		# # remap
		self.brake_data = Float64()
		self.gear_data = String()
		self.throttle_data = Float64()
		self.steering_data = Float64()
		self.pub_brake = rospy.Publisher("/brake_command", Float64, queue_size=1)
		self.pub_gear = rospy.Publisher("/gear_command", String, queue_size=1)
		self.pub_throttle = rospy.Publisher("/throttle_command", Float64, queue_size=1)
		self.pub_steering = rospy.Publisher("/steering_command", Float64, queue_size=1)

	def callback(self, time):
		steering = brake = throttle = omega = target_speed = a = 0
		gear = "forward"

		if self.goal_type == 7 and self.stop and not self.end: # Exit control when reached final goal
			rospy.loginfo("Controller Time: %.4f" %(time))
			self.end = True
			return
		elif self.end:
			return

		car_speed = math.sqrt(math.pow(self.v_x, 2) + math.pow(self.v_y, 2))

		# Calculate distance from car to goal
		x_diff = self.goal_x - self.car_x
		y_diff = self.goal_y - self.car_y
		diff_radius = math.sqrt(math.pow(x_diff,2) + math.pow(y_diff,2)+math.pow(self.car_z,2))
		if car_speed > 1:
			# Transform goal coordinates using forward kinematics
			if abs(self.yaw) > 1.0:
				self.yaw = 1
			z = 2 * math.asin(self.yaw) + rad90  # theta
			g_dx = -(math.cos(z) * x_diff - math.sin(z) * y_diff)
			g_dy = -(math.sin(z) * x_diff + math.cos(z) * y_diff)
			a = math.atan(g_dy / g_dx)  # alpha
			if g_dx < 0 and self.goal_type in [6,8,9,10,11]:
				a = -a
			# Steering adjustments
			if self.goal_type in [3,10]:
				gain = 0.8 * math.pow(car_speed / 10, 2) + 0.6
			else:
				gain = 1

			fct_z = 1.5
			omega = fct_z * a

			if omega != 0:
				# Apply Ackermann's steering
				r = car_speed / -omega
				adjusted_z = math.atan(b_wheel_base / r)
				self.steering = adjusted_z * gain
		if self.goal_type in [6,8,9,10,11]:
			gear = "reverse"
		### self.steering control for TEB // Max car steer: 1.0 rad (60 deg) ###
		#if car_speed > -1:
			# Transform goal coordinates using forward kinematics
		#	if abs(self.yaw) > 1.0:
		#		self.yaw = 1
		#	z = 2 * math.asin(self.yaw) + math.radians(90)  # theta
		#	self.g_dx = -(math.cos(z) * (self.goal_x - self.car_x) - math.sin(z) * (self.goal_y - self.car_y))
		#	g_dy = -(math.sin(z) * (self.goal_x - self.car_x) + math.cos(z) * (self.goal_y - self.car_y))
		#	a = math.atan(g_dy / self.g_dx)  # alpha

			# Proportional steering adjustments

		#	if (a < 0.5):
		#		gain = 2
		#	else:
		#		gain = math.pow(10,a)

			# Differential steering adjustments
		#	d_gain = 1
		#	dt2 = 1
		#	t2 = rospy.get_time()
		#	self.mem2.append((t2,a))
		#	elapsed2 = self.mem2[-1][0] - self.mem2[0][0]
		#	if (elapsed2 > dt2) :
		#		self.mem2.pop(0)

		#	if elapsed2 > 0:
		#		da = (self.mem2[-1][1] - self.mem2[0][1]) / elapsed2

		#	self.steering = gain * a * car_speed/10 #+ d_gain * da

		#	if abs(a) > 0.2:
		#		self.steering += d_gain * da * car_speed / 10
		#	if self.goal_type not in [6,8,9,10,11]:
		#		self.steering = -self.steering
		#	else:
		#		gear = "reverse"

		# self.prev_gdx = self.g_dx
		#if (self.prev_gdx * self.g_dx) < 0:
		#
		#	brake = 1
		#	if self.g_dx > 0:
		#		gear = "forward"
		#	else:
		#		gear = "reverse"

		if self.goal_type == 7 and diff_radius < 3:
			self.stop = True

		# self.throttle and self.brake control
		if car_speed < 0.5 :  # Start moving
			self.throttle = 0.5
			#rospy.loginfo(throttle)
		elif not self.stop:  # Outside of goal radius

			fct = [0.007, 0.007 , 0.03]  # Propotional and differential gain

			dt = 1
			t = rospy.get_time()
			if diff_radius < 10 and self.goal_type in [5,11]:
				self.mem=[]
			self.mem.append((t,car_speed))
			elapsed = self.mem[-1][0] - self.mem[0][0]
			if (elapsed > dt) :
				self.mem.pop(0)

			if elapsed > 0:
				self.accel = (self.mem[-1][1] - self.mem[0][1]) / elapsed


			increment = fct[0] *(self.vmax - car_speed)/10
			decrement = fct[2] *(car_speed)/10
			if self.goal_type not in [5,11]:
				if car_speed > 2: # Limit proportional gain if velocity hasnt reached steady state
					self.throttle += increment
				self.throttle += fct[1] * -self.accel
			else:
				if diff_radius < 30 and self.throttle >= 0.2:
					self.throttle -= decrement
		else:
			self.throttle = 0

		if brake == 1:
			self.steering = 0

		# Ignore negative values and truncate values > 1
		if self.throttle < 0:
			self.throttle = 0

		if self.throttle > 1:
			self.throttle = 1

		if brake < 0:
			brake = 0

		if brake > 1:
			brake = 1

		if abs(self.steering) > 0.6:
			self.steering = 0.6*abs(self.steering) / self.steering
		### Publish controls ###
		# No remap:
		#self.controls.brake = brake
		#if gear == "forward":
		#	self.controls.is_manual_gear = False
		#	self.controls.gear_immediate = False
		#else:
		#	self.controls.is_manual_gear = True
		#	self.controls.manual_gear = -1
		#	self.controls.gear_immediate = True
		#self.controls.throttle = throttle
		#self.controls.steering = steering
		#self.pub_controls.publish(self.controls)
		# # remap:
		self.gear_data.data = gear
		self.throttle_data.data = self.throttle
		self.steering_data.data = -self.steering
		self.brake_data.data = brake
		#
		self.pub_gear.publish(self.gear_data)
		self.pub_throttle.publish(self.throttle_data)
		self.pub_steering.publish(self.steering_data)
		self.pub_brake.publish(self.brake_data)

		#rospy.loginfo("Publishing: [D_state: %d, Z_state: %s, Throttle:  %f, Brake: %f, target_speed: %d, Speed_cur: %f, angular_z: %f, a: %f, steer: %f, goal_type: %d, diff_radius: %f]" %(case, z_state, self.throttle, self.brake,target_speed,car_speed,omega,a,self.steering,goal_type,diff_radius))


	def odom(self, msg):
		if not self.end:
			self.car_x = msg.pose.pose.position.x
			self.car_y = msg.pose.pose.position.y
			self.car_z = msg.pose.pose.position.z
			self.yaw = msg.pose.pose.orientation.z
			self.v_x = msg.twist.twist.linear.x
			self.v_y = msg.twist.twist.linear.y
			self.v_z = msg.twist.twist.linear.z

			# timer
			t = rospy.get_time()
			if not self.start:
				self.start = True
				dt = 0
				self.callback(self.t_tot)
			else:
				dt = t - self.t0

			self.t_poll += dt
			self.t_tot += dt
			self.t0 = t

			# goals
			if self.arrSize > 0:
				if self.goal_type in [5,11]: # Smaller radius for 3-point D, stop & go
				    rad = 3
				else:
				    rad = 5
				diff = math.sqrt((self.car_x - self.goal_x)**2 + (self.car_y - self.goal_y)**2 + self.car_z**2)
				if diff < rad:
					self.pose_seq.pop(0)
					self.pose_types.pop(0)
					self.arrSize -= 1
					if self.arrSize > 0:
						self.goal_x = self.pose_seq[0][0]
						self.goal_y = self.pose_seq[0][1]
						self.goal_type = self.pose_types[0]

			# callback at defined period
			if self.t_poll >= self.poll_period:
				self.t_poll = 0
				self.callback(self.t_tot)
		else:
			return

	def getGoals(self):
		rospy.loginfo("Loading goals for configuration: %d" %(self.config))
		# rospy.sleep(self.delay)

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
            # Config 9 - 8 & Cheat path
			self.pose_seq = [[0,0],[34, 0],[42, -15], [44,-30], [44,-110],[29,-124],[20,-127.5],[3,-128],[36,-130],[42,-143],[44,-158],[44,-198],[43, -240],[29, -252.5],[14,-254.5],[0, -256],[-73,-255],[-81,-241],[-84.5,-226],[-84.5,-199],[-84,-245],[-99,-254],[-114,-255],[-135.5,-255.5],[-178,-253],[-196.5,-242.5],[-207,-219],[-211.5,-198],[-210.5,-145],[-197,-133],[-175,-128.5],[-136.5,-128],[-204,-127],[-210,-115],[-212,-98],[-212,-74],[-211,-62],[-199,-51],[-170,-48],[-135,-48],[-95,-47.5],[-86,-30.5],[-85,-19],[-85,-15],[-98,-4],[-113,0],[-135,1]]
			self.pose_types = [0,8,9,10,8,9,10,11,1,2,3,4,1,2,3,4,1,2,3,5,8,9,10,6,8,9,10,6,8,9,10,11,1,2,3,4,1,2,3,4,1,2,3,1,2,3,7]
		elif self.config == 10:
		    # Config 10 - 9 & Cheat some more
		    self.pose_seq = [[0,0],[34.5, 0],[44,-30], [44,-115],[29,-124],[-0.5,-128],[37,-130],[42,-143],[44,-198],[43, -244],[29, -252.5],[0, -256],[-76,-255],[-81.5,-241],[-84.5,-197],[-84,-246],[-99,-255],[-114,-255],[-135.5,-255.5],[-178,-253],[-196.5,-242.5],[-207,-219],[-211.5,-198],[-210.5,-140],[-197,-133],[-135,-128],[-205,-127],[-210,-115],[-212,-98],[-212,-74],[-211,-60],[-199,-49],[-135,-48],[-92,-47.5],[-86,-30.5],[-86,-19],[-86,-11],[-98,-4],[-135,1]]
		    self.pose_types = [0,8,9,8,9,11,1,2,4,1,2,4,1,2,5,8,9,10,6,8,9,10,6,8,9,11,1,2,3,4,1,2,4,1,2,3,1,2,7]

		self.arrSize = len(self.pose_types)
		self.goal_x = self.pose_seq[0][0]
		self.goal_y = self.pose_seq[0][1]
		self.goal_type = self.pose_types[0]

def listener():
	control = Control()
	rospy.init_node('cmd_car_control')
	rospy.loginfo("Checking for odom...")
	data = rospy.wait_for_message("/odom", Odometry, 20) # 20s timeout
	if data != None:
		control.getGoals()
		rospy.loginfo("Initiating control loop with polling period: %.2fs"%(control.poll_period))
		rospy.Subscriber("/odom", Odometry, control.odom) # Get car position
		rospy.spin()
	else:
	    rospy.loginfo("No odom data!")

if __name__ == '__main__':
	# Constants
	rad90 = math.radians(90)
	b_wheel_base = 3 # Wheelbase length [m]
	mass = 2176.46 # Car mass [kg]
	g = 9.81 # Gravitational constant [m/s^2]
	friction = 0.01 # Rolling friction coefficient
	rho = 1.2 # Air density at NTP [kg/m^3]
	drag_coeff = 0.357 # Drag coefficient
	area = 3.389 # Car front chassis area [m^2]

	listener()
