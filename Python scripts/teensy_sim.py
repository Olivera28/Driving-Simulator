#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32
from cse400_msgs.msg import Command
from cse400_msgs.msg import Motor
from cse400_msgs.msg import IMU
import math

class Teensy_Sim:

	def __init__(self):
		# timing variables
		self.ROS_rate = 40 # 40Hz loop rate
		self.ROS_period = 1/self.ROS_rate
		
		# robot constants
		self.WHEEL_MAXVEL = 12.566 # rad/s
		self.FRONT_MAXVEL = 0.15415 # rad/s
		self.WHEEL_ACCEL = 20.5 # rad/s/s
		self.FRONT_ACCEL = 2.0 # rad/s/s
		self.TIRE_DIA = 0.08 # meter
		self.TIRE_SEP = 0.245 # meter
		self.BASE_SENSOR_OFFSET = 0.155
		self.TIRE_SCALE1 = self.TIRE_DIA/(2*self.TIRE_SEP)
		self.TIRE_SCALE2 = self.TIRE_DIA/4
		self.WHEEL_DELTA_V = self.WHEEL_ACCEL * self.ROS_period
		self.FRONT_DELTA_V = self.FRONT_ACCEL * self.ROS_period
		
		# range constants
		self.MAX_DISTANCE = 500 # millimeters
		self.RANGE_ANGLES = [math.radians(-55), math.radians(0), math.radians(55)]
		self.SEGMENTS = rospy.get_param('teensy/segments', [])

		# robot state variables
		self.left_pos = 0.0
		self.left_vel = 0.0
		self.right_pos = 0.0
		self.right_vel = 0.0
		self.x = 0.0
		self.y = 0.0
		self.prev_v = 0.0
		self.v = 0.0
		self.a = 0.0
		self.theta = 0.0
		self.prev_w = 0.0
		self.w = 0.0
		self.front_pos = 0.0
		self.front_vel = 0.0

		# motor velocity setpoints
		self.left_set = 0.0
		self.right_set = 0.0
		self.front_set = 0.0
				
		# subscriber variables
		self.sub_cmd = rospy.Subscriber("teensy/Commands", Command, self.cmd_Cb)
		self.cmd = Command()

	def cmd_Cb(self, cmd):
		self.cmd = cmd
		self.left_set = self.limit(self.cmd.left_vel, self.WHEEL_MAXVEL)
		self.right_set = self.limit(self.cmd.right_vel, self.WHEEL_MAXVEL)
		self.front_set = self.limit(self.cmd.front_vel, self.ARM_MAXVEL)

	def limit(self, set_vel, max_vel):
		if set_vel > max_vel:
			limit_set = max_vel
		elif set_vel < -max_vel:
			limit_set = -max_vel
		else:
			limit_set = set_vel
		return limit_set
	
	def update_robot_state(self):
		self.left_vel = self.accelerate_motor(self.left_set, self.left_vel, self.WHEEL_DELTA_V)
		self.left_pos += self.left_vel * self.ROS_period
		self.right_vel = self.accelerate_motor(self.right_set, self.right_vel, self.WHEEL_DELTA_V)
		self.right_pos += self.right_vel * self.ROS_period
		self.front_vel = self.accelerate_motor(self.front_set, self.front_vel, self.FRONT_DELTA_V)
		self.front_pos += self.shoulder_vel * self.ROS_period
		self.prev_w = self.w
		self.w = (self.right_vel - self.left_vel) * self.TIRE_SCALE1
		self.prev_v = self.v
		self.v = (self.right_vel + self.left_vel) * self.TIRE_SCALE2
		self.a = (self.v - self.prev_v) * self.ROS_rate
		self.x += self.v * math.cos(self.theta) * self.ROS_period
		self.y += self.v * math.sin(self.theta) * self.ROS_period
		self.theta += self.w * self.ROS_period
	
	def accelerate_motor(self, setpoint, current_vel, delta_v):
		if setpoint > current_vel:
			vel = current_vel + delta_v
			if vel > setpoint:
				vel = setpoint
		else:
			vel = current_vel - delta_v
			if vel < setpoint:
				vel = setpoint
		return vel


rospy.init_node('Teensy_sim', anonymous=True)

simulator = Teensy_Sim()

pub_millis = rospy.Publisher("teensy/Millis", UInt32, queue_size=10)
pub_motors = rospy.Publisher("teensy/Motors", Motor, queue_size=10)
pub_IMU = rospy.Publisher("teensy/IMU", IMU, queue_size=10)


millis_to_ROS = UInt32()
motor_msg = Motor()
IMU_msg = IMU()


rate = rospy.Rate(simulator.ROS_rate)
ticks = 0

print('Starting Teensy simulation node')

while not rospy.is_shutdown():
	ticks = (ticks + 1) % 32
	millis_to_ROS.data += 25
	simulator.update_robot_state()
	if ticks % 2 == 0:
		IMU_msg.quat_w = math.cos(0.5 * simulator.theta)
		IMU_msg.quat_z = math.sin(0.5 * simulator.theta)
		IMU_msg.rot_z = simulator.w * 57.295779513 # teensy BNO055 is using degrees/s
		if simulator.w == 0.0:
			IMU_msg.accel_x = simulator.a
			IMU_msg.accel_y = 0.0
		else:
			R = simulator.v/simulator.w
			IMU_msg.accel_x = simulator.a + R * (simulator.w - simulator.prev_w) * simulator.ROS_rate
			if simulator.w < 0.0:
			    IMU_msg.accel_y = -R * simulator.w * simulator.w
			else:
			    IMU_msg.accel_y = R * simulator.w * simulator.w			
		pub_IMU.publish(IMU_msg)
	if (ticks + 3) % 4 == 0:
		motor_msg.left_pos = simulator.left_pos
		motor_msg.left_vel = simulator.left_vel
		motor_msg.right_pos = simulator.right_pos
		motor_msg.right_vel = simulator.right_vel
		motor_msg.front_pos = simulator.front_pos
		motor_msg.front_vel = simulator.front_vel
		pub_motors.publish(motor_msg)
	if ticks == 7:
		pub_millis.publish(millis_to_ROS)
	rate.sleep()
