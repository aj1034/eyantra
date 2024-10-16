#!/usr/bin/env python3

'''
This python file runs a ROS 2-node of name pico_control which holds the position of Swift Pico Drone on the given dummy.
This node publishes and subsribes the following topics:

		PUBLICATIONS			SUBSCRIPTIONS
		/drone_command			/whycon/poses
		/pid_error			    /throttle_pid
						        /pitch_pid
						        /roll_pid
					
Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.	
'''

# Importing the required libraries

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node
from tf_transformations import euler_from_quaternion
import time


class Swift_Pico(Node):
	def __init__(self):
		super().__init__('pico_controller')  # initializing ros node with name pico_controller

		# This corresponds to your current position of drone. This value must be updated each time in your whycon callback
		# [x,y,z]
		self.drone_position = [0.0, 0.0, 0.0]
		self.drone_orientation = [0.0,0.0,0.0]
		self.q = [0.0, 0.0, 0.0, 0.0]

		# [x_setpoint, y_setpoint, z_setpoint]
		self.setpoint = [2, 2, 20]  # whycon marker at the position of the dummy given in the scene. Make the whycon marker associated with position_to_hold dummy renderable and make changes accordingly

		# Declaring a cmd of message type swift_msgs and initializing values
		self.cmd = SwiftMsgs()
		self.cmd.rc_roll = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_throttle = 1500

		#initial setting of Kp, Kd and ki for [roll, pitch, throttle]. eg: self.Kp[2] corresponds to Kp value in throttle axis
		#after tuning and computing corresponding PID parameters, change the parameters

		self.Kp = [0, 0, 0]
		self.Ki = [0, 0, 0]
		self.Kd = [0, 0, 0]

		#-----------------------Add other required variables for pid here --------------------------------------------
		self.PID_terms_alt        =[0,0,0]
		self.PID_terms_roll       =[0,0,0]
		self.PID_terms_pitch      =[0,0,0]

		self.error                =[0,0,0]
		self.previous_error       =[0,0,0]

		self.output               =[0,0,0]  #roll, pitch, altitude

		self.min_values = [1200,1200,1200]
		self.max_values = [1800,1800,1800]

        
		# Hint : Add variables for storing previous errors in each axis, like self.prev_error = [0,0,0] where corresponds to [pitch, roll, throttle]		#		 Add variables for limiting the values like self.max_values = [2000,2000,2000] corresponding to [roll, pitch, throttle]
		#													self.min_values = [1000,1000,1000] corresponding to [pitch, roll, throttle]
		#																	You can change the upper limit and lower limit accordingly. 
		#----------------------------------------------------------------------------------------------------------

		# # This is the sample time in which you need to run pid. Choose any time which you seem fit.
	
		self.sample_time = 0.50  # in seconds

		# Publishing /drone_command, /pid_error
		self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
		self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

		#------------------------Add other ROS 2 Publishers here-----------------------------------------------------
	

		# Subscribing to /whycon/poses, /throttle_pid, /pitch_pid, roll_pid
		self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
		self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
		self.create_subscription(PIDTune, "/roll_pid", self.set_roll_pid, 1)
		self.create_subscription(PIDTune, "/pitch_pid", self.set_pitch_pid, 1)


		#------------------------Add other ROS Subscribers here-----------------------------------------------------
	

		self.arm()  # ARMING THE DRONE

		# Creating a timer to run the pid function periodically, refer ROS 2 tutorials on how to create a publisher subscriber(Python)
		self.create_timer(self.sample_time, self.pid)


	def disarm(self):
		self.cmd.rc_roll = 1000
		self.cmd.rc_yaw = 1000
		self.cmd.rc_pitch = 1000
		self.cmd.rc_throttle = 1000
		self.cmd.rc_aux4 = 1000
		self.command_pub.publish(self.cmd)
		

	def arm(self):
		self.disarm()
		self.cmd.rc_roll = 1500
		self.cmd.rc_yaw = 1500
		self.cmd.rc_pitch = 1500
		self.cmd.rc_throttle = 1500
		self.cmd.rc_aux4 = 2000
		self.command_pub.publish(self.cmd)  # Publishing /drone_command


	# Whycon callback function
	# The function gets executed each time when /whycon node publishes /whycon/poses 
	def whycon_callback(self, msg):
		self.drone_position[0] = msg.poses[0].position.x 
		#--------------------Set the remaining co-ordinates of the drone from msg----------------------------------------------
		self.drone_position[1] = msg.poses[0].position.y
		self.drone_position[2] = msg.poses[0].position.z
		#orientation is in quaternions
		self.q[0] = msg.poses[0].orientation.x
		self.q[1] = msg.poses[0].orientation.y
		self.q[2] = msg.poses[0].orientation.z
		self.q[3] = msg.poses[0].orientation.w
		#quaterninon to euler

		self.drone_orientation = euler_from_quaternion(self.q) 
		#---------------------------------------------------------------------------------------------------------------


	# Callback function for /throttle_pid
	# This function gets executed each time when /drone_pid_tuner publishes /throttle_pid
	def altitude_set_pid(self, alt):
		self.Kp[2] = alt.kp   # This is just for an example. You can change the ratio/fraction value accordingly
		self.Ki[2] = alt.ki 
		self.Kd[2] = alt.kd 

	#----------------------------Define callback function like altitide_set_pid to tune pitch, roll--------------
	def set_roll_pid(self,roll):
		self.Kp[0] = roll.kp   
		self.Ki[0] = roll.ki
		self.Kd[0] = roll.kd 

	def set_pitch_pid(self,pitch):
		self.Kp[1] = pitch.kp 
		self.Ki[1] = pitch.ki 
		self.Kd[1] = pitch.kd 

	#----------------------------------------------------------------------------------------------------------------------

	def pid(self):
	#-----------------------------Write the PID algorithm here--------------------------------------------------------------

	# Steps:
	# 	1. Compute error in each axis. eg: error[0] = self.drone_position[0] - self.setpoint[0] ,where error[0] corresponds to error in x...
		self.error[0] = self.drone_position[0] - self.setpoint[0] #errorx
		self.error[1] = self.drone_position[1] - self.setpoint[1] #errory
		self.error[2] = self.drone_position[2] - self.setpoint[2] #errorz


	#	2. Compute the error (for proportional), change in error (for derivative) and sum of errors (for integral) in each axis. Refer "Understanding PID.pdf" to understand PID equation.
		self.PID_terms_alt[0] = self.Kp[2] * self.error[2]
		self.PID_terms_alt[1] = self.Kd[2] * (self.error[2] - self.previous_error[2])/self.sample_time
		self.PID_terms_alt[2] += self.error[2] * self.sample_time 

		self.PID_terms_pitch[0] = self.Kp[1] * self.error[1]
		self.PID_terms_pitch[1] = self.Kd[1] * (self.error[1] - self.previous_error[1])/self.sample_time
		self.PID_terms_pitch[2] += self.error[1] * self.sample_time 

		self.PID_terms_roll[0] = self.Kp[0] * self.error[0]
		self.PID_terms_roll[1] = self.Kd[0] * (self.error[0] - self.previous_error[0])/self.sample_time
		self.PID_terms_roll[2] += self.error[0] * self.sample_time 



	#	3. Calculate the pid output required for each axis. For eg: calcuate self.out_roll, self.out_pitch, etc.

		self.output[0] = self.PID_terms_roll[0] + self.PID_terms_roll[1] + self.Ki[0] * self.PID_terms_roll[2]
		self.output[1] = self.PID_terms_pitch[0] + self.PID_terms_pitch[1] + self.Ki[0] * self.PID_terms_pitch[2]
		self.output[2] = self.PID_terms_alt[0] + self.PID_terms_alt[1] + self.Ki[2] * self.PID_terms_alt[2]


	#	4. Reduce or add this computed output value on the avg value ie 1500. For eg: self.cmd.rcRoll = 1500 + self.out_roll. LOOK OUT FOR SIGN (+ or -). EXPERIMENT AND FIND THE CORRECT SIGN

		self.cmd.rc_roll      = 1500 + int(self.output[0])
		self.cmd.rc_pitch     = 1500 + int(self.output[1])
		self.cmd.rc_throttle  = 1500 + int(self.output[2])


	#	5. Don't run the pid continously. Run the pid only at the a sample time. self.sampletime defined above is for this purpose. THIS IS VERY IMPORTANT.
		# time.sleep(self.sample_time)

	#	6. Limit the output value and the final command value between the maximum(2000) and minimum(1000)range before publishing. For eg : if self.cmd.rcPitch > self.max_values[1]:
	#																														self.cmd.rcPitch = self.max_values[1]
		self.cmd.rc_roll = min(max(self.cmd.rc_roll, self.min_values[0]), self.max_values[0])
		self.cmd.rc_pitch = min(max(self.cmd.rc_pitch, self.min_values[1]), self.max_values[1])
		self.cmd.rc_throttle = min(max(self.cmd.rc_throttle, self.min_values[2]), self.max_values[2])

	#	7. Update previous errors.eg: self.prev_error[1] = error[1] where index 1 corresponds to that of pitch (eg)

		self.previous_error = self.error
	#	8. Add error_sum
		self.pid_error = self.error
		pid_error_msg = PIDError()
		pid_error_msg.roll_error = self.error[0]
		pid_error_msg.pitch_error = self.error[1]
		pid_error_msg.throttle_error = self.error[2]

		self.pid_error_pub.publish(pid_error_msg)

	#--------------------------------------------------------------------------------------------------------------------------------------------------------------
		# self.command_pub.publish(self.cmd.rc_roll)
		# self.command_pub.publish(self.cmd.rc_pitch)
		# self.command_pub.publish(self.cmd.rc_throttle)
		self.command_pub.publish(self.cmd)
		# calculate throttle error, pitch error and roll error, then publish it accordingly
		# self.pid_error_pub.publish(self.pid_error[0])
		# self.pid_error_pub.publish(self.pid_error[1])
		# self.pid_error_pub.publish(self.pid_error[2])




def main(args=None):
	rclpy.init(args=args)
	swift_pico = Swift_Pico()
	rclpy.spin(swift_pico)
	swift_pico.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
