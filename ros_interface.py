#!/usr/bin/env python2

"""
ROS Node for controlling the ARDrone 2.0 using the ardrone_autonomy package.

This ROS node subscribes to the following topics:
/vicon/ARDroneCarre/ARDroneCarre

This ROS node publishes to the following topics:
/cmd_vel_RHC

"""

from __future__ import division, print_function, absolute_import

# Import ROS libraries
import roslib
import rospy
import numpy as np

# Import class that computes the desired positions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TransformStamped, Twist, Vector3
from position_controller import PositionController


class ROSControllerNode(object):
    """ROS interface for controlling the Parrot ARDrone in the Vicon Lab.
    write code here to define node publishers and subscribers
    publish to /cmd_vel topic
    subscribe to /vicon/ARDroneCarre/ARDroneCarre for position and 
    attitude feedback
    """
    def __init__ (self):

		# Publishers
		self.pub_vel_cmd = rospy.Publisher('/cmd_vel_RHC', 
		                            Twist, queue_size=32)

		self.pub_error = rospy.Publisher('/aer1217/position_error',
						Twist,queue_size=32)

		#Subscriber
		self.sub_vicon = rospy.Subscriber('/vicon/ARDroneCarre/ARDroneCarre', 
		                                TransformStamped,
		                                self.update_quadrotor_state)
		self.sub_desired_pos = rospy.Subscriber('/aer1217/desired_position', 
		                                TransformStamped,
		                                self.update_desired_pos)

		#initialize  variables
		self.actual_pos = TransformStamped()
		self.actual_pos_prev = TransformStamped()
		self.actual_vel = np.array([0,0,0,0])

		self.desired_pos = TransformStamped()
		self.desired_pos_prev = TransformStamped()
		self.desired_vel = np.array([0,0,2,0])

		#Define Published msgs
		self.pos_error = Twist()
		self.vel_cmd_msg = Twist()

		#initialize position controller
		self.pos_controller = PositionController()

		#publish vel commands
		self.dt = 1/100.0
		self.pub_prop_vel = rospy.Timer(rospy.Duration(self.dt), \
												self.send_vel_cmd)
        

    def update_quadrotor_state(self, transfrom_stamped_msg):
		"""Callback function for sub_vicon topic, stores the quad rotor
		position and also calculates its velocity 

		Args:
			tranform_stamped_msg: msg received from subscriber
		"""

		#store received msg
		self.actual_pos = transfrom_stamped_msg

		#calculate velocity
		rotation = (self.actual_pos.transform.rotation.x,
		    self.actual_pos.transform.rotation.y,
		    self.actual_pos.transform.rotation.z,
		    self.actual_pos.transform.rotation.w)
		euler_angle = euler_from_quaternion(rotation)

		psi_act = euler_angle[2]

		pos_act = np.array([self.actual_pos.transform.translation.x,
						  self.actual_pos.transform.translation.y,
						  self.actual_pos.transform.translation.z,
						  psi_act])

		t_act = self.actual_pos.header.stamp.to_sec() + \
							self.actual_pos.header.stamp.to_nsec()/(10**9)

		rotation = (self.actual_pos_prev.transform.rotation.x,
		    self.actual_pos_prev.transform.rotation.y,
		    self.actual_pos_prev.transform.rotation.z,
		    self.actual_pos_prev.transform.rotation.w)
		euler_angle = euler_from_quaternion(rotation)

		psi_prev = euler_angle[2]

		pos_prev = np.array([self.actual_pos_prev.transform.translation.x,
							 self.actual_pos_prev.transform.translation.y,
							 self.actual_pos_prev.transform.translation.z,
							 psi_prev])

		t_prev = self.actual_pos_prev.header.stamp.to_sec() + \
						self.actual_pos_prev.header.stamp.to_nsec()/(10**9)

		try:
			delta = pos_act - pos_prev
			#Ensure difference in yaw doesn't exceed pi
			if delta[3] > np.pi:
				delta[3] = delta[3] - 2 * np.pi 
			elif delta[3] < - np.pi:
				delta[3] = delta[3] + 2 * np.pi 
			#calculate velocity
			self.actual_vel = delta/float(t_act - t_prev)
		except:
			ropsy.logwarn('Division by zero encountered when calculating \
									actual velocity')
			pass

		self.actual_pos_prev = self.actual_pos


    def update_desired_pos(self, transformstamped_msg):
		"""Callback function for sub_desired_pos topic, stores the 
		desired position and also calculates its velocity

		Args:
		tranformstamped_msg: msg received from subscriber

		"""

		#store desired position
		self.desired_pos = transformstamped_msg


		pos_act = np.array([self.desired_pos.transform.translation.x,
							self.desired_pos.transform.translation.y,
							self.desired_pos.transform.translation.z,
							self.desired_pos.transform.rotation.z])

		tdes_act = self.desired_pos.header.stamp.to_sec() + \
		self.desired_pos.header.stamp.to_nsec()/(10**9)

		pos_prev = np.array([self.desired_pos_prev.transform.translation.x,
							self.desired_pos_prev.transform.translation.y,
							self.desired_pos_prev.transform.translation.z,
							self.desired_pos_prev.transform.rotation.z])
		
		tdes_prev = self.desired_pos_prev.header.stamp.to_sec() + \
		self.desired_pos_prev.header.stamp.to_nsec()/(10**9)


		try:
			delta = pos_act - pos_prev
			#Ensure difference in yaw doesn't exceed pi
			if delta[3] > np.pi:
				delta[3] = delta[3] - 2 * np.pi
			elif delta[3] < - np.pi:
				delta[3] = delta[3] + 2 * np.pi 
			#calculate velocity
			self.desired_vel = delta/float(tdes_act - tdes_prev)
		except:
			ropsy.logwarn('Division by zero encountered when calculating \
				desired velocity')
			pass

		self.desired_pos_prev = self.desired_pos


    def send_vel_cmd(self, event):
		"""Function that sends velocity commands to ardrone 
		"""
		
		#calculate commands using position controller
		phi_c, theta_c, z_acc, psi_acc = self.pos_controller.pos_cont(self.actual_pos, \
			self.desired_pos, self.actual_vel, self.desired_vel)

		#set velocity commands
		self.vel_cmd_msg.linear.x  = phi_c
		self.vel_cmd_msg.linear.y  = theta_c
		self.vel_cmd_msg.linear.z  = np.clip(self.vel_cmd_msg.linear.z+z_acc * self.dt,-1.0,1.0)
		self.vel_cmd_msg.angular.z  = np.clip(self.vel_cmd_msg.angular.z+psi_acc * self.dt,-1000.0,1000.0)

		#record position error
		self.pos_error.linear.x = self.desired_pos.transform.translation.x - \
								self.actual_pos.transform.translation.x

		self.pos_error.linear.y = self.desired_pos.transform.translation.y - \
								self.actual_pos.transform.translation.y

		self.pos_error.linear.z = self.desired_pos.transform.translation.z - \
								self.actual_pos.transform.translation.z
		rotation = (self.actual_pos.transform.rotation.x,
		    self.actual_pos.transform.rotation.y,
		    self.actual_pos.transform.rotation.z,
		    self.actual_pos.transform.rotation.w)
		euler_angle = euler_from_quaternion(rotation)

		psi_act = euler_angle[2]
		print('yaw = ', psi_act,'yaw_error=', self.desired_pos.transform.rotation.z - psi_act)
		#Publish position error
		self.pub_error.publish(self.pos_error)	

		#publish velocity commands
		self.pub_vel_cmd.publish(self.vel_cmd_msg)

		


if __name__ == '__main__':
    # write code to create ROSControllerNode
    rospy.init_node('ros_interface')
    ROSControllerNode()
    rospy.spin()


