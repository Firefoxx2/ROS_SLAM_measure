#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
import tf


# name of the robot in the gazebo simulation
GAZEBO_ROBOT_NAME = rospy.get_param("/measure/gazebo_robot_id", 'turtlebot3')

# name of the topic where is published location information from gazebo
GAZEBO_TOPIC_TRUE_LOCATION = rospy.get_param("/measure/gazebo_topic", '/gazebo/model_states')

# name of the target frame of the estimation of location
TARGET_FRAME = rospy.get_param("/measure/target_frame", 'odom')

# name of the source frame of the estimation of location
SOURCE_FRAME = rospy.get_param("/measure/source_frame", 'base_footprint')

# frequency of actualization of the node
FREQUENCY = rospy.get_param("/measure/frequency", 10)


class Measure:
	def __init__(self):
		rospy.init_node('r1_measure_node', anonymous=False)

		# subscribe to tf topic
		self._tflistener_sub = tf.TransformListener()

		# subscribe to gazebo location topic
		self._true_location_sub = rospy.Subscriber(GAZEBO_TOPIC_TRUE_LOCATION, ModelStates, self.gazebo_callback)

		# publish location information on topic
		self._location_pub = rospy.Publisher('/measure', String, queue_size=10)
		
		# current true and estimated location
		self.estimated_location = Pose()
		self.true_location = Pose()
		
		# while node is running
		while not rospy.is_shutdown():
			try:
				# recover estimated location from tf topic
				((self.estimated_location.position.x,
				self.estimated_location.position.y, 
				self.estimated_location.position.z), 
				(self.estimated_location.orientation.x, 
				self.estimated_location.orientation.y, 
				self.estimated_location.orientation.z, 
				self.estimated_location.orientation.w)) = \
					self._tflistener_sub.lookupTransform(TARGET_FRAME, SOURCE_FRAME, rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue
			
			# publish true and estimated location
			self._location_pub.publish(self.pose_to_str(self.estimated_location) + "," + self.pose_to_str(self.true_location))
			
			# print distance for debug
			#print(self.distance(self.true_location, self.estimated_location))
			
			# delay
			rate = rospy.Rate(FREQUENCY)
			rate.sleep()

	# gazebo callback (new location information)
	def gazebo_callback(self, MS):
		# get index of the robot in the list of objects in the gazebo simulation
		i = MS.name.index(GAZEBO_ROBOT_NAME)

		# extract its location
		self.true_location = MS.pose[i]

	# convert pose to string for CSV export
	def pose_to_str(self, p:Pose) -> String:
		# create string containing only x an y coordinates and pars it into CSV format
		ret = '{},{}'.format(p.position.x, p.position.y)
		return ret
		
	# compute 2D distance between two points (debug) 
	def distance(self, p1:Pose, p2:Pose):
		# compute 2D distance between two points in euclidien way
		dist = ((p1.position.x - p2.position.x)**2 + (p1.position.y - p2.position.y)**2)**0.5
		return dist


if __name__ == '__main__':
	try:
		measure = Measure()
	except rospy.ROSInterruptException as e:
		rospy.logerr(e)