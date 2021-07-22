#!/usr/bin/env python
# licence removed for brevity

import rospy
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Time
from std_msgs.msg import Header
from std_msgs.msg import Duration

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def gazebo_model_test():

	pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
	rospy.init_node('gazebo_model_test', anonymous=True)
	rate = rospy.Rate(10) #10Hz
	while not rospy.is_shutdown():
		tracebot = JointTrajectory()
		point = JointTrajectoryPoint()
	
		tracebot.joint_names = ['left_shoulder_pan_joint', 'left_shoulder_lift_joint', 'left_elbow_joint', 'left_wrist_1_joint', 'left_wrist_2_joint', 'left_wrist_3_joint','right_shoulder_pan_joint', 'right_shoulder_lift_joint', 'right_elbow_joint', 'right_wrist_1_joint', 'right_wrist_2_joint', 'right_wrist_3_joint']
		tracebot.header.stamp = rospy.Time.now()
		
		point.positions = [1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5,1.5]
		point.velocities = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
		point.accelerations = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
		point.effort = [1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]
		point.time_from_start = rospy.Duration(2)
		tracebot.points.append(point)
		
		rospy.loginfo(tracebot)
		pub.publish(tracebot)
		rate.sleep()

if __name__ == '__main__':
	try:
		gazebo_model_test()
	except rospy.ROSInterruptException:
		pass
