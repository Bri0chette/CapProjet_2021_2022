#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

bridge = CvBridge()


def callback(data):
	messageDrone = data.data
	print(messageDrone)

	message = "Message de la station"
	publisher.publish(message)
	rate.sleep()


def listener():
	rospy.init_node('Subscriber_Station_Topic')
	rospy.Subscriber('Publisher_Drone', String, callback)
	rospy.spin()


if __name__ == '__main__':
	try:

		# Publisher
		publisher = rospy.Publisher('Publisher_Station', String)
		rospy.init_node('Publisher_Station_Topic')
		rate = rospy.Rate(33)

		while not rospy.is_shutdown():
			listener()


	except rospy.ROSInterruptException:
		pass
