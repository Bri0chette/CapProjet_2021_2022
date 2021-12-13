#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

video = cv2.VideoCapture("/home/drone/Bureau/testpython/video4.avi")

bridge = CvBridge()


def callback(data):
	messageStation = data.data
	print(messageStation)

	message = "Message du drone"
	publisher.publish(message)
	rate.sleep()


def listener():
	rospy.init_node('Subscriber_Drone_Topic')
	rospy.Subscriber('Publisher_Station', String, callback)
	rospy.spin()


if __name__ == '__main__':
	try:

		# Publisher
		publisher = rospy.Publisher('Publisher_Drone', String)
		rospy.init_node('Publisher_Drone_Topic')
		rate = rospy.Rate(33)

		while not rospy.is_shutdown():
			message = "Message du drone"
			publisher.publish(message)
			rate.sleep()

			listener()

	except rospy.ROSInterruptException:
		pass
