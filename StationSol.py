#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

bridge = CvBridge()


def callback(data):
	cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
	cv2.imshow("Image de base", cv_img)

	message = "OK"
	publisher.publish(message)
	rate.sleep()

	cv2.waitKey(1)


def listener():
	rospy.Subscriber('Publisher_Drone', Image, callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		# Init node
		rospy.init_node('Station_Topic', anonymous=True)

		# Publisher
		publisher = rospy.Publisher('Publisher_Station', Image, queue_size=10)
		rate = rospy.Rate(33)
		rospy.loginfo("Publisher Station Started")

		# Listener
		listener()
		rospy.loginfo("Listener Station Connected")

	except rospy.ROSInterruptException:
		pass
