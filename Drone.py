#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy

video = cv2.VideoCapture("/home/drone/Bureau/testpython/video4.avi")

bridge = CvBridge()


def talk():
		while True:
			success, image = video.read()

			if  success:
				message = bridge.cv2_to_imgmsg(image, "bgr8")
				publisher.publish(message)
				rate.sleep()
			else:
				video.set(cv2.CAP_PROP_POS_FRAMES, 0)


def callback(data):
	instruction = data.data
	print(instruction)


def listener():
	rospy.Subscriber('Publisher_Station', Image, callback)
	rospy.spin()


if __name__ == '__main__':
	try:
		# Init
		rospy.init_node('Drone_Topic', anonymous=True)

		# Publisher
		publisher = rospy.Publisher('Publisher_Drone', Image, queue_size=10)
		rate = rospy.Rate(33)
		rospy.loginfo("Publisher Drone Started")

		# Listener
		listener()
		rospy.loginfo("Listener Drone Connected")

		while not rospy.is_shutdown():
			talk()

	except rospy.ROSInterruptException:
		pass
