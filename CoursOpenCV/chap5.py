import cv2
import numpy as np

image = cv2.imread("../Scripts IA/Ressources/Zoro.png")

width = 200
height = 100

points1 = np.float32([[220, 280], [420, 240], [440, 320], [240, 360]])
points2 = np.float32([[0, 0], [width, 0], [width, height], [0, height]])

matrix = cv2.getPerspectiveTransform(points1, points2)

imageOutput = cv2.warpPerspective(image, matrix, (width, height))

cv2.imshow("Window", image)
cv2.imshow("Window Perspective", imageOutput)

cv2.waitKey(0)