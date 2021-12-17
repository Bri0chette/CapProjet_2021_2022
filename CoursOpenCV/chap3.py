import cv2
import numpy as np

image = cv2.imread("../Scripts IA/Ressources/Zoro.png")
print(image.shape)

imageResize = cv2.resize(image, (300, 300))
print(imageResize.shape)

imageCropped = image[0:200, 200:500]

cv2.imshow("Window", image)
cv2.imshow("Window Resize", imageResize)
cv2.imshow("Window Cropped", imageCropped)

cv2.waitKey(0)