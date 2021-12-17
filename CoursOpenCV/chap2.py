import cv2
import numpy as np
print("Package imported")

image = cv2.imread("../Scripts IA/Ressources/Zoro.png")
kernel = np.ones((5, 5), np.uint8)

imageGris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
imageBlur = cv2.GaussianBlur(image, (7, 7), 0)
imageContours = cv2.Canny(image, 100, 100)
imageDilatee = cv2.dilate(imageContours, kernel, iterations=1)
imageErodee = cv2.erode(imageDilatee, kernel, iterations=1)

cv2.imshow("Image Gris", imageGris)
cv2.imshow("Image Blur", imageBlur)
cv2.imshow("Image Contours", imageContours)
cv2.imshow("Image Dialation", imageDilatee)
cv2.imshow("Image Erodee", imageErodee)
cv2.waitKey(0)