## LIT L'IMAGE IMPORTEE ##

import cv2
import numpy

image = cv2.imread("Ressources/Zoro.png")
image2 = numpy.array_like(image)
cv2.imshow("Window", image2)

cv2.waitKey(0);



