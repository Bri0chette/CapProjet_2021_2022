import cv2
import numpy as np

image = np.zeros((512, 512, 3), np.uint8)
#print(image.shape)

#image[:] = 255, 255, 0

cv2.line(image, (0, 0), (image.shape[1], image.shape[0]), (0, 255, 0), 5)
cv2.rectangle(image, (0, 0), (200, 200), (0, 0, 255), 5)
#cv2.rectangle(image, (0, 0), (200, 200), (0, 0, 255), cv2.FILLED)
cv2.circle(image, (256, 256), 50, (255, 255, 0), 5)

cv2.putText(image, "Bonsoir", (220, 60), cv2.FONT_HERSHEY_COMPLEX, 2, (255, 255, 255), 3)

cv2.imshow("Window", image)

cv2.waitKey(0)
