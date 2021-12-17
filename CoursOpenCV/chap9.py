import cv2
import numpy as np

pathCascade = "./haarcascade_frontalface_default.xml"
faceCascade = cv2.CascadeClassifier(pathCascade)

capture = cv2.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 480)

while True :
    success, image = capture.read()
    imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    faces = faceCascade.detectMultiScale(imageGray, 1.1, 10)
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 3)
        cv2.putText(image, "Visage", (x+w//2-200,  y+h+50), cv2.FONT_HERSHEY_COMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Window", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break