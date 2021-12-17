import cv2
print("Package imported")

capture = cv2.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 480)
capture.set(10, 100)


while True:
    success, image = capture.read()
    cv2.imshow("Window", image)
    if cv2.waitKey(1) & 0xFF ==ord('q'):
        break

