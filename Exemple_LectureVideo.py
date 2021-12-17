## LIT LA VIDEO IMPORTEE EN BOUCLE ##

import cv2

video = cv2.VideoCapture("Ressources/video4.avi")

while True:
    success, image = video.read()

    # Video loop
    if success == False:
        video = cv2.VideoCapture("Ressources/video4.avi")
        success, image = video.read()

    cv2.imshow("Window", image)

    # Wait time : 30 fps video -> 1 frame per 33.33 milliseconds
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break

