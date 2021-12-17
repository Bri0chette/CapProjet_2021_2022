import cv2
import numpy as np

def empty(a):
    pass

def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]),
                                                None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank] * rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


path = "../Ressources/Zoro.png"

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 300)
cv2.createTrackbar("Hue min", "TrackBars", 40,   179, empty)
cv2.createTrackbar("Hue max", "TrackBars", 60, 179, empty)
cv2.createTrackbar("Sat min", "TrackBars", 30,   255, empty)
cv2.createTrackbar("Sat max", "TrackBars", 90, 255, empty)
cv2.createTrackbar("Val min", "TrackBars", 190,   255, empty)
cv2.createTrackbar("Val max", "TrackBars", 255, 255, empty)

while True :
    image = cv2.imread(path)
    imageHSV = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hueMin = cv2.getTrackbarPos("Hue min", "TrackBars")
    hueMax = cv2.getTrackbarPos("Hue max", "TrackBars")
    satMin = cv2.getTrackbarPos("Sat min", "TrackBars")
    satMax = cv2.getTrackbarPos("Sat max", "TrackBars")
    valMin = cv2.getTrackbarPos("Val min", "TrackBars")
    valMax = cv2.getTrackbarPos("Val max", "TrackBars")

    lower = np.array([hueMin, satMin, valMin])
    upper = np.array([hueMax, satMax, valMax])

    mask = cv2.inRange(imageHSV, lower, upper)

    imageResult = cv2.bitwise_and(image, image, mask=mask)

    imageStack = stackImages(0.5, ([image, imageHSV, mask, imageResult]))

    cv2.imshow("Window color recognition", imageStack)

    cv2.waitKey(1)
