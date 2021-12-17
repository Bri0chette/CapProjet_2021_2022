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


capture = cv2.VideoCapture(0)
capture.set(3, 640)
capture.set(4, 480)

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 300)
cv2.createTrackbar("Blue", "TrackBars", 255,   255, empty)
cv2.createTrackbar("Green", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Red", "TrackBars", 255,   255, empty)

while True :
    success, image = capture.read()

    blue = cv2.getTrackbarPos("Blue", "TrackBars")
    green = cv2.getTrackbarPos("Green", "TrackBars")
    red = cv2.getTrackbarPos("Red", "TrackBars")

    color = np.array([blue, green, red])

    mask = cv2.inRange(image, np.array([0, 0, 0]), color)

    imageFilter = cv2.bitwise_and(image, image, mask=mask)

    imageStack = stackImages(0.5, ([image, mask, imageFilter]))

    cv2.imshow("Window", imageStack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

