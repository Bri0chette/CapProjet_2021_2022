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


def getContours(image):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    return contours


path = "../Scripts IA/Ressources/out5.png"
kernel = np.ones((5, 5), np.uint8)


cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 300)
cv2.createTrackbar("Blue min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("Blue max", "TrackBars", 95, 255, empty)
cv2.createTrackbar("Green min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("Green max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Red min", "TrackBars", 150,   255, empty)
cv2.createTrackbar("Red max", "TrackBars", 255, 255, empty)


while True :
    image = cv2.imread(path)
    imageBGR = image.copy()
    blueMin = cv2.getTrackbarPos("Blue min", "TrackBars")
    blueMax = cv2.getTrackbarPos("Blue max", "TrackBars")
    greenMin = cv2.getTrackbarPos("Green min", "TrackBars")
    greenMax = cv2.getTrackbarPos("Green max", "TrackBars")
    redMin = cv2.getTrackbarPos("Red min", "TrackBars")
    redMax = cv2.getTrackbarPos("Red max", "TrackBars")

    lower = np.array([blueMin, greenMin, redMin])
    upper = np.array([blueMax, greenMax, redMax])

    maskColor = cv2.inRange(imageBGR, lower, upper)
    maskDilate = cv2.dilate(maskColor, kernel, iterations=2)
    maskErode = cv2.erode(maskDilate, kernel, iterations=2)
    mask = maskErode.copy()

    imageFiltered = cv2.bitwise_and(imageBGR, imageBGR, mask=mask)

    contours = getContours(mask)
    imageFilteredAndContours = imageFiltered.copy()
    imageFilteredAndContours = cv2.drawContours(imageFilteredAndContours, contours, -1, (255, 255, 0), 2)

    imageStack = stackImages(0.4, ([imageBGR, mask, imageFilteredAndContours]))

    cv2.imshow("Window", imageStack)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break