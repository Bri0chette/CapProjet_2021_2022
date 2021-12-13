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


def getContours(image, areaMin):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if areaMin < area:
            cv2.drawContours(imageFilteredAndContours, cnt, -1, (0 , 0, 255), 3)
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            objectCorners = len(approx)
            x, y, width, height = cv2.boundingRect(approx)

            if objectCorners == 4:
                objectType = "Window"
            else:
                objectType = "None"

            if objectType == "Window":
                cv2.rectangle(imageFilteredAndContours, (x, y), (x + width, y + height), (0, 255, 0), 3)
                cv2.putText(imageFilteredAndContours, objectType, (x + width // 2, y + height + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            return x, y, width, height
        else:
            return "none", "none", "none", "none"


video = cv2.VideoCapture("Ressources/video4.avi")
imageBlack = np.zeros((500, 500), np.uint8)

#Dilate and Erode parameters
kernelValue = 15
kernel = np.ones((kernelValue, kernelValue), np.uint8)
iterations = 2

cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 500)
cv2.createTrackbar("Blue min", "TrackBars", 150,   255, empty)
cv2.createTrackbar("Blue max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Green min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("Green max", "TrackBars", 100, 255, empty)
cv2.createTrackbar("Red min", "TrackBars", 0,   255, empty)
cv2.createTrackbar("Red max", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Kernel", "TrackBars", kernelValue, 50, empty)
cv2.createTrackbar("Iterations", "TrackBars", iterations, 10, empty)
cv2.createTrackbar("Area min", "TrackBars", 200, 1000, empty)
cv2.createTrackbar("Area max", "TrackBars", 100000, 100000, empty)


while True :
    success, image = video.read()

    #Video loop
    if success == False:
        video = cv2.VideoCapture("Ressources/video4.avi")
        success, image = video.read()

    imageBGR = image.copy()
    blueMin = cv2.getTrackbarPos("Blue min", "TrackBars")
    blueMax = cv2.getTrackbarPos("Blue max", "TrackBars")
    greenMin = cv2.getTrackbarPos("Green min", "TrackBars")
    greenMax = cv2.getTrackbarPos("Green max", "TrackBars")
    redMin = cv2.getTrackbarPos("Red min", "TrackBars")
    redMax = cv2.getTrackbarPos("Red max", "TrackBars")
    kernelValue = cv2.getTrackbarPos("Kernel", "TrackBars")
    iterations = cv2.getTrackbarPos("Iterations", "TrackBars")
    kernel = np.ones((kernelValue, kernelValue), np.uint8)
    areaMin = cv2.getTrackbarPos("Area min", "TrackBars")
    areaMax = cv2.getTrackbarPos("Area max", "TrackBars")

    lower = np.array([blueMin, greenMin, redMin])
    upper = np.array([blueMax, greenMax, redMax])

    maskColor = cv2.inRange(imageBGR, lower, upper)
    maskDilate = cv2.dilate(maskColor, kernel, iterations=iterations)
    maskErode = cv2.erode(maskDilate, kernel, iterations=iterations)
    mask = maskErode.copy()

    imageFiltered = cv2.bitwise_and(imageBGR, imageBGR, mask=mask)
    imageFilteredAndContours = imageFiltered.copy()
    #x, y, width, height -> window position, width and height
    x, y, width, height = getContours(mask, areaMin)
    if x == "none":
        centerWinX = "none"
        centerWinY = "none"
    else :
        centerWinX = int(x) + int(width) // 2
        centerWinY = int(y) + int(height) // 2

    cv2.putText(imageFilteredAndContours, "Window position : ( " + str(centerWinX) + " ; " + str(centerWinY) + " )", (10, imageFilteredAndContours.shape[0] - 10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)
    if x != "none":
        cv2.imwrite("Ressources/winOut.png", imageFilteredAndContours)

    messageX = ""
    messageY = ""

    if centerWinX == "none":
        messageX = "none"
    else:
        if int(centerWinX) < imageFilteredAndContours.shape[1] // 2 - 10:
            messageX = "DROITE"
        elif int(centerWinX) > imageFilteredAndContours.shape[1] // 2 + 10:
            messageX = "GAUCHE"
        else:
            messageX = "STABLE"

    if centerWinY == "none":
        messageY = "none"
    else:
        if int(centerWinY) < imageFilteredAndContours.shape[0] // 2 - 10:
            messageY = "BAS"
        elif int(centerWinY) > imageFilteredAndContours.shape[0] // 2 + 10:
            messageY = "HAUT"
        else:
            messageY = "STABLE"

    print("INSTRUCTIONS\tX : " + messageX + "\tY : " + messageY)

    imageStack = stackImages(0.8, ([imageBGR, mask, imageFilteredAndContours]))

    cv2.imshow("Window", imageStack)
    cv2.imshow("WindowBlack", imageBlack)

    #Wait time : 30 fps video -> 1 frame per 33.33 milliseconds
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break







