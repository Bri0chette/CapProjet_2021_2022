import cv2
import numpy as np

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
    for cnt in contours:
        area = cv2.contourArea(cnt)
        print(area)
        if area > 500:
            cv2.drawContours(imageContours, cnt, -1, (255, 255, 0), 3)
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02*perimeter, True)
            objectCorners = len(approx)
            x, y, width, height = cv2.boundingRect(approx)

            if objectCorners == 3:
                objectType = "Triangle"
            elif objectCorners == 4:
                ratio = width/float(height)
                if 0.95 < ratio and ratio < 1.05:
                    objectType = "Carre"
                else:
                    objectType = "Rectangle"
            else:
                objectType = "Cercle"

            cv2.rectangle(imageContours, (x, y), (x+width, y+height), (0, 0, 255), 3)
            cv2.putText(imageContours, objectType, (x+width//2-20,  y+height+20), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0), 2)





path = "../Ressources/Formes.jpg"

image = cv2.imread(path)
image = cv2.resize(image, (720, 480))
imageContours = image.copy()

imageBlack = np.zeros_like(image)

imageGray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
imageBlur = cv2.GaussianBlur(imageGray, (7, 7), 1)
imageCanny = cv2.Canny(imageBlur, 50, 50)

getContours(imageCanny)

imageStack = stackImages(0.8, ([image, imageGray, imageBlur], [imageCanny, imageContours, imageBlack]))

cv2.imshow("Window", imageStack)

cv2.waitKey(0)