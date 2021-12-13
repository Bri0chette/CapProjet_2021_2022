"""
ALGORITHME DE PASSAGE DE FENETRE

Cet algorithme est le même que PASSAGE DE FENETRE PARAMETRABLE. A la différence que les paramètres sont fixes, et définis antérieurement grâce à l'algorithme paramétrable.
Il est donc plus optimisé, et idéal pour l'utilisation en situation réelle.
Le code ne sera pas commenté de façon aussi complète que dans le programme PASSAGE DE FENETRE PARAMETRABLE. Vous pourrez vous y référer si des éléments vous posent problème.
"""


import cv2
import numpy as np


def empty(a):
    pass


def getContours(image, areaMin):
    contours, hierarchy = cv2.findContours(image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if areaMin < area:
            cv2.drawContours(imageFiltered, cnt, -1, (0 , 0, 255), 3)
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            objectCorners = len(approx)
            x, y, width, height = cv2.boundingRect(approx)

            if objectCorners == 4:
                objectType = "Window"
            else:
                objectType = "None"

            if objectType == "Window":
                cv2.rectangle(imageFiltered, (x, y), (x + width, y + height), (0, 255, 0), 3)
                cv2.putText(imageFiltered, objectType, (x + width // 2, y + height + 30), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

            return x, y, width, height
        else:
            return "none", "none", "none", "none"


# Source de la vidéo à lire.
video = cv2.VideoCapture("Ressources/video4.avi")


# Paramètres par défaut des valeurs minimales et maximales de couleurs (de 0 à 255) pour les filtrer dans l'image.
blueMin = 150
blueMax = 255
greenMin = 0
greenMax = 100
redMin = 0
redMax = 0

# Paramètres par défaut des fonctions d'érosion et de dilatation.
kernelValue = 15
kernel = np.ones((kernelValue, kernelValue), np.uint8)
iterations = 2

# Paramètres par défaut de l'air minimal et maximal du rectangle détecté (pour éviter de détecter des petits rectangles qui peuvent apparaîtrent dans l'image).
areaMin = 200
areaMax = 100000

# Valeurs basses et hautes pour le mask. On va garder les couleurs entre ces valeurs.
lower = np.array([blueMin, greenMin, redMin])
upper = np.array([blueMax, greenMax, redMax])


# Boucle de lecture de la vidéo.
while True :

    success, image = video.read()

    # Video loop.
    if success == False:
        video = cv2.VideoCapture("Ressources/video4.avi")
        success, image = video.read()

    mask = cv2.inRange(image, lower, upper)
    mask = cv2.dilate(mask, kernel, iterations=iterations)
    mask = cv2.erode(mask, kernel, iterations=iterations)

    imageFiltered = cv2.bitwise_and(image, image, mask=mask)

    x, y, width, height = getContours(mask, areaMin)

    if x == "none":
        centerWinX = "none"
        centerWinY = "none"
    else :
        centerWinX = int(x) + int(width) // 2
        centerWinY = int(y) + int(height) // 2

    cv2.putText(imageFiltered, "Window position : ( " + str(centerWinX) + " ; " + str(centerWinY) + " )", (10, imageFiltered.shape[0] - 10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    if x != "none":
        cv2.imwrite("Ressources/winOut.png", imageFiltered)

    messageX = ""
    messageY = ""
    targetPointXMin = imageFiltered.shape[1] // 2 - 10
    targetPointXMax = imageFiltered.shape[1] // 2 + 10
    targetPointYMin = imageFiltered.shape[0] // 2 - 10
    targetPointYMax = imageFiltered.shape[0] // 2 + 10

    if centerWinX == "none":
        messageX = "none"
    else:
        if int(centerWinX) < targetPointXMin:
            messageX = "DROITE"
        elif int(centerWinX) > targetPointXMax:
            messageX = "GAUCHE"
        else:
            messageX = "OK"

    if centerWinY == "none":
        messageY = "none"
    else:
        if int(centerWinY) < targetPointYMin:
            messageY = "BAS"
        elif int(centerWinY) > targetPointYMax:
            messageY = "HAUT"
        else:
            messageY = "OK"

    print("INSTRUCTIONS\tX : " + messageX + "\tY : " + messageY)
    if messageX == "OK" and messageY == "OK":
        print("AVANCER")

    cv2.imshow("Image de base", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Image filtrée et détection", imageFiltered)

    if cv2.waitKey(33) & 0xFF == ord('q'):
        break