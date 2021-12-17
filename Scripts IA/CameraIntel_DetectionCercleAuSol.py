import cv2
import pyrealsense2 as rs
import numpy as np


# Fonction vide pour les trackbars
def empty(a):
    pass


# Fonction de détection d'un cercle en détectant les contours
def getContours(img):
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 500 < area:
            perimeter = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True)
            objectCorners = len(approx)
            x, y, width, height = cv2.boundingRect(approx)

            # On considère la forme comme un cercle à partir de 5 coins, et on retourne ses informations
            if objectCorners > 5:
                cPositionX = x
                cPositionY = y
                cWidth = width
                cHeight = height
                cCenterX = x + width // 2
                cCenterY = y + height // 2
                return str(cPositionX), str(cPositionY), str(cWidth), str(cHeight), str(cCenterX), str(cCenterY)

    # Retour par défaut si aucun cercle n'a été trouvé
    return "none", "none", "none", "none", "none", "none"


# Capture du flux de la webcam
video = cv2.VideoCapture(".avi")
pipeline = rs.pipeline()

# Create a config object
config = rs.config()
# Tell config that we will use a recorded device from file to be used by the pipeline through playback.
#s.config.enable_device_from_file(config, args.input)

# Configure the pipeline to stream the depth stream
# Change this parameters according to the recorded bag file resolution
config.enable_stream(rs.stream.color, rs.format.bgr8, 30)
pipeline.start(config) 
colorizer = rs.colorizer()

# CIBLE SUR L'IMAGE PARAMETRABLE
targetPointX = 800
targetPointY = 540

# Trackbars pour modifier la position de la cible (pour les tests)
cv2.namedWindow("Target Point Position")
cv2.resizeWindow("Target Point Position", 500, 100)
cv2.createTrackbar("Position X", "Target Point Position", targetPointX,  int(video.get(cv2.CAP_PROP_FRAME_WIDTH)), empty)
cv2.createTrackbar("Position Y", "Target Point Position", targetPointY, int(video.get(cv2.CAP_PROP_FRAME_HEIGHT)), empty)

while True:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    H = color_frame.get_height()
    W = color_frame.get_width()

    #convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    image = color_image.copy()
    
    # Modification de l'image pour faciliter la détection de contours
    imageModified = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imageModified = cv2.GaussianBlur(imageModified, (7, 7), 2)
    imageModified = cv2.Canny(imageModified, 50, 50)

    # FPS de la vidéo
    cv2.putText(image, "FPS : " + str(video.get(cv2.CAP_PROP_FPS)),(image.shape[1] - 100, 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 255), 1)

    # Récupération de la position de la cible avec les trackbars
    targetPointX = cv2.getTrackbarPos("Position X", "Target Point Position")
    targetPointY = cv2.getTrackbarPos("Position Y", "Target Point Position")

    # Appel de la fonction getContours() pour rechercher les contours de l'image et détecter un cercle. On récupère alors les informations du cercle.
    circlePositionX, circlePositionY, circleWidth, circleHeight, circleCenterX, circleCenterY = getContours(imageModified)

    #Couleur du cercle en BGR si détecté
    if circlePositionX != "none":
        circleColorB = int(image[int(circleCenterY), int(circleCenterX), 0])
        circleColorG = int(image[int(circleCenterY), int(circleCenterX), 1])
        circleColorR = int(image[int(circleCenterY), int(circleCenterX), 2])

        # Bleu
        if circleColorB >= 150 and circleColorG <= 50 and circleColorR <= 50 :
            circleColor = "Blue"
        # Vert
        elif circleColorB <= 50 and circleColorG >= 150 and circleColorR <= 50 :
            circleColor = "Green"
        # Rouge
        elif circleColorB <= 50 and circleColorG <= 50 and circleColorR >= 150:
            circleColor = "Red"
        # Couleur non définie
        else :
            circleColor = "Unknown color"

    # Message pour les instructions envoyées au drone
    messageX, messageY = "none", "none"

    # Si un cercle a été détecté
    if circlePositionX != "none":

        # On regarde sa position et on la compare à celle de la cible pour définir les instructions.
        if int(circleCenterX) < targetPointX - 10:
            messageX = "GAUCHE"
        elif targetPointX + 10 < int(circleCenterX):
            messageX = "DROITE"
        else:
            messageX = "OK"

        if int(circleCenterY) < targetPointY - 10:
            messageY = "DEVANT"
        elif targetPointY + 10 < int(circleCenterY):
            messageY = "DERRIERE"
        else:
            messageY = "OK"

        # Si le cercle est sur la cible
        if messageX == "OK" and messageY == "OK":
            # On dessine le repérage du cercle sur l'image en blanc
            cv2.rectangle(image, (int(circlePositionX), int(circlePositionY)), (int(circlePositionX) + int(circleWidth), int(circlePositionY) + int(circleHeight)), (255, 255, 255), 1)
            cv2.rectangle(image, (int(circlePositionX) + int(circleWidth) // 2, int(circlePositionY) + int(circleHeight) // 2), (int(circlePositionX) + int(circleWidth) // 2, int(circlePositionY) + int(circleHeight) // 2), (255, 255, 255), 5)
            cv2.putText(image, circleColor + " circle", (int(circlePositionX) + int(circleWidth) // 2 - 20, int(circlePositionY) + int(circleHeight) + 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255),1)
            cv2.putText(image, "B : " + str(circleColorB), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 15),cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(image, "G : " + str(circleColorG), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(image, "R : " + str(circleColorR), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 45), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)


            # Affichage de la position du cercle dans l'image
            cv2.putText(image, "Circle Position : (" + circleCenterX + ";" + circleCenterY + ")", (image.shape[1] - 250, image.shape[0] - 6), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 255, 255), 1)
            # Envoi des instructions dans la console
            print("INSTRUCTIONS\tDESCENTE")

        # Si le cercle n'est pas sur la cible
        else:
            # On dessine le repérage du cercle sur l'image de la même couleur que le cercle
            cv2.rectangle(image, (int(circlePositionX), int(circlePositionY)), (int(circlePositionX) + int(circleWidth), int(circlePositionY) + int(circleHeight)), (circleColorB, circleColorG, circleColorR), 1)
            cv2.rectangle(image, (int(circlePositionX) + int(circleWidth) // 2, int(circlePositionY) + int(circleHeight) // 2), (int(circlePositionX) + int(circleWidth) // 2, int(circlePositionY) + int(circleHeight) // 2), (0.5 *  circleColorB, 0.5 * circleColorG, 0.5 *  circleColorR), 5)
            cv2.putText(image, circleColor + " circle", (int(circlePositionX) + int(circleWidth) // 2 - 20, int(circlePositionY) + int(circleHeight) + 15),cv2.FONT_HERSHEY_COMPLEX, 0.5, (circleColorB, circleColorG, circleColorR), 1)
            cv2.putText(image, "B : " + str(circleColorB), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (255, 0, 0), 1)
            cv2.putText(image, "G : " + str(circleColorG), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 30), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(image, "R : " + str(circleColorR), (int(circlePositionX) + int(circleWidth) + 5, int(circlePositionY) + 45), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)

            # Affichage de la position du cercle dans l'image
            cv2.putText(image, "Circle Position : (" + circleCenterX + ";" + circleCenterY + ")", (image.shape[1] - 250, image.shape[0] - 6), cv2.FONT_HERSHEY_COMPLEX, 0.5, (circleColorB, circleColorG, circleColorR), 1)
            # Envoi des instructions dans la console
            print("INSTRUCTIONS\tX : " + messageX + " Y : " + messageY)

        # Affichage de la position de la cible dans l'image
        cv2.putText(image, "Target Position : (" + str(targetPointX) + ";" + str(targetPointY) + ")",(1, image.shape[0] - 6), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)

    # Si aucun cercle n'a été détecté
    else:
        # Affichage de la position du cercle et de la cible dans l'image
        cv2.putText(image, "Target Position : (" + str(targetPointX) + ";" + str(targetPointY) + ")",(1, image.shape[0] - 6), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
        cv2.putText(image, "Circle Position : (" + circleCenterX + ";" + circleCenterY + ")",(image.shape[1] - 250, image.shape[0] - 6), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)
        # Envoi des instructions dans la console
        print("INSTRUCTIONS\tX : " + messageX + " Y : " + messageY)

    # Affichage de la cible sur l'image
    cv2.rectangle(image, (targetPointX, targetPointY), (targetPointX, targetPointY), (0, 0, 255), 5)
    cv2.putText(image, "Target", (targetPointX - 20, targetPointY + 15), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255), 1)

    # Affichage de l'image dans une fenêtre
    cv2.imshow("Window1", imageModified)
    cv2.imshow("Window", image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
