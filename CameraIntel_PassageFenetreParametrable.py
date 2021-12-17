"""
ALGORITHME DE PASSAGE DE FENETRE PARAMETRABLE

Les paramètres du programme sont paramétrables en temps réel à l'éxecution à l'aide de trackbars. Cela peut-être idéal pour les tests et le réglage précis des paramètres.
Une fois les paramètres bien définis, ils peuvent être ajoutés dans le code du programme PASSAGE DE FENETRE.

Description :
Le programme lit une vidéo filmée avec la caméra de profondeur Intel Realsense.
A chaque image de la vidéo, les couleurs (nuances de rouge, vert et bleu pour la profondeur) sont filtrées pour ne faire apparaître que la fenêtre sur un mask en noir et blanc.
On applique des modifications sur le mask pour améliorer la qualité des filtres (érosion, dilatation).
On détecte ensuite le rectangle de la fenêtre dans le mask, en récupérant ses valeurs de position dans l'image.
On peut alors envoyer des instructions dans la console pour recentrer la fenêtre dans l'image, ou dire au drone d'avancer si elle est au milieu de l'image (ou sur le point cible que l'on peut définir).
"""


import pyrealsense2 as rs
import cv2
import numpy as np



# Fonction vide utile pour les trackbars.
def empty(a):
    pass


# Fonction de détection des contours dans une image.
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


# Lancement du live
pipeline = rs.pipeline()

    # Create a config object
config = rs.config()
    # Tell config that we will use a recorded device from file to be used by the pipeline through playback.
    #s.config.enable_device_from_file(config, args.input)

    # Configure the pipeline to stream the depth stream
    # Change this parameters according to the recorded bag file resolution
config.enable_stream(rs.stream.depth, rs.format.z16, 30)
pipeline.start(config) 
    # Create colorzer object
colorizer = rs.colorizer()

# **************************** PARAMETRES DU PROGRAMME, MODIFIABLES EN TEMPS REEL A L'EXECUTION GRACE AUX TRACKBARS ****************************

# Paramètres par défaut des valeurs minimales et maximales de couleurs (de 0 à 255) pour les filtrer dans l'image.
blueMin = 0
blueMax = 0
greenMin = 0
greenMax = 130
redMin = 100
redMax = 255 

# Paramètres par défaut des fonctions d'érosion et de dilatation.
kernelValue = 15
kernel = np.ones((kernelValue, kernelValue), np.uint8)
iterations = 2

# Paramètres par défaut de l'air minimal et maximal du rectangle détecté (pour éviter de détecter des petits rectangles qui peuvent apparaîtrent dans l'image).
areaMin = 200
areaMax = 100000

# Trackbars pour changer les valeurs des paramètres pendant l'éxecution du programme, et voir l'impact en temps réel sur l'image.
# Attention, une fois les paramètres réglés, il ne sont pas changés dans le programme, il faut les noter et changer les valeurs par défaut à la main dans le code.
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 500)
cv2.createTrackbar("Blue min", "TrackBars", blueMin,   255, empty)
cv2.createTrackbar("Blue max", "TrackBars", blueMax, 255, empty)
cv2.createTrackbar("Green min", "TrackBars", greenMin,   255, empty)
cv2.createTrackbar("Green max", "TrackBars", greenMax, 255, empty)
cv2.createTrackbar("Red min", "TrackBars", redMin,   255, empty)
cv2.createTrackbar("Red max", "TrackBars", redMax, 255, empty)
cv2.createTrackbar("Kernel", "TrackBars", kernelValue, 50, empty)
cv2.createTrackbar("Iterations", "TrackBars", iterations, 10, empty)
cv2.createTrackbar("Area min", "TrackBars", areaMin, 1000, empty)
cv2.createTrackbar("Area max", "TrackBars", areaMax, 100000, empty)

# ************************************************************************************************************************************************************************************


# Boucle de lecture de la vidéo.
while True :

       # Get frameset of depth
    frames = pipeline.wait_for_frames()

    # Get depth frame
    depth_frame = frames.get_depth_frame()

    # Colorize depth frame to jet colormap
    depth_color_frame = colorizer.colorize(depth_frame)

    # Convert depth_frame to numpy array to render image in opencv
    depth_color_image = np.asanyarray(depth_color_frame.get_data())
    image = depth_color_image.copy()

    # On récupère les valeurs des trackbars à chaque image pour appliquer les modifications si il y en a eu.
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

    # Valeurs basses et hautes pour le mask. On va garder les couleurs entre ces valeurs.
    lower = np.array([blueMin, greenMin, redMin])
    upper = np.array([blueMax, greenMax, redMax])

    # Création du mask en noir et blanc.
    mask = cv2.inRange(image, lower, upper)

    # Application de l'érosion et de la dilatation pour améliorer la qualité du mask.
    mask = cv2.dilate(mask, kernel, iterations=iterations)
    mask = cv2.erode(mask, kernel, iterations=iterations)

    # Création de l'image filtrée. On applique le mask sur l'image de base pour n'afficher que les éléments qui nous intéressent.
    imageFiltered = cv2.bitwise_and(image, image, mask=mask)

    # On lance la fonction getContours(). Elle permet de détecter les contours de l'image et récupérer le contour qui correpond à la fenêtre.
    # On passe le mask en noir et blanc en paramètre, et l'air minimal de la fenêtre que l'on veut trouver, pour faciliter la détection des contours.
    # L'image imageFiltered sera aussi modifiée dans getContours() pour afficher la détection directement dessus.
    # getContours renvoie la position et la taille de la fenêtre détectée.
    x, y, width, height = getContours(mask, areaMin)

    # On vérifie si la fonction getContours() a renvoyé une valeur. Si aucune fenêtre n'a été détectée, getContours renvoie "none". Attention, la fonction renvoie des valeurs de type string.
    # On peut ainsi définir le centre de la fenêtre détectée en X et en Y. La fonction renvoie le coin supérieur gauche de la fenêtre détectée, on applique alors quelques modifications pour avoir le centre.
    # Attention, l'axe Y va vers le bas.
    if x == "none":
        centerWinX = "none"
        centerWinY = "none"
    else :
        centerWinX = int(x) + int(width) // 2
        centerWinY = int(y) + int(height) // 2

    # On ajoute du texte sur imageFiltered en bas pour afficher la position de la fenêtre détectée.
    cv2.putText(imageFiltered, "Window position : ( " + str(centerWinX) + " ; " + str(centerWinY) + " )", (10, imageFiltered.shape[0] - 10), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2)

    # On capture une image si on détecte une fenêtre, pour être sûr que le programme fonctionne bien.
    if x != "none":
        cv2.imwrite("Ressources/winOut.png", imageFiltered)

    # On crée ensuite les messages d'instructions. Il serviront à dire au drone de monter, descendre, etc, suivant la zone cible définie (ici la zone cible est le centre de l'image).
    messageX = ""
    messageY = ""
    targetPointXMin = imageFiltered.shape[1] // 2 - 10
    targetPointXMax = imageFiltered.shape[1] // 2 + 10
    targetPointYMin = imageFiltered.shape[0] // 2 - 10
    targetPointYMax = imageFiltered.shape[0] // 2 + 10

    # On vérifie où se situe la fenêtre et on définit les messages en fonction.
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

    # On envoie les messages dans la console.
    print("INSTRUCTIONS\tX : " + messageX + "\tY : " + messageY)
    if messageX == "OK" and messageY == "OK":
        print("AVANCER")

    # On affiche les images que l'on souhaite à l'écran.
    cv2.imshow("Image de base", image)
    cv2.imshow("Mask", mask)
    cv2.imshow("Image filtrée et détection", imageFiltered)

    # Temps entre deux images à traiter : pour une vidéo de 30 ips -> 1 image par 33.33 millisecondes
    # Presser 'q' pour quitter le programme
    if cv2.waitKey(33) & 0xFF == ord('q'):
        break