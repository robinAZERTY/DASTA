import cv2

# Obtenir le nombre de caméras connectées à l'ordinateur
num_cameras = 2  # Vous pouvez modifier cette valeur si nécessaire

# Ouvrir les flux vidéo pour chaque caméra connectée
capture_devices = [cv2.VideoCapture(idx+1) for idx in range(num_cameras)]

# Vérifier si les flux vidéo ont été ouverts avec succès
for idx, capture in enumerate(capture_devices):
    if not capture.isOpened():
        print(f"Impossible d'ouvrir la caméra {idx}")
        capture_devices[idx] = None

# Boucle pour afficher les flux vidéo en direct de chaque caméra
while True:
    frames = []
    for capture in capture_devices:
        if capture is not None:
            ret, frame = capture.read()
            if ret:
                frames.append(frame)
            else:
                print("Impossible de lire un frame d'une des caméras.")
                break

    # Affichage des frames de chaque caméra
    for idx, frame in enumerate(frames):
        cv2.imshow(f'Camera {idx}', frame)

    # Sortie de la boucle si 'q' est pressé
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Libérer les ressources et fermer les fenêtres
for capture in capture_devices:
    if capture is not None:
        capture.release()
cv2.destroyAllWindows()
