import pygame
from pygame.locals import *
import numpy as np
from pyquaternion import Quaternion

# Initialisation de Pygame
pygame.init()

# Paramètres de la fenêtre
width, height = 800, 600
screen = pygame.display.set_mode((width, height))
screen2 = pygame.display.set_mode((width, height))
pygame.display.set_caption("Cube en orientation quaternion")

# Couleurs
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

dist2cam = 0.5

side = 0.1

cube_vertices = np.array([[-1, -1, -1],
                          [1, -1, -1],
                          [1, 1, -1],
                          [-1, 1, -1],
                          [-1, -1, 1],
                          [1, -1, 1],
                          [1, 1, 1],
                          [-1, 1, 1]]) * side



def draw_cube(X):
    q = Quaternion(X[6], X[7], X[8], X[9])
    p = X[0:3].reshape(3)
    # On efface l'écran
    screen.fill(WHITE)
    # On tourne le cube
    cube_vertices_rotated = np.array([q.rotate(v) for v in cube_vertices])
    # On décale le cube pour qu'il soit visible
    cube_vertices_rotated[:, 2] += dist2cam
    #on ajoute la position du cube
    cube_vertices_rotated[:,:] += p
    # On projette les points sur le plan de l'écran
    cube_vertices_projected = np.array([(x / z, y / z) for x, y, z in cube_vertices_rotated])
    # On dessine les arêtes du cube
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                 (4, 5), (5, 6), (6, 7), (7, 4),
                 (0, 4), (1, 5), (2, 6), (3, 7)]:
        pygame.draw.line(screen, BLACK,
                         (width / 2 + cube_vertices_projected[i][0] * width / 3,
                          height / 2 + cube_vertices_projected[i][1] * width / 3),
                         (width / 2 + cube_vertices_projected[j][0] * width / 3,
                          height / 2 + cube_vertices_projected[j][1] * width / 3))
        
    #on ajoute l'affichage des matrices de calibration
    txt = "acc_ortho_correction:"
    font = pygame.font.Font('freesansbold.ttf', 12)
    text = font.render(txt, True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 120)
    screen.blit(text, textRect)
    for i in range(3):
        txt  = np.array2string(X[13+i*3:13+i*3+3].reshape(3), precision=3, separator=',')
        text = font.render(txt, True, BLACK, WHITE)
        textRect = text.get_rect()
        textRect.center = (width // 2, height // 2 + 140 + i*20)
        screen.blit(text, textRect)
    
    
    
    
    
        
        
'''
        ____________STATE ESTIMATE VECTOR______________

        State                   unit        index
postion(xyz)                    m           1:3
velocity(xyz)                   m/s         4:6
orientation(quaternion)                     7:10
acc_bias_correction(xyz)        m/s²        11:13
acc_ortho_correction(3*3)                   14:22
gyr_ortho_correction(3*3)                   23:31
cam1_pos(xyz)                   m           32:34
cam1_ori(quaternion)                        35:38
cam1_k                          pixel        39
cam2_pos(xyz)                   m           40:42
cam2_ori(quaternion)                        43:46
cam2_k                          pixel        47
'''
        
def draw_state(state):
    # afficher l'état sur l'écran (avec du texte) state est un np.array de taille 47

    screen.fill(WHITE)
    font = pygame.font.Font('freesansbold.ttf', 12)
    # afficher la position
    text = font.render('position: ' + str(state[0:3]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2)
    screen.blit(text, textRect)
    # afficher la vitesse
    text = font.render('vitesse: ' + str(state[3:6]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 20)
    screen.blit(text, textRect)
    # afficher l'orientation
    text = font.render('orientation: ' + str(state[6:10]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 40)
    screen.blit(text, textRect)
    # afficher le biais d'accélération
    text = font.render('biais acc: ' + str(state[10:13]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 60)
    screen.blit(text, textRect)
    # afficher la les gains d'accélération
    text = font.render('gain acc: ' + str(state[[13, 17, 21]]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 80)
    screen.blit(text, textRect)
    # afficher la les gains de gyroscope
    text = font.render('gain gyr: ' + str(state[[22, 26, 30]]), True, BLACK, WHITE)
    textRect = text.get_rect()
    textRect.center = (width // 2, height // 2 + 100)
    screen.blit(text, textRect)
    
    
    
    
    

    
def draw_Cov(cov_matrix):
    # afficher la matrice de covariance sur l'écran2
    try :
        screen.fill(WHITE)
        # cov_matrix est un np.array carré. calculer la racine carrée de chaque élément pour avoir la matrice d'écart-type
        # cov_matrix = np.sqrt(cov_matrix)
        # afficher la matrice d'écart-type sur l'écran2 avec une echelle de couleur
        # pour chaque élément de la matrice
        m = np.max(cov_matrix)
        for i in range(cov_matrix.shape[0]):
            for j in range(cov_matrix.shape[1]):
                # calculer la couleur de l'élément
                color = int(cov_matrix[i, j] / m * 100)
                # afficher le pixel correspondant à l'élément
                pygame.draw.rect(screen, (color, color, color), (i*10, j*10, 10, 10))
        
    except Exception as e:
        pass
    
if __name__ == '__main__':
    # On crée un quaternion unitaire
    q = Quaternion()
    # On crée une horloge pour contrôler la vitesse de rotation
    clock = pygame.time.Clock()
    # Boucle de contrôle
    while True:
        # On récupère les entrées utilisateur
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                exit()
        # On calcule le quaternion de rotation
        q = Quaternion(axis=[1, 0, 0], angle=np.pi / 100) * q
        # On dessine le cube
        draw_cube(q,np.array([0,0,0]))
        # On affiche le résultat
        pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)