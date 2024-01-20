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

dist2cam = 5

cube_vertices = np.array([[-1, -1, -1],
                          [1, -1, -1],
                          [1, 1, -1],
                          [-1, 1, -1],
                          [-1, -1, 1],
                          [1, -1, 1],
                          [1, 1, 1],
                          [-1, 1, 1]])


def draw_cube(q):
    # On efface l'écran
    screen.fill(WHITE)
    # On tourne le cube
    cube_vertices_rotated = np.array([q.rotate(v) for v in cube_vertices])
    # On décale le cube pour qu'il soit visible
    cube_vertices_rotated[:, 2] += dist2cam
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
        draw_cube(q)
        # On affiche le résultat
        pygame.display.flip()
        # On attend 10 ms avant de recommencer
        clock.tick(100)