import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Paramètres de la ligne en 3D
theta = np.linspace(0, 4*np.pi, 100)
x = np.cos(theta)
y = np.sin(theta)
z = theta/(4*np.pi)

# Création de la figure et de l'axe 3D
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Création de la ligne initiale
line, = ax.plot(x, y, z, 'b-')

# Fonction de mise à jour pour l'animation
def update(frame):
    # Mise à jour des données de la ligne
    line.set_data(x[:frame], y[:frame])
    line.set_3d_properties(z[:frame])

# Création de l'animation
animation = FuncAnimation(fig, update, frames=len(theta), interval=50, repeat=False)

# Affichage de la figure
plt.show()
