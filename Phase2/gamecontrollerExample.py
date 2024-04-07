import pygame
import FakeDrone
import numpy as np
import calibration
quad = FakeDrone.FakeQuad()
               



pygame.init()

#initialise the joystick module
pygame.joystick.init()

#define screen size
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 500

#create game window
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
pygame.display.set_caption("Joysticks")

#define font
font_size = 30
font = pygame.font.SysFont("Futura", font_size)

def draw_box(font):
    #project the box on the image using the calibration.prject function
    dx = 0.40
    dy = 0.40
    dz = 0.1
    vertices = np.array([[-dx/2,-dy/2,-dz/2],
                            [dx/2,-dy/2,-dz/2],
                            [dx/2,dy/2,-dz/2],
                            [-dx/2,dy/2,-dz/2],
                            [-dx/2,-dy/2,dz/2],
                            [dx/2,-dy/2,dz/2],
                            [dx/2,dy/2,dz/2],
                            [-dx/2,dy/2,dz/2]])
    #project the vertices
    projected = []
    for vertex in vertices:
        projected.append(calibration.project(vertex,quad.state.orientation.elements,quad.state.position,np.array([-2,0,-1]),np.array([ 0.5, 0.5, 0.5, 0.5 ]),300))
        # projected.append(calibration.hcmln(calibration.my_ekf.x,0,vertex))
    # print(projected[0], img.shape)
    #draw the lines
    for i, j in [(0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                    (0, 4), (1, 5), (2, 6), (3, 7)]:
            pygame.draw.line(screen,
                            (255,255,255),
                            (round(projected[i][0]+screen.get_width()/2),
                            round(projected[i][1]+screen.get_height()/2)),
                            (round(projected[j][0]+screen.get_width()/2),
                            round(projected[j][1]+screen.get_height()/2)), 1)
  
  
#function for outputting text onto the screen
def draw_text(text, font, text_col, x, y):
  img = font.render(text, True, text_col)
  screen.blit(img, (x, y))

#create clock for setting game frame rate
clock = pygame.time.Clock()
FPS = 60

#create empty list to store joysticks
joysticks = []

x = 0
y = 0
z = 0
thrust = 0

#define player colour
col = "royalblue"
#game loop
run = True
while run:

  clock.tick(FPS)

  #update background
  screen.fill(pygame.Color("midnightblue"))
  #draw player
  quad.run(x,y,z,thrust,1.0/FPS)
  # print(quad.engines[0].speed)
  draw_box(font)
  
#   player.topleft = (x, y)
#   pygame.draw.rect(screen, pygame.Color(col), player)

  #show number of connected joysticks
  draw_text("Controllers: " + str(pygame.joystick.get_count()), font, pygame.Color("azure"), 10, 10)
  for joystick in joysticks:
    draw_text("Battery Level: " + str(joystick.get_power_level()), font, pygame.Color("azure"), 10, 35)
    draw_text("Controller Type: " + str(joystick.get_name()), font, pygame.Color("azure"), 10, 60)
    draw_text("Number of axes: " + str(joystick.get_numaxes()), font, pygame.Color("azure"), 10, 85)

  for joystick in joysticks:
    #change player colour with buttons
    if joystick.get_button(0):
      col = "royalblue"
    if joystick.get_button(1):
      col = "crimson"
    if joystick.get_button(2):
      col = "fuchsia"
    if joystick.get_button(3):
      col = "forestgreen"

    #player movement with joystick
    # if joystick.get_button(14):
    #   x += 5
    # if joystick.get_button(13):
    #   x -= 5
    # if joystick.get_button(11):
    #   y -= 5
    # if joystick.get_button(12):
    #   y += 5

    #player movement with analogue sticks
    x = -joystick.get_axis(2)*0.2
    y = -joystick.get_axis(3)*0.2
    z = -joystick.get_axis(0)*0.2
    thrust = -0.07*joystick.get_axis(1)+0.58
    

  #event handler
  for event in pygame.event.get():
    if event.type == pygame.JOYDEVICEADDED:
      joy = pygame.joystick.Joystick(event.device_index)
      joysticks.append(joy)
    #quit program
    if event.type == pygame.QUIT:
      run = False

  #update display
  pygame.display.flip()

pygame.quit()