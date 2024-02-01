import cv2
import numpy as np
import random

'''
this code is for generating a dataset of videos and their end points
when we record infrared led videos, we want to know where the end points are
they can move or desepear and making them blurry, so we want to know where they are at the end of each frame

So we can simulate a 3d object moving in front of the camera, compute the 3d position of the leds, and then project them on the 2d image
'''

FRAME_SIZE = (640,480)
VIDEO_LENGTH = 10 #seconds
FPS = 30

MAX_LED_DIAMETER = 0.01 #max diameter of the leds
MIN_LED_DIAMETER = 0.003 #min diameter of the leds
MAX_LED_NUMBER = 6 #max number of leds
MAX_BACKGROUND_ARTEFACT = 10 #max level of background artefact (due to the environment, windows, etc.)
MAX_OCCLUSION_OFTENNESS = 0.5 #how often the leds are occluded
MIN_MIN_BRIGHTNESS = 50 #some leds are less bright than others
MAX_MIN_BRIGHTNESS = 150
MAX_NOISE = 10 #max level of noise
MIN_CAM_AOV = 30 #min camera angle of view
MAX_CAM_AOV = 90 #max camera angle of view


def createRandomLedsPose(n_led, min_brightness, d_min = 0.101,d_max = 0.2):
    '''
    create a random position and orientation for the leds
    '''
    ledsPos = []
    ledsOrientation = []
    ledsBrightness = []
    for i in range(n_led):
        #create a random position
        d = random.uniform(d_min,d_max)
        theta = random.uniform(0,2*np.pi)
        phi = random.uniform(0,np.pi)
        #convert it to cartesian coordinates
        x = d*np.sin(phi)*np.cos(theta)
        y = d*np.sin(phi)*np.sin(theta)
        z = d*np.cos(phi)
        ledsPos.append([x,y,z])
        yaw = random.uniform(0,2*np.pi)
        pitch = random.uniform(0,np.pi)
        roll = random.uniform(0,2*np.pi)
        ledsOrientation.append([yaw,pitch,roll])
        ledsBrightness.append(random.randint(min_brightness,255))
        
    return np.array(ledsPos),np.array(ledsOrientation),np.array(ledsBrightness)

def createRandomTrajectory(video_length,fps,camAoV,camResolution,order = 2):
    '''
    create a random trajectory of the camera
    return a list of a trajectory expressed by a taylor expansion
    '''
    return None

def createRandomOcclusion(occlusion_often, camResolution):
    '''
    create a random occlusion mask -> some shapes that move randomly and occlude occlusion_often*100 % of the image
    '''
    return np.ones((VIDEO_LENGTH*FPS,camResolution[0],camResolution[1]),dtype=np.uint8)

def createRandomBackroundArtefact(max_bright_backround_artefact, camResolution, occlusion_often=0.1, n_artefact=10):
    '''
    create a random backround artefact
    '''
    return np.zeros((camResolution[0],camResolution[1]),dtype=np.uint8)
    
def project(leds,trajectory,camAoV, camResolution, led_AoL = 200):
    '''
    project the leds on the image
    return ledsLocations only if the led is pointint to the camera (more or less led_AoL)
    '''
    return np.zeros((VIDEO_LENGTH*FPS,camResolution[0],camResolution[1]),dtype=np.uint8),np.zeros((VIDEO_LENGTH*FPS,leds[0].shape[0],2),dtype=np.uint16)


def generateVideo(n_led,max_bright_backround_artefact,occlusion_often,noise,min_brightness,camAoV = 60,camResolution = (640,480)):
    # print("generating video with {} leds, max_bright_backround_artefact {}, occlusion_often {}, noise {}, min_brightness {}".format(n_led,max_bright_backround_artefact,occlusion_often,noise,min_brightness))
    internalLedsPos,internalLedsOrientation,LedBright = createRandomLedsPose(n_led, min_brightness)
    leds = (internalLedsPos,internalLedsOrientation,LedBright)
    trajectory = createRandomTrajectory(VIDEO_LENGTH,FPS,camAoV,camResolution)
    video, ledsLocations = project(leds,trajectory,camAoV, camResolution)
    occlusion_mask = createRandomOcclusion(occlusion_often, camResolution)
    backround_artefact = createRandomBackroundArtefact(max_bright_backround_artefact,camResolution)
    
    #apply the occlusion mask
    video = video*occlusion_mask
    
    #add the backround artefact
    video = video + backround_artefact
    #add the noise
    video = video + np.random.normal(0,noise,video.shape)
    # print(video.shape)
    #remove ledsLocations that are occluded by the occlusion mask (look the ledsLocations in the occlusion mask and remove them if they are 0)
    # ledsLocations = ledsLocations[occlusion_mask[ledsLocations[:,:,0],ledsLocations[:,:,1]]==1]
    #transplate ledsLocations to indices to read in the video
    # print(ledsLocations.shape)
    indices = np.indices(ledsLocations.shape)
    # print(indices.shape)
    indices = np.array([indices[0],indices[1],indices[2]])
    # print(indices.shape)
    indices = indices[:,ledsLocations[:,:,0],ledsLocations[:,:,1]]
    print(indices.shape)
    
        
    # print(ledsLocations[:,:,0].shape)
    # ledsLocations_mask = occlusion_mask[:,ledsLocations[:,:,0].flatten(),ledsLocations[:,:,1].flatten()]==1
    ledsLocations_mask = occlusion_mask[indices[0],indices[1],indices[2]]==1
    # print(occlusion_mask.shape)
    print(ledsLocations_mask.shape)
    return video,ledsLocations



def generateDataSet(n=1000):
    inputs = [] #list of videos
    outputs = [] #list of lists of end points
    
    random.seed(0)
    
    for i in range(n):
        n_led = random.randint(1,MAX_LED_NUMBER)
        led_diameter = np.random.uniform(MIN_LED_DIAMETER,MAX_LED_DIAMETER,n_led)
        backround_artefact = random.randint(0,MAX_BACKGROUND_ARTEFACT)
        occlusion_often = random.uniform(0,MAX_OCCLUSION_OFTENNESS)
        noise = random.randint(0,MAX_NOISE)
        camAov = random.uniform(MIN_CAM_AOV,MAX_CAM_AOV)
        min_brightness = random.randint(MIN_MIN_BRIGHTNESS,MAX_MIN_BRIGHTNESS)
        newVideo,newOutputs = generateVideo(n_led,backround_artefact,occlusion_often,noise,min_brightness,camAov)
        inputs.append(newVideo)
        outputs.append(newOutputs)

# a = np.random.normal(0,10,(10,10))
# #flatten the array for only specific indices
# indices = np.indices(a.shape)
# indices = np.array([indices[0],indices[1]])
# print(a)
# print(indices.shape)
# indices = indices[:,a>0]
# print(indices)
 
# print(a[indices[0],indices[1]])
generateDataSet()