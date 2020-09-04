# Include all global viarables
import numpy as np

Degree_to_Radian = np.pi/180.0
Radian_to_Degree =  180.0/np.pi

MOTION_RESOLUTION = 0.5 #[m] path interporate resolution

# Vehicle parameter
WB = 2.89  #[m] wheel base: rear to front steer
LT = 8.0 #[m] rear to trailer wheel
W = 1.9 #[m] width of vehicle
LF = 3.836 #[m] distance from rear to vehicle front end of vehicle
LB = 0.946 #[m] distance from rear to vehicle back end of vehicle
LTF = 1.0 #[m] distance from rear to vehicle front end of trailer
LTB = 9.0 #[m] distance from rear to vehicle back end of trailer
MAX_STEER = 0.610865 #[rad] maximum steering angle 
TR = 0.5 # Tire radius [m] for plot
TW = 1.0 # Tire width [m] for plot

# for collision check
# WBUBBLE_DIST = 3.5 #distance from rear and the center of whole bubble
# WBUBBLE_R = 10.0 # whole bubble radius
# B = 4.45 # distance from rear to vehicle back end
# C = 11.54 # distance from rear to vehicle front end
# I = 8.55 # width of vehicle
# VRX = [C, C, -B, -B, C ]
# VRY = [-I/2.0, I/2.0, I/2.0, -I/2.0, -I/2.0]