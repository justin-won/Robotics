"""grocery controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import math
import numpy as np

#Initialization
print("=== Initializing Grocery Shopper...")
#Consts
MAX_SPEED = 7.0  # [rad/s]
MAX_SPEED_MS = 0.633 # [m/s]
AXLE_LENGTH = 0.4044 # m
MOTOR_LEFT = 10
MOTOR_RIGHT = 11
N_PARTS = 12
LIDAR_ANGLE_BINS = 667
LIDAR_SENSOR_MAX_RANGE = 5.5 # Meters
LIDAR_ANGLE_RANGE = math.radians(240)

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

# The Tiago robot has multiple motors, each identified by their names below
part_names = ("head_2_joint", "head_1_joint", "torso_lift_joint", "arm_1_joint",
              "arm_2_joint",  "arm_3_joint",  "arm_4_joint",      "arm_5_joint",
              "arm_6_joint",  "arm_7_joint",  "wheel_left_joint", "wheel_right_joint")

# All motors except the wheels are controlled by position control. The wheels
# are controlled by a velocity controller. We therefore set their position to infinite.
target_pos = (0.0, 0.0, 0.09, 0.07, 1.02, -3.16, 1.27, 1.32, 0.0, 1.41, 'inf', 'inf')
robot_parts=[]

for i in range(N_PARTS):
    robot_parts.append(robot.getDevice(part_names[i]))
    robot_parts[i].setPosition(float(target_pos[i]))
    robot_parts[i].setVelocity(robot_parts[i].getMaxVelocity() / 2.0)

# Enable Camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Enable GPS and compass localization
gps = robot.getDevice("gps")
gps.enable(timestep)
compass = robot.getDevice("compass")
compass.enable(timestep)

# Enable LiDAR
lidar = robot.getDevice('Hokuyo URG-04LX-UG01')
lidar.enable(timestep)
lidar.enablePointCloud()

# Enable display
display = robot.getDevice("display")

# Odometry
pose_x     = 0
pose_y     = 0
pose_theta = 0

vL = 0
vR = 0

lidar_sensor_readings = [] # List to hold sensor readings
lidar_offsets = np.linspace(-LIDAR_ANGLE_RANGE/2., +LIDAR_ANGLE_RANGE/2., LIDAR_ANGLE_BINS)
lidar_offsets = lidar_offsets[83:len(lidar_offsets)-83] # Only keep lidar readings not blocked by robot chassis

map = None



# ------------------------------------------------------------------
# Helper Functions



# Main Loop
while robot.step(timestep) != -1:
    mode = 'manual' 
#mode = 'autonomous'
    if mode == 'manual':
        key = keyboard.getKey()
        while(keyboard.getKey() != -1): pass
        if key == keyboard.LEFT :
            vL = -MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.RIGHT:
            vL = MAX_SPEED
            vR = -MAX_SPEED
        elif key == keyboard.UP:
            vL = MAX_SPEED
            vR = MAX_SPEED
        elif key == keyboard.DOWN:
            vL = -MAX_SPEED
            vR = -MAX_SPEED
        elif key == ord(' '):
            vL = 0
            vR = 0
        elif key == ord('S'):
            #reference: numpy.save(file, arr, allow_pickle=True, fix_imports=True)
            # Part 1.4: Filter map and save to filesystem
            map_bool = map > 0.2 
            map_bool = np.multiply(map_bool,1)
            def rotate90Clockwise(A):
                N = len(A[0])
                for i in range(N // 2):
                    for j in range(i, N - i - 1):
                        temp = A[i][j]
                        A[i][j] = A[N - 1 - j][i]
                        A[N - 1 - j][i] = A[N - 1 - i][N - 1 - j]
                        A[N - 1 - i][N - 1 - j] = A[j][N - 1 - i]
                        A[j][N - 1 - i] = temp
            map_bool = rotate90Clockwise(map_bool)
            np.save('map.npy', map_bool)
            print("Map file saved")
        elif key == ord('L'):
            # You will not use this portion in Part 1 but here's an example for loading saved a numpy array
            map = np.load('map.npy')
            print("Map loaded")
        else: # slow down
            vL *= 0.75
            vR *= 0.75
    
    
    
    robot.getDevice("wheel_left_joint").setVelocity(vL)
    robot.getDevice("wheel_right_joint").setVelocity(vR)
