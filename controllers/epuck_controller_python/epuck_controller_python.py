"""epuck_controller_python controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Robot
from controller import DistanceSensor

TIME_STEP = 64

MAX_SPEED = 6.28

# create the Robot instance 
robot = Robot()

# get a handler to the motors and set target position to infinity (speed control)
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

def go_forward(leftMotor, rightMotor):
    print('in go forward')
    leftSpeed = MAX_SPEED
    rightSpeed = MAX_SPEED
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
   
 
def turn_around(leftMotor, rightMotor, duration ):
    print('in turn_around\n')
    # once we are near an object turn in place  
    leftSpeed  = 0.25*MAX_SPEED
    rightSpeed = -0.25*MAX_SPEED
    # write actuators inputs
    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)
    
    # duration is our best guess for how long it'll turn 180 degrees 
    end_time= robot.getTime()+duration
    while (robot.step(TIME_STEP) != -1 and robot.getTime() < end_time):
        continue

def final_state(leftMotor, rightMotor, psValues, ps):
  print('in final_state\n')
  while (robot.step(TIME_STEP) != -1):
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
    
    # if we are in the 5cm, spin
    if(psValues[0]>=100 or psValues[7]>=100):
        print('in final_state_1\n')
        print('ps 0: ' + str(psValues[0]) + ' ps 7 ' + str(psValues[7]) + '\n')
        leftSpeed = 0.25*MAX_SPEED
        rightSpeed = -0.25*MAX_SPEED
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
    if(psValues[5] >= 90 and psValues[6] <= 85):
      print('in final_state_11\n')
      final_state_2(leftMotor, rightMotor, psValues, ps)
      return

def final_state_2(leftMotor, rightMotor, psValues, ps):
    print('in final_state_2\n')
    while (robot.step(TIME_STEP) != -1):
        psValues = []
        for i in range(8):
            psValues.append(ps[i].getValue())
     
        if(psValues[5] >= 90):
            go_forward(leftMotor, rightMotor)
        else:
            return

  
  
ps = []
psNames = [
    'ps0', 'ps1', 'ps2', 'ps3',
    'ps4', 'ps5', 'ps6', 'ps7'
]

for i in range(8):
    ps.append(robot.getDevice(psNames[i]))
    ps[i].enable(TIME_STEP)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(MAX_SPEED)
rightMotor.setVelocity(MAX_SPEED)

      

# main loop
# Perform simulation steps of TIME_STEP milliseconds
# and leave the loop when the simulation is over
#
collisions = 0
while robot.step(TIME_STEP) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    # read sensors outputs
    psValues = []
    for i in range(8):
        psValues.append(ps[i].getValue())
     
     #States 1 and 2
     #our best guess for within 0.05 m of that objects 
     #0 and 7 are the front sensors 
     #as we get closer the values increase 
   
    if ((psValues[0]>=100.0) or (psValues[7]>=100.0) and (collisions < 1)):    
        # once we are near an object turn in place  
        turn_around(leftMotor, rightMotor, 2.825)
        go_forward(leftMotor, rightMotor)
        collisions = collisions + 1
    print('collisions: ' + str(collisions) + '\n');
    # State 3
    if(collisions == 1):
       # final_state(left_motor, right_motor, ps_values[5])
       end_time= robot.getTime()+1.0
       while (robot.step(TIME_STEP) != -1 and robot.getTime() < end_time):
           continue
       final_state(leftMotor, rightMotor, psValues, ps)
       collisions = collisions + 1
    
    #if(collision == 2): 
    #   final_state(leftMotor, rightMotor, psValues, ps)
    #   collisions = collisions + 1
    
    # Final State 4
    if(collisions > 1):
        leftSpeed = 0.0
        rightSpeed = 0.0
        leftMotor.setVelocity(leftSpeed)
        rightMotor.setVelocity(rightSpeed)
   
    pass
