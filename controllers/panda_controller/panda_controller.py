"""panda_controller controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, PositionSensor
import roboticstoolbox as rtb
from spatialmath import SE3, SO3

pi = 3.14159265

# create the Robot instance.
robot = Robot()
motors = []
sensors = []
timestep = int(robot.getBasicTimeStep())

panda = rtb.models.ETS.Panda()

# Default angles:
# 0, 0, 0, -100, -90, 90, 45

# Center of board
# 0.65, 0, 0.7
# Rotation of board
# 0, 0, 1

points_C = [[0.3, -0.2, 0.5], [0.25, -0.25, 0.5], [0.2, -0.3, 0.5], 
            [0.2, -0.35, 0.5], [0.25, -0.4, 0.5], [0.3, -0.45, 0.5]]

points_U = [[0.35, -0.45, 0.5], [0.35, -0.35, 0.5], [0.35, -0.25, 0.5], 
            [0.4, -0.2, 0.5], [0.45, -0.25, 0.5], [0.45, -0.45, 0.5]]

# Initialize 
def initialize_motors():
    for i in range(7):
        motor_name = 'panda_joint' + str(i+1)
        sensor_name = motor_name + '_sensor'
        motors.append(robot.getDevice(motor_name))
        sensors.append(robot.getDevice(sensor_name))
        sensors[i].enable(timestep)
    
def set_robot_position(thetas):
    for i in range(7):
        theta = thetas[i]
        motors[i].setPosition(theta)

def get_robot_position():
    thetas = []
    for sensor in sensors:
        thetas.append(sensor.getValue())
    return thetas

def set_robot_XYZ(pos):
    x = pos[0]
    y = pos[1]
    z = pos[2]
    T_target = SE3(x, y, z)  # Define target pose in SE3 format
    print(T_target)
    q_ik = panda.ikine_LM(T_target).q  # Compute inverse kinematics
    set_robot_position(q_ik)

def check_joint_angles(thetas):
    # Joint limits
    joints_min = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]
    joints_max = [ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973]
    for i in range(7):
        theta = thetas[i]
        if theta > joints_max[i] or theta < joints_min[i]:
            return False
    return True

def inverse_kinematics(pos):
    x = pos[0]
    y = pos[1]
    z = pos[2]
    Tep = SE3(x, y, z)

    thetas = [0, 0, 0, 0, 0, 0, 0]
    success = False
    
    while not success:
        thetas = panda.ikine_LM(Tep).q
        success = check_joint_angles(thetas)
    return thetas
    
    

initialize_motors()

q = inverse_kinematics([0.6, 0, 0.8])

while robot.step(timestep) != -1:
    set_robot_position(q)
    
    
    #thetas = [0, -0.5, 0, -0.5, 0, 0, 0]
    #set_robot_XYZ([1, 2, 3]);
    #set_robot_position([0, 0.1, 0, 0, 0, 0, 0])
 


