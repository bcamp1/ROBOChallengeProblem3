"""panda_controller controller."""

# You may need to import some classes of the controller module. Ex:
from controller import Robot, Motor, PositionSensor
pi = 3.14159265

# create the Robot instance.
robot = Robot()
motors = []
sensors = []
timestep = int(robot.getBasicTimeStep())

# Default angles:
# 0, 0, 0, -100, -90, 90, 45

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

initialize_motors()

print(get_robot_position())

while robot.step(timestep) != -1:
    thetas = [0, -0.5, 0, -0.5, 0, 0, 0]
    print(get_robot_position())
    #set_robot_position(thetas)
    pass


