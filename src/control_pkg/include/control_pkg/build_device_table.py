import yaml
from device import *
import warnings
import copy

environment_data = {}
RUN_PARAM_ONCE = False

def setup_environment_params(yaml_file_path):
    global environment_data
    global RUN_PARAM_ONCE
    
    if RUN_PARAM_ONCE:
        warnings.warn("WARNING: Environment parameter setup ran more than once. Environment might break or not work properly.")
    else:
        environment_data = load_environment_data(yaml_file_path)
    
    return copy.deepcopy(environment_data)

def load_environment_data(yaml_file_path):
    global RUN_PARAM_ONCE
    
    with open(yaml_file_path, 'r') as file:
        config = yaml.safe_load(file)
        
    RUN_PARAM_ONCE = True
    return config['environment']

def build_robot_table(robot_pose_array):
    global RUN_PARAM_ONCE
    global environment_data
    
    if not RUN_PARAM_ONCE:
        raise RuntimeError("You need to run setup_environment_params with the yaml file path before building the device table.")
        
    robot_table = list()
    
    # Set up the robot table   
    for i in range(len(robot_pose_array) - environment_data['number_of_sensors']):
        new_point = Point(robot_pose_array[i].position.x, robot_pose_array[i].position.y, robot_pose_array[i].position.z)
        new_quaternion = Quaternion(robot_pose_array[i].orientation.x, robot_pose_array[i].orientation.y, robot_pose_array[i].orientation.z, robot_pose_array[i].orientation.w)
        new_robot = Device(f"ROBOT_{i+1}", new_point, new_quaternion, 100)
        robot_table.append(new_robot)

def build_sensor_table(sensor_pose_array):
    global RUN_PARAM_ONCE
    global environment_data
    
    if not RUN_PARAM_ONCE:
        raise RuntimeError("You need to run setup_environment_params with the yaml file path before building the device table.")
    
    device_table = list()
    
    # Set up the sensor table
    for i in range(len(sensor_pose_array) + environment_data['number_of_robots']):
        new_point = Point(sensor_pose_array[i].position.x, sensor_pose_array[i].position.y, sensor_pose_array[i].position.z)
        new_quaternion = Quaternion(sensor_pose_array[i].orientation.x, sensor_pose_array[i].orientation.y, sensor_pose_array[i].orientation.z, sensor_pose_array[i].orientation.w)
        new_sensor = Device(f"SENSOR_{i+1}", new_point, new_quaternion, 100)
        device_table.append(new_sensor)
        
        
        
