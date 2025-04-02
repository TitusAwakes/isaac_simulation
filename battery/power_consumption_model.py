# Based on:
# doi:10.3390/en12010027

import math

# The proposed equations should be equal to solving the definite integrals, considering the time step size as the interval of integration.

def sensor_power_consumption_model(sensor_power=0.5, timestep_size=1, v_max=1):
    """Power consumption model for the sensor.

    Args:
        sensor_power (float): Power consumption of the sensor in watts.
        timestep_size (float): Size of the time step in seconds.    

    Returns:
        _type_: _description_
    """

    t = timestep_size # 1s, timestep size
    v_max = v_max # 1m/s
    P_sensor = sensor_power

    E_sensor = ((1/v_max) * (P_sensor * t)) # Paper describes Psensor to be integrated over time, but since Psensor is constant, we can just multiply it with the time step size 

    return E_sensor

# Control power consumption model

def control_power_consumption_model(control_power=0.5, timestep_size=1, current_state="STANDBY", P_standby=0.1, starting_factor=0.7, current_velocity=0, new_velocity=0):
    """Power consumption model for the control system.
    Args:
        control_power (float): Power consumption of the control system in watts.
        timestep_size (float): Size of the time step in seconds.
        current_state (str): Current state of the control system. Can be "STANDBY", "STARTUP", or "STABLE".
        P_standby (float): Power consumption in standby mode.
        starting_factor (float): Starting factor for the power consumption model.
        current_velocity (float): Current velocity of the control system.
        new_velocity (float): New velocity of the control system."
        """

    # There's a possible issue here but I believe it is correct:
    # The paper describes the power consumption model to be integrated over dt; the solution here is the anti-derivative of the power consumption model, which would then 
    # be "defined" over the interval. The It earns confusion because t is used in two different ways:
    # 1. The time step size (timestep_size) which is constant and used to calculate the energy consumption over the time step size
    # 2. The variable of the anti-derivative.

    t = timestep_size # 1s, timestep size

    if current_state == "STANDBY":
        E_standby = (P_standby * t) # Energy consumption in standby mode, paper describes P_standby * delta_t, but since delta_t is constant, we can just multiply it with the time step size
        return E_standby
    elif current_state == "STARTUP": 
        v_delta = new_velocity - current_velocity
        E_startup = (t * ((starting_factor * v_delta) + ((t)/30) + (P_standby))) # Solution to the integral of the power consumption model for the control system. Actually I'm not too sure on this one, 
        # but I believe it is correct. We would get three anti-derivatives, so for each one we needed to define the interval. TODO: Re-solve this one.
        return E_startup
    elif current_state == "STABLE":
        E_stable = P_standby * ((t*t*t)/3)
        return E_stable
    else: 
        raise ValueError("Invalid state. Must be 'STANDBY', 'STARTUP', or 'STABLE'.")
    
def motion_system_power_consumption_model(timestep_size=1, robot_mass=1, robot_velocity=1, friction_coefficient=0.5, time_heat_constant_epsilon=0.01,
                                          time_heat_constant_lambda=0.1, speed_heat_constant=0.001,
                                          drag_coefficient=-0.5, vibration_velocity_coefficient=5):
    
    """Power consumption model for the motion system."

    Args:
        timestep_size (float): Size of the time step in seconds.
        robot_mass (float): Mass of the robot in kg.
        robot_velocity (float): Velocity of the robot in m/s.
        friction_coefficient (float): Friction coefficient.
        time_heat_constant_epsilon (float): Time heat constant epsilon.
        time_heat_constant_lambda (float): Time heat constant lambda.
        speed_heat_constant (float): Speed heat constant.
        drag_coefficient (float): Drag coefficient.
        vibration_velocity_coefficient (float): Vibration velocity coefficient.
    """

    t = timestep_size
    E_k = robot_mass * ((robot_velocity**2) / 2) # Kinetic energy of the robot
    E_f = friction_coefficient * robot_mass * robot_velocity
    E_e = ((time_heat_constant_epsilon * (t*t*t/3)) * t) + (time_heat_constant_lambda * robot_velocity * t * t) + (time_heat_constant_lambda * (t*t)/2)  # Energy consumption due to heat generation
    E_m = robot_mass * ((math.e**drag_coefficient*t)/((drag_coefficient**2) + (vibration_velocity_coefficient**2))) * ((drag_coefficient*math.cos((vibration_velocity_coefficient*t) + (robot_velocity) + (robot_mass/2)))
    + (vibration_velocity_coefficient*math.sin((vibration_velocity_coefficient*t) + robot_velocity + (robot_mass/2)))) + (robot_mass * t) # Energy consumption due to motion

    E_motion = E_k + E_f + E_e + E_m # Total energy consumption of the motion system
    
    return E_motion