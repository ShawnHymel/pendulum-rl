"""
python -m pip install pyserial matplotlib
"""
import time
import datetime
import os
import random
import logging
import math

from typing import Optional

# Encoder and stepper controls (local)
from control_comms import ControlComms, StatusCode, DebugLevel

# Third-party packages
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import wandb

# Communication settings
SERIAL_PORT = "/dev/ttyACM0"    # Check your devices
BAUD_RATE = 500000      # Must match what's in the Arduino code!
CTRL_TIMEOUT = 1.0      # Seconds
DEBUG_LEVEL = DebugLevel.DEBUG_ERROR

# Reinforcement learning settings
K_T = 5                 # Reward constant to multiply theta (angle of encoder)
K_DT = 0.05             # Reward constant to multiply dtheto/dt (angular velocity of encoder)
K_P = 2                 # Reward constant to multiply phi (angle of stepper)
K_DP = 0.05             # Reward constant to multiply dphi/dt (angular velocity of stepper)
REWARD_OOB = -500       # Reward (penalty) for having the stepper motor move out of bounds (OOB)
ENC_ANGLE_NORM = 180    # Divide by this to normalize +/-180 deg angle to +/-1
STP_MOVE_MIN = -10.0
STP_MOVE_MAX = 10.0
STP_ANGLE_MIN = -180    # Episode ends if stepper goes beyond this angle
STP_ANGLE_MAX = 180     # Episode ends if stepper goes beyond this angle
STP_ANGLE_NORM = 180    # Divide by this to normalize +/-180 deg angle to +/-1

ENV_TIMEOUT = 30.0
RESET_SETTLE_TIME = 2.0 # Seconds to wait after reset to start moving again

# Angle constants
ENC_OFFSET = 180.0      # Pendulum in the "up" position should be 0 deg
ANG_REV = 360           # Degrees in a single revolution

# Communication constants
CMD_SET_HOME = 0        # Set current stepper position as home (0 deg)
CMD_MOVE_TO = 1         # Move stepper to a particular position (deg)
CMD_MOVE_BY = 2         # Move stepper by a given amount (deg)
CMD_SET_STEP_MODE = 3   # Set step mode
CMD_SET_BLOCK_MODE = 4  # Set blocking mode
CMD_NOP = 5             # Take no action, just receive observation
STEP_MODE_1 = 0         # 1 division per step
STEP_MODE_2 = 1         # 2 divisions per step
STEP_MODE_4 = 2         # 4 divisions per step
STEP_MODE_8 = 3         # 8 divisions per step
STEP_MODE_16 = 4        # 16 divisions per step
STATUS_OK = 0           # Stepper idle
STATUS_STP_MOVING = 1   # Stepper is currently moving

# Set to desired step mode
STEP_MODE = STEP_MODE_8

def calc_angular_velocity(ang, ang_prev, dt):
    """
    Estimate engular velocity based on current and previous readings. Note that we assume that the
    object in question cannot go the long way around (e.g. more than 180 deg).
    """
    da = ang - ang_prev
    if da > (ANG_REV / 2):
        da -= ANG_REV
    elif da < -(ANG_REV / 2):
        da += ANG_REV
    
    return da / dt

class Pendulum(gym.Env):
    """
    Subclass gymnasium Env class
    
    This is the gym wrapper class that allows our agent to interact with our environment. We need
    to implement four main methods: step(), reset(), render(), and close(). We should also define
    the action_space and observation space as class members.
    
    Note: on Windows, time.sleep() is only accurate to around 10ms. As a result, setting fps_limit
    will give you a "best effort" limit.
    
    More information: https://gymnasium.farama.org/api/env/
    """
    
    def __init__(
        self,
        serial_port,
        baud_rate,
        ctrl_timeout=1.0,
        debug_level=DebugLevel.DEBUG_NONE,
        env_timeout=0.0, 
        stp_mode=STEP_MODE_8, 
        stp_blocking=False
    ):
        """
        Set up the environment, action, and observation shapes. Optional tiemout in seconds.
        """
        
        print("Creating environment")
            
        # Call superclass's constructor
        super().__init__()
        
        # Connect to Arduino board
        self.ctrl = ControlComms(timeout=ctrl_timeout, debug_level=debug_level)
        try:
            self.ctrl.close()
        except:
            pass
        ret = self.ctrl.connect(serial_port, baud_rate)
        if ret is not StatusCode.OK:
            print("ERROR: Could not connect to board")
        
        # Define action space (scalar signifying how many degrees to move stepper by)
        self.action_space = gym.spaces.Box(
            low=STP_MOVE_MIN,
            high=STP_MOVE_MAX,
            # shape=(1, 1),
            dtype=np.float32
        )
        print(self.action_space.shape)
        
        # Define observation space 
        # [encoder angle, encoder angular velocity, stepper angle, stepper angular velocity]
        self.observation_space = gym.spaces.Box(
            low=np.array([-180, -np.inf, STP_ANGLE_MIN, -np.inf]),
            high=np.array([180, np.inf, STP_ANGLE_MAX, np.inf]),
            dtype=np.float32
        )
        print(f"Obs space: {self.observation_space.shape}")
        
        # Record time from microcontroller and own elapsed time
        self.timestamp = 0
        self.timeout = env_timeout
        self.start_time = time.time()
        self.sum_dtime = 0
        self.num_steps = 0
        
        # Record previous encoder and stepper angles (to calculate velocities)
        self.angle_stp_prev = 0
        self.angle_enc_prev = 0
        
        # Set current stepper position as "home" and optionally set blocking
        self.ctrl.step(CMD_SET_STEP_MODE, [stp_mode])
        self.ctrl.step(CMD_SET_HOME, [0])
        if stp_blocking:
            self.ctrl.step(CMD_SET_BLOCK_MODE, [1])
        else:
            self.ctrl.step(CMD_SET_BLOCK_MODE, [0])
    
    def __del__(self):
        """
        Destructor: make sure to close the serial port
        """
        self.close()
    
    def step(self, action: np.ndarray):
        """
        What happens when you tell the stepper motor to do something then record the observation.
        """
        
        # Initialize return values
        obs = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        reward = 0.0
        info = {"error": False, "dtime": 0.0, "elapsed_time": 0.0}
        terminated = False
        truncated = False
        
        # Box is 2D NumPy array, action must be sent out as 1D list [...]
        action_list = action.flatten().tolist()
        
        # Move the stepper motor and wait for a response
        resp = self.ctrl.step(CMD_MOVE_BY, action_list)
        if resp:
            
            # Extract information from controller response
            status, timestamp, terminated, angles = resp
            
            # Compute lapsed time (in seconds) from previous observation (milliseconds)
            info["dtime"] = (timestamp - self.timestamp) / 1000.0
            self.timestamp = timestamp

            # HACK: Record dtimes to compute average
            self.sum_dtime += info["dtime"]
            self.num_steps += 1
            
            # Offset encoder angle so that 0 deg is up
            angles[0] -= ENC_OFFSET
            
            # Calculate velocities
            dtheta = calc_angular_velocity(angles[0], self.angle_enc_prev, info['dtime'])
            dphi = calc_angular_velocity(angles[1], self.angle_stp_prev, info['dtime'])
            self.angle_enc_prev = angles[0]
            self.angle_stp_prev = angles[1]
            
            # Construct observation (normalized)
            obs[0] = angles[0] / ENC_ANGLE_NORM
            obs[1] = dtheta / ENC_ANGLE_NORM
            obs[2] = angles[1] / STP_ANGLE_NORM
            obs[3] = dphi / STP_ANGLE_NORM
                    
            # Calculate reward if stepper is not out of bounds
            if (angles[1] >= STP_ANGLE_MIN) and (angles[1] <= STP_ANGLE_MAX):
                reward = -1 * (K_T * obs[0] ** 2 + 
                               K_DT * obs[1] ** 2 + 
                               K_P * obs[2] ** 2 +
                               K_DP * obs[3] ** 2)
            
            # Stepper motor is out of bounds--terminate episode
            else:
                reward = REWARD_OOB
                terminated = True
        
        # Something is wrong with communication
        else:
            print("ERROR: Could not communicate with Arduino")
            info["error"] = True
            terminated = True
        
        # Calculate elapsed time
        info["elapsed_time"] = time.time() - self.start_time
        
        # Check if we've exceeded the time limit
        if not terminated and self.timeout > 0.0 and info["elapsed_time"] >= self.timeout:
            truncated = True
        
        return obs, reward, terminated, truncated, info
    
    def reset(self, seed=None, return_info=True, options=None):
        """
        Return the pendulum to the starting position
        """
        
        # Initialize return values
        obs = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        info = {"error": False, "dtime": 0, "elapsed_time": 0.0}
        
        # Reset timer
        self.start_time = time.time()
        
        # Let the pendulum fall and return to the starting position
        time.sleep(RESET_SETTLE_TIME)
        resp = self.ctrl.step(CMD_MOVE_TO, [0.0])
        print(f"RESP: {resp}")
        if resp:
            
            # Extract information from controller response
            status, timestamp, terminated, angles = resp

            # HACK: Print average step time from previous episode
            if self.num_steps != 0:
                print(f"AVG STEP TIME: {self.sum_dtime / self.num_steps}")
            self.sum_dtime = 0
            self.num_steps = 0
            
            # Compute lapsed time (in seconds) from previous observation (milliseconds)
            info["dtime"] = (timestamp - self.timestamp) / 1000.0
            self.timestamp = timestamp
            
            # Offset encoder angle so that 0 deg is up
            angles[0] -= ENC_OFFSET
            
            # Calculate velocities
            dtheta = calc_angular_velocity(angles[0], self.angle_enc_prev, info['dtime'])
            dphi = calc_angular_velocity(angles[1], self.angle_stp_prev, info['dtime'])
            self.angle_enc_prev = angles[0]
            self.angle_stp_prev = angles[1]
            
            # Construct observation (normalized)
            obs[0] = angles[0] / ENC_ANGLE_NORM
            obs[1] = dtheta / ENC_ANGLE_NORM
            obs[2] = angles[1] / STP_ANGLE_NORM
            obs[3] = dphi / STP_ANGLE_NORM
            
            # Let pendulum settle for a bit
            time.sleep(RESET_SETTLE_TIME)
            
        # Something is wrong with communication
        else:
            print("ERROR: Could not communicate with Arduino")
            info["error"] = True
            
        # Calculate elapsed time
        info["elapsed_time"] = time.time() - self.start_time
        
        return obs, info
    
    def close(self):
        """
        Close connection to Arduino
        """
        self.ctrl.close()

# Create the environment for sample-factory
def make_env(
    full_env_name, 
    cfg=None,
    env_config=None, 
    render_mode: Optional[str] = None
):
    env = Pendulum(
        SERIAL_PORT,
        BAUD_RATE,
        ctrl_timeout=CTRL_TIMEOUT,
        debug_level=DEBUG_LEVEL,
        env_timeout=ENV_TIMEOUT, 
        stp_mode=STEP_MODE, 
        stp_blocking=True
    )
    return env