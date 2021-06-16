import numpy as np

def RotZ(yaw):
    yaw = np.pi/180 * yaw
    R = [[np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0           , 0          , 1]]
    return np.array(R)

def RotY(pitch):
    pitch = np.pi/180 * pitch
    R = [[np.cos(pitch), 0, -np.sin(pitch)],
        [0,              1, 0],
        [np.sin(pitch),  0, np.cos(pitch)]]

    return np.array(R)

def RotX(roll):
    roll = np.pi/180 * roll
    R = [[1, 0, 0],
        [0, np.cos(roll), np.sin(roll)],
        [0, -np.sin(roll), np.cos(roll)]]
    return np.array(R)