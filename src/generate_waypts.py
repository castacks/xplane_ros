import numpy as np

def RotZ(heading):
    psi = heading 
    R = np.array([[np.cos(heading), -np.sin(heading), 0],
                [np.sin(heading), np.cos(heading), 0],
                [0,                 0,             1]])
    return R

R = RotZ(-1.889)


wayp = np.array([[6000],
            [-300], 
            [-200]])

w = np.array([[6000],
            [-5000], 
            [-200]])
wayp = np.hstack((wayp, w))

w = np.array([[-6000],
            [-5000], 
            [-200]])
wayp = np.hstack((wayp,w))

w = np.array([[-6000],
            [0], 
            [-200]])
wayp = np.hstack((wayp,w))

print(R @ wayp)

