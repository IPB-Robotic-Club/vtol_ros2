import math
import numpy as np
import cv2
from scipy.spatial.transform import Rotation

def rotvec2quat(rvec):
    R, _ = cv2.Rodrigues(rvec)
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        S = math.sqrt(trace + 1.0) * 2
        w = 0.25 * S
        x = (R[2,1] - R[1,2]) / S
        y = (R[0,2] - R[2,0]) / S
        z = (R[1,0] - R[0,1]) / S
    elif (R[0,0] > R[1,1]) and (R[0,0] > R[2,2]):
        S = math.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2]) * 2
        w = (R[2,1] - R[1,2]) / S
        x = 0.25 * S
        y = (R[0,1] + R[1,0]) / S
        z = (R[0,2] + R[2,0]) / S
    elif R[1,1] > R[2,2]:
        S = math.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2]) * 2
        w = (R[0,2] - R[2,0]) / S
        x = (R[0,1] + R[1,0]) / S
        y = 0.25 * S
        z = (R[1,2] + R[2,1]) / S
    else:
        S = math.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1]) * 2
        w = (R[1,0] - R[0,1]) / S
        x = (R[0,2] + R[2,0]) / S
        y = (R[1,2] + R[2,1]) / S
        z = 0.25 * S
    mag = math.sqrt(x*x + y*y + z*z + w*w)
    return (x/mag, y/mag, z/mag, w/mag)

rvec = np.array([1.2, -0.5, 2.0], dtype=np.float32)
print("Mine:", rotvec2quat(rvec))
print("Scipy:", Rotation.from_rotvec(rvec).as_quat())
