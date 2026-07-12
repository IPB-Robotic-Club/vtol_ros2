import math
from scipy.spatial.transform import Rotation

# Simulate marker pose: marker is flat on the ground.
# Camera is looking straight down. 
# So camera Z is (0,0,-1) in world, marker Z is (0,0,1) in world.
# Thus, camera and marker have opposing Z axes.
# A rotation from camera to marker involves a 180 deg flip (e.g. around X axis) + some Yaw around Z.
yaw_deg = 30
# Create rotation: 180 deg around X, then 30 deg around Z
r = Rotation.from_euler('xz', [180, yaw_deg], degrees=True)
q = r.as_quat() # x, y, z, w
x, y, z, w = q

# Calculate yaw error using the formula in mission_centering:
yaw_error = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
print(f"Original yaw: {yaw_deg} deg, Calculated yaw: {math.degrees(yaw_error)} deg")
