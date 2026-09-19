import numpy as np
#Helping myself get angles of the mazzer
Rot = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])

from scipy.spatial.transform import Rotation as R
r = R.from_matrix(Rot)

angles_rad = r.as_euler('xyz')
np.set_printoptions(formatter={'float_kind':'{:f}'.format})
print(angles_rad * 180/np.pi)
