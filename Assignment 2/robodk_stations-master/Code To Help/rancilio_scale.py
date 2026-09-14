import numpy as np
# arctan
Adx = -0.868
Ady = 0.5
Bdx = 0
Bdy = -1
Eq1 = Bdx*Ady - Bdy*Adx
Eq2 = Bdx*Adx + Bdy*Ady
theta = np.arctan2(Eq1, Eq2)
print(theta*180/np.pi)
print("Rotation Matrix")
R = np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta), np.cos(theta), 0],
               [0, 0, 1]])
print(R)