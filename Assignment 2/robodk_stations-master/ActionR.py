from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)
import time

def Rotational_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0,0],
               [np.sin(theta), np.cos(theta), 0,0],
               [0, 0, 1, 0],
               [0, 0, 0, 0]])

def Rotational_matrix_x(theta):
    return np.array([[1, 0, 0,0],
               [0, np.cos(theta), -np.sin(theta),0],
               [0, np.sin(theta), np.cos(theta), 0],
               [0, 0, 0, 0]])

def Rotational_matrix_y(theta):
    return np.array([[np.cos(theta), 0, np.sin(theta), 0],
               [0, 1, 0, 0],
               [-np.sin(theta), 0, np.cos(theta), 0],
               [0, 0, 0, 0]])

def Rotational_matrix_z_inverse_z(theta):
    return np.transpose(np.array([[np.cos(theta), -np.sin(theta), 0],
                   [np.sin(theta), np.cos(theta), 0],
                   [0, 0, 1]]))

def Translation_matrix(x,y,z):
    return np.array([[0, 0, 0, x],
                  [0, 0, 0,y],
                  [0, 0, 0, z],
                  [0, 0, 0, 1]])
def Translation_matrix_inverse(x,y,z):
    return np.array([[x],
                     [y],
                     [z]])

def inverse_transform_z(theta, x, y, z):
    R_T = np.array([[ np.cos(theta),  np.sin(theta), 0],
                    [-np.sin(theta),  np.cos(theta), 0],
                    [0,               0,             1]])  # transpose of R_z(theta)
    T = np.array([x, y, z])

    Trans_inv = np.identity(4)
    Trans_inv[0:3, 0:3] = R_T
    Trans_inv[0:3, 3] = R_T @ (-T)
    return Trans_inv

def inverse_transform_matrix(R, x, y, z):
    R_T = np.transpose(R)          # inverse of a rotation matrix is its transpose
    T = np.array([x, y, z])

    Trans_inv = np.identity(4)
    Trans_inv[0:3, 0:3] = R_T
    Trans_inv[0:3, 3] = R_T @ (-T)
    return Trans_inv

def z_rotation_from_points(local_pts, world_pts, R_pre=np.eye(3)):
    lp = (R_pre @ np.asarray(local_pts, float).T).T
    wp = np.asarray(world_pts, float)
    bd, ad = lp[1] - lp[0], wp[1] - wp[0]
    cross = bd[0]*ad[1] - bd[1]*ad[0]
    dot = bd[0]*ad[0] + bd[1]*ad[1]
    return np.arctan2(cross, dot)

def ActionR():
    """

    Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.

    """
    tls.cup_tool_attach_r_ati()
    tls.cup_tool_open_ur5()

    # Once the cup tool has been attached, send the Robot to the Rancillo Scale Origin at top cover fastener (left)
    
    key12 = [-385.8, -330.2, 41.7] # RS_top_cover_fastener_left (Global)
    key13 = [-418.2, -311.2, 41.5] # RS_top_cover_fastener_right (Global)

    key39 = [0, 0, 0] # RS_top_cover_fastener_left (local)
    key40 = [0, -38, 0] #  RS_top_cover_fastener_right (local)

    # Calculate the rotation using the difference between Key 7 and Key 6.
    theta = z_rotation_from_points(np.array([key39, key40]), np.array([key12, key13]))

    # Only in the x and y direction.
    R1 = Rotational_matrix_z(theta)
    T1 = Translation_matrix(key12[0], key12[1], key12[2])

    UR_T_RS = R1 + T1

    # Once in the Rancillo Scale coordinate system, need to move to the cup centre point

    key41 = [157.52, -19, 24.24] # RS Pan Centre point

    # Adjust for cup height

    cup_height = 45
    key41_cupcentre = [157.52, -19, 24.24 + cup_height]

    theta = np.pi/2
    R2 = Rotational_matrix_y(theta) # Adjusting the tool rotation
    T2 = Translation_matrix(key41_cupcentre[0], key41_cupcentre[1], key41_cupcentre[2])
    # T2 = Translation_matrix(key41[0], key41[1], key41[2])

    RS_T_RSCentrePoint = R2 + T2

    # Rancillo Scale Centre Point in World Coordinates

    UR_T_RSCentrePoint = UR_T_RS @ RS_T_RSCentrePoint

    # Account for the cup tool offset (open configuration)

    # Cup tool angle offset
    
    key50 = [0, 0, 0] # Cup Tool Centre Point (Local)
    theta = (np.pi/180)*-50 
    R3 = Rotational_matrix_z(theta)
    T3 = Translation_matrix(key50[0], key50[1], key50[2])
    TCP_T_MT = R3 + T3
    CT_T_TCP = inverse_transform_z(theta, key50[0], key50[1], key50[2])


    key48 = [-104.5, 0, 186.62]  # Cup tool top face centre (open)
    theta = -np.pi
    R4 = Rotational_matrix_x(theta)
    T4 = Translation_matrix(key48[0], key48[1], key48[2])
    CT_T_CTtopfacecentre = R4 + T4
    CTtopfacecentre_T_CT = inverse_transform_z(theta, key48[0], key48[1], key48[2])

    # Now can calculate the Cup Tool centre point, with respect to the Robots Frame
    UR_T_TCP = UR_T_RSCentrePoint  @ CTtopfacecentre_T_CT @ CT_T_TCP

    # Intermediate 1 is used to ensure approaches cup from good angle
    Intermediate1 = [-12.690000, -128.230000, 148.850000, -30.080000, 26.540000, -219.270000]
    UR5.MoveJ(Intermediate1, blocking = True)

    T_UR_T_TCP = rm.Mat(UR_T_TCP.tolist())
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP)), blocking=True)

    tls.cup_tool_shut_ur5()

    time.sleep(1)

    Intermediate2 = [12.250000, -87.720000, 118.880000, -31.740000, 13.160000, -219.280000]
    UR5.MoveJ(Intermediate2, blocking = True)


    # Intermediate 3 is used to back out of the RS
    Intermediate3 = [-4.620000, -64.330000, 104.020000, -38.540000, -58.850000, -221.200000]
    UR5.MoveJ(Intermediate3, blocking = True)

    # Intermediate 4 is used to back out of the RS
    Intermediate4 = [-4.620000, -64.330000, 104.020000, -38.540000, -141.920000, -221.200000]
    UR5.MoveJ(Intermediate4, blocking = True)


    # Original Code for finding Cup placement (Unused)
    # cup_size = 75 + 51.5
    # key62 = [-450, 200,	cup_size]

    # UR_T_TCP_place = UR_T_TCP.copy()
    # UR_T_TCP_place[0:3, 3] = key62

    # T_UR_T_TCP_place = rm.Mat(UR_T_TCP_place.tolist())
    # UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_place)), blocking=True)

    tls.cup_tool_open_ur5()

    time.sleep(1)

    # Leave cup and move away

    Intermediate4 = [35.770000, -64.330000, 104.020000, -38.540000, -141.920000, -221.200000]
    UR5.MoveJ(Intermediate4, blocking = True)

    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_l_ati()

