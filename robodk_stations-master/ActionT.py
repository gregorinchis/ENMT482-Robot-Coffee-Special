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

def x_rotation_from_points(local_pts, world_pts, R_pre=np.eye(3)):
    lp = (R_pre @ np.asarray(local_pts, float).T).T
    wp = np.asarray(world_pts, float)
    bd, ad = lp[1] - lp[0], wp[1] - wp[0]
    cross = bd[1]*ad[2] - bd[2]*ad[1]
    dot = bd[1]*ad[1] + bd[2]*ad[2]
    return np.arctan2(cross, dot)

def ActionT():
    """
    Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.
    """

    

     # Once the tool has been attached, send the Robot to the Rancillo Origin at top cover fastener (left)
        
    key56 = [-150.8, -549.1, 179] # PC_Base front lip centre (Global)
    key57 = [-150.8,	-651,	198] # PC_Body on button centre raised (Global)

    key58 = [0, 0, 0] # PC_Base front lip centre (Local)
    key59 = [0,	-101.7,	17.3] #  PC_Body on button centre raised (Local)

    # Calculate the rotation using the difference between Key 7 and Key 6.
    theta = z_rotation_from_points(np.array([key58, key59]), np.array([key56, key57]))

    # Only in the y and z direction.
    R1 = Rotational_matrix_x(theta)
    T1 = Translation_matrix(key56[0], key56[1], key56[2])

    UR_T_RS = R1 + T1

    # Once in the Rancillo Tool Cleaner Coordinate system, need to move to the Rancillo Silicone Brush
        
    
    key60 = [-45,	-51.2,	8.2] # Silicone brush coordinates

    compressionvalue = 25
    exitheight = 15
    key60_compressed = [-45, -51.2, 8.2 - compressionvalue]
    key60_exitheight = [-45, -51.2, 8.2 + exitheight]

    theta = np.pi/2 # Make facing down
    R2 = Rotational_matrix_y(theta) # Adjusting the tool rotation
    T2 = Translation_matrix(key60[0], key60[1], key60[2])

    T2_compressed = Translation_matrix(key60_compressed[0], key60_compressed[1], key60_compressed[2])
    T2_exitheight = Translation_matrix(key60_exitheight[0], key60_exitheight[1], key60_exitheight[2])

    R_T_RTCsiliconebrush= R2 + T2
    R_T_RTCsiliconebrush_compressed = R2 + T2_compressed
    R_T_RTCsiliconebrush_exitheight = R2 + T2_exitheight

    # Rancillo tool cleaner silicone brush in World Coordinates
    UR_T_Rsiliconebrush = UR_T_RS @ R_T_RTCsiliconebrush

    UR_T_Rsiliconebrush_compressed = UR_T_RS @ R_T_RTCsiliconebrush_compressed

    UR_T_Rsiliconebrush_exitheight = UR_T_RS @ R_T_RTCsiliconebrush_exitheight


    # Rancillo Tool Adjustments

    # Rancillo tool angle offset
    
    key53 = [0, 0, 0] # Rancillo Tool Centre Point (Local)

    theta = (np.pi/180)*-50 
    R3 = Rotational_matrix_z(theta)
    T3 = Translation_matrix(key53[0], key53[1], key53[2])

    TCP_T_RT = R3 + T3
    RT_T_TCP = inverse_transform_z(theta, key53[0], key53[1], key53[2])

    # Making it flat and at VST_Basket rim
    key55 = [28.7,	0,	146.3] # VST_Basket rim (centre)

    theta = -np.pi/2
    R4 = Rotational_matrix_x(theta)
    T4 = Translation_matrix(key55[0], key55[1], key55[2])

    RT_T_RTbasketrim_locked = R4 + T4
    RTbasketrim_T_RT = inverse_transform_matrix(
        R4[0:3, 0:3], key55[0], key55[1], key55[2]
    )

    UR_T_TCP = UR_T_Rsiliconebrush @ RTbasketrim_T_RT @ RT_T_TCP

    UR_T_TCP_compressed = UR_T_Rsiliconebrush_compressed @ RTbasketrim_T_RT @ RT_T_TCP

    UR_T_TCP_exitheight = UR_T_Rsiliconebrush_exitheight @ RTbasketrim_T_RT @ RT_T_TCP

    intermediate0 = [-104.040000, -77.510000, -129.190000, -151.150000, -47.310000, -221.430000]
    UR5.MoveJ(intermediate0, blocking=True)

    intermediatefuckknows = [-103.340000, -77.310000, -147.540000,     -131.580000, -69.230000, -216.920000]
    UR5.MoveJ(intermediatefuckknows, blocking=True)

    intermediate1 = [-103.340000, -75.000000, -147.690000, -131.540000, -101.540000, -216.920000]
    UR5.MoveJ(intermediate1, blocking=True)

    intermediate2 = [-103.850000, -93.460000, -117.690000, -144.230000, -101.540000, -216.920000]
    UR5.MoveL(intermediate2, blocking=True)

    intermediate3 = [-103.850000, -93.460000, -117.690000, -144.230000, -101.540000, -32.310000]
    UR5.MoveJ(intermediate3, blocking=True)


    T_UR_T_TCP = rm.Mat(UR_T_TCP.tolist())
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP)), blocking=True)

    T_UR_T_TCP_compressed = rm.Mat(UR_T_TCP_compressed.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_compressed)), blocking=True)

    time.sleep(5)

    T_UR_T_TCP_exitheight = rm.Mat(UR_T_TCP_exitheight.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_exitheight)), blocking=True)







# # ActionT()