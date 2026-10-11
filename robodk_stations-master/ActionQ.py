from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

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


def ActionQ():
    """
    This function uses the Mazzer tool to lock the Rancilio Scale

    Global variables:
    key6: 441.4, -273.5, 41.7 -> Rancilio Scale Origin at top cover fastener (left)
    key7: 408.7, -292.4, 41.7 -> Rancilio Scale Origin at top cover fastener (right)

    Local Variables:
    Key 39: 0, 0, 0 -> Rancilio Scale Origin at top cover fastener (left)
    Key 40: 0, -38, 0 -> Rancilio Scale Origin at top cover fastener (right)
    """

    # # Get the Mazzer tool (only necessary on single actions)
    # tls.mazzer_tool_attach_r_ati() 


    # Once the tool has been attached, send the Robot to the Rancillo Scale Origin at top cover fastener (left)

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

    # Once at the Origin, need to move to the Rancillo scale lock lever

    key42 = [31.5, 21.53, -15]     # Rancillo Scale Lock Lever (local)

    R2 = Rotational_matrix_y(np.pi) # Already in the RS coordinate frame
    T2 = Translation_matrix(key42[0], key42[1], key42[2])
    RS_T_RSLockLever = R2 + T2

    # Lock Levers Pose in World Coordinates
    UR_T_RSLockLever = UR_T_RS @ RS_T_RSLockLever

    # Mazzer tool angle offset

    key50 = [0, 0, 0] # Mazzer Tool Centre Point (Local)
    theta = (np.pi/180)*-50 
    R3 = Rotational_matrix_z(theta)
    T3 = Translation_matrix(key50[0], key50[1], key50[2])
    TCP_T_MT = R3 + T3
    MT_T_TCP = inverse_transform_z(theta, key50[0], key50[1], key50[2])

    # Account for the mazzer tool offset

    key51 = [0, 0, 102.82]  # Mazzer tool offset

    theta = np.pi/2  # Mazzer tool is RS frame rotated 90 degrees
    theta2 = np.deg2rad(-45)

    R4 = Rotational_matrix_z(theta) @ Rotational_matrix_x(theta2)
    T4 = Translation_matrix(key51[0], key51[1], key51[2])
    MT_T_MTtip = R4 + T4
    MTtip_T_MT = inverse_transform_matrix(R4[0:3, 0:3], key51[0], key51[1], key51[2])

    # Now can calculate the Mazzer Tool tip centre point, with respect to the Robots Frame
    UR_T_TCP = UR_T_RSLockLever @ MTtip_T_MT @ MT_T_TCP

    # Intermediate 1 is used to avoid hitting the Rancillo Scale
    Intermediate1 = [-84.230000, -95.960000, -127.750000, -77.370000, -1.640000, 98.790000]
    UR5.MoveJ(Intermediate1, blocking = True)

    # Intermediate 2 gets close to the lever
    Intermediate2 = [-101.760000, -101.760000, -127.410000, -59.480000, 47.250000, 70.860000]
    UR5.MoveJ(Intermediate2, blocking = True)

    # Convert from numpy and move
    T_UR_T_TCP = rm.Mat(UR_T_TCP.tolist())
    UR5.MoveJ(T_UR_T_TCP, blocking=True)

    # Slide the lock lever in x to activate it

    LockLeverDisplacement_x = 25
    key42_slide = [key42[0] + LockLeverDisplacement_x, key42[1], key42[2]]
    T2_slide = Translation_matrix(key42_slide[0], key42_slide[1], key42_slide[2])
    RS_T_RSLockLever_slide = R2 + T2_slide
    UR_T_RSLockLever_slide = UR_T_RS @ RS_T_RSLockLever_slide
    UR_T_TCP_slide = UR_T_RSLockLever_slide @ MTtip_T_MT @ MT_T_TCP

    T_UR_T_TCP_slide = rm.Mat(UR_T_TCP_slide.tolist())
    UR5.MoveJ(T_UR_T_TCP_slide, blocking=True)

    # Push down on the Lock Lever (Commented out as may be uneccesary)

    LockLeverDisplacement_z = 20
    key42_down = [key42_slide[0], key42_slide[1], key42_slide[2] - LockLeverDisplacement_z]
    T2_down = Translation_matrix(key42_down[0], key42_down[1], key42_down[2])
    RS_T_RSLockLever_down = R2 + T2_down
    UR_T_RSLockLever_down = UR_T_RS @ RS_T_RSLockLever_down
    UR_T_TCP_down = UR_T_RSLockLever_down @ MTtip_T_MT @ MT_T_TCP

    T_UR_T_TCP_down = rm.Mat(UR_T_TCP_down.tolist())
    UR5.MoveJ(T_UR_T_TCP_down, blocking=True)

    visual_program = RDK.Item("Show_Rancilio_Scale_Lock", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()

    # # Moves head away from lock
    # Intermediate2 = [-124.070000, -109.180000, -108.630000, -52.250000, 123.050000, 46.640000]
    # UR5.MoveJ(Intermediate2, blocking = True)

    Intermediate3 = [-80.250000, -75.480000, -140.420000, -52.230000, 45.320000, 46.630000]
    UR5.MoveJ(Intermediate3, blocking = True)


    tls.mazzer_tool_detach_r_ati()

