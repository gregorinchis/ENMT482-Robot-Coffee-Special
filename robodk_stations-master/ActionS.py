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

def ActionS():
    """

    Remove the Rancilio tool from the group head.

    """

    # Once the tool has been attached, send the Robot to the Rancillo Origin at top cover fastener (left)
    
    key10 = [-411.3, -480.2, 348] # Rancillo Top Panel fastener, left front (Global)
    key11 = [-600.5, -370.8, 348] # Rancillo Top Panel fastener, right front (Global)

    key35 = [0, 0, 0] # Rancillo Top Panel fastener, left front (Local)
    key36 = [0, 220.42, 0] #  Rancillo Top Panel fastener, right front (Local)

    # Calculate the rotation using the difference between Key 7 and Key 6.
    theta = z_rotation_from_points(np.array([key35, key36]), np.array([key10, key11]))

    # Only in the x and y direction.
    R1 = Rotational_matrix_z(theta)
    T1 = Translation_matrix(key10[0], key10[1], key10[2])

    UR_T_RS = R1 + T1

    # Once in the Rancillo Coordinate system, need to move to the Rancillo Group Head
    

    key38 = [-14.2,	68.7, -145.4] # GroupGasket (centre)

    theta = 3*np.pi/2 + ((3.4 * np.pi)/180) # Match the group-head frame used in ActionI
    R2 = Rotational_matrix_y(theta) # Adjusting the tool rotation
    T2 = Translation_matrix(key38[0], key38[1], key38[2])

    R_T_Rgrouphead= R2 + T2

    # Group Head in World Coordinates
    UR_T_RGroupHead = UR_T_RS @ R_T_Rgrouphead

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

    # For the locked position
    theta = 0
    R4_locked = Rotational_matrix_x(theta)
    T4_locked = Translation_matrix(key55[0], key55[1], key55[2])

    RT_T_RTbasketrim_locked = R4_locked + T4_locked
    RTbasketrim_T_RT_locked = inverse_transform_matrix(
        R4_locked[0:3, 0:3], key55[0], key55[1], key55[2]
    )

    # For the UnLocked Position
    theta = np.pi/4
    R4_unlocked = Rotational_matrix_x(theta)
    T4_unlocked = Translation_matrix(key55[0], key55[1], key55[2])

    RT_T_RTbasketrim_unlocked = R4_unlocked + T4_unlocked
    RTbasketrim_T_RT_unlocked = inverse_transform_matrix(
        R4_unlocked[0:3, 0:3], key55[0], key55[1], key55[2]
    )

    # For the in between position (necessary for the moveC)
    theta = np.pi/8
    R4_mid = Rotational_matrix_x(theta)
    RTbasketrim_T_RT_mid = inverse_transform_matrix(
        R4_mid[0:3, 0:3], key55[0], key55[1], key55[2]
    )


    # Now can calculate the Cup Tool centre point, with respect to the Robots Frame

    UR_T_TCP_unlocked = UR_T_RGroupHead @ RTbasketrim_T_RT_unlocked @ RT_T_TCP # Unlocked position at the Group Head

    UR_T_TCP_locked = UR_T_RGroupHead @ RTbasketrim_T_RT_locked @ RT_T_TCP # Locked position at the group head

    UR_T_TCP = UR_T_RGroupHead @ RTbasketrim_T_RT_mid @ RT_T_TCP # Midpoint position



    fiddlefactor = -8
    RancilloToolDepth = 29.3 + fiddlefactor

    key38_withdepth = [-14.2,	68.7, -145.4 - RancilloToolDepth] # Below the GroupGasket (centre)

    theta = 3*np.pi/2 + ((3.4 * np.pi)/180) # Adding the 1.7 degree tilt 
    R2_below = Rotational_matrix_y(theta) # Adjusting the tool rotation
    T2_below = Translation_matrix(key38_withdepth[0], key38_withdepth[1], key38_withdepth[2])

    R_T_Rbelowgrouphead= R2_below + T2_below

    UR_T_RBelowGroupHead = UR_T_RS @ R_T_Rbelowgrouphead
    UR_T_TCP_below = UR_T_RBelowGroupHead @ RTbasketrim_T_RT_unlocked @ RT_T_TCP # Below the Group Head

    Intermediate1 = [-103.990000, -72.960000, -151.890000, -134.640000, -28.560000, -220.460000]
    UR5.MoveJ(Intermediate1, blocking = True)
    
    # Directly in line and out from Rancillo Tool
    Intermediate2 = [-135.170664, -89.856843, -131.207732, -136.992574, -104.728137, -219.155526]
    UR5.MoveJ(Intermediate2, blocking = True)

    # Where the Rancillo tool was left in Action I

    T_UR_T_TCP_locked = rm.Mat(UR_T_TCP_locked.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_locked)), blocking=True)

    tls.student_tool_attach()




    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_locked)), blocking=True)

    time.sleep(2)

    # # Rotate to unlock the Rancillo tool
    # T_UR_T_TCP = rm.Mat(UR_T_TCP.tolist())
    # T_UR_T_TCP_unlocked = rm.Mat(UR_T_TCP_unlocked.tolist())
    # UR5.MoveC(
    #     rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP)),
    #     rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_unlocked)),
    #     blocking=True,
    # )

        # Move from locked to unlocked in one-degree increments.
    for theta_degrees in range(0, 46, 1):
        theta = np.deg2rad(theta_degrees)
        R4_step = Rotational_matrix_x(theta)
        RTbasketrim_T_RT_step = inverse_transform_matrix(
            R4_step[0:3, 0:3], key55[0], key55[1], key55[2]
        )
        UR_T_TCP_step = UR_T_RGroupHead @ RTbasketrim_T_RT_step @ RT_T_TCP
        T_UR_T_TCP_step = rm.Mat(UR_T_TCP_step.tolist())
        UR5.MoveJ(
            rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_step)),
            blocking=True,
        )

    visual_program = RDK.Item("Hide_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()



    T_UR_T_TCP_below = rm.Mat(UR_T_TCP_below.tolist())
    UR5.MoveL(rm.UR_2_Pose(rm.Pose_2_UR(T_UR_T_TCP_below)), blocking=True)

    Intermediate3 = [-104.040000, -107.280000, -114.310000, -133.840000, 49.95, -221.430000]
    UR5.MoveJ(Intermediate3, blocking = True)

    Intermediate4 = [-104.040000, -77.510000, -129.190000, -133.840000, 49.95, -221.430000]
    UR5.MoveJ(Intermediate4, blocking = True)









# ActionS()

