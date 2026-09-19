from time import sleep
from robodk.robolink import *
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
import time
import robodk_scales
from modbus_scale_client import modbus_scale_client



UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

IP_RANCILIO_3 = "192.168.22.4"

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

def ActionP():
    """

    This function uses the Mazzer tool to Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports
    32±0.1g of water has been dispensed in the cup.

    """

    # Get the Mazzer tool (Only necessary on Action testing runs)

    # Once the tool has been attached, send the Robot to the Rancillo Scale Origin at top cover fastener (left)

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


    # Once at the Origin, need to move to the Rancillo hot water switch

    key37 = [46.8, 38.3, -63.2]     # Rancillo Switch Rocker, lowered (Local)

    R2 = Rotational_matrix_y(-3*np.pi/4) # Already in the RS coordinate frame, however rotating for tool head to be pointing into the button
    T2 = Translation_matrix(key37[0], key37[1], key37[2])
    RS_T_RHotWaterSwitch = R2 + T2

    # Rancillo hot water switch Pose in World Coordinates
    UR_T_RHotWaterSwitch = UR_T_RS @ RS_T_RHotWaterSwitch

    # Mazzer tool angle offset

    key50 = [0, 0, 0] # Mazzer Tool Centre Point (Local)
    theta = (np.pi/180)*-50 
    R3 = Rotational_matrix_z(theta)
    T3 = Translation_matrix(key50[0], key50[1], key50[2])
    TCP_T_MT = R3 + T3
    MT_T_TCP = inverse_transform_z(theta, key50[0], key50[1], key50[2])

    # Account for the mazzer tool offset

    key51 = [0, 0, 102.82]  # Mazzer tool offset

    theta = -np.pi/2  # Mazzer tool is RS frame rotated 90 degrees

    R4 = Rotational_matrix_z(theta)
    T4 = Translation_matrix(key51[0], key51[1], key51[2])
    MT_T_MTtip = R4 + T4
    MTtip_T_MT = inverse_transform_z(theta, key51[0], key51[1], key51[2])

    # Now can calculate the Mazzer Tool tip centre point, with respect to the Robots Frame
    UR_T_TCP = UR_T_RHotWaterSwitch @ MTtip_T_MT @ MT_T_TCP

    # # Intermediate 1 is used to avoid hitting the Tool Rack
    # Intermediate1 = [-80.770000, -84.230000, -103.250000, -71.230000, 90.250000, -133.53]
    # UR5.MoveJ(Intermediate1, blocking = True)

    # Intermediate 2 gets close to the button (ugly pose should probably fix)
    Intermediate2 = [24.690000, -4.820000, -74.550000, -206.910000, 68.170000, -156.910000]
    UR5.MoveJ(Intermediate2, blocking = True)

    # Convert from numpy and move
    T_UR_T_TCP = rm.Mat(UR_T_TCP.tolist())
    UR5.MoveJ(T_UR_T_TCP, blocking=True)

    # Slide the button in the Z to turn on the hot water

    LockLeverDisplacement_z = 10
    key37_up = [key37[0], key37[1], key37[2] + LockLeverDisplacement_z]
    T2_up = Translation_matrix(key37_up[0], key37_up[1], key37_up[2])
    RS_T_RHotWaterSwitch_up = R2 + T2_up
    UR_T_RHotWaterSwitch_up = UR_T_RS @ RS_T_RHotWaterSwitch_up
    UR_T_TCP_up = UR_T_RHotWaterSwitch_up @ MTtip_T_MT @ MT_T_TCP

    T_UR_T_TCP_up = rm.Mat(UR_T_TCP_up.tolist())
    UR5.MoveJ(T_UR_T_TCP_up, blocking=True)


    # Handling Scale inputs

    target = 32
    tolerance = 0.1

    client = modbus_scale_client.ModbusScaleClient(host = IP_RANCILIO_3)

    if client.server_exists() == False:
        RDK.ShowMessage("No scale detected, output will be simulated.")

    ## Scale output (grams).
    value = client.read()

    RDK.ShowMessage("Value = %f" % value)

    while (1):
        
        value = client.read() # Scale Output in grams

        if ((value) > (target - tolerance)):
            break

        RDK.ShowMessage("Value = %f" % value)

    # Slide the button in the Y to turn off the hot water

    LockLeverDisplacement_z = 10
    key37_down = [key37[0], key37[1], key37[2] - LockLeverDisplacement_z]
    T2_down = Translation_matrix(key37_down[0], key37_down[1], key37_down[2])
    RS_T_RHotWaterSwitch_down = R2 + T2_down
    UR_T_RHotWaterSwitch_down = UR_T_RS @ RS_T_RHotWaterSwitch_down
    UR_T_TCP_down = UR_T_RHotWaterSwitch_down @ MTtip_T_MT @ MT_T_TCP

    T_UR_T_TCP_down = rm.Mat(UR_T_TCP_down.tolist())
    UR5.MoveJ(T_UR_T_TCP_down, blocking=True)

