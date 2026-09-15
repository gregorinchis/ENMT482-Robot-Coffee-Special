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

# Now for the actual functions
def InitialiseSimulateB():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def HomeToMazzerScaleLockLeverRightTop():
    tls.mazzer_tool_attach_r_ati()

    #we need to arrive at the mazzer origin first
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1_1 = Rotational_matrix_z(0)
    T1_1 = Translation_matrix(40, -59.53, -15)
    MStMSLLR = R1_1 + T1_1

    theta = 15*np.pi/180
    R1 = Rotational_matrix_x(theta)
    T1 = Translation_matrix(0,0,0)
    MSLLRtMSLLRR = R1 + T1

    theta = (np.pi/180)*-50
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)


    MTCCTtMS_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])

    R4 = np.array([[1,0,0],
                    [0,-1,0],
                    [0,0,-1]])

    MStMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    midpoint = [-40.269152, -102.070235, -113.852073, -48.908772, 104.100501, -84.629458]
    UR5.MoveJ(midpoint, blocking=True)

    URtTCP = URtMS_np @ MStMSLLR @ MSLLRtMSLLRR @ MStMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(T_URtTCP, blocking=True)

def SlideInXDirectionAcrossLock():

    #we need to arrive at the mazzer origin first
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    #x is the NumberToChangeInLab
    R1_1 = Rotational_matrix_z(0)
    T1_1 = Translation_matrix(5, -59.53, -15)
    MStMSLLR = R1_1 + T1_1

    theta = 15*np.pi/180
    R1 = Rotational_matrix_x(theta)
    T1 = Translation_matrix(0,0,0)
    MSLLRtMSLLRR = R1 + T1

    theta = (np.pi/180)*-50
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)


    MTCCTtMS_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])

    R4 = np.array([[1,0,0],
                    [0,-1,0],
                    [0,0,-1]])

    MStMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    URtTCP = URtMS_np @ MStMSLLR @ MSLLRtMSLLRR @ MStMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(T_URtTCP, blocking=True)

def SlideInzDirectionAcrossLock():

    #we need to arrive at the mazzer origin first
    theta = -60*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(439.4,-277.9, 41.9)
    URtMS_np = R + T

    R1_1 = Rotational_matrix_z(0)
    T1_1 = Translation_matrix(5, -59.53, -25)
    MStMSLLR = R1_1 + T1_1

    theta = 15*np.pi/180
    R1 = Rotational_matrix_x(theta)
    T1 = Translation_matrix(0,0,0)
    MSLLRtMSLLRR = R1 + T1

    theta = (np.pi/180)*-50
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)


    MTCCTtMS_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])

    R4 = np.array([[1,0,0],
                    [0,-1,0],
                    [0,0,-1]])

    MStMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    URtTCP = URtMS_np @ MStMSLLR @ MSLLRtMSLLRR @ MStMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(T_URtTCP, blocking=True)

    midpoint = [-31.614301, -80.955259, -129.316320, -60.303246, 90.148023, -76.126285]

    UR5.MoveJ(midpoint, blocking=True)
    #UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


