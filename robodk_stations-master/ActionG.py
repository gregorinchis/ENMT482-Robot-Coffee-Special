from time import sleep
from robodk.robolink import *
from robodk import *
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
    return np.array([[np.cos(theta), 0, np.sin(theta),0],
               [0, 1, 0,0],
               [-np.sin(theta),0, np.cos(theta), 0],
               [0, 0, 0, 0]])

def Rotational_matrix_z_inverse_z(theta):
    return np.transpose(np.array([[np.cos(theta), -np.sin(theta), 0],
                   [np.sin(theta), np.cos(theta), 0],
                   [0, 0, 1]]))

def Rotational_matrix_y_inverse_y(theta):
    return np.transpose(np.array([[np.cos(theta), 0, -np.sin(theta)],
                   [0, 1, 0],
                   [-np.sin(theta), 0, np.cos(theta)]]))

def Rotational_matrix_x_inverse_x(theta):
    return np.transpose([[1, 0, 0],
               [0, np.cos(theta), -np.sin(theta)],
               [0, np.sin(theta), np.cos(theta)]])

def Translation_matrix(x,y,z):
    return np.array([[0, 0, 0, x],
                  [0, 0, 0,y],
                  [0, 0, 0, z],
                  [0, 0, 0, 1]])
def Translation_matrix_inverse(x,y,z):
    return np.array([[x],
                     [y],
                     [z]])

def Inverse_transform(R_T,T):
    A = R_T
    B = R_T@(-T)
    Trans_inv = np.identity((4))
    Trans_inv[0:3,0:3] = A
    Trans_inv[0:3,3] = B.ravel()
    print(Trans_inv)
    return(Trans_inv)

def inverse_transform_matrix(R, x, y, z):
    R_T = np.transpose(R)          # inverse of a rotation matrix is its transpose
    T = np.array([x, y, z])

    Trans_inv = np.identity(4)
    Trans_inv[0:3, 0:3] = R_T
    Trans_inv[0:3, 3] = R_T @ (-T)
    return Trans_inv

# Now for the actual functions
def MazzerPickUpToWDTTop(z):

    theta = 180*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(590.3,-98.7, 86)
    URtWDT_np = R + T

    R1_1 = Rotational_matrix_z(0)
    T1_1 = Translation_matrix(0,0,z)
    WDTtWDTT_np = R1_1 + T1_1

    R1 = Rotational_matrix_y(0)
    T1 = Translation_matrix(-29.3,0, 0)
    RTBRRtRTBRRT = R1 + T1

    #inverse matrix 
    R1_inv = Rotational_matrix_z_inverse_z(0)
    T1_inv = Translation_matrix_inverse(-29.3,0, 0)
    RTBRRTtRTBRR = Inverse_transform(R1_inv, T1_inv)

    theta = 0*np.pi/180
    R2 = Rotational_matrix_y(theta)
    T2 = Translation_matrix(0,0, 0)
    RTBRtRTBRR = R2 + T2

    #inverse matrix
    R2_inv = Rotational_matrix_y_inverse_y(theta)
    T2_inv = Translation_matrix_inverse(0,0, 0)
    RTBRRtRTBR = Inverse_transform(R2_inv, T2_inv)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(28.7,0, 146.3)
    RTtRTBR_np = R3 + T3
    
    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z(0)
    T3_inv = Translation_matrix_inverse(28.7,0, 146.3)
    RTBRtRT = Inverse_transform(R3_inv, T3_inv)
        
    R4 = Rotational_matrix_z((np.pi/180)*-50)
    T4 = Translation_matrix(0, 0, 0)
    TCPtRT_np = R4 + T4
        
        #inverse matrix
    R4_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T4_inv = Translation_matrix_inverse(0,0,0)
    RTtTCP = Inverse_transform(R4_inv,T4_inv)
    
    R5 = np.array([[0,0,1],
                   [0,1,0],
                   [-1,0,0]])
    
    RTBBRTRtRTBBRT = inverse_transform_matrix(R5, 0, 0, 0)

    R6 = np.array([[1,0,0],
                   [0,1,0],
                   [0,0,1]]) 
    WDTtRTBBRTR = inverse_transform_matrix(R6, 0, 0, 0)

    URtTCP = URtWDT_np @ WDTtWDTT_np @ WDTtRTBBRTR @ RTBBRTRtRTBBRT @ RTBRRTtRTBRR @ RTBRRtRTBR @ RTBRtRT @ RTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
        
    # convert numpy array into an RDK matrix
        
    initial_orientation = [7.776689, -76.124333, -154.780123, -123.629447, -81.857075, 139.638040]
    UR5.MoveJ(initial_orientation, blocking=True)
        
    UR5.MoveJ(UR_2_Pose(Pose_2_UR(T_URtTCP)), blocking=True)    # Run your rm.Mat through a conversion and back and it works...
def WDTTToWDT():

    theta = 180*np.pi/180
    R = Rotational_matrix_z(theta)
    T = Translation_matrix(590.3,-98.7, 86)
    URtWDT_np = R + T

    R1 = Rotational_matrix_y(0)
    T1 = Translation_matrix(-29.3,0, 0)
    RTBRRtRTBRRT = R1 + T1

    #inverse matrix 
    R1_inv = Rotational_matrix_z_inverse_z(0)
    T1_inv = Translation_matrix_inverse(-29.3,0, 0)
    RTBRRTtRTBRR = Inverse_transform(R1_inv, T1_inv)

    theta = 0*np.pi/180
    R2 = Rotational_matrix_y(theta)
    T2 = Translation_matrix(0,0, 0)
    RTBRtRTBRR = R2 + T2

    #inverse matrix
    R2_inv = Rotational_matrix_y_inverse_y(theta)
    T2_inv = Translation_matrix_inverse(0,0, 0)
    RTBRRtRTBR = Inverse_transform(R2_inv, T2_inv)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(28.7,0, 146.3)
    RTtRTBR_np = R3 + T3
    
    #inverse matrix
    R3_inv = Rotational_matrix_z_inverse_z(0)
    T3_inv = Translation_matrix_inverse(28.7,0, 146.3)
    RTBRtRT = Inverse_transform(R3_inv, T3_inv)
        
    R4 = Rotational_matrix_z((np.pi/180)*-50)
    T4 = Translation_matrix(0, 0, 0)
    TCPtRT_np = R4 + T4
        
        #inverse matrix
    R4_inv = Rotational_matrix_z_inverse_z((np.pi/180)*-50)
    T4_inv = Translation_matrix_inverse(0,0,0)
    RTtTCP = Inverse_transform(R4_inv,T4_inv)
    
    R5 = np.array([[0,0,1],
                   [0,1,0],
                   [-1,0,0]])
    
    RTBBRTRtRTBBRT = inverse_transform_matrix(R5, 0, 0, 0)

    R6 = np.array([[1,0,0],
                   [0,1,0],
                   [0,0,1]]) 
    WDTtRTBBRTR = inverse_transform_matrix(R6, 0, 0, 0)

    URtTCP = URtWDT_np @ WDTtRTBBRTR @ RTBBRTRtRTBBRT @ RTBRRTtRTBRR @ RTBRRtRTBR @ RTBRtRT @ RTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
        
    # convert numpy array into an RDK matrix
        
    UR5.MoveL(UR_2_Pose(Pose_2_UR(T_URtTCP)), blocking=True)    # Run your rm.Mat through a conversion and back and it works...
    tls.student_tool_detach() 
def WDTleftopen():
    Location = [3.462210, -85.438446, -150.300195, -124.194508, -86.064749, 140.268358]
    UR5.MoveL(Location, blocking=True)
# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


