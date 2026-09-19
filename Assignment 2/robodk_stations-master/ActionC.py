from time import sleep
from robodk.robolink import *
import tools
from scipy.spatial.transform import Rotation as R
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
import robodk.robomath as rm
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

def Rotational_matrix_z(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
               [np.sin(theta), np.cos(theta), 0],
               [0, 0, 1]])

def Rotational_matrix_x(theta):
    theta =  np.pi/180 * theta
    return np.array([[1, 0, 0],
               [0, np.cos(theta), -np.sin(theta)],
               [0, np.sin(theta), np.cos(theta)]])

def Rotational_matrix_y(theta):
    theta =  np.pi/180 * theta
    return np.array([[np.cos(theta), 0, np.sin(theta)],
               [0, 1, 0],
               [-np.sin(theta), 0, np.cos(theta)]])

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

def Inverse_transform(R_T,T):
    A = R_T
    B = R_T@(-T)
    Trans_inv = np.identity((4))
    Trans_inv[0:3,0:3] = A
    Trans_inv[0:3,3] = B.ravel()
    print(Trans_inv)
    return(Trans_inv)

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

def transform_matrix(R, x, y, z):
    R = R         # inverse of a rotation matrix is its transpose
    T = np.array([x, y, z])

    Trans = np.identity(4)
    Trans[0:3, 0:3] = R
    Trans[0:3, 3] = (T)
    return Trans

def deg_to_rad(angle):
    return np.pi/180 * angle

# Now for the actual functions
def InitialiseSimulateA():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def home_to_mazzer_button():
    tls.mazzer_tool_attach_r_ati()

    # calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    #theta = -65.2
    #R = Rotational_matrix_x(theta)
    
    #MtMtrans = transform_matrix(R,0,0,0)

    # Moving to the button 
    R = Rotational_matrix_z(0)
    MtMOBC = transform_matrix(R, 89.6, -189.5, -143)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
        
    MOBCtMOBCF = transform_matrix(R,0,0,0)

    #Now lets now rotate it by Y 
    theta = 90
    R = Rotational_matrix_y(theta)
    MOBCFtMOBCFR = transform_matrix(R,0,0,0)

    #now lets set it at an angle where it cannot hit
    theta = 45
    R = Rotational_matrix_x(theta)
            
    MOBCFRtMOBCFRR = transform_matrix(R,0,0,0)


    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    #MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)

    MTCCTtM_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCFRRtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    
    URtTCP = URtM @ MtMOBC @ MOBCtMOBCF @ MOBCFtMOBCFR @ MOBCFRtMOBCFRR @ MOBCFRRtMTCCT @ MTCCTtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    mid_joint = [-51.551494, -92.145478, -127.106194, -50.606335, 89.987358, -196.021636]
    mid_joint_pose_2 = [-56.562479, -113.922327, -115.191222, -130.450281, 229.822187, -194.760804]
    UR5.MoveJ(mid_joint, blocking=True)
    UR5.MoveJ(mid_joint_pose_2, blocking=True)
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def mazzer_button_pressed_on():
    # calculated by doing the cross product of x and y vectors and finding rotation manually
        R = np.array([[-0.874, 0.203, -0.441],
                     [-0.486, -0.367, 0.794],
                     [0, 0.908, 0.420]])
        # rotation in X Y Z [65.209002 0.008858 -150.961295]
        x = 504.4
        y = -419.7
        z = 319.5
    
        # Mazzer Frame transform
        URtM = transform_matrix(R, x, y, z)
    
        #Now making the mazzer frame flat:
        #theta = -65.2
        #R = Rotational_matrix_x(theta)
        
        #MtMtrans = transform_matrix(R,0,0,0)
    
        # Moving to the button 
        R = Rotational_matrix_z(0)
        MtMOBC = transform_matrix(R, 89.6, -189.5, -143)
    
        #Now making the mazzer frame flat:
        theta = -65.2
        R = Rotational_matrix_x(theta)
            
        MOBCtMOBCF = transform_matrix(R,0,0,0)
    
        #Now lets now rotate it by Y 
        theta = 90
        R = Rotational_matrix_y(theta)
        MOBCFtMOBCFR = transform_matrix(R,0,0,0)
    
        #now lets set it at an angle where it cannot hit
        theta = 45
        R = Rotational_matrix_x(theta)
                
        MOBCFRtMOBCFRR = transform_matrix(R,0,0,0)
    
        #press that fucker 
        theta = 0
        R = Rotational_matrix_x(theta)
        MOBCFRRtMOBCFRRP = transform_matrix(R,0,0,-10)
    
        MTtTCP = inverse_transform_z(theta, 0, 0, 0)
    
        R3 = Rotational_matrix_z(0)
        T3 = Translation_matrix(0, 0, 102.82)
        #MTtMTCCT_np = R3 + T3
        MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)
    
        MTCCTtM_np = np.array([[1,0,1,0],
                                [0,-1,0,0],
                                [0,0,0,0],
                                [0,0,0,1]])
        
        R4 = np.array([[0,1,0],
                        [1,0,0],
                        [0,0,-1]])
        
        MOBCFRRPtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)
    
        
        URtTCP = URtM @ MtMOBC @ MOBCtMOBCF @ MOBCFtMOBCFR @ MOBCFRtMOBCFRR @ MOBCFRRtMOBCFRRP @ MOBCFRRPtMTCCT @ MTCCTtMT @ MTtTCP
    
        T_URtTCP = rm.Mat(URtTCP.tolist())
        UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def mazzer_press():
# calculated by doing the cross product of x and y vectors and finding rotation manually
        R = np.array([[-0.874, 0.203, -0.441],
                     [-0.486, -0.367, 0.794],
                     [0, 0.908, 0.420]])
        # rotation in X Y Z [65.209002 0.008858 -150.961295]
        x = 504.4
        y = -419.7
        z = 319.5
    
        # Mazzer Frame transform
        URtM = transform_matrix(R, x, y, z)
    
        #Now making the mazzer frame flat:
        #theta = -65.2
        #R = Rotational_matrix_x(theta)
        
        #MtMtrans = transform_matrix(R,0,0,0)
    
        # Moving to the button 
        R = Rotational_matrix_z(0)
        MtMOBC = transform_matrix(R, 89.6, -189.5, -143)
    
        #Now making the mazzer frame flat:
        theta = -65.2
        R = Rotational_matrix_x(theta)
            
        MOBCtMOBCF = transform_matrix(R,0,0,0)
    
        #Now lets now rotate it by Y 
        theta = 90
        R = Rotational_matrix_y(theta)
        MOBCFtMOBCFR = transform_matrix(R,0,0,0)
    
        #now lets set it at an angle where it cannot hit
        theta = 45
        R = Rotational_matrix_x(theta)
                
        MOBCFRtMOBCFRR = transform_matrix(R,0,0,0)
    
        #unpress that fucker 
        theta = 0
        R = Rotational_matrix_x(theta)
        MOBCFRRtMOBCFRRP = transform_matrix(R,0,0,50)
    
        MTtTCP = inverse_transform_z(theta, 0, 0, 0)
    
        R3 = Rotational_matrix_z(0)
        T3 = Translation_matrix(0, 0, 102.82)
        #MTtMTCCT_np = R3 + T3
        MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)
    
        MTCCTtM_np = np.array([[1,0,1,0],
                                [0,-1,0,0],
                                [0,0,0,0],
                                [0,0,0,1]])
        
        R4 = np.array([[0,1,0],
                        [1,0,0],
                        [0,0,-1]])
        
        MOBCFRRPtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)
    
        
        URtTCP = URtM @ MtMOBC @ MOBCtMOBCF @ MOBCFtMOBCFR @ MOBCFRtMOBCFRR @ MOBCFRRtMOBCFRRP @ MOBCFRRPtMTCCT @ MTCCTtMT @ MTtTCP
    
        T_URtTCP = rm.Mat(URtTCP.tolist())
        UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def mazzer_wait():
    time.sleep(2)
def mazzer_button_turn_off():
# calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    #theta = -65.2
    #R = Rotational_matrix_x(theta)
    
    #MtMtrans = transform_matrix(R,0,0,0)

    # Moving to the button 
    R = Rotational_matrix_z(0)
    MtMOBC = transform_matrix(R, 93.3, -183, -159.5)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
        
    MOBCtMOBCF = transform_matrix(R,0,0,0)

    #Now lets now rotate it by Y 
    theta = 90
    R = Rotational_matrix_y(theta)
    MOBCFtMOBCFR = transform_matrix(R,0,0,0)

    #now lets set it at an angle where it cannot hit
    theta = 35
    R = Rotational_matrix_x(theta)
            
    MOBCFRtMOBCFRR = transform_matrix(R,0,0,0)


    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    #MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)

    MTCCTtM_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCFRRtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    
    URtTCP = URtM @ MtMOBC @ MOBCtMOBCF @ MOBCFtMOBCFR @ MOBCFRtMOBCFRR @ MOBCFRRtMTCCT @ MTCCTtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

def mazzer_button_turn_off_pressed():
# calculated by doing the cross product of x and y vectors and finding rotation manually
    R = np.array([[-0.874, 0.203, -0.441],
                 [-0.486, -0.367, 0.794],
                 [0, 0.908, 0.420]])
    # rotation in X Y Z [65.209002 0.008858 -150.961295]
    x = 504.4
    y = -419.7
    z = 319.5

    # Mazzer Frame transform
    URtM = transform_matrix(R, x, y, z)

    #Now making the mazzer frame flat:
    #theta = -65.2
    #R = Rotational_matrix_x(theta)
    
    #MtMtrans = transform_matrix(R,0,0,0)

    # Moving to the button 
    R = Rotational_matrix_z(0)
    MtMOBC = transform_matrix(R, 93.3, -183, -159.5)

    #Now making the mazzer frame flat:
    theta = -65.2
    R = Rotational_matrix_x(theta)
        
    MOBCtMOBCF = transform_matrix(R,0,0,0)

    #Now lets now rotate it by Y 
    theta = 90
    R = Rotational_matrix_y(theta)
    MOBCFtMOBCFR = transform_matrix(R,0,0,0)

    #now lets set it at an angle where it cannot hit
    theta = 35
    R = Rotational_matrix_x(theta)
            
    MOBCFRtMOBCFRR = transform_matrix(R,0,0,0)

    #press that fucker 
    theta = 0
    R = Rotational_matrix_x(theta)
    MOBCFRRtMOBCFRRP = transform_matrix(R,0,0,-15)
    
    theta = -50 
    R2 = Rotational_matrix_z(theta)
    T2 = Translation_matrix(0, 0, 0)
    #TCPtMT_np = R2 + T2

    MTtTCP = inverse_transform_z(theta, 0, 0, 0)

    R3 = Rotational_matrix_z(0)
    T3 = Translation_matrix(0, 0, 102.82)
    #MTtMTCCT_np = R3 + T3
    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)

    MTCCTtM_np = np.array([[1,0,1,0],
                            [0,-1,0,0],
                            [0,0,0,0],
                            [0,0,0,1]])
    
    R4 = np.array([[0,1,0],
                    [1,0,0],
                    [0,0,-1]])
    
    MOBCFRRtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    
    URtTCP = URtM @ MtMOBC @ MOBCtMOBCF @ MOBCFtMOBCFR @ MOBCFRtMOBCFRR @ MOBCFRRtMOBCFRRP @ MOBCFRRtMTCCT @ MTCCTtMT @ MTtTCP

    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveJ(rm.UR_2_Pose(rm.Pose_2_UR(T_URtTCP)), blocking=True)

    last_point = [-54.053837, -108.975276, -98.878399, -229.293149, 278.261720, -228.959691]
    time.sleep(1)
    UR5.MoveJ(last_point, blocking=True)
    UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)
# example 4x4 matrix 
#np.array([[      0,        0,                0,         x ],
#          [      0,            0,            0,         y ],
#          [      0,            0,            1,         z ],
#          [  0.000000,     0.000000,     0.000000,     1.000000 ]])


