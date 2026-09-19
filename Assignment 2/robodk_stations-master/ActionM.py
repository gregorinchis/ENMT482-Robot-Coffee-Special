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

#Key	Frame	Origin?	X	Y	Z	Location
#1	world	yes	-589.3	-221.8	212.9	CD_Top_Cover fastener (2010 o'clock)
#2	world	no	-680.4	-80.6	213.2	CD_Top_Cover fastener (1345 o'clock)
#16	local	yes	0	    0	    0	    CD_Top_Cover fastener (2010 o'clock)
#17	local	no	0	    142	    90	    CD_Top_Cover fastener (1345 o'clock)
#18	local	no	-10.2	72	    -71.8	CD_Front_Index_Pull (open)
#19	local	no	-10.2	71.7	-36.8	CD_Front_Index_Pull (shut)
#20	local	no	-148.7	71.1	31.5	Supreme_Cup rim (centre)

R_cd = np.array([[0, 0, -1, 0],
                 [0, 1, 0, 0], 
                 [1, 0, 0, 0],
                 [0, 0, 0, 0]])

T_cd = Translation_matrix(-589.3, -221.8, 212.9)



URtCD = R_cd + T_cd

print("URtCD = ", URtCD)


def InitialiseSimulateM():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

def HomeToCupDispenserRest():
    #bring the tool just above the cup dispenser slot, ready to lower into the slot
    tls.mazzer_tool_attach_r_ati()


    CDtApproach = Translation_matrix(40, 71.7, -36.8) + Rotational_matrix_z(0) # Pull up to here first before
    #   moving down to the slot.

    Flip = Rotational_matrix_x(np.pi) + Translation_matrix(0, 0, 0)  # reverse tool poke direction along CD Z

    R4 = np.array([[1,0,0],
               [0,-1,0],
               [0,0,-1]])                              # tool mount flip (reused from ActionB)
    CDtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)     # undo 102.82mm tip offset

    theta = (np.pi/180)*-50
    MTtTCP = inverse_transform_z(theta, 0, 0, 0)        # tool base -> flange twist

    URtTCP = URtCD @ CDtApproach @ Flip @ CDtMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    marker = RDK.AddFrame("CD_Approach_Debug")
    marker.setPose(T_URtTCP)
    UR5.MoveJ(T_URtTCP, blocking=True)

def LowerIntoCupDispenserSlot():
        #lower into the slot from the approach position.
    CDtShut = Translation_matrix(-10.2, 71.7, -36.8)+Rotational_matrix_z(0)  # CD origin -> index pull shut point

    Flip = Rotational_matrix_x(np.pi) + Translation_matrix(0, 0, 0)  # reverse tool poke direction along CD Z

    R4 = np.array([[1,0,0],
               [0,-1,0],
               [0,0,-1]])                              # tool mount flip (reused from ActionB)
    CDtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)     # undo 102.82mm tip offset

    theta = (np.pi/180)*-50
    MTtTCP = inverse_transform_z(theta, 0, 0, 0)        # tool base -> flange twist

    URtTCP = URtCD @ CDtShut @ Flip @ CDtMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveL(T_URtTCP, blocking=True)

def CupDispenserSlotToPullOut():
    #once the tool is in the slot, pull out the cup dispenser to key 18.
    CDtOpen = Translation_matrix(-10.2, 72, -71.8)+Rotational_matrix_z(0)  # CD origin -> index pull open point
    Flip = Rotational_matrix_x(np.pi) + Translation_matrix(0, 0, 0)  # reverse tool poke direction along CD Z

    R4 = np.array([[1,0,0],
                [0,-1,0],
                [0,0,-1]])                              # tool mount flip (reused from ActionB)
    CDtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)

    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)     # undo 102.82mm tip offset

    theta = (np.pi/180)*-50
    MTtTCP = inverse_transform_z(theta, 0, 0, 0)        # tool base -> flange twist

    URtTCP = URtCD @ CDtOpen @ Flip @ CDtMTCCT @ MTCCTtMT @ MTtTCP
    T_URtTCP = rm.Mat(URtTCP.tolist())
    UR5.MoveL(T_URtTCP, blocking=True)

if __name__ == "__main__":
    InitialiseSimulateM()
    HomeToCupDispenserRest()
    LowerIntoCupDispenserSlot()
    CupDispenserSlotToPullOut()
    LowerIntoCupDispenserSlot()


