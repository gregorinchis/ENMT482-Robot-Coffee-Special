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



#Key  Frame  Origin?  Ref  X       Y       Z       Location
#14   world  yes      wdt  590.30  -98.70  86      DT_Anvil face (centre)
#15   world  no       wdt  482.10  -98.70  37.30   DT_Plate centre drill hole
#44   local  yes      wdt  0       0       0       DT_Anvil face (centre)
#45   local  no       wdt  108     0       -42.30  DT_Plate centre drill hole
#46   local  no       wdt  27.60   -0.90   84      UK_Rotor fastener (1800 o'clock)

R_wdt = np.array([[-1, 0, 0, 0],
                 [0, -1, 0, 0], 
                 [0, 0, 1, 0],
                 [0, 0, 0, 0]])

T_wdt = Translation_matrix(590.30,  -98.70,  86)

URtWDT = R_wdt + T_wdt

print("URtWDT = ", URtWDT)


def InitialiseSimulateI():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

# key46 UK_Rotor fastener, in the wdt frame. The orbit is defined off this point
KEY46 = (27.6, -0.9, 90.0)
ORBIT_R    = np.hypot(KEY46[0], KEY46[1])       # radius about the wdt z axis
ORBIT_PSI0 = np.arctan2(KEY46[1], KEY46[0])     # starting angle of key46


def MazzerToolChain():
    """Flange <- tool base <- tip. Same chain as ActionM; R4 supplies the flip."""
    R4 = np.array([[1, 0, 0],
                   [0, -1, 0],
                   [0, 0, -1]])                         # tool mount flip (reused from ActionB/M)
    WDTtMTCCT = inverse_transform_matrix(R4, 0, 0, 0)
    #WDT to Mazzer Tool C

    MTCCTtMT = inverse_transform_z(0, 0, 0, 102.82)     # undo key51 YSR-12 tip offset

    theta = (np.pi/180)*-50
    MTtTCP = inverse_transform_z(theta, 0, 0, 0)        # tool base -> flange twist

    return WDTtMTCCT @ MTCCTtMT @ MTtTCP


def WDTPose(x, y, z):
    """World pose of the tool flange for a tip at (x, y, z) in the wdt frame."""
    target = Rotational_matrix_z(0) + Translation_matrix(x, y, z)
    return rm.Mat((URtWDT @ target @ MazzerToolChain()).tolist())


def MoveToWDT():
    # approach above key46; 
    UR5.MoveJ(WDTPose(KEY46[0], KEY46[1], KEY46[2]), blocking=True)


def RotateWDT(rotations=5, steps_per_rev=24):
    #rotate the mazzer tool around the wdt tool for 5 rotation, in a circle with the
    # central axis being wdt tool frame z axis, and radius being point 46, with z height being same as 46
    #
    # Tool orientation is held fixed through the orbit, the tip travels the circle but
    # does not spin, so wrist joint 6 does not wind up 1800 degrees and hit its limit.
    z = KEY46[2]
    for i in range(1, rotations * steps_per_rev + 1):
        psi = ORBIT_PSI0 + 2*np.pi * i / steps_per_rev
        UR5.MoveL(WDTPose(ORBIT_R*np.cos(psi), ORBIT_R*np.sin(psi), z), blocking=True)
    


if __name__ == "__main__":
    InitialiseSimulateI()
    tls.wdt_shut()
    tls.mazzer_tool_attach_r_ati()
    MoveToWDT()
    RotateWDT(rotations=5, steps_per_rev=24)
    