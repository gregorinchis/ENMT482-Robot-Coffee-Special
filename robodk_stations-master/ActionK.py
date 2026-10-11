from time import sleep
from robodk.robolink import *
from ActionJ import CarryUpright
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

"""place the Rancilio tool in the PUQ and wait 2 s"""

"""
Key	Frame	Origin	Fixture	X	Y	Z	Location
8	world	yes	puq	384.80	84.40	270.90	PUQ_Body lid apex
9	world	no	puq	344.70	124.40	270.90	PUQ_Body lid edge (2100 o'clock)

32	local	yes	puq	0	0	0	PUQ_Body lid apex
33	local	no	puq	0	-56.38	0	PUQ_Body lid edge (2100 o'clock)
34	local	no	puq	9.90	-0.30	-133.50	PUQ_Body upper clamp (centre)

"""
# PUQ frame from keys 8/9 (world) and 32/33 (local)
KEY8  = (384.8, 84.4, 270.9)     # PUQ_Body lid apex (world)
KEY9  = (344.7, 124.4, 270.9)    # PUQ_Body lid edge (world)
KEY33 = (0, -56.38, 0)           # PUQ_Body lid edge (local)
KEY34 = (9.90, -0.30, -133.50)   # PUQ_Body upper clamp (local)

theta_puq = np.arctan2(KEY9[1] - KEY8[1], KEY9[0] - KEY8[0]) - np.arctan2(KEY33[1], KEY33[0])
URtPUQ = Rotational_matrix_z(theta_puq) + Translation_matrix(*KEY8)

# Rancilio tool keypoints, in the rancilio_tool frame (key53 = ATI centre = origin)
KEY55 = (28.7, 0, 146.3)         # VST_Basket rim (centre)
KEY63_TILT = 1.7                 # rim angle to the stem (deg)

# Model seat: basket rim centred under the upper clamp (key34), 4 mm below its face.
# The settings below are measured from this point.
RIM_GAP = 4
RIM_IN_PUQ = (KEY34[0], KEY34[1], KEY34[2] - RIM_GAP)

# Insertion position, tuned at the lab (mm from the model seat)
SLIDE_IN_Z = 1   # height the tool slides in at. Too low hits the bottom bracket
SEAT_Z     = 6   # height it rises to and sits at while tamping
SEAT_IN    = 3   # how much further into the PUQ than the model seat it goes (mm)

APPROACH = 150   # stand-off in front of the PUQ, along the handle (mm)
TAMP_WAIT = 2    # seconds


def PUQtRT():
    """Rancilio tool (ATI centre) pose in the PUQ frame while seated.

    Column is at puq -x (Lab frames p7), so the handle points out the open
    front along puq +x, rim faces up:
      tool x -> puq +z,  tool y -> puq +y,  tool z (ATI -> basket) -> puq -x
    then tilt 1.7 deg about tool y so the rim is level (same as the WDT in ActionJ).
    """
    R0 = np.array([[0, 0, -1],
                   [0, 1,  0],
                   [1, 0,  0]])
    R = R0 @ Rotational_matrix_y(np.radians(KEY63_TILT))[0:3, 0:3]
    p = np.array(RIM_IN_PUQ) - R @ np.array(KEY55)

    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T


def PUQPose(dx=0, z=SEAT_Z):
    """Flange pose with the Rancilio tool in the PUQ, `dx` back out from the seat along the handle, rim `z` above the model seat."""
    offset = Rotational_matrix_z(0) + Translation_matrix(dx - SEAT_IN, 0, z)
    RTtTCP = inverse_transform_z((np.pi/180)*-50, 0, 0, 0)     # 50 deg adapter twist
    return rm.Mat((URtPUQ @ offset @ PUQtRT() @ RTtTCP).tolist())


def InsertRancilioInPUQ():
    """Robot is holding the Rancilio tool (after J). Slide it into the PUQ and let it tamp."""
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))

    # carry upright from the WDT (J leaves the arm in the right configuration),
    # slide in at SLIDE_IN_Z, then rise to SEAT_Z
    CarryUpright(PUQPose(dx=APPROACH, z=SLIDE_IN_Z))
    UR5.MoveL(PUQPose(z=SLIDE_IN_Z), blocking=True)
    UR5.MoveL(PUQPose(z=SEAT_Z), blocking=True)

    sleep(TAMP_WAIT)


def WithdrawRancilioFromPUQ():
    """Reverse of the insert, start of task L."""
    UR5.MoveL(PUQPose(z=SLIDE_IN_Z), blocking=True)
    UR5.MoveL(PUQPose(dx=APPROACH, z=SLIDE_IN_Z), blocking=True)


if __name__ == "__main__":
    RDK.setRunMode(RUNMODE_SIMULATE)
    prog = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()
    # standalone test: jump to where J leaves the robot. Rancilio tool attached,
    # pulled out of the WDT, upright. Simulation only.
    UR5.setJoints([3, -83, -146, -129, -86, 140])
    RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT).setVisible(False)
    RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL).setVisible(True, False)
    InsertRancilioInPUQ()
    WithdrawRancilioFromPUQ()
