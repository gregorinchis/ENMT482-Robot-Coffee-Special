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

"""Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan"""

"""
Key	Frame	Origin	Fixture	X	Y	Z	Location
1	world	yes	cup_dispenser	-589.30	-221.80	212.90	CD_Top_Cover fastener (2010 o'clock)
12	world	yes	rancilio_scale	-385.80	-330.20	41.70	MS_Top_Cover fastener (left)
13	world	no	rancilio_scale	-418.20	-311.20	41.50	MS_Top_Cover fastener (right)

20	local	no	cup_dispenser	-148.70	71.10	31.50	Supreme_Cup rim (centre)
39	local	yes	rancilio_scale	0	0	0	MS_Top_Cover fastener (left)
40	local	no	rancilio_scale	0	-38	0	MS_Top_Cover fastener (right)
41	local	no	rancilio_scale	157.52	-19	24.24	RS_Pan (centre)
47	local	yes	cup_tool	0	0	0	ATI-9120-011T (centre)
48	local	no	cup_tool	-104.50	0	186.62	Cup_Holder top face centre (open)
49	local	no	cup_tool	-53.50	0	186.62	Cup_Holder top face centre (shut)

"""
# Cup dispenser frame (same as ActionM)
R_cd = np.array([[0, 0, -1, 0],
                 [0, 1, 0, 0],
                 [1, 0, 0, 0],
                 [0, 0, 0, 0]])
URtCD = R_cd + Translation_matrix(-589.3, -221.8, 212.9)
KEY20 = (-148.7, 71.1, 31.5)     # Supreme_Cup rim (local)

# Rancilio scale frame from keys 12/13 (world) and 39/40 (local)
KEY12 = (-385.8, -330.2, 41.7)   # RS top cover fastener left (world)
KEY13 = (-418.2, -311.2, 41.5)   # RS top cover fastener right (world)
KEY40 = (0, -38, 0)              # RS top cover fastener right (local)
KEY41 = (157.52, -19, 24.24)     # RS_Pan centre (local)

theta_rs = np.arctan2(KEY13[1] - KEY12[1], KEY13[0] - KEY12[0]) - np.arctan2(KEY40[1], KEY40[0])
URtRS = Rotational_matrix_z(theta_rs) + Translation_matrix(*KEY12)

# Cup tool keypoints, in the cup_tool frame (key47 = ATI centre = origin)
KEY49 = (-53.5, 0, 186.62)       # Cup_Holder top face centre (shut)
CUP_HEIGHT = 74.7                # base to rim, from the station cup model
# Shut, the holder face sits right at the rim, so the cup hangs from its lip.
# Open drops the holder 51 mm (key48) to let go.

APPROACH  = 120   # stand-off before sliding the holder onto / off the cup (mm)
VIA       = 160   # via point back along the slide, close enough that the wrist stays over the table (mm)
GRIP_UP   = 20    # open holder slides round the cup this far up, clears the dispenser. Shutting lifts the cup this much (mm)
LIFT      = 40    # lift the cup clear of the dispenser tabs (30 mm) (mm)
BACK_CD   = 200   # back out this far from the dispenser before lifting, clears its top (mm)
SWITCH_Z  = 550   # safe TCP height, clear of everything (mm)
OUT_RS    = 150   # straight run onto / off the pan, starts clear of the machine (mm)
RIM_UP    = 3     # holder sits this much higher on the cup, at the dispenser and the scale (mm)

# Directions the holder slides onto the cup (world azimuth, deg). RS keeps the whole arm
# over the table (on R's line, -120.4, the upper arm leans out past the back edge).
SLIDE_AZ_CD = -140
SLIDE_AZ_RS = -100

# Right handed arm configuration, used for all of N.
CUP_REF_JOINTS = [-146, -113, -97, -150, -146, 141]


def CupPose(rim_world, slide_az, back=0, dz=0):
    """Flange pose with the shut cup holder centred on a cup rim at `rim_world`.

    Holder is horizontal (tool x up), sliding onto the cup along world azimuth
    `slide_az` (tool z). `back` backs off along the slide, `dz` moves up.
    """
    a = np.radians(slide_az)
    z = np.array([np.cos(a), np.sin(a), 0])
    x = np.array([0, 0, 1.0])
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])

    target = np.array(rim_world, float) - back*z + np.array([0, 0, dz])
    URtCT = np.identity(4)
    URtCT[0:3, 0:3] = R
    URtCT[0:3, 3] = target - R @ np.array(KEY49)          # put key49 on the rim

    CTtTCP = inverse_transform_z((np.pi/180)*-50, 0, 0, 0)   # 50 deg adapter twist
    return rm.Mat((URtCT @ CTtTCP).tolist())


# Cup rim in the dispenser, and the rim once the cup sits on the scale pan
RIM_CD = (URtCD @ np.array([*KEY20, 1]))[0:3] + np.array([0, 0, RIM_UP])
RIM_RS = (URtRS @ np.array([KEY41[0], KEY41[1], KEY41[2] + CUP_HEIGHT, 1]))[0:3] + np.array([0, 0, RIM_UP])


def GrabCupFromDispenser():
    """Cup tool attached and shut, cup dispensed (after M). Open, slide round the cup, shut to catch the lip and lift it out."""
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))

    # MoveJ in high over the via point so the tool doesn't clip the dispenser, then straight down
    # (explicit joints so the arm is in CUP_REF_JOINTS' configuration, tool given, else it solves for the flange)
    via = CupPose(RIM_CD, SLIDE_AZ_CD, back=VIA, dz=GRIP_UP)
    via_joints = UR5.SolveIK(via, rm.Mat(CUP_REF_JOINTS), UR5.PoseTool())
    UR5.MoveJ(UR5.SolveIK(AtHeight(via, SWITCH_Z), via_joints, UR5.PoseTool()), blocking=True)
    UR5.MoveL(via, blocking=True)

    # open, slide round the cup, then shut to catch the lip
    tls.cup_tool_open_ur5()
    UR5.MoveL(CupPose(RIM_CD, SLIDE_AZ_CD, back=APPROACH, dz=GRIP_UP), blocking=True)
    UR5.MoveL(CupPose(RIM_CD, SLIDE_AZ_CD, dz=GRIP_UP), blocking=True)
    tls.cup_tool_shut_ur5()

    # update visuals: cup leaves the dispenser and is on the tool
    prog = RDK.Item("Hide_Cup_Dispenser_Cup", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()
    RDK.Item("Cup_(UR5)", ITEM_TYPE_TOOL).setVisible(True, False)

    UR5.MoveL(CupPose(RIM_CD, SLIDE_AZ_CD, dz=LIFT), blocking=True)
    UR5.MoveL(CupPose(RIM_CD, SLIDE_AZ_CD, back=APPROACH, dz=LIFT), blocking=True)


def AtHeight(pose, z):
    """`pose` moved straight up or down so the TCP is at height z."""
    T = np.array([[pose[i, j] for j in range(4)] for i in range(4)])
    T[2, 3] = z
    return rm.Mat(T.tolist())


def PlaceCupOnScale():
    """Carry the cup to the Rancilio scale and set it down."""
    # back out clear of the dispenser top, carry upright to the start of the run in, then straight in
    UR5.MoveL(CupPose(RIM_CD, SLIDE_AZ_CD, back=BACK_CD, dz=LIFT), blocking=True)
    CarryUpright(CupPose(RIM_RS, SLIDE_AZ_RS, back=OUT_RS))
    UR5.MoveL(CupPose(RIM_RS, SLIDE_AZ_RS), blocking=True)


def LeaveCupOnScale():
    """Holder is open (dropped below the lip). Update visuals and back out."""
    RDK.Item("Cup_(UR5)", ITEM_TYPE_TOOL).setVisible(False, False)
    prog = RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()

    # straight back out, then up, so the MoveJ to the tool stand is clear of the machine
    UR5.MoveL(CupPose(RIM_RS, SLIDE_AZ_RS, back=OUT_RS), blocking=True)
    UR5.MoveL(AtHeight(UR5.Pose(), SWITCH_Z), blocking=True)


if __name__ == "__main__":
    RDK.setRunMode(RUNMODE_SIMULATE)
    prog = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()
    # standalone test: the state after L and M, portafilter locked in the
    # group head, a cup dispensed. Simulation only.
    for name in ("Show_Rancilio_Rancilio_Tool_Rotated", "Show_Cup_Dispenser_Cup"):
        prog = RDK.Item(name, ITEM_TYPE_PROGRAM)
        prog.RunCode()
        prog.WaitFinished()

    tls.cup_tool_attach_r_ati()
    GrabCupFromDispenser()
    PlaceCupOnScale()
    tls.cup_tool_open_ur5()
    LeaveCupOnScale()
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_r_ati()
