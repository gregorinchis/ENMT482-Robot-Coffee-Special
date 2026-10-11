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

"""Open the WDT fixture, remove the Rancilio tool and close the WDT fixture"""


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


def InitialiseSimulateJ():
    #   After creating a `Robolink()` object, items within the RoboDK station tree
    #   are able to be retrieved by name, item type, or both.
    # Work in simulation mode
    RDK.setRunMode(RUNMODE_SIMULATE)
    UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


    #Resets Simulation ready to be used
    robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    robot_program.RunCode()
    robot_program.WaitFinished()

# Rancilio tool keypoints, in the rancilio_tool frame (key53 = ATI centre = origin)
KEY55 = (28.7, 0, 146.3)       # VST_Basket rim (centre)
KEY63_TILT = 1.7               # rim angle to the stem (deg)
KEY64_DEPTH = 27.3             # VST_Basket depth

# Where the tool sits in the WDT (wdt frame). Basket bottom rests on the anvil
# face (key44), so the rim centre sits one basket depth above it.
RIM_IN_WDT = (0, 0, KEY64_DEPTH-3)
WDT_TILT = 0      # deg about tool y, matches ActionG (KEY63_TILT would be 1.7)

APPROACH = 100    # stand-off behind the ATI plate before mating (mm)
ALIGNED  = 10     # short straight-in distance before mating (mm)
LIFT     = 40     # lift the basket clear of the WDT ring after opening (mm)
RETRACT  = 150    # pull back along the handle after lifting (mm)
ABOVE    = 50     # come down onto the approach point from this far above (mm)

# Same start as ActionG, so the arm comes in over the WDT rather than sweeping past the PUQ
WDT_TOP_JOINTS = [7.776689, -76.124333, -154.780123, -123.629447, -81.857075, 139.638040]

# Arm configuration used for everything done holding the Rancilio tool (G-K).
# It reaches the Mazzer, WDT and PUQ without switching configuration, so the
# tool can be carried between them with MoveL only and never tips over.
WDT_REF_JOINTS = [2, -108, -127, -123, -88, 140]
CARRY_CLEAR = 50  # lift this far above the higher of start/target when carrying (mm)


def WDTtRT():
    """Rancilio tool (ATI centre) pose in the wdt frame while seated in the WDT.

    Handle runs along wdt +x (out towards the DT_Plate / robot), rim faces up:
      tool x -> wdt +z,  tool y -> wdt +y,  tool z (ATI -> basket) -> wdt -x
    No stem tilt here (WDT_TILT = 0): Matthew's ActionG places the tool with
    the handle level, as the station model does, and J has to mate exactly
    where G left it.
    """
    R0 = np.array([[0, 0, -1],
                   [0, 1,  0],
                   [1, 0,  0]])
    R = R0 @ Rotational_matrix_y(np.radians(WDT_TILT))[0:3, 0:3]
    p = np.array(RIM_IN_WDT) - R @ np.array(KEY55)   # put key55 on the rim point

    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    return T


def RTtTCP():
    """Rancilio tool frame -> master tool TCP (50 deg adapter twist, same as ActionL)."""

    return inverse_transform_z((np.pi/180)*-50, 0, 0, 0)


def RancilioPose(dx=0, dz=0, back=0):
    """Flange pose with the Rancilio tool seated in the WDT, offset by
    dx, dz along the wdt frame and `back` mm along tool -z (away from the basket)."""
    offset_wdt = Rotational_matrix_z(0) + Translation_matrix(dx, 0, dz)
    offset_tool = inverse_transform_z(0, 0, 0, back)   # translate -back along tool z
    return rm.Mat((URtWDT @ offset_wdt @ WDTtRT() @ offset_tool @ RTtTCP()).tolist())


def CarryUpright(target, clear=CARRY_CLEAR):
    """Move the held Rancilio tool to `target` without tipping the grounds out.

    Lift straight up, move across, lower straight down. All MoveL, so the
    basket only ever turns about the vertical and stays upright.
    """
    as_np = lambda m: np.array([[m[i, j] for j in range(4)] for i in range(4)])
    start = as_np(UR5.Pose())
    end = as_np(target)
    z = max(start[2, 3], end[2, 3]) + clear

    above_start = start.copy()
    above_start[2, 3] = z
    above_end = end.copy()
    above_end[2, 3] = z

    UR5.MoveL(rm.Mat(above_start.tolist()), blocking=True)
    UR5.MoveL(rm.Mat(above_end.tolist()), blocking=True)
    UR5.MoveL(target, blocking=True)


def SwingUpright(target):
    """Like CarryUpright, but for going round the robot (e.g. PUQ -> group head).

    A straight line across would pass next to the robot base (singularity), so
    rotate joint 1 only (the base axis is vertical, so the basket just swings
    round and stays upright), then MoveL the rest of the way.
    """
    start = UR5.Pose()
    swing = np.degrees(np.arctan2(target[1, 3], target[0, 3]) - np.arctan2(start[1, 3], start[0, 3]))
    swing = (swing + 180) % 360 - 180

    joints = UR5.Joints().list()
    joints[0] += swing
    UR5.MoveJ(joints, blocking=True)
    UR5.MoveL(target, blocking=True)


def MateRancilioInWDT():
    """Move the master tool onto the ATI plate of the Rancilio tool sitting in the WDT."""
    UR5.setTool(RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL))
    UR5.setPoseFrame(RDK.Item("UR5_Base", ITEM_TYPE_FRAME))

    UR5.MoveJ(WDT_TOP_JOINTS, blocking=True)

    # drop onto the approach point from above, then straight along the handle onto the plate
    # MoveJ to explicit joints so the arm ends up in the configuration K needs
    above = UR5.SolveIK(RancilioPose(back=APPROACH, dz=ABOVE), rm.Mat(WDT_REF_JOINTS), UR5.PoseTool())  # tool given, else it solves for the flange
    UR5.MoveJ(above, blocking=True)
    UR5.MoveL(RancilioPose(back=APPROACH), blocking=True)
    UR5.MoveL(RancilioPose(back=ALIGNED), blocking=True)
    UR5.MoveL(RancilioPose(), blocking=True)


def LiftRancilioFromWDT():
    """With the Rancilio tool attached and the WDT open, lift it out and back off."""
    # update visuals: tool leaves the WDT and is now on the robot
    prog = RDK.Item("Hide_WDT_Rancilio_Tool", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()
    RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL).setVisible(True, False)

    # lift the basket out of the ring, then back out along the handle
    UR5.MoveL(RancilioPose(dz=LIFT), blocking=True)
    UR5.MoveL(RancilioPose(dx=RETRACT, dz=LIFT), blocking=True)


if __name__ == "__main__":
    InitialiseSimulateJ()
    tls.wdt_open()
    MateRancilioInWDT()
    tls.student_tool_attach()
    LiftRancilioFromWDT()
    tls.wdt_shut()
