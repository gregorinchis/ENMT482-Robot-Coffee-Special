from time import sleep
from robodk.robolink import *
from ActionA import InitialiseSimulateA, HomeToMazzerScaleTop, MazzerScaleTopToMazzerScale, MazzerScaleToHome
from ActionB import HomeToMazzerScaleLockLeverRightTop, SlideInXDirectionAcrossLock, SlideInzDirectionAcrossLock
from ActionC import home_to_mazzer_button, mazzer_button_pressed_on, mazzer_press, mazzer_wait, mazzer_button_turn_off, mazzer_button_turn_off_pressed
from ActionL import ActionL
from ActionO import ActionO
from ActionP import ActionP
from ActionQ import ActionQ
from ActionR import ActionR
from ActionS import ActionS
from ActionT import ActionT
from ActionU import ActionU
import tools
import numpy as np

RDK = Robolink()
tls = tools.Tools(RDK)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

#   After creating a `Robolink()` object, items within the RoboDK station tree
#   are able to be retrieved by name, item type, or both.
# Work in simulation mode
RDK.setRunMode(RUNMODE_SIMULATE)
# RDK.setRunMode(RUNMODE_RUN_ROBOT)


UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


#Resets Simulation ready to be used
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()


Actions = [
    0,  # A
    0,  # B
    0,  # C
    0,  # D
    0,  # E
    0,  # F
    0,  # G
    0,  # H
    0,  # I
    0,  # J
    0,  # K
    0,  # L
    0,  # M
    0,  # N
    0,  # O
    0,  # P
    0,  # Q
    1,  # R
    0,  # S
    0,  # T
    0,  # U
    0,  # V
]

if Actions[0] == 1:
    #Action A
    HomeToMazzerScaleTop()
    MazzerScaleTopToMazzerScale()
    MazzerScaleToHome()
    # tls.rancilio_tool_detach_r_ati() # only for seperate testing
if Actions[1] == 1:
    #Action B
    HomeToMazzerScaleLockLeverRightTop()
    SlideInXDirectionAcrossLock()
    SlideInzDirectionAcrossLock()
if Actions[2] == 1:
    home_to_mazzer_button()
    mazzer_button_pressed_on()
    mazzer_press()
    mazzer_wait()
    mazzer_button_turn_off()
    mazzer_button_turn_off_pressed()

if Actions[11] == 1:
    # Action L
    # tls.rancilio_tool_attach_r_ati() # only for seperate testing
    ActionL()

if Actions[14] == 1:
    # Action O
    ActionO()

if Actions[15] == 1:
    # Action P
    # tls.mazzer_tool_attach_r_ati() # Only for seperate testing
    ActionP()

if Actions[16] == 1:
    # Action Q
    # tls.mazzer_tool_attach_r_ati() # Only for seperate testing
    ActionQ()

if Actions[17] == 1:
    # Action R
    ActionR()

if Actions[18] == 1:
    # Action S
    ActionS()
if Actions[19] == 1:
    # Action T
    # tls.rancilio_tool_attach_r_ati() # only for seperate testing
    ActionT()
if Actions[20] == 1:
    # Action U
    ActionU()

