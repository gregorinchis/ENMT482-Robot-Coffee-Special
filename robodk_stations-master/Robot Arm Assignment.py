from time import sleep
from robodk.robolink import *
# import robodk_scales
# from modbus_scale_client import modbus_scale_client
from ActionA import InitialiseSimulateA, HomeToMazzerScaleTop, MazzerScaleTopToMazzerScale, MazzerScaleToHome
from ActionB import HomeToMazzerScaleLockLeverRightTop, SlideInXDirectionAcrossLock, SlideInzDirectionAcrossLock
from ActionC import home_to_mazzer_button, mazzer_button_pressed_on, mazzer_press, mazzer_wait, mazzer_button_turn_off, mazzer_button_turn_off_pressed
from ActionD import dosing_action
from ActionE import HomeToMazzerScaleLockLeverRightTop2, SlideInXDirectionAcrossLock2, SlideInzDirectionAcrossLock2
from ActionF import HomeToMazzerPickUp, CollectMazzerTool, TopOfMazzerTool
from ActionG import MazzerPickUpToWDTTop, WDTTToWDT, WDTleftopen
from ActionI import MoveToWDT, RotateWDT
from ActionJ import MateRancilioInWDT, LiftRancilioFromWDT
from ActionK import InsertRancilioInPUQ, WithdrawRancilioFromPUQ
from ActionL import ActionL
from ActionM import HomeToCupDispenserRest, LowerIntoCupDispenserSlot, CupDispenserSlotToPullOut
from ActionN import GrabCupFromDispenser, PlaceCupOnScale, LeaveCupOnScale
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
#RDK.setRunMode(RUNMODE_SIMULATE)
RDK.setRunMode(RUNMODE_RUN_ROBOT)


UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)


#Resets Simulation ready to be used
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()


Actions = [
    0,  # A: Rancilio tool to Mazzer scale
    0,  # B: Unlock Mazzer scale
    0,  # C: Grind 15s
    0,  # D: Dose 20g grounds
    0,  # E: Lock Mazzer scale
    0,  # F: Remove Rancilio tool from Mazzer
    0,  # G: Rancilio tool into WDT
    0,  # H: Release tool, close WDT
    1,  # I: Spin WDT rotor 5 turns
    1,  # J: Remove Rancilio tool from WDT
    1,  # K: Tamp in PUQ
    1,  # L: PUQ to group head
    1,  # M: Dispense cup
    1,  # N: Cup to Rancilio scale
    1,  # O: Unlock Rancilio scale
    1,  # P: Dispense 32g water
    1,  # Q: Lock Rancilio scale
    1,  # R: Cup to customer zone
    1,  # S: Remove Rancilio tool from group head
    1,  # T: Silicone brush clean
    1,  # U: Bristle brush clean
    1,  # V: Return Rancilio tool to stand
]

if Actions[0] == 1:
    # Action A: Pick up the Rancilio tool and place it on the Mazzer Scale pan.
    tls.rancilio_tool_attach_r_ati()
    tls.wdt_shut()
    HomeToMazzerScaleTop()
    MazzerScaleTopToMazzerScale()
    MazzerScaleToHome()
    #tls.rancilio_tool_detach_r_ati() # Only for Simulation


if Actions[1] == 1:
    # Action B: Use the Mazzer tool to unlock the Mazzer Scale.
    tls.mazzer_tool_attach_r_ati()
    HomeToMazzerScaleLockLeverRightTop()
    SlideInXDirectionAcrossLock()
    SlideInzDirectionAcrossLock()

if Actions[2] == 1:
    # Action C: Use the Mazzer tool to turn the Mazzer on, wait 15s, and turn the Mazzer off.
    home_to_mazzer_button()
    mazzer_button_pressed_on()
    mazzer_press()
    mazzer_wait()
    mazzer_button_turn_off()
    mazzer_button_turn_off_pressed()

if Actions[3] == 1:
    # Action D: Use the Mazzer tool to pull the Mazzer dosing lever until the scale reports 20±0.1g of coffee grounds has been deposited in the Rancilio tool.
    dosing_action()

if Actions[4] == 1:
    # Action E: Use the Mazzer tool to lock the Mazzer Scale.
    HomeToMazzerScaleLockLeverRightTop2()
    SlideInXDirectionAcrossLock2()
    SlideInzDirectionAcrossLock2()

if Actions[5] == 1:
    # Action F: Remove the Rancilio tool from the Mazzer.
    tls.mazzer_tool_detach_r_ati()
    HomeToMazzerPickUp()
    CollectMazzerTool()
    TopOfMazzerTool()

if Actions[6] == 1:
    # Action G: Open the WDT fixture, and place the Rancilio tool into the WDT fixture.
    tls.wdt_open()
    MazzerPickUpToWDTTop(50)
    WDTTToWDT()
    WDTleftopen()
    tls.wdt_shut()

if Actions[7] == 1:
    # Action H: Release the Rancilio tool and close the WDT fixture.
    tls.wdt_shut()

if Actions[8] == 1:
    # Action I: Use the Mazzer tool to turn the WDT rotor five full revolutions.
    tls.mazzer_tool_attach_r_ati()
    MoveToWDT()
    RotateWDT(rotations=5, steps_per_rev=24)

if Actions[12] == 1:
    # Action M: Use the Mazzer tool to operate the cup dispenser.
    # tls.mazzer_tool_attach_r_ati() # only for seperate testing (I leaves it on)
    HomeToCupDispenserRest()
    LowerIntoCupDispenserSlot()
    CupDispenserSlotToPullOut()
    LowerIntoCupDispenserSlot()
    tls.mazzer_tool_detach_r_ati()

if Actions[13] == 1:
    # Action N: Use the cup tool to pick up the dispensed cup, and place it on the Rancilio Scale pan.
    # RDK.Item("Show_Cup_Dispenser_Cup", ITEM_TYPE_PROGRAM).RunCode() # only for seperate testing
    tls.cup_tool_attach_r_ati()
    GrabCupFromDispenser()
    PlaceCupOnScale()
    tls.cup_tool_open_ur5()
    LeaveCupOnScale()
    tls.cup_tool_shut_ur5()
    tls.cup_tool_detach_r_ati()

if Actions[9] == 1:
    # Action J: Open the WDT fixture, remove the Rancilio tool and close the WDT fixture.
    tls.wdt_open()
    MateRancilioInWDT()
    tls.student_tool_attach()
    LiftRancilioFromWDT()
    tls.wdt_shut()

if Actions[10] == 1:
    # Action K: Place the Rancilio tool into the PUQ fixture, and wait 2 seconds while the machine tamps the coffee grounds.
    # tls.rancilio_tool_attach_r_ati() # only for seperate testing
    InsertRancilioInPUQ()

if Actions[11] == 1:
    # Action L: Remove the Rancilio tool from the PUQ fixture, and insert it into the Rancilio group head.
    visual_program = RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    # tls.rancilio_tool_attach_r_ati() # only for on machine testing (L on its own, not after K)
    WithdrawRancilioFromPUQ()
    ActionL()

if Actions[14] == 1:
    # Action O: Use the Mazzer tool to unlock the Rancilio Scale.
    visual_program = RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    visual_program = RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    ActionO()

if Actions[15] == 1:
    # Action P: Use the Mazzer tool to operate the Rancilio hot water switch until the scale reports 32±0.1g of water has been dispensed in the cup.
    # tls.mazzer_tool_attach_r_ati() # Only for seperate testing
    ActionP()

if Actions[16] == 1:
    # Action Q: Use the Mazzer tool to lock the Rancilio Scale.
    visual_program = RDK.Item("Show_Rancilio_Scale_Read", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    # tls.mazzer_tool_attach_r_ati() # Only for seperate testing
    ActionQ()

if Actions[18] == 1:
    # Action S: Remove the Rancilio tool from the group head.
   # tls.rancilio_tool_attach_r_ati() # only for seperate testing
    visual_program = RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    ActionS()

if Actions[19] == 1:
    # Action T: Position the Rancilio tool over the Rancilio Tool Cleaner fixture silicone brush, and actuate for 5s.
    # tls.rancilio_tool_attach_r_ati() # only for seperate testing
    ActionT()

if Actions[20] == 1:
    # Action U: Position the Rancilio tool over the Rancilio Tool Cleaner fixture bristle brush, and actuate for 5s.
    ActionU()

if Actions[17] == 1:
    # Action R: Use the cup tool to carefully pick up the cup of coffee and place it in the customer zone.
    visual_program = RDK.Item("Show_Rancilio_Scale_Cup", ITEM_TYPE_PROGRAM)
    visual_program.RunCode()
    visual_program.WaitFinished()
    # visual_program = RDK.Item("Show_Rancilio_Rancilio_Tool_Rotated", ITEM_TYPE_PROGRAM)
    # visual_program.RunCode()
    # visual_program.WaitFinished()
    ActionR()

