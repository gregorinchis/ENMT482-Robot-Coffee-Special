from time import sleep
from robodk.robolink import *
from ActionA import InitialiseSimulateA, HomeToMazzerScaleTop, MazzerScaleTopToMazzerScale, MazzerScaleToHome
from ActionB import HomeToMazzerScaleLockLeverRightTop, SlideInXDirectionAcrossLock, SlideInzDirectionAcrossLock
import tools
import numpy as np
RDK = Robolink()
tls = tools.Tools(RDK)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

Actions = [0, 1]

InitialiseSimulateA()
if Actions[0] == 1:
    #Action A
    HomeToMazzerScaleTop()
    MazzerScaleTopToMazzerScale()
    MazzerScaleToHome()
    tls.rancilio_tool_detach_r_ati()
if Actions[1] == 1:
    #Action B
    HomeToMazzerScaleLockLeverRightTop()
    SlideInXDirectionAcrossLock()
    SlideInzDirectionAcrossLock()