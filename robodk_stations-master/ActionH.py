"""Release the Rancilio tool and close the WDT fixture"""

# ActionG (Matthew) already releases the tool (student_tool_detach in
# WDTTToWDT) and backs straight off 50 mm in WDTleftopen(), leaving the WDT open, so all
# that is left for H is closing the fixture.
from ActionG import *


if __name__ == "__main__":
    RDK.setRunMode(RUNMODE_SIMULATE)
    prog = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
    prog.RunCode()
    prog.WaitFinished()
    # standalone test: G first, with the Rancilio tool picked off the stand
    # (normally F leaves it on the robot beside the Mazzer)
    tls.rancilio_tool_attach_r_ati()
    tls.wdt_open()
    MazzerPickUpToWDTTop(50)
    WDTTToWDT()
    WDTleftopen()

    tls.wdt_shut()
