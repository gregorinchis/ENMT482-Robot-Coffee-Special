#*****************************************************************************
#
#   Module:         tools.py
#   Project:        UC-02-2024 Coffee Cart Modifications
#
#   Repository:     robodk_stations
#   Target:         N/A
#
#   Author:         Rodney Elliott
#   Date:           14 August 2025
#
#   Description:    Tool programs.
#
#*****************************************************************************
#
#   Copyright:      (C) 2025 Rodney Elliott
#
#   This file is part of Coffee Cart Modifications.
#
#   Coffee Cart Modifications is free software: you can redistribute it and/or
#   modify it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the license, or (at your
#   option) any later version.
#
#   Coffee Cart Modifications is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
#   Public License for more details.
#
#   You should of received a copy of the GNU General Public License along with
#   Coffee Cart Modifications. If not, see <https://www.gnu.org/licenses/>.
#
#*****************************************************************************

#*****************************************************************************
#   Doxygen file documentation
#*****************************************************************************

##
#   @file tools.py
#
#   @brief Tool programs.
#
#   __tools__ provides methods that replicate the functionality of the tool
#   programs located within the RoboDK station tree.

#*****************************************************************************
#   Modules
#*****************************************************************************

##
#   @package robodk.robolink
#
#   The robodk package is the distributed entry point of the Python API. It is
#   the common parent of all sub-packages and modules that constitute the
#   Python API.
#
#   The _robolink_ sub-module is the bridge between RoboDK and Python. Every
#   object in the RoboDK item tree can be retrieved and it is represented by
#   the object Item. An item can be a robot, a reference frame, a tool, an
#   object or any other item visible in the station tree.
from robodk.robolink import *

#*****************************************************************************
#   Class
#*****************************************************************************

class Tools():
    ##
    #   @brief Class constructor.
    #   @param[in] rdk instance of the RDK object
    def __init__(self, rdk):
        ##  RDK robolink object instance.
        self.RDK = rdk
        ##  Robot instance.
        self.UR5 = self.RDK.Item("UR5", ITEM_TYPE_ROBOT)
        ##
        #   @brief description: robot I/O dictionary.
        #
        #   The dictionary _key_ field is the I/O description as used on the
        #   robot tablet. The _value_ field is the corresponding I/O number for
        #   each robot.
        self.io = {
            "tool_attach": 0,
            "tool_detach": 1,
            "cup_open": 2,
            "cup_shut": 3,
            "wdt_open": 4,
            "wdt_shut": 5,
        }
    ##
    #   @brief Attach cup tool right (ATI).
    #
    #   With the robot in right hand configuration, retrieve the cup_tool from
    #   the ATI fixture.
    def cup_tool_attach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Cup_Tool_(ATI)", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Cup_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Cup_Tool_Shut_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Attached_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Rotated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Departure_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Attach cup tool left (ATI).
    #
    #   With the robot in left hand configuration, retrieve the cup_tool from
    #   the ATI fixture.
    def cup_tool_attach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Cup_Tool_(ATI)", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Cup_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Cup_Tool_Shut_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Attached_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Rotated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Departure_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Detach cup tool right (ATI).
    #
    #   With the robot in right hand configuration, return the cup_tool to the
    #   ATI fixture.
    def cup_tool_detach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Cup_Tool_(ATI)", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Cup_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Departure_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Rotated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Attached_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Cup_Tool_Shut_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Detach cup tool left (ATI).
    #
    #   With the robot in left hand configuration, return the cup_tool to the
    #   ATI fixture.
    def cup_tool_detach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Cup_Tool_(ATI)", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Cup_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Departure_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Rotated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Attached_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Cup_Tool_Shut_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Cup_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##  Open WDT fixture.
    def wdt_open(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["wdt_open"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["wdt_open"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        tool = self.RDK.Item("WDT_Shut", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("WDT_Open", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
    ##  Shut WDT fixture.
    def wdt_shut(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["wdt_shut"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["wdt_shut"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        tool = self.RDK.Item("WDT_Open", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("WDT_Shut", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
    ##
    #   @brief Attach mazzer tool right (ATI).
    #
    #   With the robot in right hand configuration, retrieve the mazzer_tool
    #   from the ATI fixture.
    def mazzer_tool_attach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Mazzer_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Mazzer_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Mazzer_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Attach mazzer tool left (ATI).
    #
    #   With the robot in left hand configuration, retrieve the mazzer_tool
    #   from the ATI fixture.
    def mazzer_tool_attach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Mazzer_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Mazzer_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Mazzer_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Detach mazzer tool right (ATI).
    #
    #   With the robot in right hand configuration, return the mazzer_tool
    #   to the ATI fixture.
    def mazzer_tool_detach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Mazzer_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Mazzer_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Mazzer_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
    ##
    #   @brief Detach mazzer tool left (ATI).
    #
    #   With the robot in left hand configuration, return the mazzer_tool to
    #   the ATI fixture.
    def mazzer_tool_detach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Mazzer_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Mazzer_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Mazzer_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Mazzer_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Mazzer_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
    ##
    #   @brief Attach rancilio tool right (ATI).
    #
    #   With the robot in right hand configuration, retrieve the rancilio_tool
    #   from the ATI fixture.
    def rancilio_tool_attach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Rancilio_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Rancilio_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Attach rancilio tool left (ATI).
    #
    #   With the robot in left hand configuration, retrieve the rancilio_tool
    #   from the ATI fixture.
    def rancilio_tool_attach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Rancilio_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Rancilio_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(False)
        tool = self.RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
    ##
    #   @brief Detach rancilio tool right (ATI).
    #
    #   With the robot in right hand configuration, return the rancilio_tool
    #   to the ATI fixture.
    def rancilio_tool_detach_r_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Rancilio_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Rancilio_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Mated_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Approach_R_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
    ##
    #   @brief Detach rancilio tool left (ATI).
    #
    #   With the robot in left hand configuration, return the rancilio_tool to
    #   the ATI fixture.
    def rancilio_tool_detach_l_ati(self):
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("Rancilio_Tool_(ATI)",
            ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
        # Target move.
        self.UR5.MoveJ(self.RDK.Item("Rancilio_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Mated_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Update visuals.
        tool = self.RDK.Item("Rancilio_Tool_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Rancilio_Tool_(ATI)", ITEM_TYPE_OBJECT)
        tool.setVisible(True)
        # Target move.
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Aligned_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        self.UR5.MoveL(self.RDK.Item("Rancilio_Tool_Approach_L_(ATI)",
            ITEM_TYPE_TARGET), True)
        # Set frame.
        self.UR5.setPoseFrame(self.RDK.Item("UR5_Base", ITEM_TYPE_FRAME))
        # Set tool.
        tool = self.RDK.Item("Master_Tool_(UR5)", ITEM_TYPE_TOOL)
        self.UR5.setTool(tool)
    ##  Open cup tool.
    def cup_tool_open_ur5(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["cup_open"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Cup_Tool_Open_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Toggle I/O.
        self.UR5.setDO(self.io["cup_open"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
    ##  Shut cup tool.
    def cup_tool_shut_ur5(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["cup_shut"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        tool = self.RDK.Item("Cup_Tool_Open_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(False, False)
        tool = self.RDK.Item("Cup_Tool_Shut_(UR5)", ITEM_TYPE_TOOL)
        tool.setVisible(True, False)
        # Toggle I/O.
        self.UR5.setDO(self.io["cup_shut"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
    ##  Student tool attach.
    def student_tool_attach(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
    ##  Student tool detach.
    def student_tool_detach(self):
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_attach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 1)
        # Delay for I/O to actuate.
        time.sleep(0.5)
        # Toggle I/O.
        self.UR5.setDO(self.io["tool_detach"], 0)
        # Delay for I/O to actuate.
        time.sleep(0.5)

