#*****************************************************************************
#
#   Module:         robodk_scales.py
#   Project:        UC-02-2024 Coffee Cart Modifications
#
#   Repository:     robodk_stations
#   Target:         N/A
#
#   Author:         Rodney Elliott
#   Date:           19 June 2025
#
#   Description:    Example code showing how to communicate with the scales.
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
#   @file robodk_scales.py
#
#   @brief Example code showing how to communicate with the scales.

#*****************************************************************************
#   Modules
#*****************************************************************************

##
#   @package robolink
#
#   The _robodk_ package is the distributed entry point of the RoboDK Python
#   API. It is the common parent of all sub-packages and modules that
#   constitute the Python API.
#
#   The _robolink_ sub-module is the bridge between RoboDK and Python. Items in
#   the RoboDK station tree (robots, tools, frames, programs, ...) are able to
#   be retrieved using various methods from the `Robolink()` object.
from robodk.robolink import *

##
#   @package robodialogs
#
#   The _robodialogs_ package is a dialog toolbox for the RoboDK API and RoboDK
#   add-ins. This toolbox includes user prompts, open file dialogs, messages,
#   etc.
from robodk.robodialogs import *

##
#   @package modbus_scale_client
#
#   The _modbus scale_client_ package contains the Modbus Ethernet client
#   class.
from modbus_scale_client import modbus_scale_client

#*****************************************************************************
#   Constants
#*****************************************************************************

## IPv4 address of the Mazzer scale of robot 1.
IP_MAZZER_1 = "192.168.20.3"
## IPv4 address of the Mazzer scale of robot 2.
IP_MAZZER_2 = "192.168.21.3"
## IPv4 address of the Mazzer scale of robot 3.
IP_MAZZER_3 = "192.168.22.3"

## IPv4 address of the Rancilio scale of robot 1.
IP_RANCILIO_1 = "192.168.20.4"
## IPv4 address of the Rancilio scale of robot 2.
IP_RANCILIO_2 = "192.168.21.4"
## IPv4 address of the Rancilio scale of robot 3.
IP_RANCILIO_3 = "192.168.22.4"

#*****************************************************************************
#   Code
#*****************************************************************************

##
#   @brief Create a link between RoboDK and Python.
#
#   After creating a `Robolink()` object, items within the RoboDK station tree
#   are able to be retrieved by name, item type, or both.
RDK = Robolink()

##
#   @brief Get the robot item by name.
#
#   Note that in this and all subsequent calls to the `Item()` method, both the
#   item `name` and item `type` arguments are used. Providing that all item
#   names within the RoboDK station tree are unique, the second argument is
#   optional. However it is used here out of an abundance of caution and to
#   serve as a visual reminder as to the nature of the object that is being
#   referenced.
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

##
#   @brief Instance of the __Modbus Scale Client__ class.
#   @param[in] host IPv4 address of the Modbus Ethernet server host.
client = modbus_scale_client.ModbusScaleClient(host = IP_RANCILIO_3)

if client.server_exists() == False:
    RDK.ShowMessage("No scale detected, output will be simulated.")

## Scale output (grams).
value = client.read()

RDK.ShowMessage("Value = %f" % value)
