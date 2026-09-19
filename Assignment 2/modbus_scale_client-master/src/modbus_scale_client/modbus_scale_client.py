#*****************************************************************************
#
#   Module:         modbus_scale_client.py
#   Project:        UC-02-2024 Coffee Cart Modifications
#
#   Repository:     modbus_scale_client
#   Target:         N/A
#
#   Author:         Rodney Elliott
#   Date:           6 June 2025
#
#   Description:    Weigh scale Modbus Ethernet client.
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
#   @file modbus_scale_client.py
#
#   @brief Weigh scale Modbus Ethernet client.
#
#   __Modbus Scale Client__ provides the client-side capabilities required to
#   communicate with the Mazzer and Rancilio weigh scales of project UC-02-2024
#   _Coffee Cart Modifications_.

#*****************************************************************************
#   Modules
#*****************************************************************************

##
#   @package time
#
#   The _time_ module provides various time-related functions.
import time

##
#   @package random
#
#   The _random_ module implements pseudo-random number generators for various
#   distributions.
import random

##
#   @package pyModbusTCP
#
#   The _pyModbusTCP_ package contains classes that may be used to create,
#   configure, and control Modbus Ethernet clients and servers.
from pyModbusTCP.client import ModbusClient

#*****************************************************************************
#   Class
#*****************************************************************************

## The Modbus scale client class.
class ModbusScaleClient:
    ##
    #   @brief Class constructor.
    #   @param[in] host IPv4 address of the Modbus Ethernet server.
    #
    #   Perform initial configuration of the Modbus Ethernet client.
    def __init__(self, host):
        ## IPv4 address of the server.
        self.host = host
        ## Presence of scale server.
        self.is_server = False
        ## Presence of scale broker.
        self.is_broker = False

        ## 
        #   @brief Simulated scale weight.
        #
        #   In order for students to be able to test their coffee and water
        #   dosing code outside of the laboratory environment, a simulated
        #   scale weight is provided.
        self.fake_weight = 0

        ##  @brief Previous coil state.
        #
        #   Requests to tare the scale takes the form of a rising edge on coil
        #   zero. In order to effect such edges, the current state of the coil
        #   is compared to its previous state.
        self.tare_on_rising_edge = False

        ## Instance of the _pyModbusTCP_ client class.
        self.client = ModbusClient(host, port = 2593, timeout = 2)

        if  self.client.read_coils(0, 1) is None:
            self.is_server = False
        else:
            self.is_server = True

    ##
    #   @brief Read the scale output (grams).
    #   @return Current scale weight (grams).
    #
    #   Because Modbus input registers are 16-bit, it is necessary to mangle
    #   the double-precision floating-point scale values prior to storage. It
    #   therefore also becomes necessary to de-mangle the value when reading.
    #
    #   Note that if no scale is detected, then this method will output a
    #   simulated scale weight. This value increases by between one and ten
    #   grams every time it is read, and resets back to zero once it reaches
    #   1000 grams.
    def read(self):
        if  self.client.read_coils(0, 1) is None:
            self.is_server = False
        else:
            self.is_server = True

        if self.is_server == True:
            modbus_sign = self.client.read_input_registers(0, 1)
            modbus_full = self.client.read_input_registers(1, 1)
            modbus_part = self.client.read_input_registers(2, 1)

            value_string = str(modbus_full[0]) + "." + str(modbus_part[0])
            value = float(value_string)

            if modbus_sign[0] == 1:
                value *= -1

            if value == -9999.9:
                self.is_broker = False
        
        else:
            self.fake_weight += 10 * random.random()

            if self.fake_weight >= 1000:
                self.fake_weight = 0

            value = self.fake_weight
        
        return value

    ##
    #   @brief Tare (zero) the scale output.
    #
    #   A rising edge on coil zero indicates to the server that it should tare
    #   the scale.
    def tare(self):
        if self.is_server:
            self.client.write_single_coil(0, 1)
            self.tare_on_rising_edge = True
            time.sleep(1)
            self.client.write_single_coil(0, 0)
            self.tare_on_rising_edge = False
        else:
            self.fake_weight = 0

    ##
    #   @brief Report the existence of the Modbus Ethernet broker.
    #   @return @b True if broker detected, otherwise @b False.
    #
    #   If the client is able to connect to the server, but the server is not
    #   receiving data from the broker, this indicates that there is a fault 
    #   with the scale hardware that requires urgent investigation.
    def broker_exists(self):
        if self.is_server == True:
            if self.read() == -9999.9:
                self.is_broker = False
            else:
                self.is_broker = True
        else:
            self.is_broker = False

        return self.is_broker

    ##
    #   @brief Report the existence of the Modbus Ethernet server.
    #   @return @b True if server detected, otherwise @b False.
    #
    #   If the client is able to connect to the server, and the server is
    #   receiving data from the broker, then true weight values will be
    #   available for reading. If the client is unable to connect to the
    #   server, then simulated weight values will be available instead.
    def server_exists(self):
        return self.is_server

