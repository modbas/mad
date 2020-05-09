#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Car Manipulation Inputs
#
# This file is part of Mini-Auto-Drive.
#
# Mini-Auto-Drive is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# Mini-Auto-Drive is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Mini-Auto-Drive.  If not, see <http://www.gnu.org/licenses/>.
#

Header header
uint8 carid              # virtual car id
uint64 cameraFrameId     # camera frame id
uint8 cmd                # command to car control
uint8 CMD_HALT = 0       # car halt
uint8 CMD_FORWARD = 1    # low to high speed forward drive
uint8 CMD_REVERSE = 2    # low to high speed reverse drive
uint8 CMD_SLOW = 3       # low speed forward/reverse drive
uint8 CMD_CHARGE = 4     # battery charging
float32 pedals           # motor control [ -1; 1 ]
float32 steering         # steering control [ -1; 1 ]
