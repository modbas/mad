#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Mission State
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

uint8 carid
uint8 state                     # state of the car, which is visible for the user
uint8 STATE_READY = 0           # user is able to lock the car
uint8 STATE_DRIVING = 1         # car is driving
uint8 STATE_CHARGING = 2        # car is charging, user cannot lock the car
uint8 STATE_NA = 3              # car is not available
uint8 STATE_LOCKED = 4          # car is locked
float32 v
uint32 drivenLaps
uint32 lapCount
uint32 targetParkingSpaceID
float32[2] s
float32 psi
bool timerTriggered
