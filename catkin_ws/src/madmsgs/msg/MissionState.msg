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
uint8 naviState
uint8 NAVISTATE_NOTAVAILABLE = 0
uint8 NAVISTATE_LOST = 1
uint8 NAVISTATE_WAITINGFORWAITING = 2
uint8 NAVISTATE_WAITINGFORDRIVING = 3
uint8 NAVISTATE_WAITINGFORMISSION = 4
uint8 maneuverState
uint8 MANEUVERSTATE_WAITING = 0
uint8 MANEUVERSTATE_PARKING = 3
uint8 MANEUVERSTATE_CHARGING = 5
uint8 MANEUVERSTATE_BOOTING = 7
uint32 segment
uint32 lap                                    # current lap
float32[2] s                                  # current pos. [ m ]
float32 psi                                   # current yaw angle [ rad ]
float32 v                                     # current speed [ m/s ]
bool batteryLow
bool timeout
float32[2] targetSegmentS                     # target segment display marker pos.

