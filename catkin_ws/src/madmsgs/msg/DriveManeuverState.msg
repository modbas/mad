#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Driving Maneuver State
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
uint8 carid                                   # id of the car
uint8 state                                   # type of maneuver
uint8 STATE_WAITING = 0
uint8 STATE_SAFETYHALT = 1
uint8 STATE_SAFETYSTOP = 2
uint8 STATE_PARKING = 3                       # display = DRIVING
uint8 STATE_HALT = 4
uint8 STATE_CHARGING = 5
uint8 STATE_LOCATING = 6
uint8 STATE_BOOTING = 7
uint8 STATE_ERROR = 8
float32 epsi                                  # yaw angle control deviation [ rad ]
float32 ex                                    # longitudinal control deviation [ m ]
float32 ey                                    # lateral control deviation [ m ]
float32[2] s                                  # cartesian center position [ m ]
float32 x                                     # current arc length [ m ]
float32 v                                     # speed [ m/s ]
bool batteryLow                               # equals to batteryLow of CarState
bool timeout                                  # timeout in maneuver execution
int32 segment                                 # current segment id (if no spline has been received then segment = -1)
uint32 lap                                    # current lap
