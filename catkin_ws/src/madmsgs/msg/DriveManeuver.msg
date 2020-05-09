#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Driving Maneuver Command
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
uint8 carid # id of the car
uint32 breaksLen                              # total number of breaks
float32[] breaks                              # path spline breaks
uint32[] segments                             # segment id of each break
float32[] s1                                  # s1 coordinates
float32[] s2                                  # s2 coordinates
float32[] splineCoefs1                        # path spline coefficients of s1 coordinate
float32[] splineCoefs2                        # path spline coefficients of s2 coordinate
bool periodic                                 # spline is periodic or non-periodic
float32 vmax                                  # maximum speed
uint8 type                                    # type of maneuver
uint8 TYPE_HALT = 0
uint8 TYPE_PARK = 1
uint8 TYPE_CHARGE = 2
uint8 TYPE_PATHFOLLOW = 3
float32 xManeuverEnd                          # target arc length in case of PARK
uint32 lapCount                               # number of laps in case of PARK
bool disableLaneMonitor                       # switch off car lane monitor for safety stop
