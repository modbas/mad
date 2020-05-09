#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Mission Command
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
uint8 carid             # car id
uint32 lapCount         # number of laps on cyclic course
uint32 targetSegment    # target position of the car
float32 vmax            # maximum speed [ m/s ]
float32 xManeuverEnd    # target arc length [ m ]
bool autoMission        # auto mission on lost
