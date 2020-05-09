#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Controller Reference Signals
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
uint8 carid
float32[2] s     # cartesian center position [ m ]
float32 psi      # yaw angle [ rad ]
float32 kappa    # curvature [ 1/m ]
float32 v        # center point velocity [ m/s ]
float32 x        # driven arc length of center [ m ]

