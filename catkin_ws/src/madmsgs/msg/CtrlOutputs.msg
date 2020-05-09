#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Controller Outputs
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
uint8 MODE_NOINIT = 0
uint8 MODE_INIT = 1
uint8 MODE_READY = 2
uint8 MODE_RUNNING = 3
uint8 MODE_STOP = 4
uint8 MODE_HALT = 5
uint8 mode
float32 xc
