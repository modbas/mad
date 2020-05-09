#
# Mini-Auto-Drive MAD
#
# Copyright (C) 2020 Frank Traenkle
# http://www.modbas.de
#
# Car State from Onboard-Electronics over BLE
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
uint8 carid              # virtual car id (color)
float32 batteryVoltage   # [ V ]
bool batteryLow          # battery undervoltage reached
uint8 err                # error codes from BLE
uint8 ERR_NONE = 0
uint8 ERR_CALIB = 1
uint8 ERR_OVERCURRENT = 2
uint8 ERR_RN4020_DETECT_FIRMWARE = 3
uint8 ERR_STEER_CTR = 4
uint8 ERR_CARIN_MSG_WATCHDOG = 5
uint8 ERR_UNKNOWN = 6
uint8 comerr             # communication errors
uint8 COMERR_NONE = 0
uint8 COMERR_INPUTS_ACK_TIMEOUT = 1
uint8 COMERR_STATE_TIMEOUT = 2
bool calibFlag           # calibration done OK
uint8 state
uint8 STATE_CONNECTED = 0  # internal state: SC_CONNECTED
uint8 STATE_DISCONNECT = 1 # state message timeout or internal states: SC_OFF, SC_DISCONNECT, SC_BOOT, SC_CALIBRATION
uint8 STATE_ERROR = 2      # ACK message timeout or internal state: SC_ERROR (self healing, corresponds to SAFETYHALT)
uint8 STATE_DRIVE = 3      # internal state: SC_DRIVE
uint8 STATE_DRIVE_SLOW = 4 # internal state: SC_DRIVE_SLOW
uint8 STATE_CHARGE = 5     # internal state: SC_CHARGE
float32 motorCurrent       # [ A ]
uint8 ctr                  # state message counter
uint8 inputsAckCtr         # inputs ACK counter
