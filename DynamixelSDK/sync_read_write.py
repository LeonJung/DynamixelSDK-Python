#!/usr/bin/env python
# -*- coding: utf-8 -*-

################################################################################
# Copyright 2017 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
################################################################################

# Author: Ryu Woon Jung (Leon)

#
# *********     Sync Read and Sync Write Example      *********
#
#
# Available Dynamixel model on this example : All models using Protocol 2.0
# This example is tested with two Dynamixel PRO 54-200, and an USB2DYNAMIXEL
# Be sure that Dynamixel PRO properties are already set as %% ID : 1 / Baudnum : 1 (Baudrate : 57600)
#

import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from port_handler import *
from packet_handler import * 
from group_sync_write import *

# Control table address
ADDR_PRO_TORQUE_ENABLE      = 64               # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
DXL2_ID                     = 2                 # Dynamixel#1 ID : 2
BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB1'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 10           # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 4000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position


# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler().getPacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
# groupSyncRead = groupSyncRead(portHandler, packetHandler, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

# Open port
if portHandler.openPort():
    print "Succeeded to open the port"
else:
    print "Failed to open the port"
    print "Press any key to terminate..."
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print "Succeeded to change the baudrate"
else:
    print "Failed to change the baudrate"
    print "Press any key to terminate..."
    getch()
    quit()

# aa = []

# d = bytes(30)
# print d

# d = chr(3).encode()

# deg=180
# deg_b=deg.to_bytes(1, 'little')


# print ord(d)
# groupSyncWrite.makeParam()

# print groupSyncWrite.addParam(1, [0xff, 0x02, 0x04, 0x06])
# print groupSyncWrite.addParam(1, [0xff, 0x02, 0x05, 0x06])
# print groupSyncWrite.addParam(1, [0xff, 0x02, 0x04, 0x07])
# print groupSyncWrite.addParam(2, [0xfa, 0x01, 0x00, 0x06])
# print groupSyncWrite.removeParam(2)
# print groupSyncWrite.changeParam(1, [0xfa, 0x01, 0x00, 0x07])

# groupSyncWrite.makeParam()

# groupSyncWrite.clearParam()

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print packetHandler.getTxRxResult(dxl_comm_result)
elif dxl_error != 0:
    print packetHandler.getRxPacketError(dxl_error)
else:
    print "Dynamixel#%d has been successfully connected" % DXL1_ID

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print packetHandler.getTxRxResult(dxl_comm_result)
elif dxl_error != 0:
    print packetHandler.getRxPacketError(dxl_error)
else:
    print "Dynamixel#%d has been successfully connected" % DXL2_ID

while 1:
    print "Press any key to continue! (or press ESC to quit!)"
    if getch() == chr(0x1b):
        break

    # Allocate goal position value into byte array
    param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[index])), DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[index])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[index]))]

    # Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position)
    if dxl_addparam_result != True:
        print "[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID
        break

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position)
    if dxl_addparam_result != True:
        print "[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID
        break

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print packetHandler.getTxRxResult(dxl_comm_result)

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    # # Write goal position
    # dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_GOAL_POSITION, dxl_goal_position[index])
    # if dxl_comm_result != COMM_SUCCESS:
    #     print packetHandler.getTxRxResult(dxl_comm_result)
    # elif dxl_error != 0:
    #     print packetHandler.getRxPacketError(dxl_error)

    # while 1:
    #     # Read present position
    #     dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRO_PRESENT_POSITION)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print packetHandler.getTxRxResult(dxl_comm_result)
    #     elif dxl_error != 0:
    #         print packetHandler.getRxPacketError(dxl_error)

    #     print "[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID, dxl_goal_position[index], dxl_present_position)

    #     if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
    #         break

    # Change goal position
    if index == 0:
        index = 1
    else:
        index = 0    


# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print packetHandler.getTxRxResult(dxl_comm_result)
elif dxl_error != 0:
    print packetHandler.getRxPacketError(dxl_error)

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print packetHandler.getTxRxResult(dxl_comm_result)
elif dxl_error != 0:
    print packetHandler.getRxPacketError(dxl_error)

# Close port
portHandler.closePort()