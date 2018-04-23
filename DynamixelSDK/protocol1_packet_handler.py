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

from robotis_def import *

TXPACKET_MAX_LEN        = 250
RXPACKET_MAX_LEN        = 250

# for Protocol 1.0 Packet
PKT_HEADER0             = 0
PKT_HEADER1             = 1
PKT_ID                  = 2
PKT_LENGTH              = 3
PKT_INSTRUCTION         = 4
PKT_ERROR               = 4
PKT_PARAMETER0          = 5

# Protocol 1.0 Error bit
ERRBIT_VOLTAGE          = 1       # Supplied voltage is out of the range (operating volatage set in the control table)
ERRBIT_ANGLE            = 2       # Goal position is written out of the range (from CW angle limit to CCW angle limit)
ERRBIT_OVERHEAT         = 4       # Temperature is out of the range (operating temperature set in the control table)
ERRBIT_RANGE            = 8       # Command(setting value) is out of the range for use.
ERRBIT_CHECKSUM         = 16      # Instruction packet checksum is incorrect.
ERRBIT_OVERLOAD         = 32      # The current load cannot be controlled by the set torque.
ERRBIT_INSTRUCTION      = 64      # Undefined instruction or delivering the action command without the reg_write command.

class Protocol2PacketHandler(object):
    def getTxRxResult(self, result):
        if result == COMM_SUCCESS:
            return "[TxRxResult] Communication success."
        elif result == COMM_PORT_BUSY:
            return "[TxRxResult] Port is in use!"
        elif result == COMM_TX_FAIL:
            return "[TxRxResult] Failed transmit instruction packet!"
        elif result == COMM_RX_FAIL:
            return "[TxRxResult] Failed get status packet from device!"
        elif result == COMM_TX_ERROR:
            return "[TxRxResult] Incorrect instruction packet!"
        elif result == COMM_RX_WAITING:
            return "[TxRxResult] Now recieving status packet!"
        elif result == COMM_RX_TIMEOUT:
            return "[TxRxResult] There is no status packet!"
        elif result == COMM_RX_CORRUPT:
            return "[TxRxResult] Incorrect status packet!"
        elif result == COMM_NOT_AVAILABLE:
            return "[TxRxResult] Protocol does not support This function!"
        else:
            return ""

    def getRxPacketError(self, error):
        if error & ERRBIT_VOLTAGE:
            return "[RxPacketError] Input voltage error!"

        if error & ERRBIT_ANGLE:
            return "[RxPacketError] Angle limit error!"

        if error & ERRBIT_OVERHEAT:
            return "[RxPacketError] Overheat error!"

        if error & ERRBIT_RANGE:
            return "[RxPacketError] Out of range error!"

        if error & ERRBIT_CHECKSUM:
            return "[RxPacketError] Checksum error!"

        if error & ERRBIT_OVERLOAD:
            return "[RxPacketError] Overload error!"

        if error & ERRBIT_INSTRUCTION:
            return "[RxPacketError] Instruction code error!"

        return ""

    def txPacket(self, port, txpacket):
        checksum                = 0
        total_packet_length     = txpacket[PKT_LENGTH] + 4 # 4: HEADER0 HEADER1 ID LENGTH
        written_packet_length   = 0

        if port.is_using:
            return COMM_PORT_BUSY
        port.is_using = True

        # check max packet length
        if total_packet_length > TXPACKET_MAX_LEN:
            port.is_using = False
            return COMM_TX_ERROR

        # make packet header
        txpacket[PKT_HEADER0]   = 0xFF
        txpacket[PKT_HEADER1]   = 0xFF

        # add a checksum to the packet
        for idx in range(2, total_packet_length - 1): # except header, checksum
            checksum = checksum + txpacket[idx]
        txpacket[total_packet_length - 1] = ~checksum

        # tx packet
        port.clearPort()
        written_packet_length = port.writePort(txpacket) # TODO: total_packet_length
        if total_packet_length != written_packet_length:
            port.is_using = False
            return COMM_TX_FAIL

        return COMM_SUCCESS

    def rxPacket(self, port, rxpacket=[]):
        result      = COMM_TX_FAIL

        checksum    = 0
        rx_length   = 0
        wait_length = 6 # minimum length (HEADER0 HEADER1 ID LENGTH ERROR CHKSUM)

        while True:
            packet_read = port.readPort(wait_length - rx_length)
            rx_packet += packet_read

            rx_length = rx_length + len(packet_read)
            if rx_length >= wait_length:
                idx = 0

                # find packet header
                for idx in range(0, rx_length - 1):
                    if rxpacket[idx] == 0xFF and rxpacket[idx + 1] == 0xFF:
                        break

                if idx == 0:
                    if (rxpacket[PKT_ID] > 0xFD) or (rxpacket(PKT_LENGTH) > RXPACKET_MAX_LEN) or (rxpacket[PKT_ERROR] > 0x7F):
                    # unavailable ID or unavailable Length or unavailable Error
                        #remove the first byte in the packet
                        for s in range(0, rx_length - 1):
                            rxpacket[s] = rxpacket[1 + s]
                            # memcpy(&rxpacket[0], &rxpacket[idx], rx_length - idx)
                        rx_length = rx_length - 1
                        continue

                    # re-calculate the exact length of the rx packet
                    if wait_length != rxpacket[PKT_LENGTH] + PKT_LENGTH + 1:
                        wait_length = rxpacket[PKT_LENGTH] + PKT_LENGTH + 1
                        continue

                    if rx_length < wait_length:
                        # check timeout
                        if port.isPacketTimeout() == True:
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                            else:
                                result = COMM_RX_CORRUPT
                            break
                        else:
                            continue

                    # calculate checksum
                    for i in range(2, wait_length - 1): # except header, checksum
                        checksum = checksum + rxpacket[i]
                    checksum = ~checksum

                    # verify checksum
                    if rxpacket[wait_length - 1] == checksum:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                    break
                else:
                    # check timeout
                    if port.isPacketTimeout() == True:
                        if rx_length == 0:
                            result = COMM_RX_TIMEOUT
                        else:
                            result = COMM_RX_CORRUPT
                        break
                
        port.is_using = False

        return rxpacket, result

    # NOT for BulkRead instruction
    def txRxPacket(self, port, txpacket, rxpacket=[], error=0):
        result = COMM_TX_FAIL

        # tx packet
        result = txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            return rxpacket, result, error
        
        # (Instruction == BulkRead) == this function is not available.
        if txpacket[PKT_INSTRUCTION] == INST_BULK_READ:
            result = COMM_NOT_AVAILABLE
                
        # (ID == Broadcast ID) == no need to wait for status packet or not available
        # (Instruction == action) == no need to wait for status packet
        if txpacket[PKT_ID] == BROADCAST_ID or txpacket[PKT_INSTRUCTION] == INST_ACTION:
            port.is_using = False
            return result

        # set packet timeout
        if txpacket[PKT_INSTRUCTION] == INST_READ:
            port.setPacketTimeout(txpacket[PKT_PARAMETER0+1] + 6)
        else:
            port.setPacketTimeout(6) # HEADER0 HEADER1 ID LENGTH ERROR CHECKSUM

        # rx packet
        while True:
            rxpacket, result = self.rxPacket(port, rxpacket)

            print("result : ", result)
            # print("rxpacket[PKT_ID] : ", rxpacket[PKT_ID])

            if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                break

        if result == COMM_SUCCESS and txpacket[PKT_ID] == rxpacket[PKT_ID]:
            if error != 0:
                error = rxpacket[PKT_ERROR]

        return rxpacket, result, error
        
    def ping(self, port, id, model_number=0, error=0):
        result = COMM_TX_FAIL

        txpacket = [0] * 6
        rxpacket = []

        if id >= BROADCAST_ID:
            return model_number, COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID] = id
        txpacket[PKT_LENGTH] = 2
        txpacket[PKT_INSTRUCTION] = INST_PING

        rxpacket, result, error = self.txRxPacket(port, txpacket, rxpacket, error)
        if result == COMM_SUCCESS:
            data_read = [0] * 2
            data_read, result, error = self.readTxRx(port, id, 0, 2, data_read) # Address 0 : Model Number
            if result == COMM_SUCCESS:
                model_number = DXL_MAKEWORD(data_read[0], data_read[1])

        return model_number, result, error


#######################################################

    def broadcastPing(self, port, id_list=[]):
        STATUS_LENGTH   = 14
        result          = COMM_TX_FAIL

        rx_length       = 0
        wait_length     = STATUS_LENGTH * MAX_ID

        txpacket        = [0] * 10
        rxpacket        = []

        txpacket[PKT_ID]            = BROADCAST_ID
        txpacket[PKT_LENGTH_L]      = 3
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_PING

        result = self.txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            port.is_using = False
            return id_list, result

        # set rx timeout
        port.setPacketTimeout(wait_length * 1)

        while True:
            packet_read = port.readPort(wait_length - rx_length)
            rxpacket += packet_read

            rx_length = rx_length + len(packet_read)
            if port.isPacketTimeout() == True: # or rx_length >= wait_length
                break

        port.is_using = False

        if rx_length == 0:
            return id_list, COMM_RX_TIMEOUT

        while True:
            if rx_length < STATUS_LENGTH:
                return id_list, COMM_RX_CORRUPT

            # find packet header
            for idx in range(0, rx_length - 2):
                if rxpacket[idx] == 0xFF and rxpacket[idx+1] == 0xFF and rxpacket[idx+2] == 0xFD:
                    break

            if idx == 0: # found at the beginning of the packet
                # verify CRC16
                crc = DXL_MAKEWORD(rxpacket[STATUS_LENGTH-2], rxpacket[STATUS_LENGTH-1])

                if self.updateCRC(0, rxpacket, STATUS_LENGTH - 2) == crc:
                    result = COMM_SUCCESS

                    id_list.extend(rxpacket[PKT_ID])

                    for s in range(0, rx_length - STATUS_LENGTH):
                        rxpacket[s] = rxpacket[STATUS_LENGTH + s]
                    
                    rx_length = rx_length - STATUS_LENGTH

                    if rx_length == 0:
                        return id_list, result
                
                else:
                    result = COMM_RX_CORRUPT

                    # remove header (0xFF 0xFF 0xFD)
                    for s in range(0, rx_length - 3):
                        rxpacket[s] = rxpacket[3 + s]
                    rx_length = rx_length - 3

            else:
                # remove unnecessary packets
                for s in range(0, rx_length - idx):
                    rxpacket[s] - rxpacket[idx + s]
                rx_length = rx_length - idx

        
        return id_list, result

    def action(self, port, id):
        txpacket                    = [0] * 10

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = 3
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_ACTION

        return self.txRxPacket(port, txpacket)

    def reboot(self, port, id, error=0):
        txpacket                    = [0] * 10
        rxpacket                    = []

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = 3
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_REBOOT

        return self.txRxPacket(port, txpacket, rxpacket, error)

    def factoryReset(self, port, id, option, error=0):
        txpacket                    = [0] * 11
        rxpacket                    = []

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = 4
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_FACTORY_RESET
        txpacket[PKT_PARAMETER0]    = option

        return self.txRxPacket(port, txpacket, rxpacket, error)

    def readTx(self, port, id, address, length):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * 14

        if id >= BROADCAST_ID:
            return COMM_NOT_AVAILABLE

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = 7
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_READ
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)
        txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(length)
        txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(length)        

        result = self.txPacket(port, txpacket)

        # set packet timeout
        if (result == COMM_SUCCESS):
            port.setPacketTimeout(length + 11)

        return result

    def readRx(self, port, id, length, data=[], error=0):
        result                      = COMM_TX_FAIL

        rxpacket                    = []

        while True:
            rxpacket, result = self.rxPacket(port, rxpacket)

            if result != COMM_SUCCESS or rxpacket[PKT_ID] == id:
                break

        if result == COMM_SUCCESS and rxpacket[PKT_ID] == id:
            if error != 0:
                error = rxpacket[PKT_ERROR]

            for s in range(0, length):
                data[s] = rxpacket[PKT_PARAMETER0 + 1 + s]
                # memcpy(data, &rxpacket[PKT_PARAMETER0+1], length);
        
        del rxpacket[:]; del rxpacket
        return data, result, error

    def readTxRx(self, port, id, address, length, data=[], error=0):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * 14
        rxpacket                    = []
        # (length + 11 + (length/3));  # (length/3): consider stuffing

        if id >= BROADCAST_ID:
            return data, COMM_NOT_AVAILABLE, error

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = 7
        txpacket[PKT_LENGTH_H]      = 0
        txpacket[PKT_INSTRUCTION]   = INST_READ
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)
        txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(length)
        txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(length)        

        rxpacket, result, error = self.txRxPacket(port, txpacket, rxpacket, error)
        if result == COMM_SUCCESS:
            if error != 0:
                error = rxpacket[PKT_ERROR]
            
            for s in range(0, length):
                data[s] = rxpacket[PKT_PARAMETER0 + 1 + s]
            
        del rxpacket[:]; del rxpacket
        return data, result, error        

    def read1ByteTx(self, port, id, address):
        return self.readTx(port, id, address, 1)
    def read1ByteRx(self, port, id, data=[], error=0):
        data_read = [0]
        data_read, result, error = self.readRx(port, id, 1, data_read, error)
        if result == COMM_SUCCESS:
            data = data_read[0]
        return data, result, error
    def read1ByteTxRx(self, port, id, address, data=[], error=0):
        data_read = [0]
        data_read, result, error = self.readTxRx(port, id, address, 1, data_read, error)
        if result == COMM_SUCCESS:
            data = data_read[0]
        return data, result, error

    def read2ByteTx(self, port, id, address):
        return self.readTx(port, id, address, 2)
    def read2ByteRx(self, port, id, data=[], error=0):
        data_read = [0] * 2
        data_read, result, error = self.readRx(port, id, 2, data_read, error)
        if result == COMM_SUCCESS:
            data = DXL_MAKEWORD(data_read[0], data_read[1])        
        return data, result, error
    def read2ByteTxRx(self, port, id, address, data=[], error=0):
        data_read = [0] * 2
        data_read, result, error = self.readTxRx(port, id, address, 2, data_read, error)
        if result == COMM_SUCCESS:
            data = DXL_MAKEWORD(data_read[0], data_read[1])
        return data, result, error

    def read4ByteTx(self, port, id, address):
        return self.readTx(port, id, address, 4)
    def read4ByteRx(self, port, id, data=[], error=0):
        data_read = [0] * 4
        data_read, result, error = self.readRx(port, id, 4, data_read, error)
        if result == COMM_SUCCESS:
            data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]))
        return data, result, error
    def read4ByteTxRx(self, port, id, address, data=[], error=0):
        data_read = [0] * 4
        data_read, result, error = self.readTxRx(port, id, address, 4, data_read, error)
        if result == COMM_SUCCESS:
            data = DXL_MAKEDWORD(DXL_MAKEWORD(data_read[0], data_read[1]), DXL_MAKEWORD(data_read[2], data_read[3]))   
        return data, result, error


    def writeTxOnly(self, port, id, address, length, data):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (length + 12)

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5)
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5)
        txpacket[PKT_INSTRUCTION]   = INST_WRITE
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)

        for s in range(0, length):
            txpacket[PKT_PARAMETER0+2+s] = data[s]
            # memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

        result = self.txPacket(port, txpacket)
        port.is_using = False

        del txpacket[:]; del txpacket
        return result

    def writeTxRx(self, port, id, address, length, data, error=0): 
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (length + 12)
        rxpacket                    = []
        
        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5)
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5)
        txpacket[PKT_INSTRUCTION]   = INST_WRITE
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)

        for s in range(0, length):
            txpacket[PKT_PARAMETER0+2+s] = data[s]
            # memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

        rxpacket, result, error = self.txRxPacket(port, txpacket, rxpacket, error)

        del txpacket[:]; del txpacket
        return result, error

    def write1ByteTxOnly(self, port, id, address, data):
        data_write = [data]
        return self.writeTxOnly(port, id, address, 1, data_write)
    def write1ByteTxRx(self, port, id, address, data, error=0):
        data_write = [data]
        return self.writeTxRx(port, id, address, 1, data_write)

    def write2ByteTxOnly(self, port, id, address, data):
        data_write = [DXL_LOBYTE(data), DXL_HIBYTE(data)]
        return self.writeTxOnly(port, id, address, 2, data_write)
    def write2ByteTxRx(self, port, id, address, data, error=0):
        data_write = [DXL_LOBYTE(data), DXL_HIBYTE(data)]
        return self.writeTxRx(port, id, address, 2, data_write)

    def write4ByteTxOnly(self, port, id, address, data):
        data_write = [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]
        return self.writeTxonly(port, id, address, 4, data_write)
    def write4ByteTxRx(self, port, id, address, data, error=0):
        data_write = [DXL_LOBYTE(DXL_LOWORD(data)), DXL_HIBYTE(DXL_LOWORD(data)), DXL_LOBYTE(DXL_HIWORD(data)), DXL_HIBYTE(DXL_HIWORD(data))]
        return self.writeTxRx(port, id, address, 4, data_write)

    def regWriteTxOnly(self, port, id, address, length, data):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (length + 12)

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5)
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5)
        txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)
        
        for s in range(0, length):
            txpacket[PKT_PARAMETER0+2+s] = data[s]

        result = self.txPacket(port, txpacket)
        port.is_using = False

        del txpacket[:]; del txpacket
        return result

    def regWriteTxRx(self, port, id, address, length, data, error=0):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (length + 12)
        rxpacket                    = []

        txpacket[PKT_ID]            = id
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(length+5)
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(length+5)
        txpacket[PKT_INSTRUCTION]   = INST_REG_WRITE
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(address)        

        for s in range(0, length):
            txpacket[PKT_PARAMETER0+2+s] = data[s]
            # memcpy(&txpacket[PKT_PARAMETER0+2], data, length);

        _, result, error = self.txRxPacket(port, txpacket, rxpacket, error)

        del txpacket[:]; del txpacket
        return result, error

    def syncReadTx(self, port, start_address, data_length, param, param_length):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (param_length + 14) 
        # 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        
        txpacket[PKT_ID]            = BROADCAST_ID
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7) # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7) # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION]   = INST_SYNC_READ
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address)
        txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length)
        txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length)
        
        for s in range(0, param_length):
            txpacket[PKT_PARAMETER0+4+s] = param[s]
            # memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

        result = self.txPacket(port, txpacket)
        if result == COMM_SUCCESS:
            port.setPacketTimeout((11 + data_length) * param_length)

        del txpacket[:]; del txpacket
        return param, result

    def syncWriteTxOnly(self, port, start_address, data_length, param, param_length):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (param_length + 14) 
        # 14: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H

        txpacket[PKT_ID]            = BROADCAST_ID
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 7) # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 7) # 7: INST START_ADDR_L START_ADDR_H DATA_LEN_L DATA_LEN_H CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION]   = INST_SYNC_WRITE
        txpacket[PKT_PARAMETER0+0]  = DXL_LOBYTE(start_address)
        txpacket[PKT_PARAMETER0+1]  = DXL_HIBYTE(start_address)
        txpacket[PKT_PARAMETER0+2]  = DXL_LOBYTE(data_length)
        txpacket[PKT_PARAMETER0+3]  = DXL_HIBYTE(data_length)
        
        for s in range(0, param_length):
            txpacket[PKT_PARAMETER0+4+s] = param[s]
            # memcpy(&txpacket[PKT_PARAMETER0+4], param, param_length);

        result = self.txRxPacket(port, txpacket)

        del txpacket[:]; del txpacket
        return result

    def bulkReadTx(self, port, param, param_length):
        result                      = COMM_TX_FAIL

        txpacket                    = [0] * (param_length + 10)
        # 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

        txpacket[PKT_ID]            = BROADCAST_ID
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 3) # 3: INST CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 3) # 3: INST CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION]   = INST_BULK_READ

        for s in range(0, param_length):
            txpacket[PKT_PARAMETER0+s] = param[s]
            #memcpy(&txpacket[PKT_PARAMETER0], param, param_length);

        result = self.txPacket(port, txpacket)

        if result == COMM_SUCCESS:
            wait_length = 0
            i = 0
            while i < param_length:
                wait_length = wait_length + DXL_MAKEWORD(param[i+3], param[i+4]) + 10
                i = i + 5
            port.setPacketTimeout(wait_length)

        del txpacket[:]; del txpacket
        return result

    def bulkWriteTxOnly(self, port, param, param_length):
        result                  = COMM_TX_FAIL

        txpacket                = [0] * (param_length + 10)
        # 10: HEADER0 HEADER1 HEADER2 RESERVED ID LEN_L LEN_H INST CRC16_L CRC16_H

        txpacket[PKT_ID]            = BROADCAST_ID
        txpacket[PKT_LENGTH_L]      = DXL_LOBYTE(param_length + 3) # 3: INST CRC16_L CRC16_H
        txpacket[PKT_LENGTH_H]      = DXL_HIBYTE(param_length + 3) # 3: INST CRC16_L CRC16_H
        txpacket[PKT_INSTRUCTION]   = INST_BULK_WRITE

        for s in range(0, param_length):
            txpacket[PKT_PARAMETER0+s] = param[s]
            # memcpy(&txpacket[PKT_PARAMETER0], param, param_length);

        result = self.txRxPacket(port, txpacket)

        del txpacket[:]; del txpacket
        return result