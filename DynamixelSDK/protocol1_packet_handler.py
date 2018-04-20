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

TXPACKET_MAX_LEN = 4 * 1024
RXPACKET_MAX_LEN = 4 * 1024

# for Protocol 2.0 Packet
PKT_HEADER0 = 0
PKT_HEADER1 = 1
PKT_HEADER2 = 2
PKT_RESERVED = 3
PKT_ID = 4
PKT_LENGTH_L = 5
PKT_LENGTH_H = 6
PKT_INSTRUCTION = 7
PKT_ERROR = 8
PKT_PARAMETER0 = 8

# Protocol 2.0 Error bit
ERRNUM_RESULT_FAIL = 1       # Failed to process the instruction packet.
ERRNUM_INSTRUCTION = 2       # Instruction error
ERRNUM_CRC = 3       # CRC check error
ERRNUM_DATA_RANGE = 4       # Data range error
ERRNUM_DATA_LENGTH = 5       # Data length error
ERRNUM_DATA_LIMIT = 6       # Data limit error
ERRNUM_ACCESS = 7       # Access error

ERRBIT_ALERT = 128     # When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

class Protocol1PacketHandler(object):
    def updateCRC(self, crc_accum, data_blk_ptr, data_blk_size):
        crc_table = [0x0000,
            0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
            0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027,
            0x0022, 0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D,
            0x8077, 0x0072, 0x0050, 0x8055, 0x805F, 0x005A, 0x804B,
            0x004E, 0x0044, 0x8041, 0x80C3, 0x00C6, 0x00CC, 0x80C9,
            0x00D8, 0x80DD, 0x80D7, 0x00D2, 0x00F0, 0x80F5, 0x80FF,
            0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1, 0x00A0, 0x80A5,
            0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1, 0x8093,
            0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
            0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197,
            0x0192, 0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE,
            0x01A4, 0x81A1, 0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB,
            0x01FE, 0x01F4, 0x81F1, 0x81D3, 0x01D6, 0x01DC, 0x81D9,
            0x01C8, 0x81CD, 0x81C7, 0x01C2, 0x0140, 0x8145, 0x814F,
            0x014A, 0x815B, 0x015E, 0x0154, 0x8151, 0x8173, 0x0176,
            0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162, 0x8123,
            0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
            0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104,
            0x8101, 0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D,
            0x8317, 0x0312, 0x0330, 0x8335, 0x833F, 0x033A, 0x832B,
            0x032E, 0x0324, 0x8321, 0x0360, 0x8365, 0x836F, 0x036A,
            0x837B, 0x037E, 0x0374, 0x8371, 0x8353, 0x0356, 0x035C,
            0x8359, 0x0348, 0x834D, 0x8347, 0x0342, 0x03C0, 0x83C5,
            0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1, 0x83F3,
            0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
            0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7,
            0x03B2, 0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E,
            0x0384, 0x8381, 0x0280, 0x8285, 0x828F, 0x028A, 0x829B,
            0x029E, 0x0294, 0x8291, 0x82B3, 0x02B6, 0x02BC, 0x82B9,
            0x02A8, 0x82AD, 0x82A7, 0x02A2, 0x82E3, 0x02E6, 0x02EC,
            0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2, 0x02D0, 0x82D5,
            0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1, 0x8243,
            0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
            0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264,
            0x8261, 0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E,
            0x0234, 0x8231, 0x8213, 0x0216, 0x021C, 0x8219, 0x0208,
            0x820D, 0x8207, 0x0202]

        for j in range(0, data_blk_size):
            i = ((crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF
            # print("j = ", j, " i = ", i)
            # print("crc_table =", crc_table[i])
            crc_accum = ((crc_accum << 8) ^ crc_table[i]) & 0xFFFF
            # print("crc_accum : ", hex(crc_accum))

        return crc_accum

    def addStuffing(self, packet):
        pass

    def removeStuffing(self, packet):
        packet_length_in = DXL_MAKEWORD(packet[PKT_LENGTH_L], packet[PKT_LENGTH_H])
        packet_length_out = packet_length_in

        index = PKT_INSTRUCTION
        for i in range(0, (packet_length_in - 2)): # except CRC
            if (packet[i+PKT_INSTRUCTION] == 0xFD) and (packet[i+PKT_INSTRUCTION+1] == 0xFD) and (packet[i+PKT_INSTRUCTION-1] == 0xFF) and (packet[i+PKT_INSTRUCTION-2] == 0xFF):
                # FF FF FD FD
                packet_length_out = packet_length_out - 1
                i = i + 1
            
            packet[index] = packet[i + PKT_INSTRUCTION]
            index = index + 1
        
        packet[index] = packet[PKT_INSTRUCTION+packet_length_in - 2]
        packet[index + 1] = packet[PKT_INSTRUCTION+packet_length_in - 1]

        packet[PKT_LENGTH_L] = DXL_LOBYTE(packet_length_out)
        packet[PKT_LENGTH_H] = DXL_HIBYTE(packet_length_out)


    def txPacket(self, port, txpacket):
        total_packet_length = 0
        written_packet_length = 0

        if port.is_using:
            return COMM_PORT_BUSY
        port.is_using = True

        # byte stuffing for header
        self.addStuffing(txpacket)

        # check max packet length
        total_packet_length = DXL_MAKEWORD(txpacket[PKT_LENGTH_L], txpacket[PKT_LENGTH_H]) + 7
        # 7: HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H

        # print("txpacket[PKT_LENGTH_L]", txpacket[PKT_LENGTH_L], "txpacket[PKT_LENGTH_H]", txpacket[PKT_LENGTH_H])

        # print("total_packet_length = %d ", total_packet_length)

        if total_packet_length > TXPACKET_MAX_LEN:
            port.is_using = False
            return COMM_TX_ERROR

        # make packet header
        txpacket[PKT_HEADER0] = 0xFF
        txpacket[PKT_HEADER1] = 0xFF
        txpacket[PKT_HEADER2] = 0xFD
        txpacket[PKT_RESERVED] = 0x00

        # add CRC16
        crc = self.updateCRC(0, txpacket, total_packet_length - 2) # 2: CRC16
    
        # print("crc", hex(crc))

        txpacket[total_packet_length - 2] = DXL_LOBYTE(crc)
        txpacket[total_packet_length - 1] = DXL_HIBYTE(crc)

        # print("DXL_LOBYTE(crc)", DXL_LOBYTE(crc))
        # print("txpacket[total_packet_length - 2] : ", txpacket[total_packet_length - 2])
        # print("txpacket[total_packet_length - 1] : ", txpacket[total_packet_length - 1])

        # tx packet
        port.clearPort()
        written_packet_length = port.writePort(txpacket) # TODO: total_packet_length won't be sent
        if total_packet_length != written_packet_length:
            port.is_using = False
            return COMM_TX_FAIL
        
        return COMM_SUCCESS

    def rxPacket(self, port, rxpacket):
        result = COMM_TX_FAIL

        rx_length = 0
        wait_length = 11 # minimum length (HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H)

        while True:
            packet_read = port.readPort(wait_length - rx_length)

            # print("packet_read : ", packet_read)
            rxpacket += packet_read

            # rxpacket = rxpacket + packet_read

            # print("len : ", len(packet_read))

            # print("rxpacket : ", rxpacket)

            # print ("rxpacket ord : ", ord(rxpacket))

            rx_length = rx_length + len(packet_read)
            if rx_length >= wait_length:
                idx = 0

                # find packet header
                for idx in range(0, (rx_length - 3)):
                    print ("rxpacket[idx] :", rxpacket[idx], "rxpacket[idx + 1]", rxpacket[idx + 1], "rxpacket[idx + 2]", rxpacket[idx + 2], "rxpacket[idx + 3]", rxpacket[idx + 3])
                    if (rxpacket[idx] == 0xFF) and (rxpacket[idx + 1] == 0xFF) and (rxpacket[idx + 2] == 0xFD) and (rxpacket[idx + 3] != 0xFD):
                        break
                    
                if idx == 0:
                    if (rxpacket[PKT_RESERVED] != 0x00) or (rxpacket[PKT_ID] > 0xFC) or (DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) > RXPACKET_MAX_LEN) or (rxpacket[PKT_INSTRUCTION] != 0x55):
                        # remove the first byte in the packet
                        for s in range(0, (rx_length - 1)):
                            rxpacket[s] = rxpacket[1 + s]
                        
                        rx_length = rx_length - 1
                        continue
                    
                    if wait_length != (DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1):
                        wait_length = DXL_MAKEWORD(rxpacket[PKT_LENGTH_L], rxpacket[PKT_LENGTH_H]) + PKT_LENGTH_H + 1
                        continue

                    if rx_length < wait_length:
                        if port.isPacketTimeout() == True:
                            if rx_length == 0:
                                result = COMM_RX_TIMEOUT
                                print 'COMM_RX_TIMEOUT 0'
                            else:
                                result = COMM_RX_CORRUPT
                                print 'COMM_RX_CORRUPT 1'
                            
                            break
                        else:
                            continue
                    
                    crc = DXL_MAKEWORD(rxpacket[wait_length-2], rxpacket[wait_length-1])

                    crc2 = self.updateCRC(0, rxpacket, wait_length - 2)

                    print("crc :", crc, "/ crc2 :", crc2)

                    if self.updateCRC(0, rxpacket, wait_length - 2) == crc:
                        result = COMM_SUCCESS
                    else:
                        result = COMM_RX_CORRUPT
                        print 'COMM_RX_TIMEOUT 1'
                    break
                
                else:
                    # remove unnecessary packets
                    for s in range(0, (rx_length - idx)):
                        rxpacket[s] = rxpacket[idx + s]
                    
                    rx_length = rx_length - idx
            
            else:
                if port.isPacketTimeout() == True:
                    if rx_length == 0:
                        print 'COMM_RX_TIMEOUT 2'
                        result = COMM_RX_TIMEOUT
                    else:
                        print 'COMM_RX_CORRUPT 2'
                        result = COMM_RX_CORRUPT
                    
                    break
            
        port.is_using = False

        if result == COMM_SUCCESS:
            self.removeStuffing(rxpacket)

        return result, rxpacket

    # NOT for BulkRead / SyncRead instruction
    def txRxPacket(self, port, txpacket): # rxpacket, error
        rxpacket = []
        result = COMM_TX_FAIL

        # tx packet
        result = self.txPacket(port, txpacket)
        if result != COMM_SUCCESS:
            return result

        # (Instruction == BulkRead or SyncRead) == this function is not available.
        if txpacket[PKT_INSTRUCTION] == INST_BULK_READ or txpacket[PKT_INSTRUCTION] == INST_SYNC_READ:
            result = COMM_NOT_AVAILABLE

        # (ID == Broadcast ID) == no need to wait for status packet or not available.
        # (Instruction == action) == no need to wait for status packet
        if txpacket[PKT_ID] == BROADCAST_ID or txpacket[PKT_INSTRUCTION] == INST_ACTION:
            port.is_using = False
            return result

        print "finished"

        # set packet timeout
        if (txpacket[PKT_INSTRUCTION] == INST_READ):
            port.setPacketTimeout(DXL_MAKEWORD(txpacket[PKT_PARAMETER0+2], txpacket[PKT_PARAMETER0+3]) + 11)
        else:
            port.setPacketTimeout(11)
            # HEADER0 HEADER1 HEADER2 RESERVED ID LENGTH_L LENGTH_H INST ERROR CRC16_L CRC16_H

        # rx packet
        while True:
            result, rxpacket = self.rxPacket(port, rxpacket)

            print("result : ", result)
            print("rxpacket[PKT_ID] : ", rxpacket[PKT_ID])

            if result != COMM_SUCCESS or txpacket[PKT_ID] == rxpacket[PKT_ID]:
                # except (result == COMM_SUCCESS && txpacket[PKT_ID] != rxpacket[PKT_ID])
                break

        if result == COMM_SUCCESS and txpacket[PKT_ID] == rxpacket[PKT_ID]:
            # (result == COMM_SUCCESS && txpacket[PKT_ID] == rxpacket[PKT_ID])
            # if error != 0:
            error = rxpacket[PKT_ERROR]

        return result, rxpacket, error
        
    def ping(self, port, id):
        result = COMM_TX_FAIL

        txpacket = [0] * 10
        rxpacket = [0] * 14

        if id >= BROADCAST_ID:
            return COMM_NOT_AVAILABLE

        txpacket[PKT_ID] = id
        txpacket[PKT_LENGTH_L] = 3
        txpacket[PKT_LENGTH_H] = 0
        txpacket[PKT_INSTRUCTION] = INST_PING

        result, rxpacket, error = self.txRxPacket(port, txpacket) # rxpacket, error
        if result == COMM_SUCCESS:
            model_number = DXL_MAKEWORD(rxpacket[PKT_PARAMETER0 + 1], rxpacket[PKT_PARAMETER0 + 2])

        return model_number, result, error


    # def ping(self, port):
    #     arr = [0xff, 0xff, 0xfd, 0x00, 0x01, 0x03, 0x00, 0x01, 0x19, 0x4e]

    #     port.writePort(arr)