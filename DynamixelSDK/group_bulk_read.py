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

class GroupBulkRead:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.last_result = False
        self.is_param_changed = False
        self.param = []
        self.data_list = {}

        self.clearParam()

    def makeParam(self):
        if len(self.data_list.keys()) == 0:
            return

        if self.ph.getProtocolVersion() == 1.0:
            self.param = [0] * (len(self.data_list.keys()) * 3) # ID(1) + ADDR(1) + LENGTH(1)
        else:
            self.param = [0] * (len(self.data_list.keys()) * 5) # ID(1) + ADDR(2) + LENGTH(2)
            
        idx = 0
        for id in self.data_list:
            if self.ph.getProtocolVersion() == 1.0:
                self.param[idx] = self.data_list[id][2]             # LEN
                idx = idx + 1                
                self.param[idx] = id                                # ID
                idx = idx + 1    
                self.param[idx] = self.data_list[id][1]             # ADDR
                idx = idx + 1    
            else:
                self.param[idx] = id                                # ID
                idx = idx + 1                
                self.param[idx] = DXL_LOBYTE(self.data_list[id][1]) # ADDR_L
                idx = idx + 1    
                self.param[idx] = DXL_HIBYTE(self.data_list[id][1]) # ADDR_H
                idx = idx + 1    
                self.param[idx] = DXL_LOBYTE(self.data_list[id][2]) # LEN_L
                idx = idx + 1    
                self.param[idx] = DXL_HIBYTE(self.data_list[id][2]) # LEN_H
                idx = idx + 1   

    def addParam(self, id, start_address, data_length):
        if id in self.data_list: # id already exist
            return False
        
        data = [0] * data_length
        self.data_list[id] = [data, start_address, data_length]
        
        print self.data_list

        self.is_param_changed = True
        return True

    def removeParam(self, id):
        if not id in self.data_list: # NOT exist
            return
        
        del self.data_list[id]

        print self.data_list

        self.is_param_changed = True
    
    def clearParam(self):
        if len(self.data_list.keys()) == 0:
            return

        self.data_list.clear()

        print self.data_list
        return

    def txPacket(self):
        if len(self.data_list.keys()) == 0:
            return COMM_NOT_AVAILABLE
        
        if self.is_param_changed == True or len(self.param) == 0:
            self.makeParam()

        if self.ph.getProtocolVersion() == 1.0:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_list.keys()) * 3)
        else:
            return self.ph.bulkReadTx(self.port, self.param, len(self.data_list.keys()) * 5)

    def rxPacket(self):
        self.last_result = False

        result = COMM_RX_FAIL

        if len(self.data_list.keys()) == 0:
            return COMM_NOT_AVAILABLE

        for id in self.data_list:
            self.data_list[id][0], result, _ = self.ph.readRx(self.port, id, self.data_list[id][2], self.data_list[id][0])
            if result != COMM_SUCCESS:
                return result

        if result == COMM_SUCCESS:
            self.last_result = True

        return result

    def txRxPacket(self):        
        result = COMM_TX_FAIL

        result = self.txPacket()
        if result != COMM_SUCCESS:
            return result

        return self.rxPacket()

    def isAvailable(self, id, address, data_length):
        if self.last_result == False or not id in self.data_list:
            return False

        start_addr = self.data_list[id][1]

        if (address < start_addr) or (start_addr + self.data_list[id][2] - data_length < address):
            return False

        return True
    
    def getData(self, id, address, data_length):
        if self.isAvailable(id, address, data_length) == False:
            return 0

        start_addr = self.data_list[id][1]

        if data_length == 1:
            return self.data_list[id][0][address - start_addr]
        elif data_length == 2:
            return DXL_MAKEWORD(self.data_list[id][0][address - start_addr], self.data_list[id][0][address - start_addr + 1])
        elif data_length == 4:
            return DXL_MAKEDWORD(DXL_MAKEWORD(self.data_list[id][0][address - start_addr + 0], self.data_list[id][0][address - start_addr + 1]), DXL_MAKEWORD(self.data_list[id][0][address - start_addr + 2], self.data_list[id][0][address - start_addr + 3]))
        else:
            return 0