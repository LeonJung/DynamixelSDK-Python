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

class GroupBulkWrite:
    def __init__(self, port, ph):
        self.port = port
        self.ph = ph

        self.is_param_changed = False
        self.param = []
        self.param_length = 0
        self.data_list = {}

        self.clearParam()

    def makeParam(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_list.keys()) == 0:
            return

        self.param_length = 0
        for id in self.data_list:
            self.param_length = self.param_length + 1 + 2 + 2 + self.data_list[id][2]

        self.param = [0] * self.param_length

        idx = 0
        for id in self.data_list:
            if not self.data_list[id]:
                return

            self.param[idx] = id
            idx = idx + 1
            self.param[idx] = DXL_LOBYTE(self.data_list[id][1])
            idx = idx + 1
            self.param[idx] = DXL_HIBYTE(self.data_list[id][1])
            idx = idx + 1
            self.param[idx] = DXL_LOBYTE(self.data_list[id][2])
            idx = idx + 1
            self.param[idx] = DXL_HIBYTE(self.data_list[id][2])
            idx = idx + 1

            for c in range(0, self.data_list[id][2]):
                self.param[idx] = self.data_list[id][0][c]
                idx = idx + 1

        # print self.param

    def addParam(self, id, start_address, data_length, data):
        if self.ph.getProtocolVersion() == 1.0:
            return False

        if id in self.data_list: # id already exist
            return False
        
        self.data_list[id] = [data, start_address, data_length]
        
        # print self.data_list

        self.is_param_changed = True
        return True

    def removeParam(self, id):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if not id in self.data_list: # NOT exist
            return
        
        del self.data_list[id]

        # print self.data_list

        self.is_param_changed = True
    
    def changeParam(self, id, start_address, data_length, data):
        if self.ph.getProtocolVersion() == 1.0:
            return

        if not id in self.data_list: # NOT exist
            return False

        self.data_list[id] = [data, start_address, data_length]

        # print self.data_list

        self.is_param_changed = True
        return True

    def clearParam(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_list.keys()) == 0:
            return

        self.data_list.clear()

        # print self.data_list
        return

    def txPacket(self):
        if self.ph.getProtocolVersion() == 1.0 or len(self.data_list.keys()) == 0:
            return COMM_NOT_AVAILABLE
        
        if self.is_param_changed == True or len(param) == 0:
            self.makeParam()

        return self.ph.bulkWriteTxOnly(self.port, self.param, self.param_length)