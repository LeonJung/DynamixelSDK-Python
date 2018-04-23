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
# from packet_handler import *

class GroupSyncWrite:
    def __init__(self, port, ph, start_address, data_length):
        self.port = port
        self.ph = ph
        self.start_address = start_address
        self.data_length = data_length

        self.is_param_changed = False
        self.param = []
        # self.id_list = []
        self.data_list = {}

    def makeParam(self):
        if not self.data_list:
            return

        self.param = [0] * (len(self.data_list.keys()) * (1 + self.data_length)) # ID(1) + DATA(data_length)

        idx = 0
        for id in self.data_list:
            if not self.data_list[id]:
                return

            self.param[idx] = id
            idx = idx + 1
            for c in range(0, self.data_length):
                self.param[idx] = self.data_list[id][c]
                idx = idx + 1

        print self.param

    def addParam(self, id, data):
        if id in self.data_list: # id already exist
            return False
        
        self.data_list[id] = data
        
        print self.data_list

        self.is_param_changed = True
        return True

    def removeParam(self, id):
        if not id in self.data_list: # NOT exist
            return
        
        del self.data_list[id]

        print self.data_list

        self.is_param_changed = True
    
    def changeParam(self, id, data):
        if not id in self.data_list: # NOT exist
            return False

        self.data_list[id] = data

        print self.data_list

        self.is_param_changed = True
        return True

    def clearParam(self):
        self.data_list.clear()

        print self.data_list
        return True

    def txPacket(self):
        if not self.data_list:
            return COMM_NOT_AVAILABLE
        
        if self.is_param_changed == True: # or param == 0
            self.makeParam()

        return self.ph.syncWriteTxOnly(self.port, self.start_address, self.data_length, self.param, len(self.data_list.keys()) * (1 + self.data_length))