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

def PacketHandler(protocol_version):
    if protocol_version == 1.0:
        from protocol1_packet_handler import *
        return Protocol1PacketHandler()
    elif protocol_version == 2.0:
        from protocol2_packet_handler import *
        return Protocol2PacketHandler()
    else:
        from protocol2_packet_handler import *
        return Protocol2PacketHandler()

# TODO: If something goes worse, those should be replaced with this
# packetHandler = PacketHandler.getPacketHandler(PROTOCOL_VERSION)

# class PacketHandler():
#     def getPacketHandler(self, protocol_version):
#         if protocol_version == 1.0:
#             return Protocol1PacketHandler()
#         elif protocol_version == 2.0:
#             return Protocol2PacketHandler()
#         else:
#             return Protocol2PacketHandler()