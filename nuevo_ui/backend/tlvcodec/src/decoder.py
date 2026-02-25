"""
Developed by Toby Chen, con mucho amor <3
Author Email: pc.toby.chen@gmail.com
Date: May 1, 2025
License: MIT
"""

## The TLV decoder class ##
# The decoder is used to decode the data from the TLV format
# A state machine is used to decode the data, so the decoder can decode the data in a streaming manner
# The TLV format is a type-length-value format
# For one comeplete data frame, it contains a frame header and a list of arbitrary number of TLV packets.
# The frame header is fixed and contains the device ID, frame number, and the number of TLV packets in the frame.

import ctypes
from .utils import *
import binascii # for crc32
import struct
from enum import Enum

class FrameDecodeState(Enum):
    Init = 0
    MagicNum = 1
    NumTotalBytes = 2
    WaitFullFrame = 3

class DecodeErrorCode(Enum):
    NoError = 0
    CrcError = 1
    BufferOutOfIndex = 2
    UnpackFrameHeaderError = 3
    TlvError = 4
    TlvLenError = 5

class TlvDecodeDescriptor:
    def __init__(self, bufferLen=1024):
        self.decodeState = FrameDecodeState.Init
        self.errorCode = DecodeErrorCode.NoError
        self.ofst = 0
        self.frameHeader = FrameHeader()
        self.tlvHeader = TlvHeader()
        self.buffer = bytearray(bufferLen)
        self.bufferIndex = 0

class Decoder:
    def __init__(self, callback, crc=True):
        self.callback = callback
        self.descriptor = TlvDecodeDescriptor()
        self.crc = crc

    def decode(self, bytes2decode):
        # decode the bytes one by one
        i = 0
        while i < len(bytes2decode):
            self.decodePacket(bytes2decode[i])
            i = i+1

    def decodePacket(self, Packet: bytes):
        if self.descriptor.decodeState == FrameDecodeState.Init:
            if Packet == FRAME_HEADER_MAGIC_NUM[0]:
                self.descriptor.buffer[self.descriptor.bufferIndex] = Packet
                self.descriptor.bufferIndex += 1
                self.descriptor.ofst = 1
                self.descriptor.decodeState = FrameDecodeState.MagicNum
        elif self.descriptor.decodeState == FrameDecodeState.MagicNum:
            if Packet == FRAME_HEADER_MAGIC_NUM[self.descriptor.ofst]:
                self.descriptor.buffer[self.descriptor.bufferIndex] = Packet
                self.descriptor.bufferIndex += 1
                self.descriptor.ofst += 1
                if self.descriptor.ofst >= len(FRAME_HEADER_MAGIC_NUM):
                    self.descriptor.ofst = 0
                    self.descriptor.decodeState = FrameDecodeState.NumTotalBytes
            else:
                self.resetDescriptor()
        elif self.descriptor.decodeState == FrameDecodeState.NumTotalBytes:
            self.descriptor.frameHeader.numTotalBytes.payload[self.descriptor.ofst] = Packet
            self.descriptor.bufferIndex += 1
            self.descriptor.ofst += 1
            if self.descriptor.ofst >= ctypes.sizeof(self.descriptor.frameHeader.numTotalBytes):
                self.descriptor.ofst = 0
                if (self.descriptor.bufferIndex >= self.descriptor.frameHeader.numTotalBytes.value):
                    self.descriptor.ofst = 0
                    tlvs = self.parseFrame()
                    self.callback(self.descriptor.errorCode, 
                    self.descriptor.frameHeader, tlvs)
                    self.resetDescriptor()
                else:
                    self.descriptor.decodeState = FrameDecodeState.WaitFullFrame
        elif self.descriptor.decodeState == FrameDecodeState.WaitFullFrame:
            self.descriptor.buffer[self.descriptor.bufferIndex] = Packet
            self.descriptor.bufferIndex += 1
            if self.descriptor.bufferIndex >= self.descriptor.frameHeader.numTotalBytes.value:
                tlvs = self.parseFrame()
                self.callback(self.descriptor.errorCode, self.descriptor.frameHeader, tlvs)
                self.resetDescriptor()
                
    def parseFrame(self):
        headerStruct = 'Q5I'
        tlvHeaderStruct = '2I'
        frameHeaderLen = struct.calcsize(headerStruct)
        tlvHeaderLength = struct.calcsize(tlvHeaderStruct)

        self.descriptor.errorCode = DecodeErrorCode.NoError
        frameData = self.descriptor.buffer[:self.descriptor.bufferIndex]

        tlvList = []
        # parse the frame header
        try:
            magicNum, numTotalBytes, checksum, deviceId, frameNum, numTlvs = struct.unpack(headerStruct, frameData[:frameHeaderLen])
        except:
            self.descriptor.errorCode = DecodeErrorCode.UnpackFrameHeaderError
            return tlvList
        
        # copy the frame header to the descriptor
        self.descriptor.frameHeader.checksum = checksum
        self.descriptor.frameHeader.deviceId = deviceId
        self.descriptor.frameHeader.frameNum = frameNum
        self.descriptor.frameHeader.numTlvs = numTlvs
        
        # check the frame length
        if self.descriptor.frameHeader.numTotalBytes.value > len(frameData):
            self.descriptor.errorCode = DecodeErrorCode.BufferOutOfIndex
            return tlvList
        
        # CRC32 Check
        if(self.crc):
            crc32_check = binascii.crc32(frameData[16:])
            
            if crc32_check != self.descriptor.frameHeader.checksum:
                print(f'CRC32 error: {crc32_check} != {self.descriptor.frameHeader.checksum}')
                self.descriptor.errorCode = DecodeErrorCode.CrcError
                return tlvList

        # parse the TLVs
        tlvData = frameData[frameHeaderLen:]
        for i in range(self.descriptor.frameHeader.numTlvs):
            try:
                tlvType, tlvLen = struct.unpack(tlvHeaderStruct, tlvData[:tlvHeaderLength])
            except:
                self.descriptor.errorCode = DecodeErrorCode.TlvError
                return tlvList
            tlvData = tlvData[tlvHeaderLength:]

            if tlvLen > len(tlvData):
                # [debug] print out data for debugging
                print(f'TLV length error: {tlvLen} > {len(tlvData)}')
                self.descriptor.errorCode = DecodeErrorCode.TlvLenError
                return tlvList
            tlvList.append((tlvType, tlvLen, tlvData[:tlvLen]))
            tlvData = tlvData[tlvLen:]
            
        return tlvList


    def resetDescriptor(self):  
        self.descriptor.decodeState = FrameDecodeState.Init
        self.descriptor.errorCode = DecodeErrorCode.NoError
        self.descriptor.ofst = 0
        self.descriptor.frameHeader = FrameHeader()
        self.descriptor.bufferIndex = 0