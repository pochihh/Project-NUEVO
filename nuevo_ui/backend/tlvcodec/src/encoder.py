"""
Developed by Toby Chen, con mucho amor <3
Author Email: pc.toby.chen@gmail.com
Date: May 1, 2025
License: MIT
"""

## The TLV encoder class ##
# The encoder is used to encode the data into the TLV format
# The TLV format is a type-length-value format
# For one comeplete data frame, it contains a frame header and a list of arbitrary number of TLV packets. 
# The frame header is fixed and contains the device ID, frame number, and the number of TLV packets in the frame.
# The deviceId specifies the device that sends the data; the frame is incremented for each data frame encoded.
# The TLV packets are encoded with the type, length, and value., where the type specifies the type of the data, the length specifies the length of the data, and the value specifies the actual data.
# The length of the whole data frame will be rounded up to the nearest 8 bytes when the data frame is encoded.

import ctypes
from .utils import *
import binascii # for crc32
import struct

class Encoder:
    def __init__(self, deviceId, bufferSize=1024, crc=True):
        self.frameHeader = FrameHeader()
        self.tlvHeader = TlvHeader()
        self.buffer = bytearray(bufferSize)
        self.crc = crc

        # initialize the frame header
        for i in range(len(FRAME_HEADER_MAGIC_NUM)):
            self.frameHeader.magicNum[i] = FRAME_HEADER_MAGIC_NUM[i]

        self.frameHeader.numTotalBytes.value = ctypes.sizeof(self.frameHeader)
        self.frameHeader.checksum = 0
        self.frameHeader.deviceId = deviceId
        self.frameHeader.frameNum = 0
        self.frameHeader.numTlvs = 0


    # add a tlv packet to the buffer
    # to keep the encoder simple and efficient, the value is expected to be a ctypes object
    def addPacket(self, type, length, value):
        # fill the tlv header (temporary)
        self.tlvHeader.tlvType = ctypes.c_uint32(type)
        self.tlvHeader.tlvLen = ctypes.c_uint32(length)

        # copy tlv header to the buffer
        index = self.frameHeader.numTotalBytes.value
        length2add = ctypes.sizeof(self.tlvHeader)
        self.buffer[index:index+length2add] = ctypes.string_at(ctypes.addressof(self.tlvHeader), length2add)
        self.frameHeader.numTotalBytes.value += length2add

        # copy tlv value to the buffer
        index = self.frameHeader.numTotalBytes.value
        length2add = self.tlvHeader.tlvLen
        
        if length2add > 0:
            self.buffer[index:index+length2add] = ctypes.string_at(ctypes.addressof(value), length2add)
            self.frameHeader.numTotalBytes.value += length2add
    
        # update the number of tlvs
        self.frameHeader.numTlvs += 1
        

    def wrapupBuffer(self, deviceId=None, frameNum=None):
        ## Make the buffer size a multiple of 32 ##
        # get the current buffer size and calculate the number of padding bytes needed
        index = self.frameHeader.numTotalBytes.value
        length2add = NUM_PADDING_BYTES(self.frameHeader.numTotalBytes.value)

        # add zeos to the end of the buffer
        self.buffer[index:index+length2add] = b'\x00' * length2add

        # update total packet length
        self.frameHeader.numTotalBytes.value += length2add

        ## update frame header ##
        # if deviceId is not specified, use the deviceId of the encoder
        # if frameNum is not specified, use the frame number from the frame header
        if deviceId is not None:
            self.frameHeader.deviceId = deviceId
        if frameNum is not None:
            self.frameHeader.frameNum = frameNum

        # copy frame header to the descriptor buffer
        index = 0
        length2add = ctypes.sizeof(self.frameHeader)
        self.buffer[index:index+length2add] = ctypes.string_at(ctypes.addressof(self.frameHeader), length2add)
        
        # calculate crc32 value for the whole buffer except for the magic number and total packet length in the frame header
        if (self.crc == True):
            # calculate crc32 value for the whole buffer except for the magic number (8 bytes), total packet length (4 bytes), and crc itself (4 bytes) in the frame header
            crc32 = binascii.crc32(self.buffer[16:self.frameHeader.numTotalBytes.value])
            
            # copy the crc32 value to the buffer
            self.buffer[12:16] = struct.pack('I', crc32)


        return self.frameHeader.numTotalBytes.value, self.buffer

    # reset the encoder to the initial state
    # This funciton needs to be called when the encoder is used in a loop
    def reset(self):
        # reset total bytes and number of tlvs
        self.frameHeader.numTotalBytes.value = ctypes.sizeof(self.frameHeader)
        self.frameHeader.numTlvs = 0
        self.frameHeader.checksum = 0
        
        # increment the frame number
        self.frameHeader.frameNum = self.frameHeader.frameNum + 1

        # keep the rest values to keep the programefficient
