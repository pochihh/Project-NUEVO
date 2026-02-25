import sys
import os
sys.path.append('../..')
from tlvcodec import Encoder, Decoder

import ctypes

from dataTypes import DataType, get_data_type_struct
import struct

# test encoder
encoder = Encoder(1) # device id
# encoder.addPacket(DataType.CUSTOM_TYPE_1.value, ctypes.sizeof(get_data_type_struct(DataType.CUSTOM_TYPE_1)), ctypes.c_uint32(111))
# encoder.addPacket(DataType.CUSTOM_TYPE_2.value, ctypes.sizeof(ctypes.c_uint32), ctypes.c_uint32(222))
encoder.addPacket(101, 4, ctypes.c_uint32(777))
length, buffer = encoder.wrapupBuffer()

# print the buffer withe the length as hex
print("Encoded data:")
print(f'length: {length}')
for i in range(length):
    print(f'{buffer[i]:02x}', end=' ')
print("\n")


def decoder_callback(error, frameHeader, tlvs):
    print(f"Decoder_callback: {error}")
    
    # print out tlvs
    if error.value == 0:
        for i in range(frameHeader.numTlvs):
            if tlvs[i][0] == DataType.CUSTOM_TYPE_1.value:
                uint32_value = struct.unpack('<I', tlvs[i][2])[0]
                print(f'Type CUSTOM_TYPE_1: {uint32_value}')
            elif tlvs[i][0] == DataType.CUSTOM_TYPE_2.value:
                uint32_value = struct.unpack('<I', tlvs[i][2])[0]
                print(f'Type CUSTOM_TYPE_2: {uint32_value}')
            elif tlvs[i][0] == 123:
                uint32_value = struct.unpack('<I', tlvs[i][2])[0]
                print(f'Type 123: {uint32_value}')

# test decoder
decoder = Decoder(decoder_callback) # device id

decoder.decode(buffer[0:length])

