import ctypes
from enum import Enum

# define data types
class DataType(Enum):
    # 8-bit unsigned integer
    UINT8 = 0
    # 16-bit unsigned integer
    UINT16 = 1
    # 32-bit unsigned integer
    UINT32 = 2

    # Custom type
    CUSTOM_TYPE_1 = 101
    CUSTOM_TYPE_2 = 254
    CUSTOM_TYPE_3 = 255

def get_data_type_struct(data_type):
    if data_type == DataType.UINT8:
        return ctypes.c_uint8
    elif data_type == DataType.UINT16:
        return ctypes.c_uint16
    elif data_type == DataType.UINT32:
        return ctypes.c_uint32
    elif data_type == DataType.CUSTOM_TYPE_1:
        return ctypes.c_uint32
    elif data_type == DataType.CUSTOM_TYPE_2:
        return ctypes.c_uint32
    elif data_type == DataType.CUSTOM_TYPE_3:
        return ctypes.c_uint32