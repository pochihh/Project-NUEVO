# some_file.py
import sys
# caution: path[0] is reserved for script path (or '' in REPL)
sys.path.insert(1, '/path/to/application/app/folder')
sys.path.append('../src/')
sys.path.append('../../..')


import os
from utils import *

f = FrameHeader()

f.numTotalBytes.value = 300
print(f.numTotalBytes.value)
print(f.numTotalBytes.payload[0])
print(f.numTotalBytes.payload[1])
print(f.numTotalBytes.payload[2])
print(f.numTotalBytes.payload[3])
    