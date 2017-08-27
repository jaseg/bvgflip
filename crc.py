import ctypes
import os
lib = ctypes.CDLL(os.path.join(os.path.dirname(__file__), '_crc.so'))
lib.crc32.restype = ctypes.c_uint32
def crc(data):
    if type(data) is not bytes:
        raise TypeError('This only works with bytes')
    return lib.crc32(data, len(data))
