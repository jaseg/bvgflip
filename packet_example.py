
import struct
import crc
import binascii

def set_block(ser, block, data):
    data = struct.pack('<BB8H', 0x23, block, *data)
    checksum = crc.crc(bytes(x for i in data for x in (0, 0, 0, i)))
    data += struct.pack('<H', checksum&0xffff)
    ser.write(data)
    print(binascii.hexlify(data))

def strobe(ser):
    data = bytes([0x42])
    checksum = crc.crc(bytes(x for i in data for x in (0, 0, 0, i)))
    data += struct.pack('<H', checksum&0xffff)
    ser.write(data)
    print(binascii.hexlify(data))
