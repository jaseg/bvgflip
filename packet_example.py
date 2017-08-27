
import struct
import crc

def set_block(data):
    data = struct.pack('<BB8H', 0x23, 0, *data)
    checksum = crc.crc(bytes(x for i in data for x in (0, 0, 0, i)))
    ser.write(data)
    ser.write(struct.pack('<H', checksum&0xffff))
