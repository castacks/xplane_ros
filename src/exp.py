import struct

buffer = struct.pack("<4sxH", "CONN", 46009)
print(buffer)

result = struct.unpack("<4sxH", buffer)
print(result)