from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort

import struct

# Set thrust setpoint
def send_thrust(cf, value):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.data = struct.pack("<Bf", 9, value)
    cf.send_packet(pk)


# Set angular rate setpoints
def send_rates(cf, roll, pitch, yaw, altitude):
    pk = CRTPPacket()
    pk.port = CRTPPort.COMMANDER_GENERIC
    pk.data = struct.pack("<Bffff", 8, roll, pitch, yaw, altitude)
    cf.send_packet(pk)
