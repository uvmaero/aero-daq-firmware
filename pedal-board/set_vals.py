import can

CAN_PORT = 'can0'

ID_BASE = 0x2D
ID_GET = ID_BASE + 6
ID_SET = ID_BASE + 7

bus = can.interface.Bus(CAN_PORT, bustype='socketcan')

print('Fully release pedal, then press enter')
input()
print('Reading position...')

# send message
msg_send = can.Message(arbitration_id=ID_GET, is_remote_frame=True, data=None, extended_id=False)
bus.send(msg_send)

# wait until message is received
msg_recv = bus.recv()
while msg_recv.arbitration_id != ID_GET:
    msg_recv = bus.recv()

pedal0_min = msg_recv.data[0] + (msg_recv.data[1] << 8)
pedal1_min = msg_recv.data[2] + (msg_recv.data[3] << 8)

print(f'Got position: p0: {pedal0_min}, p1: {pedal1_min}')
print()
print('Fully depress pedal, then press enter')
input()
print('Reading position...')

# send message
msg_send = can.Message(arbitration_id=ID_GET, is_remote_frame=True, data=None, extended_id=False)
bus.send(msg_send)

# wait until message is received
msg_recv = bus.recv()
while msg_recv.arbitration_id != ID_GET:
    msg_recv = bus.recv()

pedal0_max = msg_recv.data[0] + (msg_recv.data[1] << 8)
pedal1_max = msg_recv.data[2] + (msg_recv.data[3] << 8)

print(f'Got position: p0: {pedal0_max}, p1: {pedal1_max}')
print()
print('Writing values to EEPROM')

eeprom_data = [
    pedal0_min & 0xFF,
    pedal0_min >> 8,
    pedal0_max & 0xFF,
    pedal0_max >> 8,
    pedal1_min & 0xFF,
    pedal1_min >> 8,
    pedal1_max & 0xFF,
    pedal1_max >> 8
]

msg_send = can.Message(arbitration_id=ID_SET, data=eeprom_data, extended_id=False)
bus.send(msg_send)