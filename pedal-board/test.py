import can
import time

bus = can.interface.Bus('can0', bustype='socketcan')

lastTime = time.time()

while True:
    # if time.time() - lastTime > 0.1:
    #     msg = can.Message(arbitration_id=0x7B, is_remote_frame=True, data=None, extended_id=False)
    #     bus.send(msg)
    #     lastTime = time.time()
    msg_recv = bus.recv()
    if msg_recv.arbitration_id == 0x7A:
        print(f'p0: {msg_recv.data[0]}, p1: {msg_recv.data[1]}, s: {msg_recv.data[4]}')