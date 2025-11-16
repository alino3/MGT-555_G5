import InverseKinematics as IK
#import YOLO as y

import serial
import struct
import time

# adjust port/baud to your Arduino
port = 'COM5'  # Change to your port
ser = serial.Serial(port, 115200, timeout=1)
time.sleep(2)

def send_and_listen(pulses1, dir1, pulses2, dir2, pulses3, dir3):
    # Pack and send data
    data = struct.pack('>iiiBBB', pulses1, pulses2, pulses3, dir1, dir2, dir3)
    ser.write(b'\x01' + data)
    
    print(f"Sent: M1={pulses1}(dir:{dir1}), M2={pulses2}(dir:{dir2}), M3={pulses3}(dir:{dir3})")
    
    # Listen for responses
    print("Arduino responses:")
    time.sleep(1)
    while ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        print(f"  {response}")
def send_steps(s1, s2, s3):

    ser = None
    if ser is None:
        print("Serial port not open")
        return
    cmd = f"G {s1} {s2} {s3}\n".encode('ascii')
    ser.write(cmd)
    resp = ser.readline().decode().strip()
    print("Arduino:", resp)

# -----------------------------
# Example: move to (x, y) while keeping orientation
try:
    x, y, phi_desired = 0.10, 0.10, 0.0
    t1, t2 = IK.ik_scara(x, y)
    t3 = IK.end_effector(t1[0], t2[0], phi_desired=0)
    print(t1, t2, t3)
    s1, d1, s2, d2, s3, d3 = IK.angles_to_steps(t1[0], t2[0], t3)
    print("Sending steps:", s1, s2, s3)
    send_and_listen(s1, d1, s2, d2, s3, d3)
    print(s1, s2, s3)
except ValueError as e:
    print("IK error:", e)