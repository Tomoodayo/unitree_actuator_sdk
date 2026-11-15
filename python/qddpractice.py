import time
import sys
sys.path.append('../lib')
from unitree_actuator_sdk import *


serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

while True:
    data.motorType = MotorType.GO_M8010_6
    cmd.motorType = MotorType.GO_M8010_6
    cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    cmd.id   = 1
    cmd.q    = 3.14*6.33
    cmd.dq   = 0
    cmd.kp   = 0.2
    cmd.kd   = 0.0
    cmd.tau  = 0.0
    serial.sendRecv(cmd, data)
    print('\n')
    print("q: " + str(data.q))
    print("dq: " + str(data.dq))
    print("temp: " + str(data.temp))
    print("merror: " + str(data.merror))
    print("tau: " + str(data.tau))
    print('\n')
    time.sleep(0.0002) # 200 us

