import time
import sys
import csv
import math

sys.path.append('../lib')
from unitree_actuator_sdk import *

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

motor_type = MotorType.GO_M8010_6
gear_ratio = queryGearRatio(motor_type)

data.motorType = motor_type
cmd.motorType = motor_type
cmd.mode = queryMotorMode(motor_type, MotorMode.FOC)
cmd.id = 1

cmd.q = 0.0 * gear_ratio
cmd.dq = 0.0 * gear_ratio
cmd.kp = 0.0
cmd.kd = 0.1
cmd.tau = 0.0

start_time = time.perf_counter()
last_flush_time = 0.0
#初期角度
q0 = None

with open('motor_data.csv', 'w', newline='',
        encoding='utf-8-sig') as f:

    writer = csv.writer(f)
    writer.writerow([
        'Time [s]',
        'Angle [deg]',
        #'Angle absolute [deg]',
        'Torque tau [Nm]'
    ])

    try:
        while True:
            serial.sendRecv(cmd, data)
            elapsed_time = time.perf_counter() - start_time

            # 最初の受信角度を基準位置にする
            if q0 is None:
               q0 = data.q

            # 初期位置からの角度変化 [deg]
            angle = ((data.q - q0) / gear_ratio) * 180.0 / math.pi

            # 絶対角度 [deg]
            #angle_absolute = (data.q / gear_ratio) * 180.0 / math.pi

            # 出力トルクフィードバック [Nm]
            tau = data.tau

            writer.writerow([
                elapsed_time,
                angle,
                #angle_absolute,
                tau
            ])

            # 約1秒ごとにファイルへ反映
            if elapsed_time - last_flush_time >= 1.0:
                f.flush()
                last_flush_time = elapsed_time
                time.sleep(0.0002)

    except KeyboardInterrupt:
        print("記録を終了します。")
        
print("motor_data.csv に保存しました。")