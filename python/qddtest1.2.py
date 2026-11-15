import time
import sys
import csv

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
cmd.kp = 0.1
cmd.kd = 0.0
cmd.tau = 0.0

start_time = time.perf_counter()
last_flush_time = 0.0

with open('motor_data.csv', 'w', newline='',
          encoding='utf-8-sig') as f:

    writer = csv.writer(f)

    # CSVの列名
    writer.writerow([
        'Time [s]',
        'Angle q',
        'Torque tau'
    ])

    try:
        while True:
            serial.sendRecv(cmd, data)

            # 経過時間
            elapsed_time = time.perf_counter() - start_time

            # 角度とトルクを取得
            angle = ((data.q) - 0.172) * 6.33
            tau = (data.tau) * 6.33

            # 1行ずつ記録
            writer.writerow([elapsed_time, angle, tau])

            # 約1秒ごとにファイルへ反映
            if elapsed_time - last_flush_time >= 1.0:
                f.flush()
                last_flush_time = elapsed_time

            time.sleep(0.0002)

    except KeyboardInterrupt:
        print("記録を終了します。")

print("motor_data.csv に保存しました。")
