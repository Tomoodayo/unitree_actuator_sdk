
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
cmd.q = 0 * gear_ratio
cmd.dq = 0 * gear_ratio
cmd.kp = 0.1
cmd.kd = 0.0
cmd.tau = 0.0

# 記録開始時刻
start_time = time.perf_counter()

# Excelで開けるCSVファイルを作成
with open('motor_tau.csv', 'w', newline='', encoding='utf-8-sig') as f:
    writer = csv.writer(f)
    writer.writerow(['Time [s]', 'Tau'])

    try:
        while True:
            serial.sendRecv(cmd, data)

            # 経過時間とトルクを記録
            elapsed_time = time.perf_counter() - start_time
            tau = data.tau

            writer.writerow([elapsed_time, tau, theta])

            # 定期的にファイルへ書き込む
            if time.perf_counter() - start_time >= 1:
                f.flush()

            time.sleep(0.0002)

    except KeyboardInterrupt:
        print("記録を終了します。")

print("motor_tau.csv に保存しました。")