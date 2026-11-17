
import time
import sys
import csv
import math

sys.path.append('../lib')
from unitree_actuator_sdk import *

# ==========================================
# 1. モータの初期設定
# ==========================================

serial = SerialPort('/dev/ttyUSB0')
cmd = MotorCmd()
data = MotorData()

motor_type = MotorType.GO_M8010_6
gear_ratio = queryGearRatio(motor_type)

data.motorType = motor_type
cmd.motorType = motor_type

cmd.mode = queryMotorMode(motor_type, MotorMode.FOC)
cmd.id = 1

# ==========================================
# 2. 制御パラメータ
# ==========================================

KP = 0.1

KD_INPUT = 0.0
KD_OTHER = 0.01

SAMPLE_INTERVAL = 0.1

INPUT_THRESHOLD = 2.0
RETURN_THRESHOLD = 1.0
INITIAL_THRESHOLD = 0.5

# ==========================================
# 3. 初期指令値
# ==========================================

cmd.q = 0.0
cmd.dq = 0.0
cmd.kp = 0.0
cmd.kd = 0.0
cmd.tau = 0.0

# ==========================================
# 4. 初期角度の取得
# ==========================================

serial.sendRecv(cmd, data)

# qddtest1.6.pyと同じ方法で初期位置を保存
q_initial_raw = data.q

# 初期位置を目標位置に設定
# data.qとcmd.qが同じ座標系であることを前提とする
cmd.q = q_initial_raw

# ゲイン0の状態で初期位置を指令
cmd.kp = 0.0
cmd.kd = 0.0
cmd.dq = 0.0
cmd.tau = 0.0

serial.sendRecv(cmd, data)

# 初期位置からの角度変化
q0 = 0.0
previous_error = 0.0

# 初期状態
state = "INPUT"

# ==========================================
# 5. CSV記録の設定
# ==========================================

start_time = time.perf_counter()
last_sample_time = start_time
last_flush_time = 0.0

with open('motor_data.csv', 'w', newline='',
          encoding='utf-8-sig') as f:

    writer = csv.writer(f)

    writer.writerow([
        'Time [s]',
        'Angle [deg]',
        'Torque [Nm]',
        'kp',
        'kd'
    ])

    try:
        while True:

            # ==================================
            # モータとの通信
            # ==================================

            serial.sendRecv(cmd, data)

            current_time = time.perf_counter()
            elapsed_time = current_time - start_time

            # ==================================
            # 0.1秒ごとの制御・記録
            # ==================================

            if current_time - last_sample_time >= SAMPLE_INTERVAL:

                # 角度変化 [deg]
                angle = (
                    (data.q - q_initial_raw)
                    / gear_ratio
                    * 180.0 / math.pi
                )

                # トルク [Nm]
                tau = data.tau

                # 前回からの角度変化
                delta_q = angle - q0

                # 初期位置からの誤差
                error = angle

                # 初期位置に近づいているか判定
                approaching_initial = (
                    abs(error) < previous_error
                )

                # ==================================
                # 状態1：初期位置付近
                # ==================================

                if abs(error) <= INITIAL_THRESHOLD:

                    state = "INPUT"

                    cmd.kp = KP
                    cmd.kd = KD_INPUT

                # ==================================
                # 状態2：RETURN状態
                # ==================================

                elif state == "RETURN":

                    if (
                        abs(delta_q) > RETURN_THRESHOLD
                        and not approaching_initial
                    ):

                        state = "INPUT"

                        cmd.kp = KP
                        cmd.kd = KD_INPUT

                    else:

                        state = "RETURN"

                        cmd.kp = KP
                        cmd.kd = KD_OTHER

                # ==================================
                # 状態3：HOLDからRETURNへの移行
                # ==================================

                elif (
                    cmd.kd > 0
                    and abs(delta_q) > RETURN_THRESHOLD
                    and approaching_initial
                ):

                    state = "RETURN"

                    cmd.kp = KP
                    cmd.kd = KD_OTHER

                # ==================================
                # 状態4：INPUT状態
                # ==================================

                elif abs(delta_q) > INPUT_THRESHOLD:

                    state = "INPUT"

                    cmd.kp = KP
                    cmd.kd = KD_INPUT

                # ==================================
                # 状態5：HOLD状態
                # ==================================

                else:

                    state = "HOLD"

                    cmd.kp = KP
                    cmd.kd = KD_OTHER

                # ==================================
                # モータへの指令値
                # ==================================

                # 初期位置を目標位置として維持
                cmd.q = q_initial_raw

                cmd.dq = 0.0
                cmd.tau = 0.0

                # ==================================
                # CSVへの記録
                # ==================================

                writer.writerow([
                    elapsed_time,
                    angle,
                    tau,
                    cmd.kp,
                    cmd.kd
                ])

                # 約1秒ごとにファイルへ反映
                if elapsed_time - last_flush_time >= 1.0:

                    f.flush()
                    last_flush_time = elapsed_time

                # ==================================
                # 次回の比較用データを更新
                # ==================================

                q0 = angle
                previous_error = abs(error)

                last_sample_time = current_time

                # 状態の表示
                print(
                    "State:", state,
                    "Angle:", round(angle, 2),
                    "Torque:", round(tau, 3),
                    "kp:", cmd.kp,
                    "kd:", cmd.kd
                )

            time.sleep(0.0002)

    except KeyboardInterrupt:

        print("記録を終了します。")

    finally:

        # ゲインを0にして終了指令
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.tau = 0.0
        cmd.dq = 0.0

        serial.sendRecv(cmd, data)

print("motor_data.csv に保存しました。")
