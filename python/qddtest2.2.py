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

# モータの目標位置・目標速度
cmd.q = 0.0 * gear_ratio
cmd.dq = 0.0 * gear_ratio
cmd.kp = 0.1
cmd.kd = 0.0
cmd.tau = 0.0

# ===== 調整パラメータ =====
TARGET_ANGLE = 90.0
TARGET_TOLERANCE = 15.0

ANGLE_CHANGE_LIMIT = 3.0
ANGULAR_VELOCITY_LIMIT = 10.0

RETURN_ANGLE = 2.0
RETURN_VELOCITY = 5.0

KP_LOW = 0.1
KD_LOW = 0.1
KP_HIGH = 0.15
KD_HIGH = 0.15
GAIN_CHANGE_ANGLE = 45.0
# ==========================

start_time = time.perf_counter()
last_flush_time = 0.0
q0 = None

# 制御状態
state = 'APPROACH_P'
hold_reference_angle = None

with open('motor_data.csv', 'w', newline='',
          encoding='utf-8-sig') as f:

    writer = csv.writer(f)

    writer.writerow([
        'Time [s]',
        'Angle [deg]',
        'Angular velocity [deg/s]',
        'Angular velocity [rad/s]',
        'Hold reference angle [deg]',
        'Angle deviation [deg]',
        'Torque tau [Nm]',
        'Control state'
    ])

    try:
        while True:

            # モータ状態の取得
            serial.sendRecv(cmd, data)

            elapsed_time = time.perf_counter() - start_time

            # 初期角度を取得
            if q0 is None:
                q0 = data.q

            # 初期位置からの角度変化 [deg]
            angle = (
                (data.q - q0) * 180.0 / math.pi / gear_ratio
            )

            # 出力軸側の角速度
            angular_velocity_rad_s = data.dq / gear_ratio

            angular_velocity_deg_s = (
                angular_velocity_rad_s * 180.0 / math.pi
            )

            # ゲイン設定
            if abs(angle) > GAIN_CHANGE_ANGLE:
                kp = KP_HIGH
                kd = KD_HIGH
            else:
                kp = KP_LOW
                kd = KD_LOW

            # 保持基準角度からの変化
            if hold_reference_angle is None:
                angle_deviation = 0.0
            else:
                angle_deviation = angle - hold_reference_angle

            # ===== 制御状態の切り替え =====

            if state == 'APPROACH_P':

                # 90°付近まではP制御
                cmd.kp = kp
                cmd.kd = 0.0

                in_target_range = (
                    TARGET_ANGLE - TARGET_TOLERANCE
                    <= angle <=
                    TARGET_ANGLE + TARGET_TOLERANCE
                )

                velocity_within_limit = (
                    abs(angular_velocity_rad_s)
                    <= ANGULAR_VELOCITY_LIMIT
                )

                # 90°付近で角速度条件を満たしたら
                # その角度を保持基準に設定
                if in_target_range and velocity_within_limit:
                    hold_reference_angle = angle
                    state = 'HOLD_P'

            elif state == 'HOLD_P':

                # 2条件を満たす間はP制御のみ
                cmd.kp = kp
                cmd.kd = 0.0

                angle_within_limit = (
                    abs(angle_deviation) <= ANGLE_CHANGE_LIMIT
                )

                velocity_within_limit = (
                    abs(angular_velocity_rad_s)
                    <= ANGULAR_VELOCITY_LIMIT
                )

                # どちらか一方でも条件を外れたらPD制御
                if not (
                    angle_within_limit
                    and velocity_within_limit
                ):
                    state = 'RETURN_PD'

            elif state == 'RETURN_PD':

                # 初期位置への復帰はPD制御
                cmd.kp = kp
                cmd.kd = kd

                is_back_at_initial = (
                    abs(angle) <= RETURN_ANGLE
                    and abs(angular_velocity_deg_s)
                    <= RETURN_VELOCITY
                )

                # 初期位置に戻ったら待機状態へ
                if is_back_at_initial:
                    state = 'APPROACH_P'
                    hold_reference_angle = None
                    cmd.kd = 0.0

            # 出力トルクフィードバック
            tau = data.tau

            # 基準角度からの偏差
            if hold_reference_angle is not None:
                angle_deviation = angle - hold_reference_angle
            else:
                angle_deviation = 0.0

            # CSV記録
            writer.writerow([
                elapsed_time,
                angle,
                angular_velocity_deg_s,
                angular_velocity_rad_s,
                hold_reference_angle,
                angle_deviation,
                tau,
                state
            ])

            # 約1秒ごとに保存
            if elapsed_time - last_flush_time >= 1.0:
                f.flush()
                last_flush_time = elapsed_time

            time.sleep(0.0002)

    except KeyboardInterrupt:
        print('記録を終了します。')

print('motor_data.csv に保存しました。')