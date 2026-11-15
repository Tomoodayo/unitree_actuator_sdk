
import time
import sys
import csv
import math
import os
from datetime import datetime

sys.path.append('../lib')
from unitree_actuator_sdk import *

# ==========================================
# 1. モータ設定
# ==========================================

SERIAL_PORT = '/dev/ttyUSB0'
MOTOR_ID = 1

motor_type = MotorType.GO_M8010_6
gear_ratio = 6.33

serial = SerialPort(SERIAL_PORT)
cmd = MotorCmd()
data = MotorData()

cmd.motorType = motor_type
data.motorType = motor_type
cmd.id = MOTOR_ID

cmd.mode = queryMotorMode(
    motor_type, MotorMode.FOC
)

# ==========================================
# 2. 実験条件
# ==========================================

EXPERIMENT_DATE = datetime.now().strftime('%Y%m%d')

LOAD_ID = 'N0'       # N0 / L1 / L2
GAIN_ID = 'G0'       # G0 / G1
WAVE_ID = 'W1'       # W1 / W2 / W3
REPEAT = 1

# 追加慣性 [kg*m^2]
ADDED_INERTIA = None

# ==========================================
# 3. 制御パラメータ
# ==========================================

# 出力軸側のPDゲイン
KP_OUT = None        # Nm/rad
KD_OUT = None        # Nm*s/rad

# FFトルク・目標角速度
TAU_FF_OUT = 0.0
DQ_DES_OUT = 0.0

# ロータ側へ換算
KP_ROTOR = (
    KP_OUT / gear_ratio**2
    if KP_OUT is not None else None
)

KD_ROTOR = (
    KD_OUT / gear_ratio**2
    if KD_OUT is not None else None
)

# ==========================================
# 4. 励振波形設定
# ==========================================

# 中心角 [rad]：実験前に実測・設定する
CENTER_RAD = None

# 振幅 [rad]
AMPLITUDE_RAD = None

# W1用周波数 [Hz]
FREQUENCY_W1 = None

# W2用移動時間 [s]
STEP_DURATION = None

# W3用周波数群 [Hz]
FREQUENCIES_W3 = [0.2, 0.5, 1.0, 2.0, 3.0]

# W3の各周波数での周期数
CYCLES_PER_FREQ = 5

# W1の実験時間 [s]
W1_DURATION = None

# ==========================================
# 5. 記録設定
# ==========================================

LOG_HZ = 65.0
LOG_INTERVAL = 1.0 / LOG_HZ

# ==========================================
# 6. 安全上限
# ==========================================

# これらは実験責任者が決めること。
# 未設定のままではプログラムを開始しない。

MIN_ANGLE_RAD = None
MAX_ANGLE_RAD = None
MAX_SPEED_RAD_S = None
MAX_ERROR_RAD = None
MAX_TEMP_C = None
MAX_TAU_REQ_NM = None
MAX_COMM_GAP_S = None

# モータ取付方向
JOINT_SIGN = 1

# 出力軸側の角度ゼロ点補正 [rad]
ZERO_OFFSET_RAD = None

# ==========================================
# 7. 実験開始前の確認
# ==========================================

required_settings = {
    'KP_OUT': KP_OUT,
    'KD_OUT': KD_OUT,
    'CENTER_RAD': CENTER_RAD,
    'AMPLITUDE_RAD': AMPLITUDE_RAD,
    'ZERO_OFFSET_RAD': ZERO_OFFSET_RAD,
    'MIN_ANGLE_RAD': MIN_ANGLE_RAD,
    'MAX_ANGLE_RAD': MAX_ANGLE_RAD,
    'MAX_SPEED_RAD_S': MAX_SPEED_RAD_S,
    'MAX_ERROR_RAD': MAX_ERROR_RAD,
    'MAX_TEMP_C': MAX_TEMP_C,
    'MAX_TAU_REQ_NM': MAX_TAU_REQ_NM,
    'MAX_COMM_GAP_S': MAX_COMM_GAP_S
}

if WAVE_ID == 'W1':
    required_settings['FREQUENCY_W1'] = FREQUENCY_W1
    required_settings['W1_DURATION'] = W1_DURATION

elif WAVE_ID == 'W2':
    required_settings['STEP_DURATION'] = STEP_DURATION

elif WAVE_ID not in ('W1', 'W2', 'W3'):
    raise ValueError('Unknown waveform ID')

missing = [
    key for key, value in required_settings.items()
    if value is None
]

if missing:
    raise ValueError(
        '未設定の実験条件があります: ' + str(missing)
    )

if KP_OUT <= 0 or KD_OUT <= 0:
    raise ValueError('PDゲインは正の値にしてください')

if AMPLITUDE_RAD <= 0:
    raise ValueError('振幅は正の値にしてください')

if JOINT_SIGN not in (-1, 1):
    raise ValueError('JOINT_SIGNは+1または-1')

# ==========================================
# 8. CSV保存先
# ==========================================

folder_name = (
    f'qdd_sysid_{EXPERIMENT_DATE}_motor{MOTOR_ID}'
)

raw_folder = os.path.join(folder_name, 'raw')
os.makedirs(raw_folder, exist_ok=True)

file_name = (
    f'{LOAD_ID}_{GAIN_ID}_{WAVE_ID}_'
    f'repeat{REPEAT:02d}.csv'
)

csv_path = os.path.join(raw_folder, file_name)

# ==========================================
# 9. 目標角度生成
# ==========================================

def generate_target(t):

    # W1：低速正弦波
    if WAVE_ID == 'W1':

        return (
            CENTER_RAD
            + AMPLITUDE_RAD
            * math.sin(2 * math.pi * FREQUENCY_W1 * t)
        )

    # W2：滑らかな位置ステップ相当入力
    elif WAVE_ID == 'W2':

        half = STEP_DURATION / 2.0

        if t < half:
            s = t / half
            smooth = (1 - math.cos(math.pi * s)) / 2
            return CENTER_RAD + AMPLITUDE_RAD * smooth

        elif t < STEP_DURATION:
            s = (t - half) / half
            smooth = (1 - math.cos(math.pi * s)) / 2
            return (
                CENTER_RAD
                + AMPLITUDE_RAD * (1 - smooth)
            )

        return CENTER_RAD

    # W3：周波数群
    elif WAVE_ID == 'W3':

        elapsed = t

        for freq in FREQUENCIES_W3:

            duration = CYCLES_PER_FREQ / freq

            if elapsed < duration:
                return (
                    CENTER_RAD
                    + AMPLITUDE_RAD
                    * math.sin(2 * math.pi * freq * elapsed)
                )

            elapsed -= duration

        return CENTER_RAD


# ==========================================
# 10. 実験時間
# ==========================================

if WAVE_ID == 'W1':
    TOTAL_DURATION = W1_DURATION

elif WAVE_ID == 'W2':
    TOTAL_DURATION = STEP_DURATION

else:
    TOTAL_DURATION = sum(
        CYCLES_PER_FREQ / f
        for f in FREQUENCIES_W3
    )

# ==========================================
# 11. ログ項目
# ==========================================

fieldnames = [
    'Sample',
    'Time_wall',
    'Time_command_s',
    'Time_send_start_s',
    'Time_recv_end_s',
    'Communication_time_s',
    'Communication_OK',

    'Load_ID',
    'Added_inertia_kg_m2',
    'Gain_ID',
    'Wave_ID',
    'Repeat',

    'q_des_rad',
    'dq_des_rad_s',
    'tau_ff_Nm',

    'Kp_out_Nm_rad',
    'Kd_out_Nm_s_rad',
    'Kp_rotor_command',
    'Kd_rotor_command',

    'q_raw_rotor_rad',
    'dq_raw_rotor_rad_s',

    'q_output_rad',
    'dq_output_rad_s',

    'tau_telemetry_raw',
    'Temperature_C',

    'Position_error_rad',
    'Estimated_PD_effort_Nm',

    'Motor_ID',
    'Gear_ratio',
    'Motor_mode'
]

# ==========================================
# 12. モータ停止処理
# ==========================================

def stop_motor():

    try:
        cmd.mode = 0
        serial.sendRecv(cmd, data)
        print('STOP command sent')

    except Exception as e:
        print('STOP command failed:', e)
        print('ハードウェア非常停止を使用してください')

# ==========================================
# 13. 実験開始
# ==========================================

sample = 0
start_time = time.monotonic()
last_recv_time = start_time

# 出力軸側の目標角度を設定
# 事前にCENTER_RADへ手動で位置合わせしておくこと

print('System Identification Start')
print('Load:', LOAD_ID)
print('Gain:', GAIN_ID)
print('Wave:', WAVE_ID)

try:

    with open(
        csv_path,
        'w',
        newline='',
        encoding='utf-8-sig'
    ) as f:

        writer = csv.DictWriter(
            f,
            fieldnames=fieldnames
        )

        writer.writeheader()

        while True:

            command_time = time.monotonic()
            elapsed = command_time - start_time

            if elapsed > TOTAL_DURATION:
                break

            # 目標角度
            q_des = generate_target(elapsed)

            # 出力軸角度からロータ側へ換算
            q_command = (
                JOINT_SIGN
                * (q_des + ZERO_OFFSET_RAD)
                * gear_ratio
            )

            # 制御指令
            cmd.q = q_command
            cmd.dq = 0.0
            cmd.tau = 0.0
            cmd.kp = KP_ROTOR
            cmd.kd = KD_ROTOR

            # 送信時刻
            send_start = time.monotonic()

            try:
                serial.sendRecv(cmd, data)
                recv_end = time.monotonic()
                communication_ok = 1

            except Exception as e:

                recv_end = time.monotonic()
                communication_ok = 0

                print('Communication error:', e)

                writer.writerow({
                    'Sample': sample,
                    'Time_wall': datetime.now().isoformat(),
                    'Time_command_s': elapsed,
                    'Time_send_start_s': send_start - start_time,
                    'Time_recv_end_s': recv_end - start_time,
                    'Communication_time_s': recv_end - send_start,
                    'Communication_OK': 0,
                    'Load_ID': LOAD_ID,
                    'Gain_ID': GAIN_ID,
                    'Wave_ID': WAVE_ID,
                    'Repeat': REPEAT,
                    'Motor_ID': MOTOR_ID,
                    'Gear_ratio': gear_ratio
                })

                f.flush()
                raise

            # ==================================
            # モータデータ取得
            # ==================================

            q_raw = float(data.q)
            dq_raw = float(data.dq)
            tau_raw = float(data.tau)

            temperature = float(
                getattr(data, 'temp', float('nan'))
            )

            # 出力軸側へ換算
            q_output = (
                JOINT_SIGN * q_raw / gear_ratio
                - ZERO_OFFSET_RAD
            )

            dq_output = (
                JOINT_SIGN * dq_raw / gear_ratio
            )

            # 角度誤差
            error = q_des - q_output

            # 理論上のPD指令努力
            # 実測トルクではない
            tau_estimated = (
                KP_OUT * error
                - KD_OUT * dq_output
            )

            # ==================================
            # 安全監視
            # ==================================

            comm_gap = recv_end - last_recv_time

            abnormal = None

            if not MIN_ANGLE_RAD <= q_output <= MAX_ANGLE_RAD:
                abnormal = 'ANGLE LIMIT'

            elif abs(dq_output) > MAX_SPEED_RAD_S:
                abnormal = 'SPEED LIMIT'

            elif abs(error) > MAX_ERROR_RAD:
                abnormal = 'POSITION ERROR LIMIT'

            elif math.isfinite(temperature) and temperature > MAX_TEMP_C:
                abnormal = 'TEMPERATURE LIMIT'

            elif abs(tau_estimated) > MAX_TAU_REQ_NM:
                abnormal = 'ESTIMATED EFFORT LIMIT'

            elif comm_gap > MAX_COMM_GAP_S:
                abnormal = 'COMMUNICATION GAP'

            # ==================================
            # CSV記録
            # ==================================

            row = {
                'Sample': sample,
                'Time_wall': datetime.now().isoformat(),
                'Time_command_s': elapsed,
                'Time_send_start_s': send_start - start_time,
                'Time_recv_end_s': recv_end - start_time,
                'Communication_time_s': recv_end - send_start,
                'Communication_OK': communication_ok,

                'Load_ID': LOAD_ID,
                'Added_inertia_kg_m2': ADDED_INERTIA,
                'Gain_ID': GAIN_ID,
                'Wave_ID': WAVE_ID,
                'Repeat': REPEAT,

                'q_des_rad': q_des,
                'dq_des_rad_s': DQ_DES_OUT,
                'tau_ff_Nm': TAU_FF_OUT,

                'Kp_out_Nm_rad': KP_OUT,
                'Kd_out_Nm_s_rad': KD_OUT,
                'Kp_rotor_command': cmd.kp,
                'Kd_rotor_command': cmd.kd,

                'q_raw_rotor_rad': q_raw,
                'dq_raw_rotor_rad_s': dq_raw,

                'q_output_rad': q_output,
                'dq_output_rad_s': dq_output,

                'tau_telemetry_raw': tau_raw,
                'Temperature_C': temperature,

                'Position_error_rad': error,
                'Estimated_PD_effort_Nm': tau_estimated,

                'Motor_ID': MOTOR_ID,
                'Gear_ratio': gear_ratio,
                'Motor_mode': 1
            }

            writer.writerow(row)
            f.flush()

            last_recv_time = recv_end
            sample += 1

            if abnormal is not None:
                print('SAFETY ABORT:', abnormal)
                raise RuntimeError(abnormal)

            # 通信周期
            processing_time = time.monotonic() - command_time
            sleep_time = LOG_INTERVAL - processing_time

            if sleep_time > 0:
                time.sleep(sleep_time)

    print('Experiment completed')
    print('Saved:', csv_path)

except KeyboardInterrupt:

    print('Experiment interrupted by user')

except Exception as e:

    print('Experiment aborted:', e)

finally:

    stop_motor()
    print('Experiment finished')

