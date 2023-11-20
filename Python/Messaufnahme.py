import socket
import numpy as np
import serial
from threading import Thread
from imucal.management import load_calibration_info
from imucal import FerrarisCalibrationInfo
import csv
import threading
import sys
### Program for the IMU_Measurement on the Roboter ###

# Speicherdatei für CSV-FILES
csv_file_path = 'robot.csv'


# Declare some Variables

# IMU ARRAYS
IMU1 = np.empty((0, 6), float)
IMU2 = np.empty((0,6),float)
IMU1_ACC = IMU2_ACC = IMU1_GYR = IMU2_GYR = np.empty((0, 3), float)
offsets_imu1 = offsets_imu2 = np.empty((0, 6), float)
##Calibrated Imu Data
calibrated_imu1_acc = calibrated_imu1_gyr = calibrated_imu2_acc = calibrated_imu2_gyr = np.empty((0, 3), float)
##Timestamp
timestamp_np = 0
##IMU1_Data
acceleration1_x_np = acceleration1_y_np = acceleration1_z_np = gyro1_x_np = gyro1_y_np = gyro1_z_np = 0
##IMU2_Data
acceleration2_x_np = acceleration2_y_np = acceleration2_z_np = gyro2_x_np = gyro2_y_np = gyro2_z_np = 0

## Offsets except Acceleration Z-Axis
off_acc1x = off_acc1y = off_acc2x = off_acc2y = off_gyr1x = off_gyr1y = off_gyr1z = off_gyr2x = off_gyr2y = off_gyr2z = None

## Offsets Acceleration Z-Axix
off_acc1z = off_acc2z = None  # 9.81264

# Load Calibration Data for IMUS
Calibration_Data1 = 'Folder_Calibrations/cal_imu1.json'
Calibration_Data2 = 'Folder_Calibrations/cal_imu2.json'
loaded_cal_info1 = load_calibration_info(Calibration_Data1)
loaded_cal_info2 = load_calibration_info(Calibration_Data2)

# Temperature Correction Factors

sensitivity_scale_factor_gyro = 0.03
zro_variation_gyro = 0.05  # dps/°C
sensitivity_scale_factor_acc = 0.00026  # sensitivity change in %/°C
zro_g_change_acc = 0.8 * 0.0001 * 9.81  # Change 0 Point in mg/°C
ref_temp = 25


# Wait for Trigger Function
def wait_for_tcp_trigger(port: int = 9090, bind_ip: str = '0.0.0.0'):
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((bind_ip, port))
        s.listen()
        c, _ = s.accept()
        with c:
            d = c.recv(1024)
            if d == b'TRIG':
                c.sendall(b'ACK')
                return 0
            else:
                raise Exception(f'Invalid TRIG via TCP: "{d}"')
                return 1

# Read Serial Function
####################################################################
## IMPORTANT: BE CERTAIN THAT THE RIGHT TEENSY PROGRAM IS LOADED! ##
####################################################################

def read_serial():
    global calibrated_imu1_acc, calibrated_imu1_gyr, calibrated_imu2_acc, calibrated_imu2_gyr
    global timestamp_np
    global acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np
    global acceleration2_x_np, acceleration2_y_np, acceleration2_z_np, gyro2_x_np, gyro2_y_np, gyro2_z_np
    global off_acc1x, off_acc1y, off_acc2x, off_acc2y, off_gyr1x, off_gyr1y, off_gyr1z, off_gyr2x, off_gyr2y, off_gyr2z
    global off_acc1z, off_acc2z
    global IMU1, IMU1_ACC, IMU2_ACC, IMU1_GYR, IMU2_GYR
    global IMU2
    global offsets_imu1, offsets_imu2
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)  # change with your settings
    cal_mat1 = FerrarisCalibrationInfo.from_json_file(Calibration_Data1)
    cal_mat2 = FerrarisCalibrationInfo.from_json_file(Calibration_Data2)
    while True:
        ser.reset_input_buffer()

        # Read Data from Serial-Stream
        line = ser.readline().decode(errors='ignore').strip()

        if line.startswith('$'):
            if line.endswith('%'):
                line = line[1:-1].strip()
                data_array = line.split(';')
                data_array = data_array[1:-1]

                temperature_np = np.array(data_array[0], dtype=float)
                diff_temp = temperature_np - ref_temp
                corr_factor1 = diff_temp * zro_g_change_acc
                corr_factor2 = sensitivity_scale_factor_acc * diff_temp

                timestamp_np = np.array(data_array[1], dtype=float)
                acceleration1_x_np = np.array(data_array[2], dtype=float) - corr_factor1 - corr_factor2
                if off_acc1x is None:
                    off_acc1x = acceleration1_x_np

                acceleration1_y_np = np.array(data_array[3], dtype=float) - corr_factor1 - corr_factor2
                if off_acc1y is None:
                    off_acc1y = acceleration1_y_np

                acceleration1_z_np = np.array(data_array[4], dtype=float) - corr_factor1 - corr_factor2
                if off_acc1z is None:
                    off_acc1z = acceleration1_z_np - 9.81264

                acceleration2_x_np = np.array(data_array[8], dtype=float) - corr_factor1 - corr_factor2
                if off_acc2x is None:
                    off_acc2x = acceleration2_x_np

                acceleration2_y_np = np.array(data_array[9], dtype=float) - corr_factor1 - corr_factor2
                if off_acc2y is None:
                    off_acc2y = acceleration2_y_np

                acceleration2_z_np = np.array(data_array[10], dtype=float) - corr_factor1 - corr_factor2
                if off_acc2z is None:
                    off_acc2z = acceleration2_z_np - 9.81264

                gyro1_x_np = np.degrees(np.array(data_array[5], dtype=float))
                if off_gyr1x is None:
                    off_gyr1x = gyro1_x_np

                gyro1_y_np = np.degrees(np.array(data_array[6], dtype=float))
                if off_gyr1y is None:
                    off_gyr1y = gyro1_y_np

                gyro1_z_np = np.degrees(np.array(data_array[7], dtype=float))
                if off_gyr1z is None:
                    off_gyr1z = gyro1_z_np

                gyro2_x_np = np.degrees(np.array(data_array[11], dtype=float))
                if off_gyr2x is None:
                    off_gyr2x = gyro2_x_np

                gyro2_y_np = np.degrees(np.array(data_array[12], dtype=float))
                if off_gyr2y is None:
                    off_gyr2y = gyro2_y_np

                gyro2_z_np = np.degrees(np.array(data_array[13], dtype=float))
                if off_gyr2z is None:
                    off_gyr2z = gyro1_z_np

                offsets_imu1 = np.array([off_acc1x, off_acc1y, off_acc1z, off_gyr1x, off_gyr1y, off_gyr1z])
                offsets_imu2 = np.array([off_acc2x, off_acc2y, off_acc2z, off_gyr2x, off_gyr2y, off_gyr2z])
                IMU1 = np.append(IMU1, np.array(
                    [[acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np]]),
                                 axis=0)

                # IMU2 = np.append(IMU2, np.array(
                #     [[acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np]]),
                #                  axis=0)
                IMU2 = np.column_stack((acceleration2_x_np, acceleration2_y_np, acceleration2_z_np, gyro2_x_np, gyro2_y_np, gyro2_z_np))

                IMU1_ACC = IMU1[0:3]

                IMU1_ACC = IMU1[0,0:3]
                IMU1_GYR = IMU1[0,3:6]
                IMU2_ACC = IMU2[0,0:3]
                IMU2_GYR = IMU2[0,3:6]
                IMU1_GYR = IMU1_GYR - offsets_imu1[3:6]
                IMU2_GYR = IMU2_GYR - offsets_imu2[3:6]
                size_imu1 = IMU1.size
                size_imu2 = IMU2.size
                size_imu1_acc = IMU1_ACC.size
                size_imu2_acc = IMU2_ACC.size
                size_imu1_gyr = IMU1_GYR.size
                size_imu2_gyr = IMU2_GYR.size
                if size_imu1 >= 1:
                    IMU1 = np.empty((0, 6), float)
                if size_imu2 >= 1:
                    IMU2 = np.empty((0, 6), float)
                if size_imu1_acc >= 1:
                    IMU2 = np.empty((0, 3), float)
                if size_imu2_acc >= 1:
                    IMU2 = np.empty((0, 3), float)
                if size_imu1_gyr >= 1:
                    IMU2 = np.empty((0, 3), float)
                if size_imu2_gyr >= 1:
                    IMU2 = np.empty((0, 3), float)

        calibrated_imu1_acc, calibrated_imu1_gyr = cal_mat1.calibrate(IMU1_ACC, IMU1_GYR, "m/s^2", "rad/s")
        calibrated_imu2_acc, calibrated_imu2_gyr = cal_mat2.calibrate(IMU2_ACC, IMU2_GYR, "m/s^2", "rad/s")
        calibrated_imu1_acc = calibrated_imu1_acc - offsets_imu1[0:3]
        calibrated_imu2_acc = calibrated_imu2_acc - offsets_imu2[0:3]

        with open("Robot1.csv", 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp_np,calibrated_imu1_acc[0],calibrated_imu1_acc[1],calibrated_imu1_acc[2],IMU1_GYR[0],IMU1_GYR[1],IMU1_GYR[2],calibrated_imu2_acc[0],calibrated_imu2_acc[1],calibrated_imu2_acc[2],IMU2_GYR[0],IMU2_GYR[1],IMU2_GYR[2]])

        # combine_arrays = np.column_stack((calibrated_imu1_acc,IMU1_GYR,calibrated_imu2_acc,IMU2_GYR))
        #
        # np.savetxt(csv_file_path, combine_arrays, delimiter=',', fmt='%d')


print("wait for tcp-trigger")
wait_for_tcp_trigger()
data_thread = Thread(target=read_serial)
data_thread.start()

