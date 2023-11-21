import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math
import serial
from threading import Thread
from queue import Queue
import pandas as pd
from imucal.management import load_calibration_info
from imucal import FerrarisCalibrationInfo
from imucal.management import find_calibration_info_for_sensor
from pathlib import Path
import os
import csv
import time

Calibration_Data1 = 'Folder_Calibrations/cal_imu1.json'
Calibration_Data2 = 'Folder_Calibrations/cal_imu2.json'
loaded_cal_info1 = load_calibration_info(Calibration_Data1)
loaded_cal_info2 = load_calibration_info(Calibration_Data2)
deltat = 0.0012
calibrated_imu1_acc = np.empty((0,3),float)
IMU1_ACC = np.empty((0,3),float)
IMU1_GYR = np.empty((0,3),float)
IMU2_ACC = np.empty((0,3),float)
IMU2_GYR = np.empty((0,3),float)
acceleration1_x_np = 0
acceleration1_y_np = 0
acceleration1_z_np = 0
acceleration2_x_np = 0
acceleration2_y_np = 0
acceleration2_z_np = 0
gyro1_x_np = 0
gyro1_y_np = 0
gyro1_z_np = 0
gyro2_x_np = 0
gyro2_y_np = 0
gyro2_z_np = 0
IMU1 = np.empty((0,6),float)
IMU2 = np.empty((0,6),float)
pi_value = math.pi
THETA_PREDICT1 = 0
THETA_PREDICT2 = 0
timestamp_np = 0
data_sensor1_2d = np.empty((0,2),float)
data_sensor2_2d = np.empty((0,2),float)
data_encoder = np.empty((0,2),float)

angle_encoder = 0

Wait_Signal = None
#Encoder
output = 0
#Temperature-Correction

sensitivity_scale_factor_gyro = 0.03
zro_variation_gyro = 0.05                   # dps/°C
sensitivity_scale_factor_acc = 0.00026      # sensi change in %/°C
zro_g_change_acc = 0.8 * 0.0001 * 9.81      # Veränderung 0 Punkt in mg/°C
ref_temp = 25
#State Vektor INITS
X_P1 = 0
X_PRED2 = np.array([[0.001], [0.001], [9.8124], [0], [0], [0], [0]])
P_PRED2 = np.eye(7) * 500
z1 = 0
z2 = np.zeros((6, 1))

H = np.eye(3)

# R -Matrix
R1 = np.eye(3) * 0.0025
Q = np .eye(3) * 0.000085
R2 = np.diag(np.full(6,11000))

S = np.zeros((6,6),dtype=float)

offset_angle = None

h_subs1 = np.array([0])
h_subs2 = np.zeros([6,1],float)

P1 = np.eye(3) * 5000
P2 = np.eye(1) * 5000
x_k = np.array([0.001,0.001,0.001])
x_p = np.array([0.001,0.001,0.001])
offset = 0.1516
X_P1 = 0
plot_array = []
err_array = []
pitch_asin = 0
def ekf1(H,R1,P1):
    global x_p
    global x_k
    global X_P1
    global S1
    global THETA_PREDICT1
    gx = -gyro1_x_np
    gy = -gyro1_y_np
    gz = -gyro1_z_np

    ax = acceleration1_x_np
    ay = acceleration1_y_np
    az = acceleration1_z_np
    x_1 = gx + math.sin(x_k[0])*math.tan(x_k[1])*gy + math.cos(x_k[0])*math.tan(x_k[1])*gz
    x_2 = math.cos(x_k[0]) *gy - math.sin(x_k[0])*gz
    x_3 = gy * (math.sin(x_k[0])/math.cos(x_k[1])) + gz*(math.cos(x_k[0])/math.cos(x_k[1]))

    x_p[0] = x_k[0] + deltat*x_1
    x_p[1] = x_k[1] + deltat*x_2
    x_p[2] = x_k[2] + deltat*x_3

    a = 1+ deltat *(gy*math.cos(x_k[0])*math.tan(x_k[1])-gz*math.sin(x_k[1])*math.tan(x_k[2]))
    b = deltat * (1/(math.cos(x_k[1]))**2) * math.cos(x_k[0])*gz + math.sin(x_k[0])*gy
    c = 0
    d = deltat * (-gy*math.sin(x_k[0])-gz*math.cos(x_k[1]))
    e = deltat
    f= 0
    g = deltat *(gy*math.cos(x_k[0])/math.cos(x_k[1])-gz*math.sin(x_k[0])/math.cos(x_k[1]))
    r = deltat * (gy*math.sin(x_k[0])*math.sin(x_k[1])/2*math.cos(x_k[0])**2 + gz*math.cos(x_k[0])*math.sin(x_k[1])/2*math.cos(x_k[1])**2)
    k = deltat
    F = np.array([[a,b,c],[d,e,f],[g,r,k]],dtype = float)
    P_k = np.dot(F,np.dot(P1,F.T)) + Q
    pitch = math.asin(ax/math.sqrt((ax**2) + (ay**2) + (az**2)))
    roll = math.asin(-ay/math.sqrt((ax**2) + (ay**2) + (az**2))*math.cos(pitch))
    h = x_p
    z1 = np.array([roll,pitch,0])
    y1 = z1 - h
    s1 = np.dot(H,np.dot(P_k,H.T)) + R1
    k1 = np.dot(P_k,np.dot(H.T,np.linalg.inv(s1)))
    x_k = x_p + np.dot(k1,y1)
    P1 = np.dot(np.eye(3) -np.dot(k1,H),P_k)
    print(np.degrees(x_p[1]))
def update_plot(frame):
    global data_sensor1_2d
    global data_sensor2_2d
    global data_encoder
    plt.clf()
    # Zeit- und Messwerte zu dem 2D-Array hinzufügen.
    data_encoder = np.append(data_encoder,np.array([[timestamp_np,angle_encoder]]),axis=0)
    data_sensor1_2d = np.append(data_sensor1_2d, np.array([[timestamp_np, x_k[1]]]), axis=0)
    # Erstellen Sie den neuen Linienplot mit den aktualisierten Daten.
    #plt.plot(data_sensor1_2d[:, 0], data_sensor1_2d[:, 1],markersize=1, label='Sensor 1 Daten')
    plt.plot(THETA_PREDICT1, markersize=1, label='Sensor 1 Daten')

    #plt.plot(data_sensor2_2d[:, 0], data_sensor2_2d[:, 1])
    plt.plot(data_encoder[:,0],data_encoder[:,1],markersize=1)
    # Aktualisieren Achsenlimit.
    ax.relim()
    ax.autoscale_view()

    plt.xlabel('Zeit')
    plt.ylabel('Messwert')

    plt.legend()

def read_serial():
    ser = serial.Serial('/dev/ttyACM0', baudrate=115200)
    global angle_encoder
    global temperature
    global timestamp_np
    global acceleration1_x_np
    global acceleration1_y_np
    global acceleration1_z_np
    global acceleration2_x_np
    global acceleration2_y_np
    global acceleration2_z_np
    global gyro1_x_np
    global gyro1_y_np
    global gyro1_z_np
    global gyro2_x_np
    global gyro2_y_np
    global gyro2_z_np
    global X1
    global X2
    global F
    global P
    global H
    global Q
    global R
    global Y
    global IMU1
    global IMU1_ACC
    global IMU1_GYR
    global IMU2
    global IMU2_ACC
    global IMU2_GYR
    global acc1_mean
    global acc2_mean
    global gyr1_mean
    global gyr2_mean
    global calibrated_imu1_acc
    global calibrated_imu1_gyr
    global calibrated_imu2_acc
    global calibrated_imu2_gyr
    global mean_acc1_x
    global mean_acc1_y
    global mean_acc1_z
    global mean_acc2_x
    global mean_acc2_y
    global mean_acc2_z
    global mean_gyr1_x
    global mean_gyr1_y
    global mean_gyr1_z
    global mean_gyr2_x
    global mean_gyr2_y
    global mean_gyr2_z
    global diff_temp
    global corr_factor1
    global corr_factor2
    global offset_angle
    global z
    global Wait_Signal
    cal_mat1 = FerrarisCalibrationInfo.from_json_file(Calibration_Data1)
    cal_mat2 = FerrarisCalibrationInfo.from_json_file(Calibration_Data2)

    while True:
        ser.reset_input_buffer()

        # Daten von der seriellen Schnittstelle lesen
        line = ser.readline().decode(errors='ignore').strip()

        if line.startswith('$'):
            if line.endswith('%'):
                clean_line = line[1:-1].strip()
                # Note: line is now in format:
                # "; 31.21 ; 1461.19 ; -0.11 ; -0.36 ; 9.90 ; -0.00 ; 0.00 ; 0.00 ; -0.15 ; -0.06 ; 9.85 ; -0.01 ; 0.01 ; 0.00 ;"
                data_array = clean_line.split(';')
                data_array = data_array[1:-1]

                temperature_np_imu1 = np.array(data_array[0], dtype=float)
                diff_temp_imu1 = temperature_np_imu1 - ref_temp
                corr_factor1_imu1 = diff_temp_imu1 * zro_g_change_acc
                corr_factor2_imu1 = sensitivity_scale_factor_acc * diff_temp_imu1

                temperature_np_imu2 = np.array(data_array[1], dtype=float)
                diff_temp_imu2 = temperature_np_imu2 - ref_temp
                corr_factor1_imu2 = diff_temp_imu2 * zro_g_change_acc
                corr_factor2_imu2 = sensitivity_scale_factor_acc * diff_temp_imu2

                timestamp_np = np.array(data_array[2], dtype=float)
                acceleration1_x_np = np.array(data_array[3], dtype=float) - corr_factor1_imu1 - corr_factor2_imu1
                # if off_acc1x is None:
                #     off_acc1x = acceleration1_x_np

                acceleration1_y_np = np.array(data_array[4], dtype=float) - corr_factor1_imu1 - corr_factor2_imu1
                # if off_acc1y is None:
                #     off_acc1y = acceleration1_y_np

                acceleration1_z_np = np.array(data_array[5], dtype=float) - corr_factor1_imu1 - corr_factor2_imu1
                # if off_acc1z is None:
                #     off_acc1z = acceleration1_z_np - 9.81264

                acceleration2_x_np = np.array(data_array[9], dtype=float) - corr_factor1_imu2 - corr_factor2_imu2
                # if off_acc2x is None:
                #     off_acc2x = acceleration2_x_np

                acceleration2_y_np = np.array(data_array[10], dtype=float) - corr_factor1_imu2 - corr_factor2_imu2
                # if off_acc2y is None:
                #     off_acc2y = acceleration2_y_np

                acceleration2_z_np = np.array(data_array[11], dtype=float) - corr_factor1_imu2 - corr_factor2_imu2
                # if off_acc2z is None:
                #     off_acc2z = acceleration2_z_np - 9.81264

                gyro1_x_np = np.degrees(np.array(data_array[6], dtype=float))
                # if off_gyr1x is None:
                #     off_gyr1x = gyro1_x_np

                gyro1_y_np = np.degrees(np.array(data_array[7], dtype=float))
                # if off_gyr1y is None:
                #     off_gyr1y = gyro1_y_np

                gyro1_z_np = np.degrees(np.array(data_array[8], dtype=float))
                # if off_gyr1z is None:
                #     off_gyr1z = gyro1_z_np

                gyro2_x_np = np.degrees(np.array(data_array[12], dtype=float))
                # if off_gyr2x is None:
                #     off_gyr2x = gyro2_x_np

                gyro2_y_np = np.degrees(np.array(data_array[13], dtype=float))
                # if off_gyr2y is None:
                #     off_gyr2y = gyro2_y_np

                gyro2_z_np = np.degrees(np.array(data_array[14], dtype=float))
                # if off_gyr2z is None:
                #     off_gyr2z = gyro2_z_np
                angle_encoder = np.array(data_array[15], dtype=float)
                if offset_angle is None:
                    offset_angle = angle_encoder
                angle_encoder = angle_encoder - offset_angle
                IMU1 = np.append(IMU1, np.array(
                    [[acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np]]),
                                 axis=0)

                IMU2 = np.append(IMU2, np.array(
                    [[acceleration2_x_np, acceleration2_y_np, acceleration2_z_np, gyro2_x_np, gyro2_y_np, gyro2_z_np]]),
                                 axis=0)

                IMU1_ACC = IMU1[:,0:3]
                IMU1_GYR = IMU1[:,3:6]
                IMU2_ACC = IMU2[:,0:3]
                IMU2_GYR = IMU2[:,3:6]
                size_imu1 = IMU1.size
                size_imu2 = IMU2.size
                if size_imu1 >= 1:
                    IMU1 = np.empty((0,6),float)
                if size_imu2 >= 1:
                    IMU2 = np.empty((0,6),float)

        Wait_Signal = True
        calibrated_imu1_acc, calibrated_imu1_gyr = cal_mat1.calibrate(IMU1_ACC, IMU1_GYR, "m/s^2", "rad/s")
        calibrated_imu2_acc, calibrated_imu2_gyr = cal_mat2.calibrate(IMU2_ACC, IMU2_GYR, "m/s^2", "rad/s")
        z2[0,0] = calibrated_imu2_acc[0,0]
        z2[1,0] = calibrated_imu2_acc[0,1]
        z2[2,0] = calibrated_imu2_acc[0,2]
        z2[3,0] = -gyro2_x_np
        z2[4,0] = -gyro2_y_np
        z2[5,0] = -gyro2_z_np
        extended_kalmanfilter1 = Thread(target=ekf1, args=(H, R1, P1,))
        extended_kalmanfilter1.start()
    return angle_encoder, acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, acceleration2_x_np, acceleration2_y_np, acceleration2_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np, gyro2_x_np, gyro2_y_np, gyro2_z_np



#
data_thread = Thread(target=read_serial)

data_thread.start()
# Erstellen Sie eine Figur und eine Axes-Instanz, um den Plot anzuzeigen.
fig, ax = plt.subplots()
# Erstellen Sie die Animation und starten Sie sie.
ani = FuncAnimation(fig, update_plot, interval=200,cache_frame_data=False)
plt.show()


