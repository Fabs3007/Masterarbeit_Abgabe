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
import sympy as sp
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
deltat = 0.0012
angle_encoder = 0
#Encoder
output = 0
#Temperature-Correction

sensitivity_scale_factor_gyro = 0.03
zro_variation_gyro = 0.05                   # dps/°C
sensitivity_scale_factor_acc = 0.00026      # sensi change in %/°C
zro_g_change_acc = 0.8 * 0.0001 * 9.81      # Veränderung 0 Punkt in mg/°C
ref_temp = 25
#State Vektor INITS
X_PRED1 = np.array([[0.1], [0.1], [0.1], [0.1], [0.1], [0.1], [0.1]])
X_PRED2 = np.array([[0.1], [0.1], [0.1], [0.1], [0.1], [0.1], [0.1]])
P_PRED1 = np.eye(7) * 500
P_PRED2 = np.eye(7) * 500
z1 = np.zeros((6, 1))
z2 = np.zeros((6, 1))
# Sympy Symbols
accx,accy,accz,wx,wy,wz,theta = sp.symbols('accx accy accz wx wy wz theta')

# state matrix
f = sp.Matrix([[accx],[accy],[accz],[wx],[wy],[wz],[sp.atan2(accx, sp.sqrt(accy*accy + accz*accz))+deltat*wy]])

# Calculating partial derivatives of f with respect to variables
F = f.jacobian([accx, accy, accz, wx, wy, wz, theta])

# Observation Matrix
h = sp.Matrix([[accx],[accy],[accz],[wx],[wy],[wz]])

# Jacobian of Obversation Matrix
H = h.jacobian([accx, accy, accz, wx, wy, wz,theta])

# Q - Matrix
Q = np.diag([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001])
# R -Matrix
R = np.diag([0.0529, 0.00025, 0.0529, 0.00025, 0.0529, 0.00025])

S = np.zeros((6,6),dtype=float)

offset_angle = None

def ekf1(F,Q,f,h,R):
    global X_PRED1
    global P_PRED1
    global S1
    global THETA_PREDICT1
    substituted_values = {accx:X_PRED1[0,0],accy:X_PRED1[1,0],accz:X_PRED1[2,0],wx:X_PRED1[3,0],wy:X_PRED1[4,0],wz:X_PRED1[5,0],theta:X_PRED1[6,0]}
    f_numerical = np.array(f.subs(substituted_values)).astype(float)
    X_PRED1 = f_numerical
    F_numerical = np.array(F.subs(substituted_values)).astype(float)
    P_PRED1 = np.dot(F_numerical,np.dot(P_PRED1,F_numerical.T)).astype(float) + Q # Matrix Multiplikation kontrollieren
    h_numerical = np.array(h.subs(substituted_values)).astype(float)
    H_numerical = np.array(H.subs(substituted_values)).astype(float)
    y = z1 - h_numerical
    S1 = np.dot(H_numerical,np.dot(P_PRED1,H_numerical.T)) + R             # Matrix Multiplikation kontrollieren
    INVS = np.linalg.inv(S1)
    K = np.dot(P_PRED1,np.dot(H_numerical.T,INVS))
    X_PRED1 = X_PRED1 + np.dot(K,y)
    P_PRED1 = np.dot((np.eye(7)-np.dot(K,H_numerical)),P_PRED1)
    THETA_PREDICT1 = np.degrees(X_PRED1[6,0])
    return X_PRED1,P_PRED1,THETA_PREDICT1

def ekf2(F,Q,f,h,R):
    global X_PRED2
    global P_PRED2
    global S2
    global THETA_PREDICT2
    substituted_values = {accx:X_PRED2[0,0],accy:X_PRED2[1,0],accz:X_PRED2[2,0],wx:X_PRED2[3,0],wy:X_PRED2[4,0],wz:X_PRED2[5,0],theta:X_PRED2[6,0]}
    f_numerical = np.array(f.subs(substituted_values)).astype(float)
    X_PRED2 = f_numerical
    F_numerical = np.array(F.subs(substituted_values)).astype(float)
    P_PRED2 = np.dot(F_numerical,np.dot(P_PRED2,F_numerical.T)).astype(float) + Q # Matrix Multiplikation kontrollieren
    h_numerical = np.array(h.subs(substituted_values)).astype(float)
    H_numerical = np.array(H.subs(substituted_values)).astype(float)
    y = z2 - h_numerical
    S2 = np.dot(H_numerical,np.dot(P_PRED2,H_numerical.T)) + R             # Matrix Multiplikation kontrollieren
    INVS = np.linalg.inv(S2)
    K = np.dot(P_PRED2,np.dot(H_numerical.T,INVS))
    X_PRED2 = X_PRED2 + np.dot(K,y)
    P_PRED2 = np.dot((np.eye(7)-np.dot(K,H_numerical)),P_PRED2)
    THETA_PREDICT2 = np.degrees(X_PRED2[6,0])
    return X_PRED2,P_PRED2,THETA_PREDICT2
def update_plot(frame):
    global data_sensor1_2d
    global data_sensor2_2d
    # Zeit- und Messwerte zu dem 2D-Array hinzufügen.
    if THETA_PREDICT1 > 100:
        None
    elif THETA_PREDICT1 < -100:
        None
    else:
        data_sensor1_2d = np.append(data_sensor1_2d, np.array([[timestamp_np, THETA_PREDICT1]]), axis=0)
    if THETA_PREDICT2 > 100:
        None
    if THETA_PREDICT2 < -100:
        None
    else:
        data_sensor2_2d= np.append(data_sensor2_2d, np.array([[timestamp_np, THETA_PREDICT2]]), axis=0)
    # Erstellen Sie den neuen Linienplot mit den aktualisierten Daten.
    plt.plot(data_sensor1_2d[:, 0], data_sensor1_2d[:, 1])
    plt.plot(data_sensor2_2d[:, 0], data_sensor2_2d[:, 1])
    # Aktualisieren Achsenlimit.
    ax.relim()
    ax.autoscale_view()

    plt.xlabel('Zeit')
    plt.ylabel('Messwert')

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
    cal_mat1 = FerrarisCalibrationInfo.from_json_file(Calibration_Data1)
    cal_mat2 = FerrarisCalibrationInfo.from_json_file(Calibration_Data2)
    while True:
        ser.reset_input_buffer()

        # Daten von der seriellen Schnittstelle lesen
        line = ser.readline().decode(errors='ignore').strip()

        if line.startswith('$'):
            if line.endswith('%'):
                line = line[1:-1].strip()
                data_array = line.split(';')
                data_array = data_array[1:-1]
                angle_encoder = np.array(data_array[0],dtype=float)
                if offset_angle is None:
                    offset_angle = angle_encoder
                angle_encoder = angle_encoder - offset_angle
                print(angle_encoder)
                temperature_np = np.array(data_array[1],dtype=float)
                diff_temp = temperature_np - ref_temp
                corr_factor1 = diff_temp * zro_g_change_acc
                corr_factor2 = sensitivity_scale_factor_acc * diff_temp
                timestamp_np = np.array(data_array[2],dtype=float)
                acceleration1_x_np = np.array(data_array[3], dtype=float) - corr_factor1 - corr_factor2
                acceleration1_y_np = np.array(data_array[4], dtype=float) - corr_factor1 - corr_factor2
                acceleration1_z_np = np.array(data_array[5], dtype=float) - corr_factor1 - corr_factor2
                acceleration2_x_np = np.array(data_array[9], dtype=float) - corr_factor1 - corr_factor2
                acceleration2_y_np = np.array(data_array[10], dtype=float) - corr_factor1 - corr_factor2
                acceleration2_z_np = np.array(data_array[11], dtype=float) - corr_factor1 - corr_factor2
                gyro1_x_np = np.degrees(np.array(data_array[6], dtype=float))
                gyro1_y_np = np.degrees(np.array(data_array[7], dtype=float))
                gyro1_z_np = np.degrees(np.array(data_array[8], dtype=float))
                gyro2_x_np = np.degrees(np.array(data_array[12], dtype=float))
                gyro2_y_np = np.degrees(np.array(data_array[13], dtype=float))
                gyro2_z_np = np.degrees(np.array(data_array[14], dtype=float))
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
                extended_kalmanfilter1 = Thread(target = ekf1,args=(F,Q,f,h,R,))
                extended_kalmanfilter2 = Thread(target=ekf2, args=(F, Q, f, h, R,))
                extended_kalmanfilter1.start()
                extended_kalmanfilter2.start()


        calibrated_imu1_acc, calibrated_imu1_gyr = cal_mat1.calibrate(IMU1_ACC, IMU1_GYR, "m/s^2", "rad/s")
        calibrated_imu2_acc, calibrated_imu2_gyr = cal_mat2.calibrate(IMU2_ACC, IMU2_GYR, "m/s^2", "rad/s")
        z1[0,0] = calibrated_imu1_acc[0,0]
        z1[1,0] = calibrated_imu1_acc[0,1]
        z1[2,0] = calibrated_imu1_acc[0,2]
        z1[3,0] = gyro1_x_np
        z1[4,0] = gyro1_y_np
        z1[5,0] = gyro1_z_np
        z2[0,0] = calibrated_imu2_acc[0,0]
        z2[1,0] = calibrated_imu2_acc[0,1]
        z2[2,0] = calibrated_imu2_acc[0,2]
        z2[3,0] = gyro2_x_np
        z2[4,0] = gyro2_y_np
        z2[5,0] = gyro2_z_np
        with open("Testfile.csv", 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([THETA_PREDICT1,THETA_PREDICT2,angle_encoder])




    return angle_encoder, acceleration1_x_np, acceleration1_y_np, acceleration1_z_np, acceleration2_x_np, acceleration2_y_np, acceleration2_z_np, gyro1_x_np, gyro1_y_np, gyro1_z_np, gyro2_x_np, gyro2_y_np, gyro2_z_np



#
data_thread = Thread(target=read_serial)
data_thread.start()

# Erstellen Sie eine Figur und eine Axes-Instanz, um den Plot anzuzeigen.
fig, ax = plt.subplots()

# Erstellen Sie die Animation und starten Sie sie.
ani = FuncAnimation(fig, update_plot, interval=200)

plt.show()


