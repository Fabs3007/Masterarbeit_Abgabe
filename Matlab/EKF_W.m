clear all;
close all;
% Save Name
save = 'richard_capture_2023-08-28_16-06-16_';
diff_save = strcat(save,'imu2_diff');
imu1_save = strcat(save,'imu1');
imu2_save = strcat(save,'imu2');
% Matrix for Noise
captures = readmatrix("captures\richard_capture_2023-08-28_15-46-10_imu.csv");

% Matrix for the Filter
testprofile = readmatrix("Wippe_Measurements\Wippe_negative_Schwingung_V2.csv");

grnd_truth = readmatrix("captures/richard_capture_2023-08-28_16-13-32_imu.csv");
grnd_truth_degree = grnd_truth(1:end,23)*(180/pi)-90;
% Interpolation

% Zeitintervall IMU-Messung
Zeitintervall_Array1 = (0.0061:0.0061:(0.0061 * (length(testprofile(:,1)))) - 0.0061)';

% Zeitintervall Roboter
Zeitintervall_Array2 = (0.008:0.008:(0.008 * (length(grnd_truth(:,1)))) - 0.008)';

% Interpolation Roboter -> IMU-Messung
int_array = interp1(Zeitintervall_Array2, grnd_truth_degree(1:length(grnd_truth(:,1))-1,1), Zeitintervall_Array1, 'linear');
 
% Neues Array für Roboter-Messung mit neuen Zeitschritten
grnd_truth_int(:,1) = Zeitintervall_Array1;
grnd_truth_int(:,2) = int_array;

timestamp = zeros(1,length(testprofile(:,1)));
timestamp2 = zeros(1,length(captures(:,1)));
% Number of Samples
N = length(testprofile(:,1));
mean_gyr1 = mean(testprofile(:,7:9));
cal_gx1 = testprofile(:,7) - mean_gyr1(1);
cal_gy1 = testprofile(:,8) - mean_gyr1(2);
cal_gz1 = testprofile(:,9) - mean_gyr1(3);
mean_gyr2 = mean(testprofile(:,13:15));
cal_gx2 = testprofile(:,13) - mean_gyr2(1);
cal_gy2 = testprofile(:,14) - mean_gyr2(2);
cal_gz2 = testprofile(:,15) - mean_gyr2(3);

% Time between Measurements
for i = 1:length(testprofile)-1
deltat(i) = (testprofile(i+1,1)-testprofile(i,1))/1000000;
timestamp(i+1) = timestamp(i) + deltat(i);
end

pitch1 = atan2(testprofile(:,4),testprofile(:,6))*(180/pi);
pitch3 = asin(testprofile(:,4)./sqrt((testprofile(:,4)).^2+testprofile(:,5).^2+testprofile(:,6).^2))*180/pi;
pitch2 = atan2(testprofile(:,10),testprofile(:,12))*(180/pi);
angle_gyr1 = 0;
angle_gyr2 = 0;
for i = 1:N-1
angle_gyr1(i+1) = angle_gyr1(i) + deltat(i)*testprofile(i,8);
angle_gyr2(i+1) = angle_gyr2(i) + deltat(i)*testprofile(i,14);
end
B = 0.00002595;
yZero = zeros(N);
x_k = [0.001 0.001 0.001];          % init x_
x_p = [0.001 0.001 0.001];          % init prädiktion
x_k2 = [0.001 0.001 0.001];          % init x_
x_p2 = [0.001 0.001 0.001];          % init prädiktion
R1 = [0.0227 0 0;
      0 0.0224 0;
      0 0 0.0225];
R1 = R1 * 1.8;
% R1 = R1* 1.03;
  % Q_substituted1 = [0.00012 0 0;
  %                     0 0.00212 0;
  %                     0 0 0.00012];
R2 = [0.0227 0 0;
      0 0.0224 0;
      0 0 0.0225];
R2 = R2 * 0.45;
Q_substituted1 = eye(3) * B;
Q_substituted2 = eye(3) * B;
P1 = eye(3)*500;

P2 = eye(3)*500;
offset = abs(0-mean(testprofile(1:30,4)));
% offset2 = abs(0-mean(testprofile(1:30,10)));
offset2 = 0;
% offset = 0;
for i = 1:N-1
    % Predict
    gx = deg2rad(-cal_gx1(i));     % GyroX
    gy = deg2rad(-cal_gy1(i));     % GyroY
    gz = deg2rad(-cal_gz1(i));      % GyroZ
    x_cast(i,1) = gx + sin(x_k(i,1)) * tan(x_k(i,2)) * gy + cos(x_k(i,1)) * tan(x_k(i,2)) * gz;       % Hilfsvariable
    x_cast(i,2) = cos(x_k(i,1))*gy-sin(x_k(i,1))*gz;                                                  % Hilfsvariable
    x_cast(i,3) = (sin(x_k(i,1))/cos(x_k(i,2)))*gy+(cos(x_k(i,1))/cos(x_k(i,2)))*gz;                  % Hilsvariable

    x_p(i,1) = x_k(i,1) + deltat(i)*x_cast(i,1);
    x_p(i,2) = x_k(i,2) + deltat(i)*x_cast(i,2);
    x_p(i,3) = x_k(i,3) + deltat(i)*x_cast(i,3);

    a = (gy*cos(x_k(i,1))*tan(x_k(i,2))-gz*sin(x_k(i,1))*tan(x_k(i,2)));
    
    b = sec(x_k(i,2))^2* (cos(x_k(i,1))*gz +sin(x_k(i,1))*gy);
    c = 0;
    d = (-gy*sin(x_k(i,1))-gz*cos(x_k(i,1)));
    e = 0;
    f = 0;
    g = (gy*(cos(x_k(i,1))/cos(x_k(i,2)))-gz*(sin(x_k(i,1))/cos(x_k(i,2))));
    h = (gy*(sin(x_k(i,1))*sin(x_k(i,2))/(2*(cos(x_k(i,2))^2)))+gz*(cos(x_k(i,1))*sin(x_k(i,2))/(2*(cos(x_k(i,2))^2))));
    k = 0;

    F1 = [a b c;
         d e f;
         g h k];
    F = eye(3) + F1*deltat(i);
    P_PRED1 = F * P1 * F' + Q_substituted1;

    % Update
    pitch_atan = 1.05*atan2(testprofile(i,4)-offset,testprofile(i,6));
    pitch_asin = asin(testprofile(i,4)-offset/sqrt((testprofile(i,4)-offset)^2+testprofile(i,5)^2+testprofile(i,6)^2));
    roll = asin(-testprofile(i,5)/(sqrt((testprofile(i,4)-offset)^2+testprofile(i,5)^2+testprofile(i,6)^2)*cos(pitch_atan)));
    
    roll_array(i) = roll;
    pitch_array(i) = pitch_asin;
    pitch_array_atan(i) = pitch_atan;
    
    
    z1 = [roll;
        pitch_atan;
        0];
         
    h_substituted1 = x_p(i,:)';
    y1 = z1 - h_substituted1;
    H = [1 0 0;
        0 1 0;
        0 0 1];
    S1 = H * P_PRED1 * H' + R1;
    K1 = P_PRED1 * H' *inv(S1);
    % K1_Array(:,i) = K1*y1;
    x_k(i+1,1:3) = x_p(i,1:3) + (K1*y1)';
    winkel(i) = 180/pi*x_p(i,2);
    P1 = (eye(3) - K1*H) * P_PRED1;
end


for i = 1:N-1
    % Predict
    gx = deg2rad(-testprofile(i,13));     % GyroX
    gy = deg2rad(-testprofile(i,14));     % GyroY
    gz = deg2rad(-testprofile(i,15));     % GyroZ
    x_cast2(i,1) = gx + sin(x_k2(i,1)) * tan(x_k2(i,2)) * gy + cos(x_k2(i,1)) * tan(x_k2(i,2)) * gz;       % Hilfsvariable
    x_cast2(i,2) = cos(x_k2(i,1))*gy-sin(x_k2(i,1))*gz;                                                  % Hilfsvariable
    x_cast2(i,3) = (sin(x_k2(i,1))/cos(x_k2(i,2)))*gy+(cos(x_k2(i,1))/cos(x_k2(i,2)))*gz;                  % Hilsvariable

    x_p2(i,1) = x_k2(i,1) + deltat(i)*x_cast2(i,1);
    x_p2(i,2) = x_k2(i,2) + deltat(i)*x_cast2(i,2);
    x_p2(i,3) = x_k2(i,3) + deltat(i)*x_cast2(i,3);

    a = (gy*cos(x_k2(i,1))*tan(x_k2(i,2))-gz*sin(x_k2(i,1))*tan(x_k2(i,2)));
    b =  sec(x_k2(i,2))^2* (cos(x_k2(i,1))*gz +sin(x_k2(i,1))*gy);
    c = 0;
    d = (-gy*sin(x_k2(i,1))-gz*cos(x_k2(i,1)));
    e = 0;
    f = 0;
    g = (gy*(cos(x_k2(i,1))/cos(x_k2(i,2)))-gz*(sin(x_k2(i,1))/cos(x_k2(i,2))));
    h = (cos(x_k2(i,1))*gz+sin(x_k2(i,1))*gy)*sin(x_k2(i,2))/cos(x_k2(i,2))^2;
    k = 0;

    F2 = [a b c;
         d e f;
         g h k];
    F =  eye(3)*F2*deltat(i);
    P_PRED2 = F * P2 * F' + Q_substituted2;

    % Update
    pitch_atan2 = atan2(testprofile(i,10)-offset2,testprofile(i,12));
    pitch_asin2 = asin(testprofile(i,10)/sqrt((testprofile(i,10)-offset2)^2+testprofile(i,11)^2+testprofile(i,12)^2));
    roll2 = asin(-testprofile(i,11)/(sqrt((testprofile(i,10)-offset2)^2+testprofile(i,11)^2+testprofile(i,12)^2)*cos(pitch_atan)));
    

    z2 = [roll2;
        pitch_atan2;
        0];
         
    h_substituted2 = x_p2(i,:)';
    y2 = z2 - h_substituted2;
    H = [1 0 0;
        0 1 0;
        0 0 1];
    S2 = H * P_PRED2 * H' + R2;
    K2 = P_PRED2 * H' *inv(S2);
    % K1_Array(:,i) = K1*y1;
    x_k2(i+1,1:3) = x_p2(i,1:3) + (K2*y2)';
    winkel2(i) = 180/pi*x_p2(i,2);
    P2 = (eye(3) - K2*H) * P_PRED2;

end


L= (N-1);

gnd_truth = rad2deg(grnd_truth(1:end,23))-90;
diff_array = abs(winkel(1:L)' - testprofile(1:L,17));

diff_fzw = winkel(:)-winkel2(:);
figure(1);
subplot(2,2,1)
plot(timestamp(1,1:L),winkel(1:L));
hold on;
plot(timestamp(1,1:L),testprofile(1:L,17));
xlabel("Time in Sekunden")
ylabel("Nickwinkel in Grad")
legend("EKF IMU1","Encoder")
title("Plots Robot")
set(gca,'FontSize',14);
subplot(2,2,2)
plot(timestamp(1,1:L),pitch1(1:L))
xlabel("Time in Sekunden")
ylabel("Nickwinkel in Grad")
legend("Pitch Acc1")
set(gca,'FontSize',14);
subplot(2,2,3)
plot(timestamp(1,1:L),angle_gyr1(1:L))
hold on;
plot(timestamp(1,1:L),yZero(1:L),'b--','LineWidth',1);
hold off;
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
legend("Integriertes Gyro")
title("Winkel von Gyroskop")
set(gca,'FontSize',14);
figure(2)
subplot(2,2,1)
plot(timestamp(1,1:L),winkel2(1:L));
xlabel("Time in Sekunden")
ylabel("Nickwinkel in Grad")
title('EKF IMU2')
legend("EKF IMU2");
set(gca,'FontSize',14);
subplot(2,2,2)
plot(timestamp(1,1:L),pitch2(1:L))
xlabel("Time in Sekunden")
ylabel("Nickwinkel in Grad")
legend("Pitch Acc2")
title("Winkel Accelerometer2")
set(gca,'FontSize',14);
subplot(2,2,3)
plot(timestamp(1,1:L),diff_array(1:L))
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
legend("Diff")
title("Diff Gndtruth und EKF2")
set(gca,'FontSize',14);
subplot(2,2,4)
plot(timestamp(1,1:L),angle_gyr2(1:L))
hold on;
plot(timestamp(1,1:L),yZero(1:L),'b--','LineWidth',1);
hold off;
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
legend("Int. Gyro")
title("Winkel von Gyroskop")
set(gca,'FontSize',14);

figure(3)
subplot(1,2,1)
plot(timestamp(1:L),testprofile(1:L,17))
hold on;
plot(timestamp(1,1:L),winkel(1:L));
hold off;
xlabel("Time in Sekunden")
ylabel("Nickwinkel in Grad")
title('EKF IMU2')
legend('Gndtruth',"EKF IMU1");
set(gca,'FontSize',14);
subplot(1,2,2)
plot(timestamp(1,1:L),diff_array(1:L))
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
title("Diff Gndtruth und EKF1")
set(gca,'FontSize',14);
figure(4)
plot(timestamp(1:L),winkel(1:L))
hold on;
plot(timestamp(1:L),testprofile(1:L,17))
hold off;
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
legend("EKF IMU1","Encoder")
title("EKF")
set(gca,'FontSize',14);
figure(5)
plot(timestamp(1:end-1),diff_fzw(:))
xlabel("Zeit in Sekunden")
ylabel("Fahrzeugwinkel")
title("Fahrzeugwinkel")
set(gca,'FontSize',14);
variable_info = whos;
total_memory =sum([variable_info.bytes]);
memory_in_kb = total_memory / 1024;
memory_in_mb = total_memory / (1024^2);
memory_in_gb = total_memory / (1024^3);
fprintf('Speicherbedarf: %.2f Bytes (%.2f KB, %.2f MB, %.2f GB)\n', total_memory, memory_in_kb, memory_in_mb, memory_in_gb);
