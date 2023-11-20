% EKF VARIABLES
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
testprofile = readmatrix("Wippe_Measurements\Schraeglage.csv");
Data = testprofile;
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


% Time between Measurements
for i = 1:length(testprofile)-1
deltat(i) = (testprofile(i+1,1)-testprofile(i,1))/1000000;
timestamp(i+1) = timestamp(i) + deltat(i);
end

pitch1 = atan2(testprofile(:,4),testprofile(:,6))*(180/pi);
pitch2 = atan2(testprofile(:,10),testprofile(:,12))*(180/pi);
angle_gyr1 = 0;
angle_gyr2 = 0;
for i = 1:N-1
angle_gyr1(i+1) = angle_gyr1(i) + deltat(i)*testprofile(i,8);
angle_gyr2(i+1) = angle_gyr2(i) + deltat(i)*testprofile(i,14);
end
B = 0.00009;
yZero = zeros(N);
x_k = [0.001 0.001 0.001];          % init x_
x_p = [0.001 0.001 0.001];          % init prädiktion
x_k2 = [0.001 0.001 0.001];          % init x_
x_p2 = [0.001 0.001 0.001];          % init prädiktion
R1 = [0.0227 0 0;
      0 0.0224 0;
      0 0 0.0225];

R1 = R1* 10.03;
  % Q_substituted1 = [0.00012 0 0;
  %                     0 0.00212 0;
  %                     0 0 0.00012];
R2 = [0.0227 0 0;
      0 0.0224 0;
      0 0 0.0225];
R2 = R2 * 100.03;
Q_substituted1 = eye(3) * B;
Q_substituted2 = eye(3) * B;
P1 = eye(3)*500;

P2 = eye(3)*500;
offset = 0.05;
offset2 = abs(0-mean(testprofile(1:30,10)));


% LKF VARIABLES



mean_gyr1 = mean(Data(:,7:9));
mean_gyr2 = mean(Data(:,13:15));

calgx1 = Data(:,7)-mean_gyr1(1);
calgy1 = -Data(:,8) - mean_gyr1(2);
calgz1 = Data(:,9) - mean_gyr1(3);

calgx2 = Data(:,13) - mean_gyr2(1);
calgy2 = Data(:,14) - mean_gyr2(2);
calgz2 = Data(:,15) - mean_gyr2(3);



pitch = atan2(Data(:,4)-offset,Data(:,6))*180/pi;
pitch_gyr = 0;
pitch2 = atan2(Data(:,10)-offset,Data(:,12))*180/pi;
for i = 1:N-1
    pitch_gyr(i+1) = pitch_gyr(i) + deltat(i)*(calgy1(i));
end
% Kalman-Filter Parameters

% initiate Kalman-Filter


    % State-Vector       
    x_hat = zeros(N,1);                 % Predict Matrix
x_hat2 = zeros(N,1);  
    % State Matrix 
    F = 1;
    F2 = 1;

    % Process Noise Covariance fix Values [not tuned]
        
    Q = B;

    % Measurement Covariance Noise [not tuned]

    R = 0.0225;
   
    % Observation Matrix H

    H = 1;
    H2 = 1;
    % Measurement Vector

    zk = zeros(1,1);

    % Covariance Matrix of Estimate

    Pk = eye(1)*5000;     % Beginnt mit großer unsicherheit
Pk2 = eye(1)*5000;
    % Kalman Gain

    K = 0;
    K2 = 0;

    Estimate = zeros(1,N);
 Estimate2 = zeros(1,N);
 % Kalman - Equations
 % INIT
 X_HAT_ = 0.001;
% EKF
for i = 1:N-1
    % Predict
    gx = deg2rad(calgx1(i));     % GyroX
    gy = deg2rad(calgy1(i));     % GyroY
    gz = deg2rad(calgz1(i));      % GyroZ
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
    pitch_atan = atan2(testprofile(i,4)-offset,testprofile(i,6));
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



% EKF 2
for i = 1:N-1
    % Predict
    gx = deg2rad(-calgx2(i));     % GyroX
    gy = deg2rad(-calgy2(i));     % GyroY
    gz = deg2rad(-calgz2(i));      % GyroZ
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
    pitch_atan = atan2(testprofile(i,10)-offset2,testprofile(i,12));
    pitch_asin = asin(testprofile(i,10)-offset2/sqrt((testprofile(i,10)-offset2)^2+testprofile(i,11)^2+testprofile(i,12)^2));
    roll = asin(-testprofile(i,11)/(sqrt((testprofile(i,10)-offset2)^2+testprofile(i,11)^2+testprofile(i,12)^2)*cos(pitch_atan)));
    
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
    winkel2(i) = 180/pi*x_p(i,2);
    P1 = (eye(3) - K1*H) * P_PRED1;
end
L = (N/2) -405;
diff_ekf = winkel(:)-winkel2(:);
figure(1)
subplot(1,2,1)
plot(timestamp(1:end-1),winkel(:))
hold on;
plot(timestamp(1:end),testprofile(:,17))
hold off;
title("EKF 1")
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
set(gca,'FontSize',14);
subplot(1,2,2)
plot(timestamp(1:end-1),winkel2(:))
xlabel("Zeit in Sekunden")
ylabel("Nickwinkel in Grad")
title("EKF 2")
set(gca,'FontSize',14);
figure(3)
plot(timestamp(1:L),-diff_ekf(1:L))
hold on;
plot(timestamp(1:L),-testprofile(1:L,17))
hold off;
xlabel("Zeit in Sekunden")
ylabel("Fahrzeugwinkel in Grad")
title("Fahrzeugwinkel")
legend("EKF FZW","Encoder FZW")
set(gca,'FontSize',14)
figure(4)
subplot(2,2,1)
plot(timestamp(1:L),pitch(1:L))
subplot(2,2,2)
plot(timestamp(1:L),angle_gyr1(1:L))
hold on;
plot(timestamp(1:L),angle_gyr2(1:L))
hold off;
subplot(2,2,3)
plot(timestamp(1:L),testprofile(1:L,4:6))
subplot(2,2,4)
plot(timestamp(1:L),testprofile(1:L,7:9))
figure(7)
% plot(timestamp(1:L),testprofile(1:L,10:12))
hold on;
plot(timestamp(1:L),testprofile(1:L,18:20))
hold off;
