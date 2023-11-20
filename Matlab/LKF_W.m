% Clear

close all hidden;
clear variables;



% Input Data
Data = readmatrix("Wippe_Measurements\Schraeglage.csv");            % Put Measurements here

% Number of Samples
N = length(Data(:,1));

% Timestamps

timestamp = zeros(1,length(Data(:,1)));

offset = abs(mean(0-Data(1:30,4)));
offset2 = abs(mean(0-Data(1:30,10)));
mean_gyr1 = mean(Data(:,21:23));
mean_gyr2 = mean(Data(:,27:29));

calgx1 = Data(:,7) -mean_gyr1(1);
calgy1 = -Data(:,8) - mean_gyr1(2);
calgz1 = Data(:,9) - mean_gyr1(3);

calgx2 = Data(:,13) - mean_gyr2(1);
calgy2 = Data(:,28) - mean_gyr2(2);
calgz2 = Data(:,15) - mean_gyr2(3);



% Time between Measurements
for i = 1:length(Data)-1
deltat(i) = (Data(i+1,1)- Data(i,1))/1000000;       % us -> s
timestamp(i+1) = timestamp(i) + deltat(i);
end


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
        
    Q = 0.0000055;

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
 X_HAT_2 = 0.001;
 % Estimate
 for i = 1:N-1
 % Subs syms in F
 X_HAT = X_HAT_- + deltat(i) * -calgy1(i);
 P_pred = F*Pk*F' + Q;

 % Update
 K = P_pred*H'*inv(H*P_pred*H'+R);

 % Feed ZK with Measurements

 zk = atan2(Data(i,4)-offset,Data(i,6))*180/pi;
 % Calculate Estimate with Kalmangain

 X_HAT_ = X_HAT + K*(zk-H*X_HAT);
 Pk = (eye(1)-K*H)*P_pred*(eye(1)-K*H)'+K*R*K';
 Estimate(:,i) = X_HAT;
 end


 for i = 1:N-1
 % Subs syms in F
 X_HAT2 = X_HAT_2- + deltat(i) * -calgy2(i);
 P_pred2 = F2*Pk2*F2' + Q;

 % Update
 K2 = P_pred2*H2'*inv(H2*P_pred2*H2'+R);

 % Feed ZK with Measurements

 zk2= atan2(Data(i,10)-offset2,Data(i,12))*180/pi;
 % Calculate Estimate with Kalmangain

 X_HAT_2 = X_HAT2 + K2*(zk2-H2*X_HAT2);
 Pk2 = (eye(1)-K*H)*P_pred2*(eye(1)-K2*H2)'+K2*R*K2';
 Estimate2(:,i) = X_HAT2;
 end

 KF(:,1) = timestamp(:);
 KF(:,2) = Estimate(1,:)';
 KF(:,3) = Data(:,17);
 % Calculate Diff
 Estimate_t = Estimate';
 Deltas = abs(Data(:,17) - KF(:,2));
 Q_estimate = cov(Deltas);
diff_lkf = Estimate(:)-Estimate2(:);
L = (N/2)-400;
 % Plotting
figure(1)
subplot(2,2,1)
plot(timestamp(1,1:end-1),Estimate(1,1:end-1));
hold on;
plot(timestamp(1,1:end-1),Data(1:end-1,17));
hold off;
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title('Linear-KF')
legend('LKF IMU1','Encoder')
set(gca,'FontSize',14);
subplot(2,2,2)
plot(timestamp(1:end-1),Deltas(1:end-1))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title("Differenz Encoder und LKF")
set(gca,'FontSize',14);
subplot(2,2,3)
plot(timestamp(1,1:end-1),pitch(1:end-1))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title('Winkel Acc.')
set(gca,'FontSize',14);
subplot(2,2,4)
plot(timestamp(1,1:end-1),pitch_gyr(1:end-1))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title('Winkel Gyroskop')
set(gca,'FontSize',14);




figure(2)
subplot(1,2,1)
plot(timestamp(1,1:end-1),Estimate(1,1:end-1));
hold on;
plot(timestamp(1,1:end-1),Data(1:end-1,17));
hold off;
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title('Linear-KF')
legend('LKF IMU1','Encoder')
set(gca,'FontSize',14);
subplot(1,2,2)
plot(timestamp(1:end-1),Deltas(1:end-1))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title("Differenz Encoder und LKF")
set(gca,'FontSize',14);
figure(3)
subplot(1,2,1)
plot(timestamp(1:L),Estimate(1:L))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title("LKF IMU1")
set(gca,'FontSize',14);
subplot(1,2,2)
plot(timestamp(1:L),Estimate2(1:L))
xlabel('Zeit in Sekunden')
ylabel('Nickwinkel in Grad')
title("LKF IMU2")
set(gca,'FontSize',14);
figure(4)
plot(timestamp(1:L),-diff_lkf(1:L))
hold on;
plot(timestamp(1:L),-Data(1:L,17))
hold off;
xlabel("Zeit in Sekunden")
ylabel("Fahrzeugwinkel in Grad")
legend("FZ-Winkel-LKF","encoder")
title("Fahrzeugwinkel")
set(gca,'Fontsize',14)
  variable_info = whos;
total_memory =sum([variable_info.bytes]);
memory_in_kb = total_memory / 1024;
memory_in_mb = total_memory / (1024^2);
memory_in_gb = total_memory / (1024^3);
fprintf('Speicherbedarf: %.2f Bytes (%.2f KB, %.2f MB, %.2f GB)\n', total_memory, memory_in_kb, memory_in_mb, memory_in_gb);