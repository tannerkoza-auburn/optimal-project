%% Optimal Estimation Project - AHRS Particle Filter

clear
clc
close all

%% Data Import

% NOTE: If unable to find file or directory, run dataParser.m or check your
% file name.
load('vn300.mat');

% Extract Fields from IMU Structure
acc = imu.acc;
gyro = imu.gyro;

%% Time Parameters
if ~isfield(imu,'time')
    fs = input('What was your sampling frequency in Hz? ');
    dt = 1/fs;
    mag = imu.mag;
else
    mag = mag.mag;
end
numSamps = min(length(gyro),length(mag)); % # of Samples

%% Particle Filter Parameters

N = 500; % Number of Particles
[roll, pitch] = acc2RP(acc(1,:));
yaw = mag2Y(mag(1,:),roll,pitch);
 % Initial Quaternions for Particles
qP = [ones(1,N); zeros(3,N)];
% qP(:,1) = eul2Quat(roll,pitch,yaw);
wP = (1/N)*ones(N,1); % Initial Particle Weights

[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:))/10; % Gyro Floor Noise Standard Deviation
qHatL = zeros(4,numSamps);
qHatL(:,1) = [1 0 0 0];

%% Particle Filter

for i = 2:numSamps

    if isfield(imu,'time')
        dt = imu.time(i)-imu.time(i-1);
    end

    for j = 1:N

        % Time Update
        gyroP = gyro(i,:) + sigmaGyro*rand; % Particle Gyro

        F = [1 -0.5*gyroP(1)*dt -0.5*gyroP(2)*dt -0.5*gyroP(3)*dt;...
            0.5*gyroP(1)*dt 1 0.5*gyroP(3)*dt -0.5*gyroP(2)*dt;...
            0.5*gyroP(2)*dt -0.5*gyroP(3)*dt 1 0.5*gyroP(1)*dt;...
            0.5*gyroP(3)*dt 0.5*gyroP(2)*dt -0.5*gyroP(1)*dt 1];

        qP(:,j) = F*qP(:,j); % Particle Propagation

    end

    % Measurement Update
    [roll, pitch] = acc2RP(acc(i,:));
    yaw = mag2Y(mag(i,:),roll,pitch);
    qM = eul2Quat(roll,pitch,yaw);

    % Likelihood Function
    qMdcm = quat2dcm(qM');
    qPdcm = quat2dcm(qP');
    dcmDiff = qMdcm - qPdcm;
    L = 1/vecnorm((dcmDiff(:,1,:)).*vecnorm(dcmDiff(:,2,:)).*vecnorm(dcmDiff(:,3,:)));
    wP = wP.*permute(L,[3 2 1]);
    wP = wP/sum(wP);

    % Orientation Estimate
    qHat = sum(wP'.*qP,2);

    % Resampling
    nEff = 1/sum(wP.^2);
    nEffT = 0.5*N; % Effective Particle Threshold

    if nEff<nEffT
        idx = resample(wP,N);
        qP = qP(:,idx);
        wP = (1/N)*ones(N,1);
    end

    % Log
    qHatL(:,i) = qHat;

end

%% Conversion
eulPF = quat2eul(qHatL');
eulVN = quat2eul(imu.quat);

figure
plot(rad2deg(eulPF(:,3)))
hold on
plot(rad2deg(eulVN(:,3)))