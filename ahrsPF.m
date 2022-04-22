%% Optimal Estimation Project - AHRS Particle Filter

clear
clc
close all

%% Data Import

% NOTE: If unable to find file or directory, run dataParser.m or check your
% file name.
load('softsysIMU.mat');

% Extract Fields from IMU Structure
acc = imu.acc;
gyro = imu.gyro;
mag = imu.mag;

%% Time Parameters

fs = 128; % Sampling Frequency (Hz)
dt = 1/fs; % Sampling Period (s)
numSamps = length(gyro); % # of Samples

%% Particle Filter Parameters

N = 500; % Number of Particles
qP = [ones(1,N); zeros(3,N)]; % Initial Quaternions for Particles
wP = (1/N)*ones(N,1); % Initial Particle Weights

[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:))/100; % Gyro Floor Noise Standard Deviation
qHatL = zeros(4,numSamps);

%% Particle Filter

for i = 1:numSamps

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