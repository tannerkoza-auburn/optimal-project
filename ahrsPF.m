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
wP = (1/N)*ones(1,N); % Initial Particle Weights

[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:)); % Gyro Floor Noise Standard Deviation

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
    theta = atan2(-acc(i,1),sqrt(acc(i,2)^2 + acc(i,3)^2)); % Pitch
    phi = atan2(acc(i,2),acc(i,3)); % Roll
    psi = atan2(mag(i,3)*sin(phi) - mag(i,2)*cos(phi), ...
        mag(i,1)*cos(theta) + mag(i,2)*sin(theta)*sin(phi) ...
        + mag(i,3)*sin(theta)*cos(phi)); % Yaw

    qM = [cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
        cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
        cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
        sin(psi/2)*cos(theta/2)*cos(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2)];

end