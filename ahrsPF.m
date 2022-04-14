%% Optimal Estimation Project - AHRS Particle Filter

clear
clc
close all

% Nicholas Ott questions from paper:
    % Eq (22) -> what is DCM_*q_hat? (second DCM)
    % Eq (24) -> what is C?
    
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
q(:,:,:) = [ones(1,N); zeros(3,N)]; % Initial Quaternion
[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:)); % 

depth = 1; % I want to track how particles evolve over time so add a third dimension to matrix of particles

%% Particle Filter

for i = 1:numSamps

    for j = 1:N
    
        % Time Update
        gyroP = gyro(i,:) + sigmaGyro*rand; % Particle Gyro (applies the same input plus random noise to all particles)

        A = [1 -0.5*gyroP(1)*dt -0.5*gyroP(2)*dt -0.5*gyroP(3)*dt;... 
            0.5*gyroP(1)*dt 1 0.5*gyroP(3)*dt -0.5*gyroP(2)*dt;...
            0.5*gyroP(2)*dt -2.5*gyroP(3)*dt 1 0.5*gyroP(1)*dt;...
            0.5*gyroP(3)*dt 0.5*gyroP(2)*dt -0.5*gyroP(1)*dt 1];
        
        q(:,j,depth+1) = A*q(:,j,depth); % Propagation

    end
    
    depth = depth + 1;

end