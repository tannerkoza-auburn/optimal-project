%% Optimal Estimation Project - AHRS Particle Filter

clear
clc
close all

% Nicholas Ott questions from paper:
    % Eq (22) -> what is DCM_*q_hat? (second DCM)
    % Eq (24) -> what is C?
    % Eq (15) -> we need to define DCM (this is probably simple I am just
        % not there yet
    
    
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
[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:)); % 

q_hat = [1;0;0;0]; % overall state estimate

q_calc = 0;

W = 1/N*ones(1,N); % particle filter weights
C = 1; % Confirm what this is (but I think this is a variance term in the posterior weight calcualtion)

%% Particle Filter

for i = 1:numSamps

    for j = 1:N

        % Time Update (think this is also wrong)
        samp = gyro(i,:)' + sigmaGyro'.*rand(3,1); % uniform distribution (mean is gyro noise, variance is noise floor)
        qP(:,j) = q_hat(:,i); 
        gyroP = gyro(i,:)' + samp; % Particle Gyro (resampling based on gyro measurements and noise parameters)
        
        % thoughts: quaternion particle is the mean from the previous time
        % step, resampling occurs by propogating the quaternion through a
        % time update with random noise creating unique particles
        
        F = [1 -0.5*gyroP(1)*dt -0.5*gyroP(2)*dt -0.5*gyroP(3)*dt;...
            0.5*gyroP(1)*dt 1 0.5*gyroP(3)*dt -0.5*gyroP(2)*dt;...
            0.5*gyroP(2)*dt -0.5*gyroP(3)*dt 1 0.5*gyroP(1)*dt;...
            0.5*gyroP(3)*dt 0.5*gyroP(2)*dt -0.5*gyroP(1)*dt 1];
        
        qP(:,j) = F*qP(:,j); % Particle Propagation (time Update) % propogate from the same estimate?
        
        dcm_time = dcm_calc(qP(:,j)); % DCM from quaternion estimate
        
        % Measurement Update
        theta = atan2(-acc(i,1),sqrt(acc(i,2)^2 + acc(i,3)^2)); % Pitch
        phi = atan2(acc(i,2),acc(i,3)); % Roll
        psi = atan2(mag(i,3)*sin(phi) - mag(i,2)*cos(phi), ...
            mag(i,1)*cos(theta) + mag(i,2)*sin(theta)*sin(phi) ...
            + mag(i,3)*sin(theta)*cos(phi)); % Yaw
        
        % --- Measured Quaternion! --- %
        qM(:,j) = [cos(psi/2)*cos(theta/2)*cos(phi/2) + sin(psi/2)*sin(theta/2)*sin(phi/2);
                   cos(psi/2)*cos(theta/2)*sin(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2);
                   cos(psi/2)*sin(theta/2)*cos(phi/2) + sin(psi/2)*cos(theta/2)*sin(phi/2);
                   sin(psi/2)*cos(theta/2)*cos(phi/2) - sin(psi/2)*sin(theta/2)*cos(phi/2)];
        
        dcm_meas = dcm_calc(qM(:,j));
        
        % --- Likeliehood calc --- %
        dcm_diff = dcm_meas-dcm_time; % difference in DCM matrices
        rX = dcm_diff(:,1); % first column
        rY = dcm_diff(:,2); % second column
        rZ = dcm_diff(:,3); % third column
        
        L = 1/(norm(rX)*norm(rY)*norm(rZ)); % Calculate volume of ellipsoid defined by all 3 axes (rX,rY,rZ)
        
        W(j) = W(j)*L; % dont't think this is wrong
                
    end
    
    W = W./sum(W); % normalize weights
    
    for j = 1:N
        
        q_calc = q_calc + W(j)*qP(:,j); % I dont think this is wrong
           
    end
    
    q_hat(:,i+1) = q_calc; % state estimate is overall weighted sum 
    
    q_calc = 0;
        
end

%% Converting to Euler Angles

[r,c] = size(q_hat);

q_hat = q_hat';

for i = 1:r
    eul(i,:) = quat2eul(q_hat(i,:),'XYZ');
end

figure()
subplot(3,1,1)
plot(eul(:,1),'.')
title('Roll')
subplot(3,1,2)
plot(eul(:,2),'.')
title('Pitch')
subplot(3,1,3)
plot(eul(:,3),'.')
title('Yaw')