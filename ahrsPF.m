%% Optimal Estimation Project - AHRS Particle Filter

clear
clc
close all

% Nicholas Ott questions from paper:
    % Eq (22) -> what is DCM_*q_hat? (second DCM)
    % Eq (24) -> what is C?
    % Eq (15) -> we need to define DCM (this is probably simple I am just
        % not there yet
        
% Resampling Thoughts:
    % Effective Sample Size method
        % when effective sample size gets below threshold, then resampling
        % is needed
        % cv = (1/N)*sum(N*w(i)-1)^2 from i = 1 -> N
        % ESS = M/(1+cv^2)
        
        % When ESS < ESS_threshold, perform resampling
        % ESS_threshold = 0.5*N (50% of particles are ineffective)
        
% Resampling algorithm is it's own function


    
%% Data Import

% NOTE: If unable to find file or directory, run dataParser.m or check your
% file name.
load('vn300Data.mat');
% load('softsysIMU.mat');

% Extract Fields from IMU Structure
acc = imu.acc;
gyro = imu.gyro;
mag = imu.mag;

orient = imu.odom;

%% Time Parameters

% fs = 128; % Sampling Frequency (Hz)
% dt = 1/fs; % Sampling Period (s)

dt = mean(diff(imu.time));
numSamps = length(gyro); % # of Samples

%% Initialization

theta = atan2(-acc(1,1),sqrt(acc(1,2)^2 + acc(1,3)^2)); % Pitch
phi = atan2(acc(1,2),acc(1,3)); % Roll
psi = atan2(mag(1,3)*sin(phi) - mag(1,2)*cos(phi), ...
              mag(1,1)*cos(theta) + mag(1,2)*sin(theta)*sin(phi) ...
            + mag(1,3)*sin(theta)*cos(phi)); % Yaw
        
q0 = eul2quat([psi theta phi])';

n = 4;
%% Particle Filter Parameters

N = 500; % Number of Particles

ESS_thresh = 1*N; % when 90% of particles are "ineffective" resample

qP = [q0(1)*ones(1,N); q0(2)*ones(1,N); q0(3)*ones(1,N); q0(4)*ones(1,N)]; % Initial Quaternions for Particles
[start,stop] = staticGyro(gyro, 0.2); % Static Indices
sigmaGyro = std(gyro(start:stop,:))*2; % 

q_hat = q0; % overall state estimate

W = 1/N*ones(1,N); % particle filter weights
C = 1; % Confirm what this is (but I think this is a variance term in the posterior weight calcualtion)

%% Particle Filter

for i = 2:numSamps-1

    for j = 1:N

        % Time Update (think this is also wrong)
        samp = sigmaGyro'.*rand(3,1); % uniform distribution (mean is gyro noise, variance is noise floor)
        gyroP = gyro(i,:)' + samp; % Particle Gyro (resampling based on gyro measurements and noise parameters)
        
        % thoughts: quaternion particle is the mean from the previous time
        % step, resampling occurs by propogating the quaternion through a
        % time update with random noise creating unique particles
        
        F = [1 -0.5*gyroP(1)*dt -0.5*gyroP(2)*dt -0.5*gyroP(3)*dt;...
            0.5*gyroP(1)*dt 1 0.5*gyroP(3)*dt -0.5*gyroP(2)*dt;...
            0.5*gyroP(2)*dt -0.5*gyroP(3)*dt 1 0.5*gyroP(1)*dt;...
            0.5*gyroP(3)*dt 0.5*gyroP(2)*dt -0.5*gyroP(1)*dt 1];
        
        qP(:,j) = F*qP(:,j); % Particle Propagation (time Update)
        
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
    
    W_norm = W./sum(W); % normalize weights
    
    ESS = 1/(sum(W_norm.^2));
    
    q_hat(:,i+1) = sum(W_norm.*qP,2);
    
    if ESS<ESS_thresh
        
        % --- Perform Resampling --- % 
            % Try resampling from select w/ replacement strategy
            Index = resample(W_norm);
            qP = qP(:,Index);
            W = (1/N)*ones(1,N);
            
    end
    
    if i == 1018
        disp('Pause')
    end
                
end

disp('ended')
%% Converting to Euler Angles

% [r,c] = size(q_hat);
% 
% q_hat = q_hat';
% 
% for i = 1:c
%     eul(i,:) = quat2eul(q_hat(i,:));
% end
% 
% [r,c] = size(orient);
% 
% for i = 1:r
%    truth(i,:) = quat2eul(orient(i,:)); 
% end
% 
% yaw = rad2deg(eul(:,1));
% pitch = rad2deg(eul(:,2));
% roll = rad2deg(eul(:,3));
% 
% yaw_truth = rad2deg(truth(:,1));
% pitch_truth = rad2deg(truth(:,2));
% roll_truth = rad2deg(truth(:,3));
% 
% figure()
% subplot(3,1,1)
% plot(roll,'.')
% hold on
% plot(roll_truth,'.')
% title('Roll')
% subplot(3,1,2)
% plot(pitch,'.')
% hold on
% plot(pitch_truth,'.')
% title('Pitch')
% subplot(3,1,3)
% plot(yaw,'.')
% hold on
% plot(yaw_truth,'.')
% title('Yaw')