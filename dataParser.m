% Optimal Estimation Project - Data Parser

clear
clc

useText = 0;

useBag = 1; % bag parser is setup for vectornav IMU!

%% Restructure Data

if useText
    
% Directory Declaration
inDir = 'data/rawData/';
inFiles = [dir(strcat(inDir,'*.txt')); dir(strcat(inDir,'*.bag'))];
outDir = 'data/structData';

% Incorrect Working Directory Handling
numFiles = length(inFiles);

if numFiles == 0
    error('No files have been read. Run this file from the root directory.')
end

% IMU Topic
imuTopic = '';

% Structure Population
for i = 1:numFiles

    % Create Destination File
    thisFile = inFiles(i).name;
    [~, ~, fileExt] = fileparts(thisFile);
    outFile = strcat(strrep(thisFile,fileExt,''),'.mat');
    destFile = fullfile(outDir,outFile);

    if strcmp(fileExt,'.txt') == true

        % Import Data
        imuTable = readtable(thisFile);
        imuArr = table2array(imuTable);

        % Populate Struct
        imu.acc = imuArr(:,1:3);
        imu.gyro = imuArr(:,4:6);
        imu.mag = imuArr(:,7:9);

    else
        % Populate Struct
        imu = importIMUBag(inDir,thisFile,imuTopic);
    end

    save(destFile,"imu")
    clearvars imu
end

end

if useBag
    
    bagfilename = 'vn300_ahrsPF_2022-04-18-14-21-51.bag';
    bag = rosbag(bagfilename);
    
    %show all topics
    topics=bag.AvailableTopics;
    %select topic
    selected_topic = select(bag,'Topic','/vectornav/IMU');
    %read topic message
    struct = readMessages(selected_topic,'DataFormat','struct');
    
    % --- Gyro and Accel --- %
    
    for i = 1:length(struct)
       
        % --- IMU Gyro --- %
        imu.gyro(i,1) = struct{i}.AngularVelocity.X;
        imu.gyro(i,2) = struct{i}.AngularVelocity.Y;
        imu.gyro(i,3) = struct{i}.AngularVelocity.Z;
        
        % --- IMU Accel --- %
        imu.acc(i,1) = struct{i}.LinearAcceleration.X;
        imu.acc(i,2) = struct{i}.LinearAcceleration.Y;
        imu.acc(i,3) = -struct{i}.LinearAcceleration.Z;
        
        imu.time(i) = double(struct{i}.Header.Stamp.Sec)+double(struct{i}.Header.Stamp.Nsec)*1e-9;
        
    end
    
    % --- Mag --- %
    
    selected_topic = select(bag,'Topic','/vectornav/Mag');
    struct = readMessages(selected_topic,'DataFormat','struct');
    
    for i = 1:length(struct)
        
       imu.mag(i,1) = struct{i}.MagneticField.X;
       imu.mag(i,2) = struct{i}.MagneticField.Y;
       imu.mag(i,3) = struct{i}.MagneticField.Z;
        
    end
    
    selected_topic = select(bag,'Topic','/vectornav/Odom');
    struct = readMessages(selected_topic,'DataFormat','struct');
    
    for i = 1:length(struct)
       
        imu.odom(i,1) = struct{i}.Pose.Pose.Orientation.W;
        imu.odom(i,2) = struct{i}.Pose.Pose.Orientation.X;
        imu.odom(i,3) = struct{i}.Pose.Pose.Orientation.Y;
        imu.odom(i,4) = struct{i}.Pose.Pose.Orientation.Z;
        
    end
    
    save('data/structData/vn300Data.mat','imu');
    
end