function out = dcm_calc(roll,pitch,yaw)

    % purpose of function is to generate the direction cosine matrix (DCM)
    % for the purposes of the measurement update in the particle filter in
    % a magneticlly clean environment

    % CONFIRM THIS:
psi = roll;
phi = pitch;
theta = yaw;
        
    out = [cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);...
           cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);...
           -sin(theta)         sin(phi)*cos(theta)                            cos(phi)*cos(theta)];



end