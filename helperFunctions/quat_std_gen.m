function out = quat_std_gen(start,stop,acc,mag)

    [accelRow,accelCol] = size(acc);
    
    if accelRow>accelCol
        acc = acc';
        mag = mag';
    end
    
    acc = acc(:,start:stop);
    mag = mag(:,start:stop);

    for i = start:stop-1
       
        theta = atan2(-acc(1,i),sqrt(acc(2,i)^2 + acc(3,i)^2)); % Pitch
        
        phi = atan2(acc(2,i),acc(3,i)); % Roll
        
        psi = atan2(mag(3,i)*sin(phi) - mag(2,i)*cos(phi), ...
                      mag(1,i)*cos(theta) + mag(2,i)*sin(theta)*sin(phi) ...
                    + mag(3,i)*sin(theta)*cos(phi)); % Yaw
                
        quat(i,:) = eul2quat([psi theta phi]);

    end
    
out = std(quat);
    
end