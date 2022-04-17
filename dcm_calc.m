function out = dcm_calc(q)

    % purpose of function is to generate the direction cosine matrix (DCM)
    % for the purposes of the measurement update in the particle filter in
    % a magneticlly clean environment

    % Source: Howard's Dissertation:
    
    out = [q(1)^2+q(2)^2-q(3)^2-q(4)^2 2*(q(2)*q(3)-q(1)*q(4))     2*(q(1)*q(3)+q(2)*q(4));...
           2*(q(1)*q(4)+q(2)*q(3))     q(1)^2-q(2)^2+q(3)^2-q(4)^2 2*(q(3)*q(4)-2*q(1)*q(2));...
           2*(q(2)*q(4)-q(1)*q(3))     2*(q(1)*q(2)+q(3)*q(4))     q(1)^2-q(2)^2-q(3)^2+q(4)^2];
end