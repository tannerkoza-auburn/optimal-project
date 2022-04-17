clc; close all;

[r,c] = size(q_hat);

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