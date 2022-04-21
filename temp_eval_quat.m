clc; close all;

meas = quat2eul(q_hat');

time = quat2eul(orient);

figure()
subplot(3,1,1)
plot(meas(:,2),'.')
hold on
plot(time(:,2),'.')
title('Pitch')
legend('PF','VN 300')
subplot(3,1,2)
plot(meas(:,3),'.')
hold on
plot(time(:,3),'.')
title('Roll')
legend('PF','VN 300')
subplot(3,1,3)
plot(meas(:,1),'.')
hold on
plot(time(:,1),'.')
title('Yaw')
legend('PF','VN 300')