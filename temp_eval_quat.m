clc; close all;

meas = quat2eul(q_hat');

time = quat2eul(orient);

figure()
subplot(3,1,1)
plot(rad2deg(meas(:,2)),'.')
hold on
plot(rad2deg(time(:,2)),'.')
title('Pitch')
legend('PF','VN 300')
subplot(3,1,2)
plot(rad2deg(meas(:,3)),'.')
hold on
plot(rad2deg(time(:,3)),'.')
title('Roll')
legend('PF','VN 300')
subplot(3,1,3)
plot(rad2deg(meas(:,1)),'.')
hold on
plot(rad2deg(time(:,1)),'.')
title('Yaw')
legend('PF','VN 300')