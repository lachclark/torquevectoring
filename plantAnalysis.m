% for generating step-response plots, etc to analyse the
% bike model state space systems used to tune controllers

% run 'makePlants.m' beforehand to populate the workspace with plants.
% Each plant corresponds to the car at different operating speeds from
% 1-15m/s at increments of 2m/s (ie: 1, 3, 5...15m/s). If the index of a
% plant is 'i', then its corresponding operating speed = 2i + 1
% eg: the operating point of plants(:,:,3) = 2*3 + 1 = 7m/s

% for comparing step response of all systems
% right click + IO Selector to select bottom left plot of interest
% (ie: delta torque to yaw rate plot is most important for TV
figure('Name', 'Step Response Comparison');
step(plants(:,:,1),'b',plants(:,:,2),'g',plants(:,:,3),'r', plants(:,:,4),'c',plants(:,:,5),'m',plants(:,:,6),'y', plants(:,:,7),'k',plants(:,:,8),'--b')
legend('1m/s','3m/s','5m/s','7m/s','9m/s','11m/s','13m/s','15m/s')

figure('Name', 'Pole-Zero Map Comparison');
iopzplot(plants(:,:,1),'b',plants(:,:,2),'g',plants(:,:,3),'r', plants(:,:,4),'c',plants(:,:,5),'m',plants(:,:,6),'y', plants(:,:,7),'k',plants(:,:,8),'--b')
legend('1m/s','3m/s','5m/s','7m/s','9m/s','11m/s','13m/s','15m/s')

