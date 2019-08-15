%make bike model plants linearised around various longitudinal velocity
%values, these are the 'operating points' for each plant.
%The velocity values are a sequence of odd numbers starting from 1
%eg: 8 operating points gives 1,3,5...15 (2i+1 given i=0:7)

operating_points = 8;

for k = 0:operating_points-1
    plants(:,:,k+1) = makeBikeModel(2*k+1);
end

%step(plants)
%bode(plants)
%yes, all plants at different operating points are stable (output 1111111..)
%isstable(plants,'elem')
%step(plants(:,:,1),'b',plants(:,:,2),'g',plants(:,:,3),'r',plants(:,:,4),'c',plants(:,:,5),'m',plants(:,:,6),'y',plants(:,:,7),'k',plants(:,:,8),'--b')
%step(bike_model_1,'b',bike_model_3,'g',bike_model_5,'r',bike_model_7,'c',bike_model_9,'m',bike_model_11,'y',bike_model_13,'k',bike_model_15,'--b')
%legend('1m/s','3m/s','5m/s','7m/s','9m/s','11m/s','13m/s','15m/s')

%make a PID block to control the torque delta at the rear wheels
PID_T = tunablePID('PID_T', 'PID');
PID_T.InputName = 'yaw_rate_error';
PID_T.OutputName = 'delta_torque';

%make a block to calculate the yaw rate error
%(inputs - yaw_ref, yaw_rate; output - yaw_rate_error
errorCalc = sumblk('yaw_rate_error = yaw_ref - yaw_rate');

%make block that essentially feeds input steering angle straight to its output
unityBlk = tunableGain('unity',1,1);
unityBlk.InputName = 'steer_angle_i';
unityBlk.OutputName = 'steer_angle_o';
unityBlk.Gain.Minimum = 0.999999;
unityBlk.Gain.Maximum = 1.000001;

%connect all the blocks together in desired configuration (see diagram*), store in C0
%this C0 is the controller section of the overall system 
C0 = connect(PID_T,errorCalc,unityBlk,{'yaw_ref','yaw_rate','steer_angle_i'},{'delta_torque','steer_angle_o'});

%connect the C0 controller to the plant state space model and tune the PID
%gains for this plant
[plants(:,:,1),C,gam,Info] = looptune(plants(:,:,1),C0,10);

%show the the PID gains, stored in 'C' variable
showTunable(C)