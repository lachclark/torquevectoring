function [theModel] = makeBikeModel(Vxo)
%MAKEBIKEMODEL returns a state-space model linearised around a certain Vxo

% the parameters (constants) necessary to descibe the linear state space model
%are:
%   Cyr - Cornering stiffness at rear wheel [Nrad^?1]
%   Cyf - Cornering stiffness at front wheel [Nrad^?1]
%   Izz - Inertia moment 120 [Kgm^2]
%   m - Mass [Kg]
%   Lf - Front wheelbase [m]
%   Lr - Rear wheelbase [m]
%   Gr - gear ratio [-]
%   tr - half the track of the car [m]
%   Rw - Radius of the wheel [m]
%   Vxo - Longitudinal Velocity [ms^-1] ** 
%   (**assumed constant throughout trajectory but this can vary (eg: 0-40),
%      Need to use some kind of lookup table for the PID controller gains
%      for varied ranges of long. velocity)
%   
%   (see Dissertacao.pdf p24 onwards for more info)

% (Inputs):
%   [delta] - Steering angle ? [rad]
%   Mz - assisting yaw moment [Nm]
%   (or ?T - torque difference at the rear wheels [Nm]*)
%
% Xdot = [Vy_dot
%         Yaw_doubledot]

% X = [Vy
%      Yaw_dot]
%
% U = [Mz
%      ?]

%acquired values from ts_18(?) CAD
Cyr = 37987.102;
Cyf = 33632.623;
Izz = 110;
m = 340;
Lf = 0.55;
Lr = 0.525;
Gr = 2.2;
tr = 0.525;
Rw = 0.232;

% *for calculating ?T from Mz (assisting yaw moment)
%  (uncomment Gr, tr, Rw first)
k = Rw/(2*tr*Gr);

% 'A' matrix calculations
A11 = -(Cyf+Cyr)/(m*Vxo);
A12 = (Lr*Cyr-Lf*Cyf)/(m*Vxo)-Vxo;
A21 = (Lr*Cyr-Lf*Cyf)/(Izz*Vxo);
A22 = -(Lf^2*Cyf+Lr^2*Cyr)/(Izz*Vxo);

% 'B' matrix calculations 
B11 = 0;
B12 = Cyf/(m*Vxo);
B21 = 1/(k*Izz);
B22 = (Lf*Cyf)/Izz;

A = [A11 A12;
     A21 A22];

B = [B11 B12;
     B21 B22];

% 'C' = I
C = eye(2);

% No 'D' feedforward matrix (yet?)
D = 0;

states = {'lateral_velocity','yaw_rate'};
inputs = {'delta_torque','steer_angle_o'};
outputs = states;

theModel = ss(A,B,C,D,'StateName', states, 'InputName', inputs, 'OutputName', outputs);

end

