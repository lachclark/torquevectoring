operating_points = 8;



% for ts_18

m = 336.83;

Cyr = 20000;

Cyf = 18200;

Lf = 0.918;

Lr = 0.612;



% % for 'base'

% m = 600;
% 
% Cyr = 17000;
% 
% Cyf = 11500;
% 
% Lf = 2.56;
% 
% Lr = 1.02;



% calculate understeer coefficient for the car

ku = (Lr*m)/(Cyf*(Lf+Lr)) - (Lf*m)/(Cyr*(Lf+Lr));



for k = 0:operating_points-1

    plants(:,:,k+1) = makeBikeModel(2*k+1);

end