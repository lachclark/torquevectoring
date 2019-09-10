operating_points = 8;

m = 340;
Cyr = 37987.102;
Cyf = 33632.623;
Lf = 0.55;
Lr = 0.525;

% calculate understeer coefficient for the car
ku = (Lr*m)/(Cyf*(Lf+Lr)) - (Lf*m)/(Cyr*(Lf+Lr));



for k = 0:operating_points-1

    plants(:,:,k+1) = makeBikeModel(2*k+1);

end