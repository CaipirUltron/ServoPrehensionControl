% Calculation of the restriction Jacobian matrices.

function [ Jr1, Jr2, Jc2, Jr3, Jc3 ] = Finger_Differential_Rel( thetaHand )

% Parameters
kr = 9/4;

% Motor angles
om1 = thetaHand(1);
om2 = thetaHand(2);
om3 = thetaHand(3);

% Parameters of the empirical model
[dK11,dK12,dK13] = Kronecker_Delta ( om1 );
[dK21,dK22,dK23] = Kronecker_Delta ( om2 );
[dK31,dK32,dK33] = Kronecker_Delta ( om3 );

% Restriction jacobians
Jr1 = [ dK11 ; kr*(1-dK12) ; -dK11*dK13 ];
Jr2 = [ dK21 ; kr*(1-dK22) ; -dK21*dK23 ; 0 ];
Jr3 = [ dK31 ; kr*(1-dK32) ; -dK31*dK33 ; 0 ];

% Coupled jacobians of finger 2 and 3
Jc2 = [ 0 0 0  1 ]';
Jc3 = [ 0 0 0 -1 ]';

end