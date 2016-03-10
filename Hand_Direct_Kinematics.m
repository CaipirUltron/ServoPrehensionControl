% Direct kinematics of the hand. Receives the 4-vector with the four motor 
% angles of the robot hand. Returns the position vectors and rotation 
% matrices associated to each finger. All angles are in radians.

function [ p1, R1, p2, R2, p3, R3 ] = Hand_Direct_Kinematics( thetaHand )

% Length of the phalanges and constant displacement of the fingers, related
% to the palm frame (both in millimeters).
l0 = 22;
l1 = 57;
l2 = 38;
l3 = 22;
pp1 = [  0   -44.5  -10 ];
pp2 = [ 36.5  44.5  -32 ];
pp3 = [-36.5  44.5  -32 ];

% Empirical model for the finger angles
[ o1 , o2 , o3 ] = Finger_Angles(thetaHand);
o11 = o1(1);
o12 = o1(2);
o13 = o1(3);
o21 = o2(1);
o22 = o2(2);
o23 = o2(3);
o24 = o2(4);
o31 = o3(1);
o32 = o3(2);
o33 = o3(3);
o34 = o3(4);

% Sine and cosine conventions for summation
s4_2 = sin(o24);
s4_3 = sin(o34);
c4_2 = cos(o24);
c4_3 = cos(o34);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s123_1 = sin(o11+o12+o13);
s123_2 = sin(o21+o22+o23);
s123_3 = sin(o31+o32+o33);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
c123_1 = cos(o11+o12+o13);
c123_2 = cos(o21+o22+o23);
c123_3 = cos(o31+o32+o33);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ls1 = l1*sin(o11)+l2*sin(o11+o12)+l3*s123_1;
ls2 = l1*sin(o21)+l2*sin(o21+o22)+l3*s123_2;
ls3 = l1*sin(o31)+l2*sin(o31+o32)+l3*s123_3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
lc1 = l1*cos(o11)+l2*cos(o11+o12)+l3*c123_1;
lc2 = l1*cos(o21)+l2*cos(o21+o22)+l3*c123_2;
lc3 = l1*cos(o31)+l2*cos(o31+o32)+l3*c123_3;

% Position vector for each finger (with respect to the palm frame)
p1 = [ pp1(1) ; ls1 + pp1(2) ; lc1 + pp1(3) ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p2 = [  (lc2 + l0)*s4_2 + pp2(1)  ;  pp2(2) - ls2  ;   (lc2 + l0)*c4_2 + pp2(3) ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
p3 = [ +(lc3 + l0)*s4_3 + pp3(1)  ;  pp3(2) - ls3  ;  +(lc3 + l0)*c4_3 + pp3(3) ]; %%%---> verificar sinais!!

% Rotation matrices for each finger
R1 = [  1      0       0        ;...
        0    c123_1   s123_1    ;...
        0   -s123_1   c123_1    ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R2 = [   c4_2   s123_2*s4_2   c123_2*s4_2   ;...
           0      c123_2       -s123_2      ;...
        -s4_2   s123_2*c4_2   c123_2*c4_2   ];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
R3 = [   c4_3  -s123_3*s4_3  -c123_3*s4_3   ;...
           0      c123_3       -s123_3      ;...
         s4_3   s123_3*c4_3   c123_3*c4_3   ];

end