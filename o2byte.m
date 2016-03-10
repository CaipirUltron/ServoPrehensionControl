% Conversion from motor angles to bytes. The motor angles are given in 
% radians. Flag is a 4-four vector that indicates when the control 
% algorithm is generating byte values above byte_maxlim and/or under 
% byte_minlim (+1/-1 indicates above/under in the respective byte).

function [ byte , flag ] = o2byte( thetaHand )
    
% Parameters
fact = [ (1/deg2rad(0.4)) (-1/deg2rad(0.13)) ];
offset = [ deg2rad(2.4) deg2rad(-17.82) ];
byte_maxlim = 255;
byte_minlim = 6;

% Initialize byte and flag vectors
byte = byte_minlim*[ 1 1 1 1 ];
flag = [ 0 0 0 0 ];
for i = 1:4
    if i <= 3
        byte(i) = fact(1)*(thetaHand(i)+offset(1));
    else
        byte(i) = fact(2)*(thetaHand(4)+offset(2));
    end
    if ( byte(i) >= byte_maxlim )
        byte(i) = byte_maxlim;
        flag(i) = 1;
    elseif ( byte(i) <= byte_minlim )
        byte(i) = byte_minlim;
        flag(i) = -1;
    else
        flag(i) = 0;
    end
end

byte = round(byte);
    
end

