% Conversion from angular velocity of the motors to bytes. The velocities 
% are given in radians/second. Flag is a 4-four vector that indicates when 
% the control algorithm is generating byte values above byte_maxlim and/or 
% under byte_minlim (+1/-1 indicates above/under in the respective byte).

function [ byte , flag ] = os2byte( dom )
    
% Parameters
fact = [ 34.5  100 ];
offset = [ -6.5 -6.5 ];
byte_maxlim = 255;
byte_minlim = 0;

% Initialize byte and flag vectors
byte = byte_minlim*[ 1 1 1 1 ];
flag = [ 0 0 0 0 ];
for i = 1:4
    if i <= 3
        byte(i) = fact(1)*dom(i)+offset(1);
    else
        byte(i) = fact(2)*dom(i)+offset(2);
    end
    if ( byte(i) > byte_maxlim )
        byte(i) = byte_maxlim;
        flag(i) = 1;
    elseif ( byte(i) < byte_minlim )
        byte(i) = byte_minlim;
        flag(i) = -1;
    else
        flag(i) = 0;
    end
end

byte = round(byte);
    
end

