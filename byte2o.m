% Conversion from byte to motor angles. The motor angles are given in 
% radians.

function [ thetaHand ] = byte2o( byte )
    
% Parameters
fact = [ deg2rad(0.4) deg2rad(-0.13) ];
offset = [ deg2rad(-2.4) deg2rad(17.82) ];
% thetaHand_maxlim = [ deg2rad(99.6) deg2rad(17.04) ];
% thetaHand_minlim = [ 0 deg2rad(-15.33) ];

% Initialize byte vector
thetaHand = [ 0 ; 0 ; 0 ; 0 ];
for i = 1:4
    if i <= 3
%         k = 1;
        thetaHand(i) = fact(1)*byte(i)+offset(1);
    else
%         k = 2;
        thetaHand(4) = fact(2)*byte(4)+offset(2);
    end
%     if ( thetaHand(i) >= thetaHand_maxlim(k) )
%         thetaHand(i) = thetaHand_maxlim(k);
%     elseif ( thetaHand(i) <= thetaHand_minlim(k) )
%         thetaHand(i) = thetaHand_minlim(k);
%     end
end
    
end

