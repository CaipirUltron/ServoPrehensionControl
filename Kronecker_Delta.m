% Calculation of the modified Kronecker delta.

function [ dK1 , dK2 , dK3 ] = Kronecker_Delta( o )

% Parameters
lim = [ deg2rad(53.6) deg2rad(55) deg2rad(44.5)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if o <= lim(1)
    dK1 = 1;
else
    dK1 = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if o <= lim(2)
    dK2 = 1;
else
    dK2 = 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if o <= lim(3)
    dK3 = 1;
else
    dK3 = 0;
end
    
end