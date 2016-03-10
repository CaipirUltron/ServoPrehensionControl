% Calculation of the joint angles for each finger, based on the empirical
% model. All angles are expressed in radians.

function [ o1 , o2 , o3 ] = Finger_Angles( om )

ok1 = [0 0 0];
ok2 = [0 0 0];
ok3 = [0 0 0];

lim = [ deg2rad(53.6) deg2rad(55) deg2rad(44.5)];
offset = deg2rad(21.2);
kr = 9/4;

for i = 1:3
    if om(i)<= lim(3)
        ok1(i) = om(i)-offset;
        ok2(i) = 0;
        ok3(i) = -ok1(i);
    elseif ((om(i)>lim(3))&&(om(i)<=lim(1)))
        ok1(i) = om(i)-offset;
        ok2(i) = 0;
        ok3(i) = -(lim(3) - offset);
    elseif ((om(i)>lim(1))&&(om(i)<=lim(2)))
        ok1(i) = lim(1) - offset;
        ok2(i) = 0;
        ok3(i) = -(lim(3) - offset);
    elseif om(i)>lim(3)
        ok1(i) = lim(1) - offset;
        ok2(i) = kr*(om(i) - lim(2));
        ok3(i) = -(lim(3) - offset);
    end
end

o1 = [ ok1(1) ok2(1) ok3(1) ];         % finger 1
o2 = [ ok1(2) ok2(2) ok3(2)  om(4) ];   % finger 2
o3 = [ ok1(3) ok2(3) ok3(3) -om(4) ];   % finger 3

end
