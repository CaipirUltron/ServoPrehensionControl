function [ pod , pod_dot ] = ref_gen_obj( xo,xo_old,h )

% Reference
if (xo > 20)
    pod = 20;
    pod_dot = 0;
elseif (xo < -20)
    pod = -20;
    pod_dot = 0;
else
    pod = xo;
    pod_dot = (xo-xo_old)/h;
end

% pod_dot = 0;

end