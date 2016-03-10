% Generate references for IBVS experiments

function [ evd , evd_dot ] = ref_gen_image( time, Total_time, varargin )
 
%     limiar1 = Total_time/6;
%     limiar2 = 2*(Total_time/6);
%     limiar3 = 3*(Total_time/6);
%     limiar4 = 4*(Total_time/6);
%     limiar5 = 5*(Total_time/6);
%     
% %     Regulation
%     if time < limiar1
% %         vm = 5526;
%         vm = 1000;
% %         vm = 320;    % IBVS                 % centroid    (pixel)
% %         vm = 325;    % HVS                  % bias     (pixel)
%     elseif time < limiar2
% %         vm = 5526 + 800;
%         vm = 1000-30;
% %         vm = 290;    % IBVS                 % bias     (pixel)
% %         vm = 310;                           % bias     (pixel)
%     elseif time < limiar3
% %         vm = 5526 + 800;
%         vm = 1000-30;
% %         vm = 290;    % IBVS                 % bias     (pixel)
% %         vm = 310;                     % bias     (pixel)
%     elseif time < limiar4
% %         vm = 5526 - 800;
%         vm = 1000+30;
% %         vm = 350;    % IBVS                 % bias     (pixel)
% %         vm = 340;                     % bias     (pixel)
%     elseif time < limiar5
%         vm = 1000+30;
% %         vm = 350;    % IBVS                 % bias     (pixel)
% %         vm = 340;                     % bias     (pixel)
%     elseif time < Total_time
%         vm = 1000;
% %         vm = 320;    % IBVS                 % bias     (pixel)
% %         vm = 325;                     % bias     (pixel)
%     end

    if ~isempty(varargin)
        vm = varargin{1};
    else
%         vm = 320;     % planar IBVS
%         vm = 6592;
        vm = 1000;
%         vm = 325;     % planar HVS
    end
    
    a = 30;
    T = 10;
    wn = 2*pi/T;
    
    evd = vm + a*sin(wn*time);
    evd_dot = wn*a*cos(wn*time);

end