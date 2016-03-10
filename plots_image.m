% Plota gráficos dos experimentos de manipulação do objeto
close all

% load ('IBVS_planar_A30_T20_K5_released');
% load ('IBVS_planar_A30_T20_K5')

TIT = 12; LBL = 12; LEG = 13; LWR = 1.5; LW = 1.5;

total_time = 60;

% Track
fig1 = figure(1);
set(fig1, 'Position', [403 246 560 1200]);
subplot(4,1,1);
    plot(time_vec,exp(ref_vec(1,:)),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(a) Object pos, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,exp(state_vec(1,:)),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg1 = legend('$p_{od}$','$p_{o}$','Orientation','Horizontal');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time 260 340]);
subplot(4,1,2);
    plot(time_vec,ref_vec(2,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Rel pos fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg2 = legend('$p_{rd}$','$p_{r}$','Orientation','Horizontal');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -160 -60]);
subplot(4,1,3);
    plot(time_vec,ref_vec(3,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(c) Rel pos fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg3 = legend('$p_{rd}$','$p_{r}$','Orientation','Horizontal');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');  
    axis([0 total_time 20 150]);
subplot(4,1,4);
    plot(time_vec,ref_vec(4,:),'-.','Linewidth',LWR,'Color',[0 0.7 0]); hold on; grid on;
    title('(d) Rel pos fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    plot(time_vec,state_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    leg4 = legend('$p_{rd}$','$p_{r}$','Orientation','Horizontal');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -4 2]);

% Errors
fig2 = figure(2);
subplot(2,2,1)
    plot(time_vec,error_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Object pos. error, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$e_{o,y}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     axis([0 total_time -15 10]);
subplot(2,2,2)
    plot(time_vec,error_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(b) Rel. pos. error, fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$e_{r,y}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -10 80]);
subplot(2,2,3)
    plot(time_vec,error_vec(3,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(c) Rel pos error, fingers 2-3','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$e_{r,x}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -80 10]);
subplot(2,2,4)
    plot(time_vec,error_vec(4,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(d) Rel pos error, fingers 2-3','Interpreter','latex','FontSize',TIT);
    ylabel('$mm$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$e_{r,y}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -5 5]);

% Position Control Signals
fig3 = figure(3);
subplot(2,2,1)
    plot(time_vec,v_ctrl_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Pos. ctrl. obj.','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$v_{o,y}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -600 600]);
subplot(2,2,2)
    plot(time_vec,v_ctrl_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Pos. ctrl. fingers 1-2, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$v_{r,y}\,\,1\!-\!2$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
    axis([0 total_time -200 1000]);
subplot(2,2,3)
    plot(time_vec,v_ctrl_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
    title('(c) Pos. ctrl. fingers 2-3, x-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$v_{r,x}\,\,2\!-\!3$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
    axis([0 total_time -1000 300]);
subplot(2,2,4)
    plot(time_vec,v_ctrl_vec(4,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
    title('(d) Pos. ctrl. fingers 2-3, y-axis','Interpreter','latex','FontSize',TIT);
    ylabel('$mm\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$v_{r,y}\,\,2\!-\!3$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','SouthEast'); 
    axis([0 total_time -50 50]);

fig4 = figure(4);    
subplot(2,2,1)
    plot(time_vec,dstate_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1]); hold on; grid on;
    title('(a) Joint ctrl finger 1','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg1 = legend('$\dot{\theta}_{m1}$');
    set(leg1,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -1 1]);
subplot(2,2,2)
    plot(time_vec,dstate_vec(2,:),'-','Linewidth',LW,'Color',[0 0.7 0]); hold on; grid on;
    title('(b) Joint ctrl finger 2','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg2 = legend('$\dot{\theta}_{m2}$');
    set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -1 1]);
subplot(2,2,3)
    plot(time_vec,dstate_vec(3,:),'-','Linewidth',LW,'Color',[1 0 0]); hold on; grid on;
    title('(c) Joint ctrl finger 3','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg3 = legend('$\dot{\theta}_{m3}$');
    set(leg3,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -1 1]);
subplot(2,2,4)
    plot(time_vec,dstate_vec(4,:),'-','Linewidth',LW,'Color',[0 0 0]); hold on; grid on;
    title('(d) Joint ctrl scissor','Interpreter','latex','FontSize',TIT);
    ylabel('$rad\,\,s^{-1}$','Interpreter','latex','FontSize',LBL);
    xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
    leg4 = legend('$\dot{\theta}_{m4}$');
    set(leg4,'FontSize',LEG,'Interpreter','latex','Location','NorthEast'); 
    axis([0 total_time -1 1]);

    
% % Error 2
% fig4 = figure(4);
% subplot(2,2,1)
%     plot(time_vec,error_vec(1,:),'-','Linewidth',LW,'Color',[0 0 1])
%     title('(a) Object pos error','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL)
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg1 = legend('$e_{o,y}$');
%     set(leg1,'FontSize',LEG,'Interpreter','latex','Location','SouthEast');
%     grid on
%     axis([0 total_time -10 10]);
%     hold on
% subplot(2,2,2)
%     plot(time_vec,error_vec(2,:),'-','Linewidth',LW,'Color',[0 0 1]);
%     hold on
%     plot(time_vec,error_vec(3,:),'-.','Linewidth',LW,'Color',[0 0.7 0]);
%     hold on
%     plot(time_vec,error_vec(4,:),'--','Linewidth',LW,'Color',[1 0 0]);
%     title('(b) Rel pos error','Interpreter','latex','FontSize',TIT);
%     ylabel('$mm$','Interpreter','latex','FontSize',LBL)
%     xlabel('Time, $s$','Interpreter','latex','FontSize',LBL);
%     leg2 = legend('$e_{r,y}\,\,1\!-\!2$','$e_{r,x}\,\,2\!-\!3$','$e_{r,y}\,\,2\!-\!3$');
%     set(leg2,'FontSize',LEG,'Interpreter','latex','Location','NorthEast');
%     grid on
%     axis([0 total_time -20 20]);
   

%  saveas(fig1,'track_state_obj','fig')
%  saveas(fig2,'track_error_obj','fig')
%  saveas(fig3,'track_pos_ctrl_obj','fig')  
%  saveas(fig4,'track_joint_ctrl_obj','fig')

%  saveas(fig1,'track_state_obj','jpeg')
%  saveas(fig2,'track_error_obj','jpeg')
%  saveas(fig3,'track_pos_ctrl_obj','jpeg')
%  saveas(fig4,'track_joint_ctrl_obj','jpeg')

% saveas(fig1,'track_state_obj','epsc')
% saveas(fig2,'track_error_obj','epsc')
% saveas(fig3,'track_pos_ctrl_obj','epsc')
% saveas(fig4,'track_joint_ctrl_obj','epsc')