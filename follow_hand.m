% Pass cartesian reference for object kinematic control from image

% Operation mode
op_mode = 1; % gripper mode
% op_mode = 2; % simulation mode 
anima_mode = 0; % do not display animation
% anima_mode = 1; % display animation

% Catch mode
catch_mode = 0; % do not catch object
% catch_mode = 1; % catch object

% Grasping selection matrix
So = [ 0 1 0 ];
Sr = [ 0 1 0 0 0 0 ;...
       0 0 0 1 0 0 ;... 
       0 0 0 0 1 0 ];
   
if (op_mode == 1)
    if ~exist('gripper','var')
        % Initialize gripper
        gripper = robotiq();
        gripper.Connect();
        gripper.ChangeMode('Individual');
    end
    if catch_mode == 1
        % Obtain grasping parameters
        [ GP , DELTA ] = gripper.Detect();
        pause(2);
    end
    % Posição relativa inicial do grasping
    dp12_const = DELTA(1,:)';
    dp23_const = DELTA(2,:)';
else
    % Define initial conditions for simulation
    thetaHand = deg2rad([ 30  ; 30 ; 30 ; -10 ]);
    % Cinemática direta dos dedos
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics( thetaHand );
    obj_dim = abs(p2(2)-p1(2)); % largura do objeto em y (mm)
    r = obj_dim/2;              % raio do objeto
    % Calcula os parâmetros de grasping.
    GP = [   0   -r  0 ;...
           p2(1)  r  0 ;...
           p3(1)  r  0 ];
    % Posição relativa inicial do grasping
    dp12_const = p1 - p2;
    dp23_const = p2 - p3;
end

% Matriz de parâmetros intrínsecos da câmera RGB do Kinect e sua inversa
RGB_intrinsics = [515.0882          0            316.0346 ;...
                     0           516.8755        269.9951 ;...
                     0              0               1                 ];
                 
% Parâmetros para a conversão Área - Depth
a0 = 7621;
z0 = 400;

% Coordenadas iniciais para a janela
Lxneg=55;
Lxpos=55;
Lyneg=55;
Lypos=55;
r = 20; % raio da esfera imaginária (mm)

% Take picture and select initial point
[~, RGB] = Kinect();
data = RGB/255;
imshow(data);
[xc,yc] = ginput(1);

% Parâmetros iniciais
param = [ Lxneg, Lxpos, Lyneg, Lypos, xc, yc ];
xo_old = 0;

% Initialize time parameters
it = 1;
h = 0.1;
time = 0;
total_time = 60;

while time <= total_time
    tic
    
    % Measures object centroid and area
    [ Pball,Area,~,param ] = detecta_bola( param , 'on' );
    
    % Calculation of cartesian object coordinates from image coord.
    xv = Pball(1); yv = Pball(2); av = Area;
    zc = z0*sqrt(a0/av);    % estimated depth
    xo = zc*( xv - RGB_intrinsics(1,3) ) / ( RGB_intrinsics(1,1) );
    yo = zc*( yv - RGB_intrinsics(2,3) ) / ( RGB_intrinsics(2,2) );
    
    % Ar matrix
    Ar = [   eye(3)   -eye(3)  zeros(3,3) ;...
           zeros(3,3)  eye(3)   -eye(3)   ];
    
    if (op_mode == 1)
        % Ask for gripper status
        status = gripper.Receive();
        thetaHand = byte2o( status(:,1) );
    end
    
    % Hand direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics ( thetaHand );
    % Relative vectors
    dp12 = p1 - p2;
    dp23 = p2 - p3;
    % Estimate of object position
    po = mean( [ (p1(2)-GP(1,2)) ; (p2(2)-GP(2,2)) ; (p3(2)-GP(3,2)) ] );
    
    % Definition of the object state
    eo = po;
    % Definition of the relative state
    er = Sr*[ dp12 ; dp23 ];
    % Complete state vector
    e = [ eo ; er ]; % ---> state vector
    
    % References
    [eod,eod_dot] = ref_gen_obj(xo,xo_old,h);
    erd = Sr*[ dp12_const ; dp23_const ];     % relative reference
    ed = [ eod ; erd ];                       % desired state vector
    
    % Atualiza pos. antiga
    xo_old = xo;
    
    % Velocity feedforward terms
    erd_dot = zeros(3,1);
    ed_dot = [ eod_dot ; erd_dot ]; % derivative of desired state vector
    
    % State error
    error = ed - e;
    
    % Cartesian control signals
    sigma_obj = 2;
    sigmar1 = 10;
    sigmar2 = 10;
    sigmar3 = 10;
    K = [  sigma_obj                    zeros(1,3)                  ;...
           zeros(3,1)  [ sigmar1 0 0 ; 0 sigmar2 0 ; 0 0 sigmar3 ]  ];
    v = K*error + ed_dot;
    
    % Angles (for jacobian matrix calculation) - all in degrees
    [ o1 , o2 , o3 ] = Finger_Angles ( thetaHand );
    
    % Correction for the Robotiq gripper
    o1(4) = 0;
    
    % Geometric jacobian of each finger
    [ J1, J2, J3 ] = Finger_Jacobians( o1 , o2 , o3 );
    
    % Correction for the Robotiq gripper
    J1 = J1(:,1:3);
    
    % Linear parts
    JP1 = J1(1:3,:);
    JP2 = J2(1:3,:);
    JP3 = J3(1:3,:);
    
    % Calculation of the restriction Jacobian matrices
    [ Jr1, Jr2, Jc2, Jr3, Jc3 ] = Finger_Differential_Rel( thetaHand );
    
    % Complete hand jacobian matrix
    Jhand = Hand_Jacobian( J1, J2, J3, Jr1, Jr2, Jc2, Jr3, Jc3 );
    
    % Position hand jacobian matrix
    JhandP = Hand_Jacobian( JP1, JP2, JP3, Jr1, Jr2, Jc2, Jr3, Jc3 );
    
    % Object jacobian matrix
    Jo = Object_Jacobian( Jhand , GP );
    JPo = Jo(1:3,:);
    
    % State jacobian matrix
    Jstate = [    So*JPo    ;...
               Sr*Ar*JhandP ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
%     dom = pinv(Jstate(1,:))*v(1);
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    if (op_mode == 1)
        % Conversion between angles and speeds into data to gripper
        [ pos_byte , pos_flag ] = o2byte( thetaHand );
        [ vel_byte , vel_flag ] = os2byte( abs(dThetaHand) );
        
        % Position and Velocity
%         pose = [ pos_byte(1) vel_byte(1) 0 ; pos_byte(2) vel_byte(2) 0 ; pos_byte(3) vel_byte(3) 0 ; pos_byte(4) vel_byte(4) 0 ];
        pose = [ pos_byte(1) vel_byte(1) 255 ; pos_byte(2) vel_byte(2) 255 ; pos_byte(3) vel_byte(3) 255 ; pos_byte(4) vel_byte(4) 255 ];
        
        % Send data to gripper
        gripper.Send(pose);
    end
    
    if (anima_mode == 1)
        % Call animation
        clf(gcf);
        Animate(o1,o2,o3,thetaHand(4));
        [x,y,z]=sphere;
        mesh(x*(r/10)+po/10,y*(r/10),z*(r/10)+(p1(3)+p2(3))/20);
        hold on;
        pause(0.01);
    end
    
    % Time of simulation
    it = it + 1;
    h = toc;
    time = time + h;
    clc
    
    if ( xc^2 + yc^2 + (zc - 500)^2 ) < r^2
        fprintf(1,'Dentro \n');
    else
        fprintf(1,'Fora \n');
    end
    
    fprintf(1,'P = [%f %f %f]^T \n\n',xo,yo,zc);
    fprintf(1,'po_dot = %f \n\n',eod_dot);

end
