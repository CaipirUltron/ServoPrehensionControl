
% Initialize gripper
if ~exist('gripper','var')
        gripper = robotiq();
        gripper.Connect();
        gripper.ChangeMode('Individual');
end

% Catch mode
catch_mode = 0; % do not catch object
% catch_mode = 1; % catch object

% Inicializa Kinect
% [~,~] = getimagedata(1);
% [~,~] = getimagedata(2);

% Matriz de parâmetros intrínsecos da câmera RGB do Kinect e sua inversa
RGB_intrinsics = [515.0882          0            316.0346 ;...
                     0           516.8755        269.9951 ;...
                     0              0               1                 ];

% Selection matrices
Sv = [1 0 0]; %  ---> x
% Sv = [0 1 0]; %  ---> y
% Sv = [0 0 1]; %  ---> area / depth
Sr = [ 0 1 0 0 0 0 ;...
       0 0 0 1 0 0 ;... 
       0 0 0 0 1 0 ];

% Parâmetros para a conversão Área - Depth 
a0 = 7621;             % ( bola maior - vermelha )
z0 = 400;

% a0 = 4381;           % ( bola menor - verde )
% z0 = 430;

if catch_mode == 1
    % Obtain grasping parameters
    [ GP , DELTA ] = gripper.Detect();
    pause(2);
end

% Posição relativa inicial do grasping
dp12_const = DELTA(1,:)';
dp23_const = DELTA(2,:)';

% Take initial sample of RGB and depth
[depth, RGB]=Kinect();

% Parâmetros iniciais para a janela de depth
Lxneg=5; Lxpos=5; Lyneg=5; Lypos=5;
imagesc(depth);
[xc,yc] = ginput(1);
xc = round(xc); yc = round(yc);
param = [ Lxneg, Lxpos, Lyneg, Lypos, xc, yc ];

% Measures INITIAL object depth (depth remains constant during planar experiment)
[ Depth,~ ] = detect_depth( param );

% Parâmetros iniciais para a janela RGB
Lxneg=25; Lxpos=25; Lyneg=25; Lypos=25;
data = RGB/255;
imshow(data);
[xc,yc] = ginput(1);
param = [ Lxneg, Lxpos, Lyneg, Lypos, xc, yc ];

% Measures INITIAL object centroid
% (depth is considered constant
[ Pball,~,~,param ] = detecta_bola( param , 'on');

% Rotation matrix from camera to palm ( fixed )
Rcp = [ 0 1 0 ; 0 0 -1 ; -1 0 0 ];

% Initialize time parameters
it = 1;
h = 0.1;
time = 0;
total_time = 60;

while time <= total_time
tic
    % Ar matrix
    Ar = [   eye(3)   -eye(3)  zeros(3,3) ;...
           zeros(3,3)  eye(3)   -eye(3)   ];
       
    % Ask for gripper status
    status = gripper.Receive();
    thetaHand = byte2o( status(:,1) );
    
    % Hand direct kinematics (angles in radians)
    [ p1 , ~ , p2 , ~ , p3 , ~ ] = Hand_Direct_Kinematics ( thetaHand );
    % Relative vectors
    dp12 = p1 - p2;
    dp23 = p2 - p3;
    
    % Measures object centroid and area
    [ Pball,~,~,param ] = detecta_bola( param );
    
    % Writes it on the screen
%     fprintf(1,'P = [%f %f]^T \n\n',Pball(1),Pball(2));
%     fprintf(1,'Area = %f [pixel^2] \n\n',Area);
    
    % Definition of the image-based state
    xv = Pball(1); yv = Pball(2); 
    zc = Depth;
    ev = Sv*[ xv ; yv ; log(zc) ];   % HVS state
    er = Sr*[ dp12 ; dp23 ];    % the relative state
    % Complete state vector
    e = [ ev ; er ]; % ---> state vector
    
    % References
%     [ evd , evd_dot ] = ref_gen_image( time, total_time, Pball(1) );
    [ evd , evd_dot ] = ref_gen_image( time, total_time );
    erd = Sr*[ dp12_const ; dp23_const ];     % relative reference
    erd_dot = zeros(3,1);
    
    if isequal(Sv,[0 0 1])
        ed = [ log(evd) ; erd ];                % desired state vector
        ed_dot = [ (1/evd)*evd_dot ; erd_dot ]; % derivative of desired state vector
    else
        ed = [ evd ; erd ];
        ed_dot = [ evd_dot ; erd_dot ]; % derivative of desired state vector
    end
    
    % State error
    error = ed - e;
    
    % Image control signal
    sigma_v = 10;  % HVS scalar gain (centroid)
    sigmar1 = 1; sigmar2 = 1; sigmar3 = 1;  % relative scalar gains
    K = [  sigma_v                    zeros(1,3)                  ;...
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
    
    % IBVS interaction matrix
    Omega_z = (1/zc)*( RGB_intrinsics - [ zeros(3,2) [ xv ; yv ; 0 ] ]);
    
    % IBVS image Jacobian matrix
    Jv = Omega_z*Rcp*JPo;
    
    % State jacobian matrix
    Jstate = [    Sv*Jv    ;...
               Sr*Ar*JhandP ];
    
    % Joint control signals (through inverse jacobian calculation)
    dThetaHand = Jstate\v;
%     dom = pinv(Jstate(1,:))*v(1);
    
    % Integration
    thetaHand = thetaHand + h*dThetaHand;
    
    % Conversion between angles and speeds into data to gripper
    [ pos_byte , pos_flag ] = o2byte( thetaHand );
    [ vel_byte , vel_flag ] = os2byte( abs(dThetaHand) );
    
    % Position and Velocity
    pose = [ pos_byte(1) vel_byte(1) 255 ; pos_byte(2) vel_byte(2) 255 ; pos_byte(3) vel_byte(3) 255 ; pos_byte(4) vel_byte(4) 255 ];
    
    % Send data to gripper
    gripper.Send(pose);
    
    % Register data
    time_vec(it) = time;
    error_vec(:,it) = error;
    ref_vec(:,it) = ed;
    state_vec(:,it) = e;
    joint_ctrl_vec(:,it) = thetaHand;
    v_ctrl_vec(:,it) = v;
    dstate_vec(:,it) = dThetaHand;
    
    % Time of simulation
    it = it + 1;
    h = toc;
    time = time + h;
    clc
    
end