% Computes the object jacobian matrix. 
% TERMINAR!!!

function [ Jobject ] = Object_Jacobian( Jhand , GP )

% Skew-simmetric matrices for the adjoints
S1 = [0 -GP(1,3) GP(1,2) ; GP(1,3) 0 -GP(1,1) ; -GP(1,2) GP(1,1) 0];
S2 = [0 -GP(2,3) GP(2,2) ; GP(2,3) 0 -GP(2,1) ; -GP(2,2) GP(2,1) 0];
S3 = [0 -GP(3,3) GP(3,2) ; GP(3,3) 0 -GP(3,1) ; -GP(3,2) GP(3,1) 0];
% Adjoint matrices for the three contact points
A1 = [eye(3) -S1 ; zeros(3,3) eye(3)];
A2 = [eye(3) -S2 ; zeros(3,3) eye(3)];
A3 = [eye(3) -S3 ; zeros(3,3) eye(3)];
% Stacking of adjoint matrices
A = [A1 ; A2 ; A3];
% Annihilator matrix of A
Adagger = pinv(A);
% Atil = [A2 -A1 zeros(6,6)];
Atil = [inv(A1) -inv(A2) zeros(6,6)];
% Selection matrix (selects y direction)
% S = [0 1 0 0 0 0];
% Constraint matrices from the contact model
H1 = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)];
H2 = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)];
H3 = [zeros(3,6); 0 0 0 1 0 0 ; zeros(2,6)];
% Stacking of the H matrices
H = [H1 zeros(6,12) ; zeros(6,6) H2 zeros(6,6) ; zeros(6,12) H3];
% Constraint jacobian matrices
Jcm = Atil*Jhand;
Jcw = Atil*H';
% Unconstrained jacobian matrices
Jum = Adagger*Jhand;
Juw = Adagger*H';
% Object jacobian
Jobject = (Jum-Juw*pinv(Jcw)*Jcm);

end

