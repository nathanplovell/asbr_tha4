%% ME384R - ASBR - THA4
% Written by Clara Summerford and Nathan Lovell
%
% Generates the 'panda' robot object for use in all future ASBR
% programming assignments involving the Franka Emika Panda robot. The
% 'panda' variable is a struct containing the screws (space frame) of the
% panda as well as the transformation matrices for the space frame, the
% seven joints, and the end-effector of the panda in the zero position.
% These screws were derived from the DH parameters of the panda found in
% literature (namely, Gaz et al. 2019).
%
% This version differs from the original (PandaParams.m) as the matrix M,
% the configuration of the end-effector in zero position, is translated to
% the end of the tool (a 100mm long, 5mm diameter cylinder)

clear
clc

%% Define Variables

% t1-t7 represent the joint angles for joints 1-7 symbolically
syms t1 t2 t3 t4 t5 t6 t7 real

% Panda link lengths
d1 = 0.333;
d3 = 0.316;
d5 = 0.384;
a4 = 0.0825;
a5 = -0.0825;
a7 = 0.088;
df = 0.107;

% Transformation matrices of Panda joints (Derived by hand for Panda in
% zero position)
T1 = [1 0 0 0; 0 1 0 0; 0 0 1 .333; 0 0 0 1];
T2 = [1 0 0 0; 0 0 1 0; 0 -1 0 .333; 0 0 0 1];
T3 = [1 0 0 0; 0 1 0 0; 0 0 1 .649; 0 0 0 1];
T4 = [1 0 0 0.0825; 0 0 -1 0; 0 1 0 0.649; 0 0 0 1];
T5 = [1 0 0 0; 0 1 0 0; 0 0 1 1.033; 0 0 0 1];
T6 = [1 0 0 0; 0 0 -1 0; 0 1 0 1.033; 0 0 0 1];
T7 = [1 0 0 0.088; 0 -1 0 0; 0 0 -1 1.033; 0 0 0 1]; 
Ts = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
M = [1.0,0,0,0.088;0,-1.0,0,0;0,0,-1.0,0.826;0,0,0,1.0]; % 0.926 - 0.100 for the tool
Tbs = [1.0,0,0,-0.088;0,-1.0,0,0;0,0,-1.0,0.926;0,0,0,1.0]; % For Adjoint matrix, later

% Rotation matrices 
x_vec = [1; 0; 0];
y_vec = [0; 1; 0];
z_vec = [0; 0; 1];

function R_x = rotx(t)
    R_x = [1 0 0; 0 cos(t) -sin(t); 0 sin(t) cos(t)];
end

function R_y = roty(t)
    R_y = [cos(t) 0 sin(t); 0 1 0; -sin(t) 0 cos(t)];
end

function R_z = rotz(t)
    R_z = [cos(t) -sin(t) 0; sin(t) cos(t) 0; 0 0 1];
end

% Rotations in terms of the symbolic thetas
r1 = rotz(t1);
r2 = roty(t2);
r3 = rotz(t3);
r4 = roty(-t4); % -y direction, same as Gaz et al.
r5 = rotz(t5);
r6 = roty(-t6); % -y direction, same as Gaz et al.
r7 = rotz(-t7);

%% Develop Panda screws
w1 = [0 0 1]'; % Linear velocity
q1 = [0 0 d1]'; % Vector from base frame to joint axis of rotation
v1 = [0 0 0]'; % Angular velocity
S1 = [w1; v1]; % Screw vector for joint 1

w2 = r1*y_vec;
q2 = q1;
v2 = cross(-w2, q2);
S2 = [w2; v2];

w3 = r1*r2*z_vec;
q3 = q2 + r1*r2*[0 0 d3]';
v3 = cross(-w3,q3);
S3 = [w3; v3];

w4 = r1*r2*r3*-y_vec;
q4 = q3 + r1*r2*r3*[a4 0 0]';
v4 = cross(-w4,q4);
S4 = [w4; v4];

w5 = r1*r2*r3*r4*z_vec;
q5 = q4 + r1*r2*r3*r4*[a5 0 d5]';
v5 = cross(-w5,q5);
S5 = [w5; v5];

w6 = r1*r2*r3*r4*r5*-y_vec;
q6 = q5;
v6 = cross(-w6,q6);
S6 = [w6; v6];

w7 = r1*r2*r3*r4*r5*r6*-z_vec;
q7 = q5 + r1*r2*r3*r4*r5*r6*[a7 0 0]';
v7 = cross(-w7,q7);
S7 = [w7; v7];

% Space Jacobian (all screws compiled together)
J = [w1 w2 w3 w4 w5 w6 w7;
    v1 v2 v3 v4 v5 v6 v7];

%% Develop 'panda' struct

% S1-S7 = screws corresponding to panda joints
panda.screws = [S1 S2 S3 S4 S5 S6 S7];

% T1-T7 = T matrices of panda joints in home position
panda.transf.T1 = T1; 
panda.transf.T2 = T2; 
panda.transf.T3 = T3; 
panda.transf.T4 = T4; 
panda.transf.T5 = T5; 
panda.transf.T6 = T6; 
panda.transf.T7 = T7; 

% M = End-effector frame with respect to space frame (aka Tsb)
panda.transf.M = M;

% Ts = space frame (aka world/home frame)
panda.transf.Ts = Ts;

% Tbs = space frame with respect to end-effector frame
panda.transf.Tbs = Tbs;

% Joint Limits - first column are lower limits, second calumn are upper
% limits. Rows correspond to joint numbers.
panda.limits = [deg2rad(-166) deg2rad(166);
                   deg2rad(-101) deg2rad(101);
                   deg2rad(-166) deg2rad(166);
                   deg2rad(-176) deg2rad(-4);
                   deg2rad(-166) deg2rad(166);
                   deg2rad(-1) deg2rad(215);
                   deg2rad(-166) deg2rad(166)];