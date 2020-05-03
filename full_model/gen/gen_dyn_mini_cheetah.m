clear all

syms g real
syms LB LH LK MB MH MK IB IH IK real
syms CoMxB CoMyB CoMxH CoMyH CoMxK CoMyK real

syms x y theta real
syms xdot ydot thetadot real
syms qfh qfk qbh qbk real
syms qfhdot qfkdot qbhdot qbkdot real

%% --- variable lists ---
% Physical parameters of the robot
m_list_params = {
    'g'  'p(1)'; %Gravity
    
    'LB' 'p(2)'; %Lengths of links
    'LH' 'p(3)';
    'LK' 'p(4)';
    
    'MB' 'p(5)'; %Total mass of links
    'MH' 'p(6)';
    'MK' 'p(7)';
    
    'IB' 'p(8)'; %Inertia tensor term in local link frame
    'IH' 'p(9)';
    'IK' 'p(10)';

    'CoMxB' 'p(11)'; %Positio of CoM in local link frame
    'CoMyB' 'p(12)';
    
    'CoMxH' 'p(13)';
    'CoMyH' 'p(14)';
    
    'CoMxK' 'p(15)';
    'CoMyK' 'p(16)'};

% floating base positions
m_list_q = {
    'x' 'q(1)';
    'y' 'q(2)';
    'theta' 'q(3)';
    'qfh' 'q(4)';
    'qfk' 'q(5)';
    'qbh' 'q(6)';
    'qbk' 'q(7)'};

% floating base velocities
m_list_qdot = {
    'xdot' 'qdot(1)';
    'ydot' 'qdot(2)';
    'thetadot' 'qdot(3)';
    'thetadot' 'qdot(4)';
    'thetadot' 'qdot(5)';
    'thetadot' 'qdot(6)';
    'thetadot' 'qdot(7)'};


%% --- variables ---
q = [x y theta qfh qfk qbh qbk]';
qdot = [xdot ydot thetadot qfhdot qfkdot qbhdot qbkdot]';

%% Forward Kinematics for floating base, front hip, and back hip

rB = [x; y];
R0B = [cos(theta) -sin(theta); sin(theta) cos(theta)];
T0B = [R0B rB; 0 0 1];

rFH = [LB/2; 0];
RBFH = [1 0; 0 1];
TBFH = [RBFH RBFH*rFH; 0 0 1];
T0FH = T0B*TBFH;

rBH = [-LB/2; 0];
RBBH = [1 0; 0 1];
TBBH = [RBBH RBBH*rBH; 0 0 1];
T0BH = T0B*TBBH;

pB = T0B(1:2,3);
write_fcn_m('fcn_pB.m',{'q','p'},[m_list_q;m_list_params],{pB,'pB'});

pFH = T0FH(1:2,3);
write_fcn_m('fcn_pFH.m',{'q','p'},[m_list_q;m_list_params],{pFH,'pFH'});

pBH = T0BH(1:2,3);
write_fcn_m('fcn_pBH.m',{'q','p'},[m_list_q;m_list_params],{pBH,'pBH'});

% Forward Kinematics for front knee, back knee, front toe, and back toe

rK = [0; -LH];
RFHFK = [cos(qfh) -sin(qfh); sin(qfh) cos(qfh)];
TFHFK = [RFHFK RFHFK*rK; 0 0 1];
T0FK = T0FH*TFHFK;

RBHBK = [cos(qbh) -sin(qbh); sin(qbh) cos(qbh)];
TBHBK = [RBHBK RBHBK*rK; 0 0 1];
T0BK = T0BH*TBHBK;

rtoe = [0; -LK];
RFKFtoe = [cos(qfk) -sin(qfk); sin(qfk) cos(qfk)];
TFKFtoe = [RFKFtoe RFKFtoe*rtoe; 0 0 1];
T0Ftoe = T0FK*TFKFtoe;

RBKBtoe = [cos(qbk) -sin(qbk); sin(qbk) cos(qbk)];
TBKBtoe = [RBKBtoe RBKBtoe*rtoe; 0 0 1];
T0Btoe = T0BK*TBKBtoe;

pFK = T0FK(1:2,3);
write_fcn_m('fcn_pFK.m',{'q','p'},[m_list_q;m_list_params],{pFK,'pFK'});

pBK = T0BK(1:2,3);
write_fcn_m('fcn_pBK.m',{'q','p'},[m_list_q;m_list_params],{pBK,'pBK'});

pFtoe = T0Ftoe(1:2,3);
write_fcn_m('fcn_pFtoe.m',{'q','p'},[m_list_q;m_list_params],{pFtoe,'pFtoe'});

pBtoe = T0Btoe(1:2,3);
write_fcn_m('fcn_pBtoe.m',{'q','p'},[m_list_q;m_list_params],{pBtoe,'pBtoe'});

% Jacobians for toes with respect to ground
JFtoe = jacobian(pFtoe, q);
JBtoe = jacobian(pBtoe, q);
write_fcn_m('fcn_JFtoe.m',{'q','p'},[m_list_q;m_list_params],{JFtoe,'JFtoe'});
write_fcn_m('fcn_JBtoe.m',{'q','p'},[m_list_q;m_list_params],{JBtoe,'JBtoe'});

% Jacobians for toes with respect to floating base (for swing control)

% Center of Mass (com) positions and velocities of each link
pBCoM = T0B*[CoMxB; CoMyB; 1];
pBCoM = pBCoM(1:2,1);
vBCoM = jacobian(pBCoM,q) * qdot;

pFHCoM = T0FH*[RFHFK RFHFK*[CoMxH; CoMyH]; 0 0 1];
pFHCoM = pFHCoM(1:2,3);
vFHCoM = jacobian(pFHCoM,q) * qdot;

pFKCoM = T0FK*[RFKFtoe RFKFtoe*[CoMxK; CoMyK]; 0 0 1];
pFKCoM = pFKCoM(1:2,3);
vFKCoM = jacobian(pFKCoM,q) * qdot;

pBHCoM = T0BH*[RBHBK RBHBK*[CoMxH; CoMyH]; 0 0 1];
pBHCoM = pBHCoM(1:2,3);
vBHCoM = jacobian(pBHCoM,q) * qdot;

pBKCoM = T0BK*[RBKBtoe RBKBtoe*[CoMxK; CoMyK]; 0 0 1];
pBKCoM = pBKCoM(1:2,3);
vBKCoM = jacobian(pBKCoM,q) * qdot;

CoM = (MB*pBCoM + MH*pFHCoM + MH*pBHCoM + MK*pFKCoM + MK*pBKCoM)/(MB + 2*MH + 2*MK);
write_fcn_m('fcn_CoM.m',{'q', 'p'},[m_list_q;m_list_params],{CoM,'CoM'});

%% Angular velocity jacobians
% We can only rotate along z axis except for virtual prismatic joints
ux = 0;
uy = 0;
utheta = 1;
uqfh = 1;
uqfk = 1;
uqbh = 1;
uqbk = 1;

JB = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; ux uy utheta 0 0 0 0];
JFH = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; ux uy utheta uqfh 0 0 0];
JFK = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; ux uy utheta uqfh uqfk 0 0];
JBH = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; ux uy utheta 0 0 uqbh 0];
JBK = [0 0 0 0 0 0 0; 0 0 0 0 0 0 0; ux uy utheta 0 0 uqbh uqbk];
JwFtoe = JFK; %Angular velocity Jacobian of foot.
JwBtoe = JBK; %Angular velocity Jacobian of foot.

% Angular velocity of frames in respect to the world frame
wB = JB * qdot;
wFH = JFH * qdot;
wFK = JFK * qdot;
wBH = JBH * qdot;
wBK = JBK * qdot;

%% --- Energy and Lagrangian ---
R0B(3,3) = 1;
KEB = 0.5 * vBCoM' * MB * vBCoM + 0.5 * wB' * R0B * diag([0 0 IB]) * transpose(R0B) * wB;
R0FK = T0FK(1:2,1:2);
R0FK(3,3) = 1;
KEFH = 0.5 * vFHCoM' * MH * vFHCoM + 0.5 * wFH' * R0FK * diag([0 0 IH]) * transpose(R0FK) * wFH;
R0Ftoe = T0Ftoe(1:2,1:2);
R0Ftoe(3,3) = 1;
KEFK = 0.5 * vFKCoM' * MK * vFKCoM + 0.5 * wFK' * R0Ftoe * diag([0 0 IK]) * transpose(R0Ftoe) * wFK;
R0BK = T0FK(1:2,1:2);
R0BK(3,3) = 1;
KEBH = 0.5 * vBHCoM' * MH * vBHCoM + 0.5 * wBH' * R0BK * diag([0 0 IH]) * transpose(R0BK) * wBH;
R0Btoe = T0Btoe(1:2,1:2);
R0Btoe(3,3) = 1;
KEBK = 0.5 * vBKCoM' * MK * vBKCoM + 0.5 * wBK' * R0Btoe * diag([0 0 IK]) * transpose(R0Btoe) * wBK;

% Kinetic energy
KE = simplify(KEB + KEFH + KEFK + KEBH + KEBK);   

% Potential energy
PE = MB*[0 g]*pBCoM + MH*[0 g]*pFHCoM + MK*[0 g]*pFKCoM + MH*[0 g]*pBHCoM + MK*[0 g]*pBKCoM;

%To calculate the actuation selection matrix:
Upsilon = [qfh qfk qbh qbk]; %where control torques go: front and back hips and knees only, first three virtual joints are passive

%% --- Euler-Lagrange Equation ---
[De, Ce, Ge, Be] = std_dynamics(KE,PE,q,qdot, Upsilon);

write_fcn_m('fcn_De.m',{'q', 'p'},[m_list_q;m_list_params],{De,'De'});
write_fcn_m('fcn_Ce.m',{'q', 'qdot', 'p'},[m_list_q;m_list_qdot;m_list_params],{Ce,'Ce'});
write_fcn_m('fcn_Ge.m',{'q', 'p'},[m_list_q;m_list_params],{Ge,'Ge'});
write_fcn_m('fcn_Be.m',{'q', 'p'},[m_list_q;m_list_params],{Be,'Be'});

%% --- Holonomic Constraints ---
% The Holonomic Constraints are toe position does not change during stance
% We already have fcn_JFtoe.m and fcn_JBtoe.m

JdotFtoe = sym('JdotFtoe',size(JFtoe));
for ii = 1:size(JFtoe,2)
    JdotFtoe(:,ii) = jacobian(JFtoe(:,ii),q) * qdot;
end
write_fcn_m('fcn_JdotFtoe.m',{'q', 'p'},[m_list_q;m_list_params],{JdotFtoe,'JdotFtoe'});

JdotBtoe = sym('JdotBtoe',size(JBtoe));
for ii = 1:size(JBtoe,2)
    JdotBtoe(:,ii) = jacobian(JBtoe(:,ii),q) * qdot;
end
write_fcn_m('fcn_JdotBtoe.m',{'q', 'p'},[m_list_q;m_list_params],{JdotBtoe,'JdotBtoe'});

% Inverse Kinematics for front hip and front knee
% Assume we know the coordinates of floating base and toe, solve for joints
syms pFtoex pFtoey real

% front toe coordinates
m_list_pFtoe = {
    'pFtoex' 'pFtoe(1)';
    'pFtoey' 'pFtoe(2)'};

% Law of cosines
LFHFtoe = sqrt((pFH(1) - pFtoex)^2 + (pFH(2) - pFtoey)^2);
qfk = pi - acos((LH^2+LK^2-LFHFtoe^2)/2/LH/LK);
LBFtoe = sqrt((pB(1) - pFtoex)^2 + (pB(2) - pFtoey)^2);
qfh = acos(((LB/2)^2+LFHFtoe^2-LBFtoe^2)/2/(LB/2)/LFHFtoe) - acos((LH^2+LFHFtoe^2-LK^2)/2/LH/LFHFtoe) - pi/2;
qf = [qfh; qfk];
write_fcn_m('fcn_IKfront.m',{'q','p','pFtoe'},[m_list_q;m_list_params;m_list_pFtoe],{qf,'qf'});

% Inverse Kinematics for back hip and front knee
% Assume we know the coordinates of floating base and toe, solve for joints
syms pBtoex pBtoey real

% back toe coordinates
m_list_pBtoe = {
    'pBtoex' 'pBtoe(1)';
    'pBtoey' 'pBtoe(2)'};

% Law of cosines
LBHBtoe = sqrt((pBH(1) - pBtoex)^2 + (pBH(2) - pBtoey)^2);
qbk = pi - acos((LH^2+LK^2-LBHBtoe^2)/2/LH/LK);
LBBtoe = sqrt((pB(1) - pBtoex)^2 + (pB(2) - pBtoey)^2);
qbh = pi/2 - acos(((LB/2)^2+LBHBtoe^2-LBBtoe^2)/2/(LB/2)/LBHBtoe) - acos((LH^2+LBHBtoe^2-LK^2)/2/LH/LBHBtoe);
qb = [qbh; qbk];
write_fcn_m('fcn_IKback.m',{'q','p','pBtoe'},[m_list_q;m_list_params;m_list_pBtoe],{qb,'qb'});
