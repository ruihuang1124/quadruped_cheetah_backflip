function p = get_params()
% Get parameters for Mini Cheetah in 2D
% The parameters are derived based on urdf file from:
% https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/gym/pybullet_data/mini_cheetah/mini_cheetah.urdf

% Gravity
g = 9.81;

% Link length for body, thigh, and shank
LB = 0.19*2;
LH = 0.209;
LK = 0.18;

% Link mass for body, thigh, and shank
MB = 3.3/2; %because we are only using half of the legs
MH = 0.634;
MK = 0.064;

% Link inertia for body, thigh, and shank
IB = 0.036203/2; %because we are only using half of the legs
IH = 0.02103;
IK = 0.000248;

% CoM for body, thigh, and shank
CoMxB = 0;
CoMyB = 0;

CoMxH = 0;
CoMyH = -0.02;

CoMxK = 0;
CoMyK = -0.209; % Needs to verify this term, seems weird

p.params = [g, LB, LH, LK, MB, MH, MK, IB, IH, IK, ...
    CoMxB, CoMyB, CoMxH, CoMyH, CoMxK, CoMyK];

end

