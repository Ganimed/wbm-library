function [dchi,visualization] = forwardDynamics(t,chi,CONFIG)
%FORWARDDYNAMICS is the function that will be integrated in the forward
%                dynamics integrator.
%
%             [dchi,visualization] = FORWARDDYNAMICS(t,chi,config) takes
%             as input the current time step, T; the robot state, CHI
%             [13+2ndof x 1]; the structure CONFIG which contains the
%             user-defined parameters.
%             The output are the vector to be integrated, DCHI [13+2ndof x1]
%             and the structure VISUALIZATION which contains all the parameters
%             used to generate the plots in the visualizer.
%
% Author : Gabriele Nava (gabriele.nava@iit.it)
% Genova, May 2016

% ------------Initialization----------------
% waitbar
waitbar(t/CONFIG.tEnd,CONFIG.wait)

%% Robot Configuration
ndof                  = CONFIG.ndof;
gain                  = CONFIG.gainsInit;
qjInit                = CONFIG.qjInit;
xCoMRef               = CONFIG.xCoMRef;

%% Robot State
STATE                 = robotState(chi,CONFIG);
RotBase               = STATE.RotBase;
omegaBaseWorld        = STATE.omegaBaseWorld;
quatBase              = STATE.quatBase;
VelBase               = STATE.VelBase;
dqj                   = STATE.dqj;
qj                    = STATE.qj;
PosBase               = STATE.PosBase;

%% Set the robot state (for wbm functions)
wbm_setWorldFrame(RotBase,PosBase,[0 0 -9.81]')
wbm_updateState(qj,dqj,[VelBase;omegaBaseWorld]);

%% Robot Dynamics
DYNAMICS              = robotDynamics(STATE,CONFIG);
Jc                    = DYNAMICS.Jc;
M                     = DYNAMICS.M;
h                     = DYNAMICS.h;
H                     = DYNAMICS.H;
m                     = M(1,1);

%% Robot Forward kinematics
FORKINEMATICS         = robotForKinematics(STATE,DYNAMICS);
RFootPoseEul          = FORKINEMATICS.RFootPoseEul;
LFootPoseEul          = FORKINEMATICS.LFootPoseEul;
xCoM                  = FORKINEMATICS.xCoM;

%% Joint limits check
% jointLimitsCheck(qj,t);

%% CoM and joint trajectory generator
trajectory.JointReferences.ddqjRef = zeros(ndof,1);
trajectory.JointReferences.dqjRef  = zeros(ndof,1);
trajectory.JointReferences.qjRef   = qjInit;
trajectory.desired_x_dx_ddx_CoM    = trajectoryGenerator(xCoMRef,t,CONFIG);

%% Balancing controller
controlParam    =  runController(gain,trajectory,DYNAMICS,FORKINEMATICS,CONFIG,STATE);
tau             =  controlParam.tau;
fc              =  controlParam.fc;

%% State derivative computation
omegaWorldBase  = transpose(RotBase)*omegaBaseWorld;
dquatBase       = quaternionDerivative(omegaWorldBase,quatBase);
NuQuat          = [VelBase;dquatBase;dqj];
dNu             = M\(Jc'*fc + [zeros(6,1); tau]-h);
% state derivative
dchi            = [NuQuat;dNu];

%% Parameters for visualization
visualization.qj          = qj;
visualization.JointRef    = trajectory.JointReferences;
visualization.xCoM        = xCoM;
visualization.poseFeet    = [LFootPoseEul;RFootPoseEul];
visualization.H           = H;
visualization.HRef        = [m*trajectory.desired_x_dx_ddx_CoM(:,2);zeros(3,1)];
visualization.fc          = fc;
visualization.f0          = controlParam.f0;
visualization.tau         = tau;

end