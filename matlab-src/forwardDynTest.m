% Script to integrate and test forward dynamics
addpath('./../build');
wholeBodyModel('model-initialise','icubGazeboSim');

%[jl1, jl2] = wholeBodyModel('joint-limits');
load('./jointLimits.mat');

param.ndof = length(jl1);
param.tau = @(t)zeros(param.ndof,1);

%initial conditions

floating_base = 'l_sole';

qjInit = 0.5*(jl1+jl2);%(0.55*jl2 + 0.45*jl1);% + rand(32,1)./100;
%zeros(32,1);

%disp('Initial Joint Configuration');
%disp(qjInit);
wholeBodyModel('update-state',qjInit,zeros(param.ndof,1),zeros(6,1));
             %wholeBodyModel('forward-kinematics',qj,refLink1)
             
             
%T_baseInit = wholeBodyModel('forward-kinematics', qjInit,floating_base);
%T_baseInit(4:end) = convertAxisAngle2Quaternion(T_baseInit(4:end));
[qj,T_baseInit,qjDot,vb] = wholeBodyModel('get-state');

qjDotInit = zeros(length(qjInit),1);
v_baseInit = zeros(6,1);

xInit = [T_baseInit;qjInit;qjDotInit;v_baseInit];

func = @(t,qv)forwardDynamics(t,qv,param);
%pause;

disp('Starting Integration');
options = odeset('RelTol',1e-4,'AbsTol',1e-8);
%options = odeset('I
[t,x] = ode15s(func,[0 1],xInit,options);



