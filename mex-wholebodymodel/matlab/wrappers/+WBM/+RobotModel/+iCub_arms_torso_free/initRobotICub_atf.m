function [wbm_icub, ndof] = initRobotICub_atf(wf2fixlnk)
    %% Initialize the model of the iCub robot with an arms and torso free configuration:
    %
    %  Note: The legs in this model of the URDF-file are stiffened and not movable.
    %
    %  Source for the joint configurations,
    %       [1] Yarp-WholeBodyInterface: <https://github.com/robotology/yarp-wholebodyinterface/blob/master/app/robots/icubGazeboSim/yarpWholeBodyInterface.ini>

    %% Base (default) model parameters:
    icub_model = WBM.wbmRobotModel;
    icub_model.urdf_robot_name = which('model_arms_torso_free.urdf');
    icub_model.wf_R_b_init     = eye(3,3);
    icub_model.wf_p_b_init     = zeros(3,1);
    icub_model.g_wf            = [0; 0; -9.81];
    %icub_model.ndof            = 0;        % unknown at the moment
    %icub_model.urdf_fixed_link = 'l_sole'; % (default) optional

    %% Base robot config:
    icub_config = WBM.wbmHumanoidConfig;
    icub_config.nCstrs           = 2;
    icub_config.ccstr_link_names = {'l_sole', 'r_sole'};

    % Setup the body of the iCub robot with the initial body (joint) positions (in degrees):
    [icub_config.body, jnt_names_body] = WBM.RobotModel.iCub_arms_torso_free.setupBodyICub_atf();
    icub_config.jpos_torso     = [0; 0; 0];
    icub_config.jpos_left_arm  = [0; 30; 0; 45; 0; 0; 0];
    icub_config.jpos_left_leg  = [25.5; 0.1; 0; -18.5; -5.5; -0.1];
    icub_config.jpos_right_arm = icub_config.jpos_left_arm;
    icub_config.jpos_right_leg = icub_config.jpos_left_leg;

    % Concatenate all joint-positions of the body:
    % Note: The order of the joints must be set in the same order as defined in the listing of ROBOT_MEX_WBI_TOOLBOX in [1].
    qj_init_body = vertcat(icub_config.jpos_torso, icub_config.jpos_left_arm, icub_config.jpos_right_arm, ...
                           icub_config.jpos_left_leg, icub_config.jpos_right_leg);

    [rev_jnt_names, ndof] = WBM.utilities.getJointNamesFromURDF(icub_model.urdf_robot_name, 'revolute');
    qj_init               = WBM.utilities.initJointsFromNameList(rev_jnt_names, ndof, jnt_names_body, qj_init_body);

    % Friction coefficients (optional):
    %icub_model.frict_coeff.v = repmat(0.75, ndof, 1); % viscous friction
    %icub_model.frict_coeff.c = repmat(0.1, ndof, 1);  % Coulomb friction

    % Init-State Parameters:
    icub_config.init_state_params.x_b     = zeros(3,1);
    icub_config.init_state_params.qt_b    = zeros(4,1);
    icub_config.init_state_params.q_j     = qj_init * (pi/180.0); % in radians
    icub_config.init_state_params.dx_b    = zeros(3,1);
    icub_config.init_state_params.omega_b = zeros(3,1);
    icub_config.init_state_params.dq_j    = zeros(ndof,1);

    %% Initialize the mex-WholeBodyModel for the iCub robot:
    if ~exist('wf2fixlnk', 'var')
        % by default: the world frame will NOT be set to a fixed link ...
        wbm_icub = WBM.WBM(icub_model, icub_config);
        return
    end
    % else, user specific ...
    wbm_icub = WBM.WBM(icub_model, icub_config, wf2fixlnk);
end
