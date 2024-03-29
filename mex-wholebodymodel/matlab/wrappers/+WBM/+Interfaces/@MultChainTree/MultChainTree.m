classdef MultChainTree < WBM.Interfaces.IMultChainTree
    properties(Dependent)
        % public properties for fast get/set methods:
        name@char       % the name of the robot.
        manuf@char      % the name of the manufacturer (annotation)
        comment@char    % general comment (annotation)
        wbm_info@struct % general information about whole body model of the robot.
        wbm_params@WBM.wbmRobotParams % base model and configuration parameters of the robot.
        plotopt3d@WBM.wbmSimConfig
        base_link@char        % floating-base link (fixed reference link) of the robot.
        ee_links              % kinematic links of the end-effectors (hands) that are controlled by the system.
        link@char             % current kinematic link of the robot that is controlled by the system.
        gravity@double vector % gravity vector (direction of the gravity)
        base@double    matrix % base transform of the robot (pose of the robot)
        tool@double    matrix % tool transform (from the end-effector to the tool-tip)
        qlim@double    matrix % joint limits, [qmin qmax] (Nx2)
        n@uint16       scalar % number of joints (equivalent to number of DoFs)

        %interface % interface to a real robot platform
    end

    properties(Access = protected)
        mwbm@WBM.Interfaces.IWBM
        mwbm_info = struct( 'robot_name',  '', ...
                            'robot_manuf', '', ...
                            'comment',     '' );
        mlnk_name@char = '';
        mlnk_names_ee
    end

    methods
        function bot = MultChainTree(robot_wbm, lnk_name, opt)
            % some error checks ...
            if (nargin < 2)
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
            end
            if ~isa(robot_wbm, 'WBM.Interfaces.IWBM')
                error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end

            if (nargin == 3)
                % options:
                if isstruct(opt)
                    if isempty(opt)
                        error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.EMPTY_DATA_TYPE);
                    end
                    % set some annotation informations about the robot:
                    bot.mwbm_info.robot_name  = opt.name;
                    bot.mwbm_info.robot_manuf = opt.manufacturer;
                    bot.mwbm_info.comment     = opt.comment;

                    % set the end-effector links (hands) to be controlled by the system:
                    if ~isempty(opt.ee_links)
                        bot.ee_links = opt.ee_links;
                    end
                    % plot options:
                    if ~isempty(opt.plotopt3d)
                        bot.mwbm.sim_config = opt.plotopt3d;
                    end
                else
                    error('MultChainTree::MultChainTree: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
                end
            end
            % initialization:
            bot.mwbm = robot_wbm;
            % set the initial link to be controlled by the system ...
            bot.link = lnk_name;

            % try to get some informations about the robot directly from the WBM ...
            if ( isempty(bot.mwbm_info.robot_name) && ~isempty(robot_wbm.robot_name) )
                bot.mwbm_info.robot_name = robot_wbm.robot_name;
            end
            if isempty(bot.mwbm_info.robot_manuf)
                bot.mwbm_info.robot_manuf = robot_wbm.robot_manuf;
            end
        end

        function newBot = copy(bot)
            newBot = WBM.utilities.copyObj(bot);
        end

        function delete(bot)
            delete(bot);
        end

        function [q_j, dq_j, wf_H_b, v_b] = getstate(bot)
            switch nargout
                case 2
                    [~,q_j,~,dq_j] = getState(bot.mwbm);
                case 4
                    [vqT_b, q_j, v_b, dq_j] = getState(bot.mwbm);
                    wf_H_b = WBM.utilities.tfms.frame2tform(vqT_b);
                otherwise
                    error('MultChainTree::getstate: %s', WBM.wbmErrorMsg.WRONG_NARGOUT);
            end
        end

        function setcontact(bot, cmode, varargin) % for closed-loop & non-closed loop contacts
            switch cmode
                case 'foot'
                    bot.mwbm.foot_conf = footConfig(bot.mwbm, varargin{:});
                case {'hand', 'gripper'}
                    bot.mwbm.hand_conf = handConfig(bot.mwbm, cmode, varargin{:});
                otherwise
                    error('MultChainTree::setcontact: %s', WBM.wbmErrorMsg.STRING_MISMATCH);
            end
        end

        function ddq_j = accel(bot, q_j, dq_j, tau)
            ddq_j = jointAcc(bot.mwbm, tau, q_j, dq_j);
        end

        function c_qv = coriolis(bot, q_j, dq_j)
            c_qv = corForces(bot.mwbm, q_j, dq_j);
        end

        function tau_fr = friction(bot, dq_j)
            tau_fr = frictForces(bot.mwbm, dq_j);
        end

        function g_q = gravload(bot, q_j)
            g_q = gravityForces(bot.mwbm, q_j);
        end

        function [g_q, wf_J_b] = gravjac(bot, q_j)
            stFltb   = getBaseState(bot.mwbm);
            base_lnk = bot.mwbm.base_link; % reference link (fixed link)

            % gravity bias forces g(q):
            g_q = gravForces(bot.mwbm, q_j, stFltb);
            % Jacobian of the base frame (reference frame):
            wf_J_b = jacob(bot.mwbm, base_lnk, q_j, stFltb);
        end

        function tau_j = rne(bot, q_j, dq_j, ddq_j)
            tau_j = invHybridDyn(bot.mwbm, q_j, dq_j, ddq_j);
        end

        function [t, stmChi] = fdyn(bot, tspan, stvChi_0, fhCtrlTrqs, ode_opt, varargin)
            [t, stmChi] = fwdDyn(bot.mwbm, tspan, stvChi_0, fhCtrlTrqs, ode_opt, varargin{:});
        end

        function wf_H_lnk = fkine(bot, q_j)
            wf_H_lnk = fwdKin(bot.mwbm, bot.mlnk_name, q_j);
        end

        function wf_H_lnk = A(bot, lnk_name, q_j)
            wf_H_lnk = linkFrame(bot.mwbm, lnk_name, q_j);
        end

        function wf_H_ee = T0_n(bot, q_j) % computes the forward kinematics of the end-effectors (hands).
            if isempty(bot.mlnk_names_ee)
                error('MultChainTree::T0_n: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end

            if iscell(bot.mlnk_names_ee)
                % both hands ...
                stFltb = getBaseState(bot.mwbm);
                wf_H_ee = cell(1,2);
                for i = 1:2
                    vqT_ee = fwdKin(bot.mwbm, bot.mlnk_names_ee{1,i}, q_j, stFltb);
                    wf_H_ee{1,i} = WBM.utilities.tfms.frame2tform(vqT_ee);
                end
                return
            end
            % else, only one hand is in use ...
            vqT_ee  = fwdKin(bot.mwbm, bot.mlnk_names_ee, q_j);
            wf_H_ee = WBM.utilities.tfms.frame2tform(vqT_ee);
        end

        function djdq_lnk = jacob_dot(bot, q_j, dq_j)
            djdq_lnk = jacobDot(bot.mwbm, bot.mlnk_name, q_j, dq_j);
        end

        function wf_J_lnk = jacob0(bot, q_j, varargin)
            % options:
            opt.rpy   = false;
            opt.eul   = false;
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin); % function from the RTB of Peter Corke.

            stFltb   = getBaseState(bot.mwbm);
            wf_J_lnk = jacob(bot.mwbm, bot.mlnk_name, q_j, stFltb);

            if opt.rpy
                % compute the analytical Jacobian with the Euler rotation rate in ZYX (RPY) order:
                wf_H_lnk = fwdKin(bot.mwbm, bot.mlnk_name, q_j, stFltb);
                Er_inv   = WBM.utilities.tfms.tform2angRateTF(wf_H_lnk, 'eul', 'ZYX');
                if (rcond(Er_inv) < eps)
                    error('MultChainTree::jacob0: %s', WBM.wbmErrorMsg.SINGULAR_MAT);
                end
                wf_rX_lnk = eye(6,6);
                wf_rX_lnk(4:6,4:6) = Er_inv;

                wf_J_lnk = wf_rX_lnk * wf_J_lnk;
            elseif opt.eul
                % compute the analytical Jacobian with the Euler rotation rate in ZYZ order:
                wf_H_lnk = fwdKin(bot.mwbm, bot.mlnk_name, q_j, stFltb);
                Er_inv   = WBM.utilities.tfms.tform2angRateTF(wf_H_lnk, 'eul', 'ZYZ');
                if (rcond(Er_inv) < eps)
                    error('MultChainTree::jacob0: %s', WBM.wbmErrorMsg.SINGULAR_MAT);
                end
                wf_rX_lnk = eye(6,6);
                wf_rX_lnk(4:6,4:6) = Er_inv;

                wf_J_lnk = wf_rX_lnk * wf_J_lnk;
            end

            if opt.trans
                % return the translational sub-matrix of the Jacobian:
                wf_J_lnk = wf_J_lnk(1:3,:);
            elseif opt.rot
                % return the rotational sub-matrix of the Jacobian:
                wf_J_lnk = wf_J_lnk(4:6,:);
            end
        end

        function wf_J_ee = jacobn(bot, q_j, varargin) % Jacobian of the ee-frames.
            if isempty(bot.mlnk_names_ee)
                error('MultChainTree::jacobn: %s', WBM.wbmErrorMsg.EMPTY_ARRAY);
            end
            % options:
            opt.trans = false;
            opt.rot   = false;
            opt = tb_optparse(opt, varargin);

            % compute the Jacobians of the end-effectors (hands) which
            % are controlled by the system:
            if iscell(bot.mlnk_names_ee)
                % both hands are used ...
                stFltb  = getBaseState(bot.mwbm);
                wf_J_ee = cell(1,2);
                for i = 1:2
                    wf_J_ee{1,i} = jacob(bot.mwbm, bot.mlnk_names_ee{1,i}, q_j, stFltb);
                end

                if opt.trans
                    % translational sub-matrices:
                    wf_J_ee{1,1} = wf_J_ee{1,1}(1:3,:);
                    wf_J_ee{1,2} = wf_J_ee{1,2}(1:3,:);
                elseif opt.rot
                    % rotational sub-matrices:
                    wf_J_ee{1,1} = wf_J_ee{1,1}(4:6,:);
                    wf_J_ee{1,2} = wf_J_ee{1,2}(4:6,:);
                end
                return
            end
            % else, only one hand is in use ...
            wf_J_ee = jacob(bot.mwbm, bot.mlnk_names_ee, q_j);
            if opt.trans
                wf_J_ee = wf_J_ee(1:3,:);
            elseif opt.rot
                wf_J_ee = wf_J_ee(4:6,:);
            end
        end

        function M = inertia(bot, q_j)
            M = massMatrix(bot.mwbm, q_j);
        end

        function Mx = cinertia(bot, q_j)
            stFltb = getBaseState(bot.mwbm);

            M        = massMatrix(bot.mwbm, q_j, stFltb);
            wf_J_lnk = jacob(bot.mwbm, bot.mlnk_name, q_j, stFltb);
            % calculate the Cartesian mass matrix (pseudo-kinetic energy matrix)
            % Mx = (J * M^(-1) * J^T)^(-1) in operational space:
            [Mx,~,~] = WBM.utilities.tfms.cartmass(wf_J_lnk, M);
        end

        function payload(bot, pl_lnk_data)
            payload(bot.mwbm, pl_lnk_data);
        end

        function f_pl = pay(bot, fhTotCWrench, f_cp, tau, q_j, dq_j)
           f_pl = ploadForces(bot.mwbm, fhTotCWrench, f_cp, tau, q_j, dq_j);
        end

        function resv = islimit(bot, q_j)
            resv = isJntLimit(bot.mwbm, q_j);
        end

        function plot3d(bot, stmChi, sim_tstep, vis_ctrl)
            if (nargin == 4)
                visFwdDyn(bot.mwbm, stmChi, sim_tstep, vis_ctrl);
                return
            end
            % else, use the default vis-control values ...
            visFwdDyn(bot.mwbm, stmChi, sim_tstep);
        end

        function set.name(bot, robot_name)
            bot.mwbm_info.robot_name = robot_name;
        end

        function robot_name = get.name(bot)
            robot_name = bot.mwbm_info.robot_name;
        end

        function set.manuf(bot, robot_manuf)
            bot.mwbm_info.robot_manuf = robot_manuf;
        end

        function robot_manuf = get.manuf(bot)
            robot_manuf = bot.mrobot_manuf;
        end

        function set.comment(bot, comment)
            bot.mwbm_info.comment = comment;
        end

        function comment = get.comment(bot)
            comment = bot.mwbm_info.comment;
        end

        function wbm_info = get.wbm_info(bot)
            wbm_info = bot.mwbm_info;
        end

        function wbm_params = get.wbm_params(bot)
            wbm_params = bot.mwbm.robot_params;
        end

        function set.plotopt3d(bot, sim_config)
            bot.mwbm.sim_config = sim_config;
        end

        function sim_config = get.plotopt3d(bot)
            sim_config = bot.mwbm.sim_config;
        end

        function set.base_link(bot, rlnk_name)
            bot.mwbm.base_link = rlnk_name;
        end

        function rlnk_name = get.base_link(bot)
            rlnk_name = bot.mwbm.base_link;
        end

        function set.link(bot, lnk_name)
            if isempty(lnk_name)
                error('MultChainTree::set.link: %s', WBM.wbmErrorMsg.EMPTY_STRING);
            end
            bot.mlnk_name = lnk_name;
        end

        function lnk_name = get.link(bot)
            lnk_name = bot.mlnk_name;
        end

        function set.ee_links(bot, lnk_names)
            if ( iscellstr(lnk_names) && isrow(lnk_names) )
                if (size(lnk_names,2) ~= 2)
                    error('MultChainTree::set.ee_links: %s', WBM.wbmErrorMsg.WRONG_ARR_SIZE);
                end
                % both ee-links (hands) will be used ...
                bot.mlnk_names_ee = lnk_names;
            elseif ischar(lnk_names)
                % only one ee-link (hand) will be used ...
                bot.mlnk_names_ee = lnk_names;
            else
                error('MultChainTree::set.ee_links: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
        end

        function lnk_names = get.ee_links(bot)
            lnk_names = bot.mlnk_names_ee;
        end

        function set.gravity(bot, g_wf)
            bot.mwbm.gravity = g_wf;
        end

        function g_wf = get.gravity(bot)
            g_wf = bot.mwbm.g_wf;
        end

        function set.base(bot, wf_H_b)
            bot.mwbm.base_tform = wf_H_b;
        end

        function wf_H_b = get.base(bot)
            wf_H_b = bot.mwbm.base_tform;
        end

        function set.tool(bot, ee_H_tt)
            bot.mwbm.tool_tform = ee_H_tt;
        end

        function ee_H_tt = get.tool(bot)
            ee_H_tt = bot.mwbm.tool_tform;
        end

        function jlmts = get.qlim(bot)
            jl    = bot.mwbm.jlimits;
            jlmts = horzcat(jl.lwr, jl.upr);
        end

        function ndof = get.n(bot)
            ndof = bot.mwbm.ndof;
        end

    end
end
