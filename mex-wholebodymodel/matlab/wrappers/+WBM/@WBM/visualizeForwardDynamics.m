% Copyright (C) 2015-2018, by Martin Neururer
% Author: Martin Neururer
% E-mail: martin.neururer@student.tuwien.ac.at / martin.neururer@gmail.com
% Date:   January-May, 2018
%
% Departments:
%   Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia and
%   Automation and Control Institute - TU Wien.
%
% This file is part of the Whole-Body Model Library for Matlab (WBML).
%
% The development of the WBM-Library was made in the context of the master
% thesis "Learning Task Behaviors for Humanoid Robots" and is an extension
% for the Matlab MEX whole-body model interface, which was supported by the
% FP7 EU-project CoDyCo (No. 600716, ICT-2011.2.1 Cognitive Systems and
% Robotics (b)), <http://www.codyco.eu>.
%
% Permission is granted to copy, distribute, and/or modify the WBM-Library
% under the terms of the GNU Lesser General Public License, Version 2.1
% or any later version published by the Free Software Foundation.
%
% The WBM-Library is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
% GNU Lesser General Public License for more details.
%
% A copy of the GNU Lesser General Public License can be found along
% with the WBML. If not, see <http://www.gnu.org/licenses/>.

function visualizeForwardDynamics(obj, stmPos, sim_config, sim_tstep, vis_ctrl)
    % Visualizes the computed *forward dynamics*, i.e. the motions, of a given
    % robot model within a specified time interval :math:`[t_0, t_f]`.
    %
    % Arguments:
    %   stmPos    (double, matrix): Position and orientation data from the *forward
    %                               dynamics states* of the integration output matrix
    %                               :math:`\mathcal{X}` of the given robot model.
    %   sim_config (:class:`~WBM.wbmSimConfig`): Configuration object with the configuration
    %                                            settings for the visualizer of the robot
    %                                            simulation.
    %   sim_tstep (double, scalar): Simulation time-step size to control the visualization
    %                               speed of the robot simulation.
    %
    %                               In order to keep the simulation close to real time, the
    %                               *simulation step size* :math:`t_{{step}_s}` is the same
    %                               *step size* :math:`t_{step}` of the given time interval
    %                               :math:`[t_0, t_f]` (see :meth:`WBM.intForwardDynamics`
    %                               -- ``tspan``).
    %
    %                               To slow down the visualization speed of the robot
    %                               simulation (slow motion), :math:`t_{{step}_s}` can
    %                               be increased by a *factor* :math:`x > 1` such that
    %                               :math:`t_{{step}_s} = x\cdot t_{step}`.
    %   vis_ctrl          (struct): Data structure to control the graphic elements of
    %                               the robot that should be drawn during the simulation
    %                               (*optional*).
    %
    %                               The structure is specified by following data fields:
    %
    %                                  - ``drawJnts`` (*logical, scalar*): Draw the joint nodes
    %                                    of the robot (default: *true*).
    %                                  - ``drawCom``  (*logical, scalar*): Draw the node of the
    %                                    center of mass (default: *true*).
    %                                  - ``drawSkel`` (*logical, scalar*): Draw the skeleton of
    %                                    the robot (default: *true*).
    %                                  - ``drawBody`` (*logical, scalar*): Draw the body parts,
    %                                    i.e. the hull, of the robot (default: *true*).
    %                                  - ``vis_speed`` (*double, scalar*): Visualization speed
    %                                    to keep the simulation close to real-time when the
    %                                    simulation step size is changed (default: 1.0).
    %
    %                               **Note:** If the control structure is not defined, then by
    %                               default all graphic elements of the robot are enabled and
    %                               will be drawn in the simulation. In dependency of the
    %                               complexity of the given robot model and the performance of
    %                               the computer this may affect the visualization speed of the
    %                               robot simulation.
    % See Also:
    %   :meth:`WBM.simulateForwardDynamics`, :meth:`WBM.intForwardDynamics` and
    %   :meth:`WBM.getPositionsData`.
    switch nargin
        case 4
            % draw all graphic elements of the simulated robot ...
            vis_ctrl.drawJnts = true;
            vis_ctrl.drawCom  = true;
            vis_ctrl.drawSkel = true;
            vis_ctrl.drawBody = true;
            % use the default visualization speed value ...
            % note: use a different speed value in order to make the visualization speed
            %       close to real-time when the simulation time step is changed.
            vis_ctrl.vis_speed = 1.0;
        case 5
            if ~isstruct(vis_ctrl)
                error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
            end
            if ~isfield(vis_ctrl, 'vis_speed')
                % add default speed value ...
                vis_ctrl.vis_speed = 1.0;
            end
        otherwise
            error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    if ~isa(sim_config, 'WBM.wbmSimConfig')
        error('WBM::visualizeForwardDynamics: %s', WBM.wbmErrorMsg.WRONG_DATA_TYPE);
    end
    % initial plot of the animation environment (reset all subplots in the
    % main figure and reinitialize them with a new ground floor) ...
    sim_config = initSimEnvironment(obj, sim_config);

    visualizeSimRobot(obj, stmPos, sim_config, sim_tstep, vis_ctrl);
end
%% END of visualizeForwardDynamics.


%% ENVIRONMENT INITIALIZATION, LIGHT, TRAJECTORIES, LEGEND, ZOOM & SHIFT AXES:

function sim_config = initSimEnvironment(obj, sim_config)
    nAxes = sim_config.nAxes;

    % clear and reset all subplots of the figure window (in reverse order):
    for i = nAxes:-1:1
        % set the axes of the subplot to the current axes ...
        set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,i));

        sim_config.gfx_objects{1,i} = []; % clear the graphics container
        cla; % delete all graphics objects from the current axes ...
        axis off;
        hold on; % retain new plots in the current axes ...
    end

    % if some axes are defined to be zoomed and shifted, zoom and shift each
    % specified axes with the corresponding zoom factor and shift values:
    if ~isempty(sim_config.zoom_axes)
        zoomShiftAxes(sim_config);
    end

    % draw the colored ground (rectangle) and the corresponding origin point of the axis:
    % note: for correct redrawing of the scene, the ground objects will be stored in reverse order.
    gfx_ground = gobjects(2,1);
    gfx_ground(2,1) = fill3(sim_config.environment.grnd_shape(1,1:4), sim_config.environment.grnd_shape(2,1:4), ...
                            sim_config.environment.grnd_shape(3,1:4), sim_config.environment.grnd_color, ...
                            'EdgeColor', sim_config.environment.grnd_edge_color);
    gfx_ground(1,1) = plot3(0, 0, 0, 'LineStyle', 'none', 'Marker', '.', 'MarkerSize', sim_config.environment.orig_pt_size, ...
                            'MarkerEdgeColor', sim_config.environment.orig_pt_color);
    sim_config.gfx_objects{1,1} = gfx_ground;

    % if activated, set and show a point light:
    if sim_config.show_light
        gldta = opengl('data');
        hlt = setPointLight(sim_config.light_pos, gldta.Software);
        sim_config.gfx_objects{1,1} = vertcat(sim_config.gfx_objects{1,1}, hlt);
    end

    % if some target points are given, draw them in the
    % environment and add them to the graphics array:
    trg_pts = sim_config.target_pts;
    if ~isempty(trg_pts)
        if ~iscolumn(trg_pts)
            error('initSimEnvironment: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
        end
        nTrg    = size(trg_pts,1);
        gfx_trg = gobjects(nTrg,1);

        for i = 1:nTrg
            gfx_trg(i,1) = getGObj(trg_pts(i,1));
        end
        sim_config.gfx_objects{1,1} = vertcat(gfx_trg, sim_config.gfx_objects{1,1});
    end

    % if some link trajectories are given, draw them in the
    % environment and add them to the graphics array:
    traj = sim_config.trajectories;
    if ~isempty(traj)
        if ~iscolumn(traj)
            error('initSimEnvironment: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
        end
        nTraj    = size(traj,1);
        len      = 2*nTraj;
        gfx_traj = gobjects(len,1);

        j = -1;
        for i = 1:nTraj
            j = j + 2;
            gobj = getGObj(traj(i,1));

            gfx_traj(j,1)   = gobj(1,1);
            gfx_traj(j+1,1) = gobj(2,1);
        end
        sim_config.gfx_objects{1,1} = vertcat(gfx_traj, sim_config.gfx_objects{1,1});

        if sim_config.show_legend
            showTrajLegend(sim_config, traj, gfx_traj, nTraj, obj.mwbm_model.yarp_robot_type);
        end
    end

    % if some volume bodies for the scenario are given, create and draw these
    % volume bodies in the simulation environment:
    vb_objects = sim_config.environment.vb_objects;
    if ~isempty(vb_objects)
        if ~iscolumn(vb_objects)
            error('initSimEnvironment: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
        end
        nObj    = size(vb_objects,1);
        gfx_vbo = gobjects(nObj,1);

        for i = 1:nObj
            if (vb_objects(i,1).frame ~= vb_objects(i,1).init_frame)
                % set the frame to the initial condition ...
                setInitFrame(vb_objects(i,1));
            end
            gfx_vbo(i,1) = getGObj(vb_objects(i,1));
        end
        % add the volume body objects to the graphics list ...
        sim_config.gfx_objects{1,1} = vertcat(gfx_vbo, sim_config.gfx_objects{1,1});
    end
end

function hlt = setPointLight(lpos, srender)
    % set the light properties:
    hlt = light(gca, 'Position', lpos, 'Style', 'local', 'Visible', 'on');
    if ~srender
        % hardware-accelerated rendering ...
        lighting gouraud;
    else
        % software rendering ...
        lighting flat;
    end
    material dull;
end

function showTrajLegend(sim_config, traj, gfx_traj, nTraj, robot_type)
    labels = cell(1,nTraj);
    htl    = gobjects(1,nTraj);
    j = -1;
    for i = 1:nTraj
        j = j + 2;
        if ( ~isempty(traj(i,1).jnt_annot_pos) && ...
              strncmp(robot_type, 'iCub', 4) ) % type string starts with "iCub"
            grp_name = traj(i,1).jnt_annot_pos{1,1};
            idx      = traj(i,1).jnt_annot_pos{1,2};
            labels{1,i} = WBM.utilities.getJointAnnotationICub(grp_name, idx);
        else
            labels{1,i} = traj(i,1).urdf_link_name;
        end
        htl(1,i) = gfx_traj(j,1);
    end

    % static legend:
    legend(htl, labels, 'Interpreter', 'none', 'FontSize', sim_config.lgd_font_sz, ...
           'TextColor', sim_config.lgd_font_color, 'Color', sim_config.lgd_bkgrd_color, ...
           'EdgeColor', sim_config.lgd_edge_color, 'Location', sim_config.lgd_location, ...
           'Orientation', sim_config.lgd_orient);
    setappdata(gca, 'LegendColorbarListeners', []);
    setappdata(gca, 'LegendColorbarManualSpace', 1);
    setappdata(gca, 'LegendColorbarReclaimSpace', 1);
end

function zoomShiftAxes(sim_config)
    % scale the axis limits of the specified axes to the given zoom factor:
    axes_idx = uint8(sim_config.zoom_axes(:,1));
    zoom_fac = sim_config.zoom_axes(:,2);

    for i = 1:size(axes_idx,1)
        ax_idx = axes_idx(i,1);
        zfac   = zoom_fac(i,1);

        if ( (zfac > 0) && (zfac ~= 1) )
            set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,ax_idx));

            if WBM.utilities.isAxesZoomed(gca), zoom out; end
            zoom reset; % remember the current zoom setting (initial zoom setting)

            ax_lmts = axis(gca); % get the current axis limits
            ax_lmts(1,1:2) = zoomAxis(ax_lmts(1,1:2), zfac); % lim_x
            ax_lmts(1,3:4) = zoomAxis(ax_lmts(1,3:4), zfac); % lim_y
            ax_lmts(1,5:6) = zoomAxis(ax_lmts(1,5:6), zfac); % lim_z
            axis(gca, ax_lmts);

            zoom(gca, zfac);
        end
    end

    if ~isempty(sim_config.shift_axes)
        % shift the specified axes to the given shift values:
        axes_idx  = sim_config.shift_axes{1,1};
        shift_vec = sim_config.shift_axes{1,2};

        for i = 1:size(axes_idx,1)
            ax_idx = axes_idx(i,1);
            sh_vec = shift_vec(i,1:3);

            set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,ax_idx));

            ax_lmts = axis(gca);
            ax_lmts(1,1:2) = ax_lmts(1:2) + sh_vec(1,1); % lim_x
            ax_lmts(1,3:4) = ax_lmts(3:4) + sh_vec(1,2); % lim_y
            ax_lmts(1,5:6) = ax_lmts(5:6) + sh_vec(1,3); % lim_z
            axis(gca, ax_lmts);
        end
    end
    % set back the current axes handle to the first ...
    set(sim_config.hWndFigure, 'CurrentAxes', sim_config.hAxes(1,1));
end

function ax_lim = zoomAxis(ax_lim, zfac)
    % Scale the axis limits for zooming.
    % Source: <https://stackoverflow.com/questions/33022127/work-with-the-zoom-in-the-figure>
    ax_min = ax_lim(1,1);
    ax_max = ax_lim(1,2);
    r_zf = 1/zfac; % reciprocal zoom factor

    d_ax   = ax_max - ax_min; % distance btw. the limits of the axis
    lim_s  = d_ax * r_zf;     % new limit scaled to the zoom factor
    sh_val = ax_max + ax_min; % shift value (sum of both limits)
    % set the new scaled and shifted axis limits ...
    ax_lim(1,1) = (-lim_s + sh_val) * 0.5; % new min.
    ax_lim(1,2) = ( lim_s + sh_val) * 0.5; % new max.
end
