function sim_config = initSimConfig_iCub()
	% List of joint names for constructing the iCub-Robot in the visualizer:
	%																  idx:
	joint_names = { 'root_link'; ...								%  1,
					'r_hip_1'; 'r_lower_leg'; 'r_sole'; ...			%  2,  3,  4,
					'l_hip_1'; 'l_lower_leg'; 'l_sole'; ...			%  5,  6,  7,
					'neck_1'; ...									%  8,
					'r_shoulder_1'; 'r_elbow_1'; 'r_gripper'; ...	%  9, 10, 11,
					'l_shoulder_1'; 'l_elbow_1'; 'l_gripper'; ...	% 12, 13, 14,
					'com' };										% 15

	% Set of joint-pair indexes to describe the configuration of the iCub-robot's
	% skeleton. Each pair of joints is connected with a rigid link (edge) to form
	% a kinematic chain of the robot. The index-pairs denotes the index-position
	% of the given joint names in the above joint name list. The index-set is used
	% to create a set of position parameters that describes the full configuration
	% of the robot system.
	%
	% Joint pair indexes for the x, y and z-positions (translations) in the 3D-space:
	%						x1 x2   y1 y2   z1 z2
	joint_pair_idx = uint8([ 1  8    1  8    1  8;
							 1  2    1  2    1  2;
							 2  3    2  3    2  3;
							 3  4    3  4    3  4;
							 1  5    1  5    1  5;
							 5  6    5  6    5  6;
							 6  7    6  7    6  7;
							 8  9    8  9    8  9;
							 9 10    9 10    9 10;
							10 11   10 11   10 11;
							 8 12    8 12    8 12;
							12 13   12 13   12 13;
							13 14   13 14   13 14]);

	% List with constant scale factors to define the sizes of the shapes (patches) for
	% the links (edges) of the robot's skeleton. They form the shape of the robot's body:
	shape_geom.size_sf = [0.07   0.03;
						  0.04   0.02;
						  0.03   0.02;
						  0.025  0.02;
						  0.04   0.02;
						  0.03   0.02;
						  0.025  0.02;
						  0.03   0.02;
						  0.025  0.02;
						  0.02   0.02;
						  0.03   0.02;
						  0.025  0.02;
						  0.02   0.02];
		
	% Connection matrix to define which vertices are to connect for creating the shapes
	% of the links or the shapes of the feets. Each row represents one polygon (rectangle):
	shape_geom.faces = uint8([1 2 3 4;
							  1 4 8 5;
							  5 8 7 6;
							  7 3 2 6;
							  2 6 5 1;
							  3 7 8 4]);

	% Joint indices of the joints for the left and the right foot:
    foot_geom.joints = uint8([4 7]);

    % Base size values to create the feets for the robot:
    foot_geom.base_sz.width  = 0.025;
    foot_geom.base_sz.height = 0.015;
    %foot_geom.base_sz = [0.025 0.015]; % optional

    % List with additional size values to define the foot dimensions:
    %						x:     y:  z:
    %					   len    wid hgt
    foot_geom.shape_ds = [-0.002   0   0; % back up
						  -0.002   0   0;
						  -0.01    0   0; % back bottom
						  -0.01    0   0;
						   0.025   0   0; % front up
						   0.025   0   0;
						   0.11    0   0; % front bottom
						   0.11    0   0];

	% Create the body model of the animated robot in the simulation:
	sim_body = WBM.wbmSimBody(joint_names, joint_pair_idx);

	% Geometry properties for the shape of the robot's body:
	sim_body.shape_geom = shape_geom;
	sim_body.foot_geom  = foot_geom;

	% Draw properties for the body of the robot and the ground floor:
	sim_body.draw_prop.joints.color     = WBM.wbmColor.orange;
	sim_body.draw_prop.links.color      = 'black';
	sim_body.draw_prop.shape.face_color = WBM.wbmColor.royalblue4;
	sim_body.draw_prop.shape.edge_color = WBM.wbmColor.royalblue4;
	sim_body.draw_prop.ground_color     = WBM.wbmColor.papayawhip;

	% Create the configuration object for the WBM-Simulator:
    sim_config = WBM.genericSimConfig(sim_body, 'iCub-Simulator:');
end