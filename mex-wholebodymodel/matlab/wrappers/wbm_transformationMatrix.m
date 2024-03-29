function wf_H_lnk = wbm_transformationMatrix(varargin)
    % WBM_TRANSFORMATIONMATRIX computes the homogeneous transformation matrix H of a specified
    % link (frame) of the robot w.r.t. the current joint configuration q_j.
    %
    % Input Arguments:
    %
    %   Normal mode:
    %       wf_R_b         -- (3 x 3) rotation matrix from the base frame 'b' to the world frame 'wf'.
    %       wf_p_b         -- (3 x 1) position vector from the base frame 'b' to the world frame 'wf'.
    %       q_j            -- (n_dof x 1) joint angle vector in [rad].
    %       urdf_link_name -- String matching URDF name of the link.
    %
    %   Optimized mode:
    %       urdf_link_name -- String matching URDF name of the link.
    %
    % Output Arguments:
    %   wf_H_lnk -- (4 x 4) homogeneous transformation matrix relative from the link frame 'lnk'
    %               to the world frame 'wf'.

    % Copyright (C) 2017 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
    % Author: Martin Neururer (martin.neururer@gmail.com); Genova, Jan 2017
    %
    % This function is part of the Whole-Body Model Library for Matlab (WBML).
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
    switch nargin
        case 1
            wf_H_lnk = yarpWBM('transformation-matrix', varargin{1,1});
        case 4
            wf_H_lnk = yarpWBM('transformation-matrix', reshape(varargin{1,1}, 9, 1), varargin{1,2}, ...
                                                                  varargin{1,3}, varargin{1,4});
        otherwise
            wbm_narginError('wbm_transformationMatrix');
    end
end
