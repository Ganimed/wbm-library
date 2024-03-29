function cX_v = comixvelcp(varargin)
    %% Dual (spatial) cross product operator for a
    %  mixed velocity vector (v, w)^T in R^6:
    %
    % Sources:
    %   [1] Multibody Dynamics Notation, S. Traversaro & A. Saccon, Eindhoven University of Technology,
    %       Department of Mechanical Engineering, 2016, <http://repository.tue.nl/849895>, p. 11, eq. (53).
    %   [2] Rigid Body Dynamics Algorithms, Roy Featherstone, Springer, 2008, pp. 23-25, eq. (2.31).
    switch nargin
        case 2
            v = varargin{1,1}; % linear velocity
            w = varargin{1,2}; % angular velocity
        case 1
            v_m = varargin{1,1};
            WBM.utilities.chkfun.checkCVecDim(v_m, 6, 'comixvelcp');

            % get the linear and angular velocity
            % of the mixed velocity v_m ...
            v = v_m(1:3,1);
            w = v_m(4:6,1);
        otherwise
            error('comixvelcp: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end
    % skew-symmetric matrices of the velocities ...
    Sv = WBM.utilities.tfms.skewm(v);
    Sw = WBM.utilities.tfms.skewm(w);

    % create the dual cross operator:
    cX_v = zeros(6,6);
    cX_v(1:3,1:3) = Sw;
    cX_v(4:6,1:3) = Sv;
    cX_v(4:6,4:6) = Sw;
end
