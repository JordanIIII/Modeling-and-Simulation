% Computes the geometric Jacobian matrix of a serial chain mechanism
% Input: unit_twists - cell array of n 6x1 matrices - unit twists of the joints T_i^{(i-1), (i-1)}
%        H0s - cell array of (at least) n-1 4x4 matrices - reference configurations of the joints H_i^{i-1}(0)
%        q - nx1 matrix - joint variables q^i

function J = get_jacobian(unit_twists, H0s, q)
    n = length(unit_twists);
    
    % Step 1: Acquire Hs and t_tilda
    Hs = direct_kinematics(unit_twists, H0s, q);
    t_tilda = tilda_twist(unit_twists);

    % Step 2: Inverse of Hs
    invHs = cell(1,n);
    for i=1:n
        H = Hs{i};
        R = H(1:3, 1:3);
        p = H(1:3, 4);
        Rt = R.';
        invHs{i} = [Rt, -Rt*p;
                    0 0 0 1];
    end

    % Step 3: Change of Coordinate of Twist
    J = zeros(6, n); % Jacobian matrix
    
    for i=1:n
        T = t_tilda{i};     % tilda(T)_i^(i-1),(i-1): Unit twist expressed in (i-1)th frame
        H = Hs{i};          % H_i^0: Coordinate representation of configuration
        invH = invHs{i};    % F_0î: Inverse of H_i^0

        % Change of coordinate
        Tm = H * T * invH;  % Matrix form
        w = [Tm(3,2);
             Tm(1,3);
             Tm(2,1)];
        v = Tm(1:3, 4);

        Tv = [w; v];        % Vector form

        J(:, i) = Tv;       % Jacobian Matrix
    end
end