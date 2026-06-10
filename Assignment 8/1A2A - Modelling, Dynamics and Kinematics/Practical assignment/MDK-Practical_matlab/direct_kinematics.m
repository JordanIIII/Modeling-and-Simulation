% Computes the configuration all the links of a serial chain mechanism relative to the base frame Psi_0
% Input: unit_twists - cell array of n 6x1 matrices - unit twists of the joints T_i^{(i-1), (i-1)}
%        H0s - cell array of n 4x4 matrices - reference (q = 0) configurations of the links H_i^{i-1}(0)
%        q - nx1 matrix - joint variables q^i
% Output: Hs - cell array of n+1 4x4 matrices - current configurations of the links 0 to n of the serial chain mechanism relative to the base frame Psi_0

function Hs = direct_kinematics(unit_twists, H0s, q)
    % unit_twists = {T1, T2, T3, T4, T5, T6};
    % H0s = {H0_1, H0_2, H0_3, H0_4, H0_5, H0_6};

    n = length(unit_twists);
        % Create 1-by-(n+1) cell array of empty cells
        % Each cell store the homogeneous matrices of links 0-n
        Hs = cell(1, n+1);

    % Link 1~n
    % Step 1: Calculate matrix twists - tilda(T)
    %         The elementary function: tilda_twist.m
    %         Tilda matrices are 4-by-4, and t_tilda stores 6 twist matrices 1~n
    t_tilda = tilda_twist(unit_twists);
    
    % Step 2: Calculate Hs_i^(i-1)
    Hs_pair = cell(1,n);
    for i=1:n
        Hs_pair{i} = expm(t_tilda{i} * q(i)) * H0s{i};
    end

    % Step 3: Calculate Hs relative to the base frame Psi(0)
              % Apply chain rule to acquire current configurations
    cur_Hs = eye(4);
    Hs{1} = cur_Hs;    % Link 0 (Configuration of the base link)
    for i=1:n
        cur_Hs = cur_Hs * Hs_pair{i};
        Hs{i+1} = cur_Hs;
    end
end