% Computes the joint velocities dq to move the end effector to the setpoint
% Input: q - nx1 matrix - joint variables q^i
%        setpoint - 3x1 matrix - desired end-effector position
%        dsetpoint - 3x1 matrix - desired end-effector velocity
%        H0s - cell array of n 4x4 matrices - reference (q = 0) configurations of the links H_i^{i-1}(0)
%        unit_twists - cell array of n 6x1 matrices - unit twists of the joints T_i^{(i-1), (i-1)}
% Output: dq - nx1 matrix - joint velocities dq^i

function dq = calculate_dq(q, setpoint, dsetpoint, H0s, unit_twists)

% Call Homogeneous matrices expressed in Psi_0
Hs = direct_kinematics(unit_twists, H0s, q);
J = get_jacobian(unit_twists, H0s, q);

% Step 1: Extract current position pee(t) and Calculate error
H_ee = Hs{end};  % Take the last element of cell of Hs -> configuration of EE
p_ee = H_ee(1:3, 4);    % 3-by-1 current position of EE

e = setpoint - p_ee;    % EE velocity

% Step 2: Calculate EE velocity
k = 50;
dp_ee = dsetpoint + k* e;

% Limit the end-effector velocity
if norm(dp_ee) > 700
    dp_ee = 700*(dp_ee/norm(dp_ee)); 
end

% Step 3: Change of coordinate to Psi_e
H0_e = [eye(3) p_ee;
        0 0 0 1];

R = H0_e(1:3, 1:3);
p = H0_e(1:3, 4);
Rt = R.';
H0_e = [Rt, -Rt*p;
         0 0 0 1];

% Step 4: Calculate adjoint matrix
p = H0_e(1:3, 4);
px = p(1);
py = p(2);
pz = p(3);
p_tilda = [0 -pz py;
           pz 0 -px;
           -py px 0];
Re = H0_e(1:3, 1:3);

% Adjoint matrix (6-by-6)
Adj= [Re zeros(3, 3);
      p_tilda*Re Re];

% Step 5:Expression of twist in Psi_e
Ad_J = Adj * J;
% Extract last three rows
J_bar = Ad_J(4:6, :);

% Step 6: Compute Moore-Penrose pseudo-invers
MP_J = J_bar' / (J_bar * J_bar');

% Step 7: Desired linear velocity of EE
dq = MP_J * dp_ee;

end