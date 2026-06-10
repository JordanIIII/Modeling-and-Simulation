% Calculate a matrix form of twist from the input of a vector twist
function t_tilda = tilda_twist(unit_twists)
    n = length(unit_twists);
    t_tilda = cell(1, n);
    
    % Calculate i-th w_tilda and t_tilda for all twists
    for i=1:n
        T = unit_twists{i};
        w = T(1:3);
        v = T(4:6);

        w_tilda = [0 -w(3) w(2);
                   w(3) 0 -w(1);
                   -w(2) w(1) 0];

        t_tilda{i} = [w_tilda, v;
                      0 0 0 0];
    end
end