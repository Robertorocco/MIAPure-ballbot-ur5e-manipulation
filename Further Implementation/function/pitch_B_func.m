function M = pitch_B_func(theta)
% AUTO-GENERATED: returns B matrix for scheduling theta
theta_min = -0.261799; theta_max = 0.261799; 
theta = min(max(theta, theta_min), theta_max);

M = zeros(4,1);
M(1,1) = polyval([0 0 0 0 0], theta);
M(2,1) = polyval([-0.20184688449634747 -3.6013392822984911e-16     0.28156494886986833  1.2181206393332252e-17    -0.29638866175376399], theta);
M(3,1) = polyval([0 0 0 0 0], theta);
M(4,1) = polyval([1.4009127161415373   2.199504334224085e-15     -1.7521628369080235 -8.8645220148596527e-17       2.975470523881274], theta);
end
