function M = roll_B_func(theta)
% AUTO-GENERATED: returns B matrix for scheduling theta
theta_min = -0.261799; theta_max = 0.261799; 
theta = min(max(theta, theta_min), theta_max);

M = zeros(4,1);
M(1,1) = polyval([0 0 0 0 0], theta);
M(2,1) = polyval([-0.21925089508820819 -4.5239626301308055e-15     0.29795033430329682  1.2532257761571967e-16    -0.30719110831344948], theta);
M(3,1) = polyval([0 0 0 0 0], theta);
M(4,1) = polyval([1.5032467034763417  3.6366635771719867e-14     -1.8409507744747191 -1.0196550378786071e-15      3.0195420591996762], theta);
end
