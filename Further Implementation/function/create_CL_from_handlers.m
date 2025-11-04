function [Am_func, Bm_func, Cm, Dm, Pm_func] = create_CL_from_handlers(A_func, B_func, K_func)

theta_points_deg = -15:3:15;
theta_points_rad = deg2rad(theta_points_deg);
num_points = length(theta_points_rad);

isConstA = false; isConstB = false; isConstK = false;
 try
      A = A_func();
      isConstA = true;
 catch 
     
 end
 try
      B = B_func();
      isConstB = true;
 catch
   
 end
 try
      K = K_func();
      isConstK = true;
 catch
     
 end


%% If all constant -> compute single CL matrices and return constant handles
if isConstA && isConstB && isConstK
        n = size(A,1); m = size(B,2);
    A_cl = A - B * K;
    
    % feedforward scaling to make 1st state ('theta') unity steady-state (best-effort)
    C_phi = zeros(1,n); C_phi(1) = 1;
    N = -1 / (C_phi * inv(A_cl) * B);   
    Bm = B * N;
    Cm = eye(n);
    Dm = zeros(n, size(Bm,2));
    
    Q = eye(n);
    try
        Pm = lyap(A_cl', Q);
    catch
            error('Unable to compute Pm; Am is likely not Hurwitz.');
    end
    Am_func = A_cl;
    Bm_func = Bm;
    Pm_func = Pm;
    return;
end


%% --- Preallocate containers ---
% first evaluate one point to infer sizes
theta0 = theta_points_rad(1);
A0 = A_func(theta0);
B0 = B_func(theta0);

n = size(A0,1);
m = size(B0,2);

Am_model = cell(1, num_points);
Bm_model = cell(1, num_points);
Pm_model = cell(1, num_points);
for i = 1:num_points
    theta = theta_points_rad(i);
    % Obtain A B and K  (try common calling signatures)
        try
            A = A_func(theta);
        catch
            try
                A = A_func([], theta);
            catch
                error('Unable to call A_func(theta). Provide A_func as @(theta) A.');
            end
        end
        try
            B = B_func(theta);
        catch
            try
                B = B_func([], theta);
            catch
                error('Unable to call B_func(theta). Provide B_func as @(theta) B.');
            end
        end

        try
            K = K_func(theta);
        catch
            try
                K = K_func([], theta);
            catch
                error('Unable to call K_func(theta). Provide K_func as @(theta) K.');
            end
        end

    % Compute closed-loop Reference model matrices Am matrix
    A_cl = A - B * K;
    Am_model{i} = A_cl;
    % Compute feedforward scaling N so the 'phi' output (3rd state) steady-state gain = 1
    C_phi = zeros(1, n); C_phi(3) = 1;
    N = -1 / (C_phi * inv(A_cl) * B);
    Bm_model{i} = B * N;
     
    % Solve Lyapunov: Am' * P + P * Am = -Q  => P = lyap(Am', Q)
    Q=eye(n);
    try
        Pm_model{i} = lyap(A_cl', Q);
    catch
            Pm_model = nan(n_expected);
            error('Unable to compute Pm: Am must be Hurwitz. Check K or theta.');
    end
end

%% Step 3: Fit Polynomials to the Gain Elements
disp('Fitting polynomials to gain schedules...');
poly_order = 4;

% Preallocate coefficient storage
coeffs_Am = cell(n,n);
coeffs_Bm = cell(n,m);
coeffs_Pm = cell(n,n);

% Fit State Matrix 
for row = 1:n
    for col =1:n
        data_points = cellfun(@(m) m(row, col), Am_model);
        coeffs_Am{row,col} = polyfit(theta_points_rad, data_points, poly_order);
    end
end

% Fit Input Matrix 
for row = 1:n
    for col =1:m
        data_points = cellfun(@(m) m(row, col), Bm_model);
        coeffs_Bm{row,col} = polyfit(theta_points_rad, data_points, poly_order);
    end
end

% Fit Lyap Matrix 
for row = 1:n
    for col =1:n
        data_points = cellfun(@(m) m(row, col), Pm_model);
        coeffs_Pm{row,col} = polyfit(theta_points_rad, data_points, poly_order);
    end
end
%% Step 4: Create Final, Callable Functions

disp('Creating final function handles for Am, Bm, Pm...');
%% --- Create callable function handles that evaluate the polynomials ---
Am_func = @(theta) eval_matrix_poly(theta, coeffs_Am, n);
Bm_func = @(theta) eval_matrix_poly(theta, coeffs_Bm, n, m);
Pm_func = @(theta) eval_matrix_poly(theta, coeffs_Pm, n);

% Cm and Dm are constant
Cm = eye(n);
Dm = zeros(n, m);
end

function M = eval_matrix_poly(theta, coeffs_cell, nrows, ncols)
    % If ncols not provided, assume square and infer from coeffs_cell
    if nargin < 4
        ncols = nrows;
    end
    M = zeros(nrows, ncols);
    for r = 1:nrows
        for c = 1:ncols
            coeffs = coeffs_cell{r,c};
            if isempty(coeffs)
                M(r,c) = 0;
            else
                M(r,c) = polyval(coeffs, theta);
            end
        end
    end
end