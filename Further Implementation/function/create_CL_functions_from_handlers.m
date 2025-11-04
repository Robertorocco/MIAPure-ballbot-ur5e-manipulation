function [Am_func, Bm_func, Cm, Dm, Pm_func] = create_CL_functions_from_handlers(A_func, B_func, K_func, prefix)
% CREATE_CL_FROM_HANDLERS_GENERATE_FILES
% Generates closed-loop Am, Bm, Pm matrices and exports them as standalone .m files
% usable in MATLAB Function blocks or anywhere else.
%
% INPUTS:
%   A_func, B_func, K_func : function handles (LPV or constant)
%   prefix : string used to name generated files (e.g., 'roll')
%
% OUTPUTS:
%   Am_func, Bm_func, Pm_func : thin handles calling the generated functions
%   Cm, Dm : identity / zero matrices (constant)
%
% EXAMPLE:
%   [Am,Bm,Cm,Dm,Pm] = create_CL_functions_from_handlers(A_roll,B_roll,K_roll,'roll');

theta_points_deg = -15:3:15;
theta_points_rad = deg2rad(theta_points_deg);
num_points = length(theta_points_rad);
target_folder = fullfile(pwd, 'function');

%% Detect if A,B,K are constant
isConstA = false; isConstB = false; isConstK = false;
try A = A_func(); isConstA = true; catch, end
try B = B_func(); isConstB = true; catch, end
try K = K_func(); isConstK = true; catch, end

%% If all constant -> just compute one CL
if isConstA && isConstB && isConstK
    n = size(A,1); m = size(B,2);
    A_cl = A - B*K;
    
    % Feedforward scaling (1st state)
    C_phi = zeros(1,n); C_phi(1) = 1;
    N = -1 / (C_phi * inv(A_cl) * B);   
    B_cl = B*N;
    
    Cm = eye(n);
    Dm = zeros(n, size(B_cl,2));
    
    Q = eye(n);
    P_cl = lyap(A_cl', Q);
    
    % Write constant matrices to files
    write_matrix_func(fullfile(target_folder, [prefix '_Am_func']), A_cl);
    write_matrix_func(fullfile(target_folder, [prefix '_Bm_func']), B_cl);
    write_matrix_func(fullfile(target_folder, [prefix '_Pm_func']), P_cl);
    
    Am_func = @(theta) feval([prefix '_Am_func'], theta);
    Bm_func = @(theta) feval([prefix '_Bm_func'], theta);
    Pm_func = @(theta) feval([prefix '_Pm_func'], theta);
    return;
end

%% --- Preallocate containers ---
theta0 = theta_points_rad(1);
A0 = A_func(theta0); B0 = B_func(theta0);
n = size(A0,1); m = size(B0,2);

Am_model = cell(1, num_points);
Bm_model = cell(1, num_points);
Pm_model = cell(1, num_points);

for i = 1:num_points
    theta = theta_points_rad(i);
    A = try_call(A_func, theta);
    B = try_call(B_func, theta);
    K = try_call(K_func, theta);
    
    A_cl = A - B*K;
    Am_model{i} = A_cl;
    
    C_phi = zeros(1,n); C_phi(3) = 1;
    N = -1 / (C_phi * inv(A_cl) * B);
    Bm_model{i} = B*N;
    
    Q = eye(n);
    Pm_model{i} = lyap(A_cl', Q);
end

%% Step 3: Fit polynomials to each element
poly_order = 4;

coeffs_Am = fit_matrix_polynomials(Am_model, theta_points_rad, n, n, poly_order);
coeffs_Bm = fit_matrix_polynomials(Bm_model, theta_points_rad, n, m, poly_order);
coeffs_Pm = fit_matrix_polynomials(Pm_model, theta_points_rad, n, n, poly_order);

% Ensure the folder exists
if ~exist(target_folder, 'dir')
    mkdir(target_folder);
end

%% Step 4: Write .m files
write_matrix_func(fullfile(target_folder, [prefix '_Am_func']), coeffs_Am, n, n,theta_points_rad(1),theta_points_rad(end));
write_matrix_func(fullfile(target_folder, [prefix '_Bm_func']), coeffs_Bm, n, m,theta_points_rad(1),theta_points_rad(end));
write_matrix_func(fullfile(target_folder, [prefix '_Pm_func']), coeffs_Pm, n, n,theta_points_rad(1),theta_points_rad(end));

%% Step 5: Return thin handles
Am_func = @(theta) feval([prefix '_Am_func'], theta);
Bm_func = @(theta) feval([prefix '_Bm_func'], theta);
Pm_func = @(theta) feval([prefix '_Pm_func'], theta);

Cm = eye(n);
Dm = zeros(n, m);

end

%% --- HELPER FUNCTIONS ---
function M = try_call(F, theta)
% Tries multiple calling signatures for A,B,K
try M = F(theta);
catch
    try M = F([],theta);
    catch
        error('Cannot call function handle');
    end
end
end

function coeffs = fit_matrix_polynomials(Mcell, theta_vec, nrows, ncols, order)
coeffs = cell(nrows,ncols);
for r=1:nrows
    for c=1:ncols
        data_points = cellfun(@(m) m(r,c), Mcell);
        coeffs{r,c} = polyfit(theta_vec, data_points, order);
    end
end
end

function write_matrix_func(file_path, coeffs_cell, nrows, ncols, theta_min, theta_max)
% WRITE_MATRIX_FUNC  Write a codegen-friendly .m file that returns matrix M(theta)
% file_path   : full path or relative path without extension (e.g. fullfile(target_folder,'roll_Am_func'))
% coeffs_cell : if nargin < 3 then coeffs_cell is a numeric matrix (constant-mode)
%               otherwise it's a cell of coefficient vectors (polynomial-mode)
% nrows,ncols : number of rows/cols for polynomial-mode
% theta_min, theta_max : scalar bounds in radians for clamping (only used in poly mode)

% Ensure directory exists and separate name from folder
[folder, base_name, ~] = fileparts(file_path);
if isempty(folder), folder = pwd; end
if ~exist(folder,'dir'), mkdir(folder); end

fname = base_name;
fname_full = fullfile(folder, [fname '.m']);
fid = fopen(fname_full, 'w');
if fid == -1
    error('Cannot open file for writing: %s', fname_full);
end

% Write function header (use base name only)
fprintf(fid, 'function M = %s(theta)\n', fname);
fprintf(fid, '%% AUTO-GENERATED by create_CL_functions_from_handlers\n');

% Decide constant vs polynomial mode
if nargin < 3 || isempty(nrows)
    % constant numeric matrix provided in coeffs_cell
    mat = coeffs_cell;
    [r,c] = size(mat);
    % Write matrix literal preserving rows with semicolons
    fprintf(fid, '%% constant matrix. theta input ignored.\n');
    fprintf(fid, 'M = [\n');
    for rr = 1:r
        rowstr = sprintf(' %0.17g ', mat(rr,:));
        % replace spaces with ' ' and preserve formatting; add semicolon for each row except last
        if rr < r
            fprintf(fid, '  %s;\n', strtrim(rowstr));
        else
            fprintf(fid, '  %s\n', strtrim(rowstr));
        end
    end
    fprintf(fid, '];\n');
else
    % polynomial mode: coeffs_cell is a cell(nrows,ncols) of coef vectors
    % safety: ensure theta bounds are provided
    if nargin < 5
        error('theta_min and theta_max must be provided for polynomial mode.');
    end
    fprintf(fid, '%% polynomial-mode: evaluate elementwise polynomials\n');
    fprintf(fid, 'theta_min = %g; theta_max = %g; \n', theta_min, theta_max);
    fprintf(fid, '%% clamp theta to [theta_min, theta_max]\n');
    fprintf(fid, 'theta = min(max(theta, theta_min), theta_max);\n\n');
    fprintf(fid, 'M = zeros(%d,%d);\n', nrows, ncols);
    for r = 1:nrows
        for c = 1:ncols
            v = coeffs_cell{r,c}(:)'; % row vector
            fprintf(fid, 'M(%d,%d) = polyval([%s], theta);\n', r, c, num2str(v,'%.17g '));
        end
    end
end

fprintf(fid, 'end\n');
fclose(fid);

% Add the folder to path so the function is immediately callable (optional but convenient)
% (Do not add repeatedly in long loops; but harmless.)
if ~isempty(folder)
    addpath(folder);
end
end
