
% FILENAME: analyze_results_imagesc.m
% =========================================================================
% Correlation heatmaps (Pearson & Spearman)
% Uses imagesc() for precise control and publication-quality output.
% =========================================================================
%% 1. Load Data
clc; close all;
disp('Loading simulation results...');
load('risultati_simulazione.mat');
%load('ris_op_.mat');
valid_results = results_table(isfinite(results_table.Cost), :);
fprintf('Analyzing %d valid tests out of %d total.\n', ...
    height(valid_results), height(results_table));
%% 2. Prepare Data
InitialDist = sqrt(valid_results.StartX.^2 + valid_results.StartY.^2);
InitialTilt = sqrt(valid_results.StartRoll.^2 + valid_results.StartPitch.^2);
% Usa sintassi LaTeX nei label
param_vars = {'$K_p^{pos}$', '$K_p^{inner}$', '$K_i^{inner}$', ...
              '$Q_1$', '$Q_2$', '$Q_3$', '$Q_4$'};
condition_vars = {'InitialDist', 'InitialTilt'};
output_vars = {'RMSE', 'MaxDist', 'ConvTime', '$OP\_RMSE$', 'Cost'};
output_vars = {'RMSE', 'MaxDist', 'ConvTime', 'Cost'};
param_data = valid_results{:, {'Kp_pos', 'Kp_inner', 'Ki_inner', 'Q1', 'Q2', 'Q3', 'Q4'}};
condition_data = [InitialDist, InitialTilt];
%output_data = valid_results{:, {'RMSE', 'MaxDist', 'ConvTime', 'OP_RMSE', 'Cost'}};
output_data = valid_results{:, {'RMSE', 'MaxDist', 'ConvTime', 'Cost'}};

data_for_corr = [param_data, condition_data, output_data]; 
labels = [param_vars, condition_vars, output_vars];
n = length(labels);
%% 3. Utility function for plotting triangular correlation matrix
function plot_corr_matrix(corr_matrix, labels, title_str, file_name)
    % Imposta triangolo inferiore a NaN
    corr_matrix(tril(true(size(corr_matrix)))) = NaN;
    % Imposta NaN e Inf come nero
    corr_matrix(isinf(corr_matrix)) = NaN;
    nan_mask = isnan(corr_matrix);
    % Crea mappa colori estesa (summer + nero)
    base_cmap = summer(256);
    cmap = [0 0 0; base_cmap];  % aggiungi nero in cima
    figure('Color', 'w', 'Position', [100 100 900 700]);
    hold on;
    % Disegna la matrice
    imagesc(corr_matrix, [-1 1]);
    colormap(cmap);
    colorbar('southoutside');
    axis equal tight;
    % Imposta tick con interpretazione LaTeX
    set(gca, 'YDir', 'normal', ...
        'XTick', 1:length(labels), 'XTickLabel', labels, ...
        'YTick', 1:length(labels), 'YTickLabel', labels, ...
        'FontSize', 12, 'TickLabelInterpreter', 'latex');
    title(title_str, 'Interpreter', 'latex', 'FontSize', 16);
    % Sovrascrivi le celle NaN con rettangoli neri
    [row_nan, col_nan] = find(nan_mask);
    for k = 1:length(row_nan)
        rectangle('Position', [col_nan(k)-0.5, row_nan(k)-0.5, 1, 1], ...
            'FaceColor', [0 0 0], 'EdgeColor', 'none');
    end
    % Scrivi numeri (2 cifre significative)
    for i = 1:length(labels)
        for j = 1:length(labels)
            val = corr_matrix(i,j);
            if ~isnan(val)
                text(j, i, sprintf('%.2f', val), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'FontSize', 10, ...
                    'FontWeight', 'bold');
            end
        end
    end
    box on;
    exportgraphics(gcf, file_name, 'ContentType', 'vector');
end
%% 4. Pearson correlation
disp('Calculating Pearson correlation...');
corr_pearson = corrcoef(data_for_corr);
plot_corr_matrix(corr_pearson, labels, ...
    'Linear Correlation (Pearson)', ...
    'Pearson_Triangular.pdf');
%% 5. Spearman correlation
disp('Calculating Spearman correlation...');
corr_spearman = corr(data_for_corr, 'Type', 'Spearman');
plot_corr_matrix(corr_spearman, labels, ...
    'Monotonic Correlation (Spearman)', ...
    'Spearman_Triangular.pdf');
disp('Analysis complete. Check the generated figures.');