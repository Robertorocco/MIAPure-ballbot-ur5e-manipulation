function [cost, metrics] = calculate_mrac_cost(params, modelName, p, t_end, dt, x0, waypoints_ts, ...
                                             Am_x, Bm_x, P_x, Am_y, Bm_y, P_y, Am_z, Bm_z, Pz, ...
                                             weights, baseline_metrics)
%CALCULATE_MRAC_COST Runs one simulation and calculates the cost.
%
%   This function is called by the optimizer.
    % Assign parameters from the optimizer
    alpha = params(1);
    beta  = params(2);
    gamma = params(3);
    
    cost = 1e10; % Default high cost in case of failure
    
    %% --- MODIFIED --- %%
    % Updated metrics struct: removed 'peak_control', added 'saturation_penalty'
    metrics = struct('itae', 1e10, 'settling_time', 1e10, 'overshoot', 1e10, 'control_effort', 1e10, 'saturation_penalty', 1e10);
    try
        % --- 1. Run the Simulation ---
        simIn = Simulink.SimulationInput(modelName);
        
        % Pass all required MATLAB variables to the model's workspace
        simIn = simIn.setVariable('p', p);
        simIn = simIn.setVariable('Am_y', Am_y);
        simIn = simIn.setVariable('Bm_y', Bm_y);
        simIn = simIn.setVariable('Am_x', Am_x);
        simIn = simIn.setVariable('Bm_x', Bm_x);
        simIn = simIn.setVariable('Am_z', Am_z);
        simIn = simIn.setVariable('Bm_z', Bm_z);
        simIn = simIn.setVariable('Px', P_x);
        simIn = simIn.setVariable('Py', P_y);
        simIn = simIn.setVariable('Pz', Pz);
        simIn = simIn.setVariable('x0', x0);
        simIn = simIn.setVariable('waypoints_ts', waypoints_ts);
        
        % Pass the parameters we are tuning
        simIn = simIn.setVariable('alpha', alpha);
        simIn = simIn.setVariable('beta', beta);
        simIn = simIn.setVariable('gamma', gamma);
        
        % Set simulation settings
        simIn = simIn.setModelParameter('StopTime', num2str(t_end));
        simIn = simIn.setModelParameter('FixedStep', num2str(dt));
        simIn = simIn.setModelParameter('CaptureErrors', 'on'); % Catch sim errors        simOut = sim(simIn);
        simOut = sim(simIn);
        % --- 2. Extract Logged Data ---
        t = simOut.tout;
        e_tracking = permute(simOut.e_sim.signals.values,[3 1 2]); % (N x 10)
        x_state    = permute(simOut.x_sim.signals.values,[3 1 2]); % (N x 10)
        tau_input  = permute(simOut.tau_sim.signals.values,[1 2 3]); % (N x 3)
        
        K_hat = permute(simOut.K_hat_sim.signals.values,[3 2 1]);
        Kr_hat = permute(simOut.Kr_hat_sim.signals.values,[1 2 3]);
        Ke_hat = permute(simOut.Ke_hat_sim.signals.values,[1 2 3]);
        all_gains = [K_hat, Kr_hat, Ke_hat];
        % --- 3. Calculate Raw Metrics ---
        
        % Metric 1: ITAE (Integral of Time-weighted Absolute Error)
        error_subset = e_tracking(:, [1, 3, 5, 7]);
        metrics.itae = trapz(t, t .* sum(abs(error_subset), 2));

        % Metric 2: Settling Time of Adaptation Gains
        if size(all_gains, 1) > 2
            gain_derivative = diff(all_gains) ./ diff(t);
            gain_deriv_norm = vecnorm(gain_derivative, 2, 2); 
            threshold = 0.05 * max(gain_deriv_norm);
            if threshold < 1e-6
                metrics.settling_time = t(end);
            else
                last_unsettled_index = find(gain_deriv_norm > threshold, 1, 'last');
                if isempty(last_unsettled_index)
                    metrics.settling_time = t(2); 
                else
                    metrics.settling_time = t(last_unsettled_index + 1);
                end
            end
        else
            metrics.settling_time = t(end);
        end
        % Metric 3: Response Overshoot (Stability)
        max_tilt_x = max(abs(x_state(2:end, 1))); % theta_x
        max_tilt_y = max(abs(x_state(2:end, 5))); % theta_y
        metrics.overshoot = max(max_tilt_x, max_tilt_y);
        
        % Metric 4: Control Input (Energy)
        metrics.control_effort = trapz(t, sum(tau_input.^2, 2));

        %% --- MODIFIED --- %%
        % Metric 5: Saturation Penalty (Integral of Over-Threshold)
        torque_threshold = 20.0; % [Nm] Your motor limit
        
        % Find all torques over the threshold
        abs_tau = abs(tau_input);
        violations = max(0, abs_tau - torque_threshold);
        
        % Square the violations (to heavily penalize large ones)
        violations_sq = violations.^2;
        
        % Sum the penalties from all 3 motors
        summed_violations = sum(violations_sq, 2);
        
        % Integrate the penalty over time
        metrics.saturation_penalty = trapz(t, summed_violations);
        
        
        % --- 4. Calculate Final Cost ---
        if isempty(baseline_metrics)
            % This is the baseline run. Cost is not used.
            cost = 0;
        else
            % This is an optimization run. Normalize and weigh the metrics.
            
            %% --- MODIFIED --- %%
            % Normalize new metric
            % Use 1e-6 to prevent divide-by-zero if baseline had no violations
            norm_saturation = metrics.saturation_penalty / max(baseline_metrics.saturation_penalty, 1e-6); 
            
            % Normalize existing metrics
            norm_itae = metrics.itae / max(baseline_metrics.itae, 1e-6);
            norm_settling = metrics.settling_time / max(baseline_metrics.settling_time, 1e-6);
            norm_overshoot = metrics.overshoot / max(baseline_metrics.overshoot, 1e-6);
            norm_control = metrics.control_effort / max(baseline_metrics.control_effort, 1e-6);
            
            % Weighted sum of normalized metrics
            cost = weights.itae * norm_itae + ...
                   weights.settling_time * norm_settling + ...
                   weights.overshoot * norm_overshoot + ...
                   weights.control_effort * norm_control + ...
                   weights.saturation_penalty * norm_saturation;
        end
        
        if isnan(cost) || isinf(cost)
            cost = 1e10; 
        end
    catch ME
        fprintf('Simulation failed for params [%.2f, %.2f, %.2f]: %s\n', ...
                alpha, beta, gamma, ME.message);
        cost = 1e10; 
    end
end