%% generate_phase1_figures_en.m
% Regenerates Phase 1 bench identification figures (Figs 2, 3, and 4) in crisp
% English at 600 DPI directly from verified raw experimental logs and saved
% Standalone MATLAB script for Phase 1 identification figures (600 DPI).
%
% Outputs:
%   figures/final/Fig2_Actuator_Inertia_Friction.png
%   figures/final/Fig3_Time_Delay_Distribution.png
%   figures/final/Fig4_Torque_Speed_Envelope.png

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(fileparts(scriptDir));
rawRoot   = fullfile(repoRoot, 'data', 'raw', 'bench_phase1');
matPath   = fullfile(repoRoot, 'results', 'phase1_actuator_params.mat');
outDir    = fullfile(repoRoot, 'figures', 'final');
if ~exist(outDir, 'dir'), mkdir(outDir); end

load(matPath, 'actuatorParams');

%% ========================================================================
%  FIGURE 2: TEST 1 - INERTIA & COULOMB/VISCOUS FRICTION IDENTIFICATION
%  ========================================================================
t1Files = dir(fullfile(rawRoot, 'test1_inertia_friction', 't1_step_*.csv'));
all_tau = []; all_alpha = []; all_omega = []; all_rep = [];
for k = 1:numel(t1Files)
    T = readtable(fullfile(t1Files(k).folder, t1Files(k).name));
    t = T.time_s;
    tau_m = T.torque_meas_Nm;
    vel_rad = deg2rad(T.vel_meas_deg_s);
    if numel(vel_rad) >= 15
        vel_filt = smoothdata(vel_rad, 'gaussian', 7);
    else
        vel_filt = vel_rad;
    end
    dt = mean(diff(t)); if dt <= 0, dt = 0.0083; end
    alpha = gradient(vel_filt, dt);
    mask = (t >= 0.08 & t <= 0.85 & (abs(alpha) > 1.5 | abs(vel_filt) > 0.15));
    if any(mask)
        all_tau   = [all_tau; tau_m(mask)];
        all_alpha = [all_alpha; alpha(mask)];
        all_omega = [all_omega; vel_filt(mask)];
        all_rep   = [all_rep; T.rep_id(mask)];
    end
end

X_reg = [all_alpha, all_omega, sign(all_omega)];
idxTrain = (all_rep <= 3);
idxVal   = (all_rep >= 4);
beta = (X_reg(idxTrain, :)' * X_reg(idxTrain, :)) \ (X_reg(idxTrain, :)' * all_tau(idxTrain));
J_est = beta(1); b_est = beta(2); tc_est = beta(3);
y_pred_val = X_reg(idxVal, :) * beta;
y_true_val = all_tau(idxVal);
R2_val = 1 - sum((y_true_val - y_pred_val).^2) / sum((y_true_val - mean(y_true_val)).^2);

fig2 = figure('Units', 'pixels', 'Position', [60 60 1200 490], 'Color', 'w');
set(fig2, 'PaperPositionMode', 'auto');

subplot(1, 2, 1); hold on; grid on; box on;
scatter(all_alpha, all_tau, 14, [0.18 0.48 0.75], 'filled', 'MarkerFaceAlpha', 0.25, ...
    'DisplayName', 'Bench step telemetry (N = 30 runs)');
a_grid = linspace(min(all_alpha)*0.95, max(all_alpha)*0.95, 120);
plot(a_grid, J_est * a_grid, 'r-', 'LineWidth', 2.4, ...
    'DisplayName', sprintf('Inertial slope J \\alpha (J = %.4f kg\\cdotm^2)', J_est));
plot(a_grid, J_est * a_grid + tc_est, 'k--', 'LineWidth', 1.1, ...
    'DisplayName', sprintf('\\pm\\tau_c Coulomb band (\\pm%.3f N\\cdotm)', tc_est));
plot(a_grid, J_est * a_grid - tc_est, 'k--', 'LineWidth', 1.1, 'HandleVisibility', 'off');
xlabel('Angular Acceleration \alpha [rad/s^2]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Electromagnetic Torque \tau_m [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title(sprintf('(a) Effective Rotor Inertia (Out-of-sample Val R^2 = %.3f)', R2_val), ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8.5);
xlim([-100 105]); ylim([-1.9 2.05]);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

subplot(1, 2, 2); hold on; grid on; box on;
tau_fric = all_tau - J_est * all_alpha;
scatter(all_omega, tau_fric, 14, [0.50 0.24 0.65], 'filled', 'MarkerFaceAlpha', 0.24, ...
    'DisplayName', 'Inertial residual (\tau_m - J \alpha)');
w_grid = linspace(min(all_omega)*0.95, max(all_omega)*0.95, 250);
tau_f_model = b_est * w_grid + tc_est * tanh(w_grid / 0.08);
plot(w_grid, tau_f_model, 'k--', 'LineWidth', 2.3, ...
    'DisplayName', sprintf('Coulomb + Viscous Fit (\\tau_c=%.3f N\\cdotm, b=%.4f)', tc_est, b_est));
xlabel('Angular Velocity \omega [rad/s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Friction Torque (\tau_m - J \alpha) [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title(sprintf('(b) Dry & Viscous Friction: \\tau_c = %.3f N\\cdotm, b = %.4f N\\cdotm\\cdots/rad', tc_est, b_est), ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8.5);
xlim([-9.2 9.2]); ylim([-0.95 0.85]);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig2, fullfile(outDir, 'Fig2_Actuator_Inertia_Friction.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig2 (J=%.5f, b=%.5f, tc=%.5f, R2=%.3f)\n', J_est, b_est, tc_est, R2_val);

%% ========================================================================
%  FIGURE 3: TEST 2 - END-TO-END HIL TRANSPORT DELAY DISTRIBUTION
%  ========================================================================
t2Files = dir(fullfile(rawRoot, 'test2_time_delay', 't2_pulse_*.csv'));
delays_ms = [];
sample_found = false;
sample_t = []; sample_cmd = []; sample_meas = []; sample_tc = 0; sample_tr = 0;

for k = 1:numel(t2Files)
    T = readtable(fullfile(t2Files(k).folder, t2Files(k).name));
    t = T.time_s; p_cmd = T.pos_cmd_deg; p_meas = T.pos_meas_deg;
    idx_cmd = find(abs(p_cmd) >= 2.0, 1, 'first');
    if isempty(idx_cmd), continue; end
    t_cmd = t(idx_cmd);
    dir_s = sign(p_cmd(idx_cmd));
    idx_resp = find(t >= t_cmd & dir_s * (p_meas - p_meas(idx_cmd)) >= 0.20, 1, 'first');
    if ~isempty(idx_resp)
        t_resp = t(idx_resp);
        td_k = (t_resp - t_cmd) * 1000.0;
        if td_k > 0 && td_k < 95.0
            delays_ms(end+1, 1) = td_k; %#ok<AGROW>
            if contains(t2Files(k).name, '+15deg_rep01') && ~sample_found
                sample_found = true;
                sample_t = t; sample_cmd = p_cmd; sample_meas = p_meas;
                sample_tc = t_cmd; sample_tr = t_resp;
            end
        end
    end
end
mean_Td = mean(delays_ms); std_Td = std(delays_ms);
ci95_Td = 1.96 * std_Td / sqrt(numel(delays_ms));

fig3 = figure('Units', 'pixels', 'Position', [60 60 1200 490], 'Color', 'w');
set(fig3, 'PaperPositionMode', 'auto');

subplot(1, 2, 1); hold on; grid on; box on;
td_ex = (sample_tr - sample_tc) * 1000.0;
fill([sample_tc sample_tr sample_tr sample_tc], [-3.5 -3.5 18.0 18.0], ...
    [0.92 0.92 0.94], 'EdgeColor', 'none', 'FaceAlpha', 0.85, ...
    'DisplayName', 'Pure transport delay window T_d');
plot(sample_t, sample_cmd, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Step command \theta_{cmd}');
plot(sample_t, sample_meas, 'b-', 'LineWidth', 2.4, 'DisplayName', 'Measured encoder response \theta_{meas}');
plot([sample_tc sample_tr], [2.8 2.8], 'r-', 'LineWidth', 2.2, 'HandleVisibility', 'off');
text(sample_tr + 0.008, 4.1, sprintf('T_{d,min} = %.1f ms', td_ex), ...
    'HorizontalAlignment', 'left', 'FontName', 'Arial', 'FontSize', 9.2, ...
    'FontWeight', 'bold', 'Color', [0.75 0.08 0.08], ...
    'BackgroundColor', [1 1 1 0.85], 'Margin', 1.5);
xlim([0.05 0.275]); ylim([-3.5 18.0]);
xlabel('Time [s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Steering Angle \theta [deg]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) Fast Step-Command Delay Detection (+15\circ Pulse)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8.5);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

subplot(1, 2, 2); hold on; grid on; box on;
h_b = histogram(delays_ms, 12, 'FaceColor', [0.85 0.37 0.02], 'EdgeColor', 'k', ...
    'FaceAlpha', 0.78, 'DisplayName', 'Measured trials');
x_g = linspace(24, 96, 200);
pdf_g = (1.0 / (std_Td * sqrt(2*pi))) * exp(-0.5 * ((x_g - mean_Td) / std_Td).^2) * numel(delays_ms) * h_b.BinWidth;
plot(x_g, pdf_g, 'k-', 'LineWidth', 2.2, 'DisplayName', 'Gaussian fit');
xline(mean_Td, 'r--', 'LineWidth', 2.2, 'DisplayName', sprintf('Mean \\mu = %.1f ms', mean_Td));
ci_lo = mean_Td - ci95_Td; ci_hi = mean_Td + ci95_Td;
fill([ci_lo ci_hi ci_hi ci_lo], [0 0 10.4 10.4], [1.0 0.25 0.25], ...
    'FaceAlpha', 0.14, 'EdgeColor', [0.85 0.15 0.15], 'LineStyle', ':', ...
    'LineWidth', 1.2, 'DisplayName', sprintf('95%% CI [%.1f, %.1f] ms', ci_lo, ci_hi));
text(25.0, 9.65, sprintf(['HIL Chain: USB + Thread + FOC\n' ...
    '\\mu = %.2f ms | \\sigma = %.2f ms\n' ...
    '95%% CI: [%.2f, %.2f] ms\n' ...
    'Valid: N = %d / %d trials'], mean_Td, std_Td, ci_lo, ci_hi, numel(delays_ms), numel(t2Files)), ...
    'VerticalAlignment', 'top', 'HorizontalAlignment', 'left', ...
    'FontName', 'Arial', 'FontSize', 7.8, 'BackgroundColor', [0.99 0.99 0.99], ...
    'EdgeColor', [0.60 0.60 0.65], 'Margin', 3.5);
xlim([24 98]); ylim([0 10.4]);
xlabel('Pure Transport Delay T_d [ms]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Trial Count', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title(sprintf('(b) HIL Chain Latency Distribution (N = %d Valid Steps)', numel(delays_ms)), ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 8.3);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig3, fullfile(outDir, 'Fig3_Time_Delay_Distribution.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig3 (N=%d/%d, mean=%.2f ms, std=%.2f ms)\n', numel(delays_ms), numel(t2Files), mean_Td, std_Td);

%% ========================================================================
%  FIGURE 4: STATIC TORQUE CALIBRATION & DYNAMIC CAPACITY ENVELOPE
%  ========================================================================
Tcal = readtable(fullfile(rawRoot, 'calibration_finalff_torque.csv'), 'CommentStyle', '#');
p_fit = polyfit(Tcal.tau_dem_Nm, Tcal.tau_meas_Nm, 1);
tau_fit = polyval(p_fit, Tcal.tau_dem_Nm);
R2_cal = 1 - sum((Tcal.tau_meas_Nm - tau_fit).^2) / sum((Tcal.tau_meas_Nm - mean(Tcal.tau_meas_Nm)).^2);
rmse_cal = sqrt(mean((Tcal.tau_meas_Nm - tau_fit).^2));
V_ohmic_stat = abs(Tcal.iq_meas_A) * 0.534;

% Exact Test 5 transient sweep statistics per load level (mean +- SD over 5 reps)
load_lvls = actuatorParams.load_levels_Nm(:);
t5_tags = {'0.0', '0.5', '1.0', '1.5', '2.0'};
w_mean_t5 = zeros(size(load_lvls)); w_std_t5 = zeros(size(load_lvls));
iq_mean_t5 = zeros(size(load_lvls));
for i = 1:numel(t5_tags)
    f5 = dir(fullfile(rawRoot, 'test5_torque_speed_load', sprintf('t5_load_%sNm_rep*.csv', t5_tags{i})));
    vp = zeros(numel(f5), 1); ip = zeros(numel(f5), 1);
    for r = 1:numel(f5)
        Tr = readtable(fullfile(f5(r).folder, f5(r).name));
        vp(r) = max(abs(smoothdata(Tr.vel_meas_deg_s, 'gaussian', 5)));
        ip(r) = max(abs(Tr.iq_meas_A));
    end
    w_mean_t5(i) = mean(vp); w_std_t5(i) = std(vp); iq_mean_t5(i) = mean(ip);
end

fig4 = figure('Units', 'pixels', 'Position', [60 60 1240 500], 'Color', 'w');
set(fig4, 'PaperPositionMode', 'auto');

% Panel (a): Static Torque Calibration & Ohmic Drop
subplot(1, 2, 1); hold on; grid on; box on;
yyaxis left;
scatter(Tcal.tau_dem_Nm, Tcal.tau_meas_Nm, 52, [0.05 0.35 0.72], 'filled', ...
    'DisplayName', 'Measured torque K_t I_q');
t_line = linspace(-2.1, 2.1, 100);
plot(t_line, polyval(p_fit, t_line), 'b-', 'LineWidth', 2.0, ...
    'DisplayName', sprintf('Linear fit (slope=%.3f, R^2=%.4f)', p_fit(1), R2_cal));
ylabel('Measured Torque \tau_{em} = K_t I_q [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylim([-2.35 2.35]);
axL = gca; axL.YColor = [0.05 0.35 0.72];

yyaxis right;
plot(Tcal.tau_dem_Nm, abs(Tcal.iq_meas_A), 'r^--', 'LineWidth', 1.6, 'MarkerSize', 6, ...
    'MarkerFaceColor', [0.85 0.20 0.15], 'DisplayName', 'Measured |I_q| [A]');
ylabel('Quadrature Current |I_q| [A]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylim([0 4.5]);
axL.YColor = [0.78 0.15 0.12];

xlabel('Demanded FFB Load \tau_{dem} [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) Static FOC Torque Calibration (N = 11 Levels, V_{bus} = 24.3 V)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
text(0.0, 4.30, sprintf(['K_t = 0.550 N\\cdotm/A (k_{cal} = %.3f N\\cdotm/unit)\n' ...
    'RMSE = %.3f N\\cdotm | R^2 = %.4f\n' ...
    'Static ohmic drop |I_q|R \\leq %.2f V\n' ...
    'Dynamic transient peak: %.2f A (%.2f V)'], ...
    p_fit(1)*2.0, rmse_cal, R2_cal, max(V_ohmic_stat), max(iq_mean_t5), max(iq_mean_t5)*0.534), ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
    'FontName', 'Arial', 'FontSize', 8.1, 'BackgroundColor', [0.99 0.99 0.99], ...
    'EdgeColor', [0.60 0.60 0.65], 'Margin', 4);
xlim([-2.15 2.15]);
legend('Location', 'southeast', 'FontSize', 8.2);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

% Panel (b): Dynamic Capability Envelope E_act(tau_load)
subplot(1, 2, 2); hold on; grid on; box on;
fill([0 1.0 1.0 0], [0 0 500 500], [0.90 0.95 1.0], 'EdgeColor', 'none', 'FaceAlpha', 0.75, ...
    'DisplayName', 'Closed-loop HIL FFB band (|\tau_{load}| \leq 1.0 N\cdotm)');

tau_grid = linspace(0, 2.45, 250);
w0_env = actuatorParams.Omega_max_noload_deg_s;
env_45 = w0_env * max(0, 1 - (tau_grid / 2.45).^4.5);
env_18 = w0_env * max(0, 1 - (tau_grid / 2.45).^1.8);
env_10 = w0_env * max(0, 1 - (tau_grid / 2.45).^1.0);

area(tau_grid, env_45, 'FaceColor', [0.22 0.55 0.85], 'FaceAlpha', 0.14, 'EdgeColor', 'none', ...
    'HandleVisibility', 'off');
plot(tau_grid, env_45, 'b-', 'LineWidth', 2.4, ...
    'DisplayName', 'MPC voltage-plateau envelope (\gamma = 4.5, \tau_{stall} = 2.45 N\cdotm)');
plot(tau_grid, env_18, '-.', 'Color', [0.10 0.55 0.25], 'LineWidth', 1.8, ...
    'DisplayName', 'Smooth back-EMF rolloff (\gamma = 1.8, \Omega_0 = 462.5\circ/s)');
plot(tau_grid, env_10, ':', 'Color', [0.45 0.45 0.48], 'LineWidth', 1.4, ...
    'DisplayName', 'Linear derating (\gamma = 1.0)');

plot(0, actuatorParams.Omega_max_noload_deg_s, 'p', 'MarkerSize', 12, ...
    'MarkerFaceColor', [0.95 0.72 0.10], 'MarkerEdgeColor', 'k', ...
    'DisplayName', sprintf('Test 3 no-load saturation (\\Omega_0 = %.1f\\circ/s)', actuatorParams.Omega_max_noload_deg_s));
errorbar(load_lvls, w_mean_t5, w_std_t5, 'o', 'Color', [0.06 0.28 0.58], ...
    'MarkerFaceColor', [0.20 0.52 0.86], 'MarkerSize', 7, 'LineWidth', 1.5, ...
    'DisplayName', 'Test 5 100\circ transient sweep (inertial-limited)');

xlim([-0.08 2.60]); ylim([0 510]);
xlabel('Load Torque Magnitude |\tau_{load}| [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Admissible Steering Rate \Omega_{max} [deg/s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Rate-Load Capability Envelope E_{act}(\tau_{load})', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 7.8);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig4, fullfile(outDir, 'Fig4_Torque_Speed_Envelope.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig4 (R2_cal=%.5f, Omega0=%.1f deg/s)\n', R2_cal, actuatorParams.Omega_max_noload_deg_s);
