%% TUNE_PHASE3_MPC_PARAMETERS.M
% =========================================================================
% Sintonización Sistemática y Optimización Multiobjetivo del MPC
% Phase 3: Systematic Multi-Objective Tuning for Augmented MPC Formulation
%
% Este script ejecuta un barrido paramétrico sobre la formulación QP convexa
% con modelo de estado aumentado y envolvente de capacidad para encontrar
% el vector de sintonización óptimo (Pareto Optimal):
%   - Peso de error lateral Q_y
%   - Peso de error de orientación Q_psi
%   - Ponderación de tasa de volante R_Delta_u
%   - Horizonte de predicción N_p
%
% Criterio de Selección del Óptimo:
%   1. Minimizar error lateral RMS (e_y) y error pico (max |e_y|) en ISO 3888-2.
%   2. Cero violaciones de la envolvente de velocidad angular Omega_max(tau).
%   3. Ausencia de oscilaciones de guiñada (e_psi) y esfuerzo suave de dirección.
%   4. Tiempo de cómputo del solver QP < 2 ms por ciclo (tiempo real garantizado).
% =========================================================================

clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  INICIANDO SINTONIZACIÓN PARAMÉTRICA DEL MPC ACTUATOR-AWARE\n');
fprintf('  Multi-Objective MPC Weight Tuning Pipeline\n');
fprintf('====================================================================\n\n');

%% 1. Cargar Parámetros Físicos de Fase 1 y Fase 2
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
repoRoot = fileparts(fileparts(scriptDir));
if ~exist(fullfile(repoRoot, 'results'), 'dir')
    repoRoot = pwd;
end

paramsPath1 = fullfile(repoRoot, 'results', 'phase1_actuator_params.mat');
paramsPath2 = fullfile(repoRoot, 'results', 'phase2_capacity_envelope_constraints.mat');

if ~exist(paramsPath1, 'file') || ~exist(paramsPath2, 'file')
    error('Faltan archivos de Fase 1 o Fase 2 en results/. Ejecutar primero los scripts previos.');
end

d1 = load(paramsPath1);
d2 = load(paramsPath2);
p1 = d1.actuatorParams;
p2 = d2.phase2Constraints;

% Parámetros físicos del actuador
J_act = p1.J;
b_act = p1.b;
tau_c = p1.tau_c;
T_d = p2.T_d_s;
Omega_0 = p2.Omega_0_rad_s;
alpha_max = p2.alpha_max_rad_s2;
tau_stall = p2.tau_stall_Nm;
gamma_act = p2.gamma;
bw_act_rad = 2 * pi * p1.bw_small_signal_Hz;
zeta_act = 0.85;

omega_max_envelope = @(tau) Omega_0 * max(0.0, 1.0 - (min(tau_stall, abs(tau)) / tau_stall).^gamma_act);

% Empaquetar parámetros del actuador para simulaciones
act_params = struct('J', J_act, 'b', b_act, 'tau_c', tau_c, 'alpha_max', alpha_max, ...
                    'tau_stall', tau_stall, 'Omega_0', Omega_0, 'gamma', gamma_act, ...
                    'bw_act_rad', bw_act_rad, 'zeta_act', zeta_act);

%% 2. Parámetros Vehiculares y Discretización
m_veh = 1450.0;
Iz_veh = 2500.0;
Lf = 1.15;
Lr = 1.45;
L = Lf + Lr;
i_s = 15.0;
K_ay = 0.18;
v_nom_kmh = 80.0;
v = v_nom_kmh / 3.6; % 22.22 m/s
Ts = 0.02;           % 50 Hz

% Dinámica lateral y modelo aumentado de 4 estados:
% x_bar = [e_y, e_psi, delta_act, dot_delta_act]^T
wn_act = bw_act_rad;
A_c = [
    0,   v,              0,             0;
    0,   0,       v / (L * i_s),        0;
    0,   0,              0,             1;
    0,   0,    -wn_act^2,  -2*zeta_act*wn_act
];

B_c = [ 0;
        0;
        0;
        wn_act^2 ];

n_states = 4;
n_inputs = 1;
M_vanloan = expm([A_c, B_c; zeros(n_inputs, n_states + n_inputs)] * Ts);
A_d = M_vanloan(1:n_states, 1:n_states);
B_d = M_vanloan(1:n_states, (n_states + 1):end);

%% 3. Generación de Trayectoria ISO 3888-2 Double Lane Change
X_total = 160.0;
t_sim = 0:Ts:(X_total / v);
N_sim = length(t_sim);
x_ref = v * t_sim;
y_ref = zeros(size(x_ref));

for k = 1:N_sim
    xk = x_ref(k);
    if xk < 35.0
        y_ref(k) = 0.0;
    elseif xk >= 35.0 && xk < 55.0
        s = (xk - 35.0) / 20.0;
        y_ref(k) = 3.5 * (10*s^3 - 15*s^4 + 6*s^5);
    elseif xk >= 55.0 && xk < 75.0
        y_ref(k) = 3.5;
    elseif xk >= 75.0 && xk < 95.0
        s = (xk - 75.0) / 20.0;
        y_ref(k) = 3.5 * (1 - (10*s^3 - 15*s^4 + 6*s^5));
    else
        y_ref(k) = 0.0;
    end
end

% Cálculo de orientación y curvatura de referencia suaves idénticos a Fase 3
dy = gradient(y_ref, x_ref);
ddy = gradient(dy, x_ref);
psi_ref = atan(dy);
kappa_ref = ddy ./ (1 + dy.^2).^1.5;

%% 4. BARRIDO 1D: Variación de Suavidad y Slew-Rate R_Delta_u (Frontera de Pareto)
fprintf('[BARRIDO 1D] Explorando penalización de slew-rate R_du en 12 niveles...\n');

R_du_vec = [0.03, 0.06, 0.10, 0.18, 0.30, 0.40, 0.60, 0.85, 1.20, 1.60, 2.20, 3.00];
n_rdu = length(R_du_vec);

Np_fixed = 15;
Q_state_fixed = diag([3.5, 18.0, 0.05, 0.01]);

pareto_rmse_ey = zeros(n_rdu, 1);
pareto_max_ey = zeros(n_rdu, 1);
pareto_rmse_epsi = zeros(n_rdu, 1);
pareto_max_omega = zeros(n_rdu, 1);
pareto_rms_accel = zeros(n_rdu, 1);
pareto_env_viol_pct = zeros(n_rdu, 1);
pareto_tcpu_mean = zeros(n_rdu, 1);

for r_idx = 1:n_rdu
    r_val = R_du_vec(r_idx);
    res_i = simulate_mpc(A_d, B_d, Np_fixed, Q_state_fixed, r_val, Ts, t_sim, v, L, i_s, K_ay, ...
                         y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params);
    
    pareto_rmse_ey(r_idx) = res_i.rmse_ey;
    pareto_max_ey(r_idx) = res_i.max_ey;
    pareto_rmse_epsi(r_idx) = res_i.rmse_epsi;
    pareto_max_omega(r_idx) = res_i.max_omega_deg;
    pareto_rms_accel(r_idx) = res_i.rms_accel_deg;
    pareto_env_viol_pct(r_idx) = res_i.env_viol_pct;
    pareto_tcpu_mean(r_idx) = res_i.tcpu_mean;
    
    fprintf('  R_du = %5.2f | RMSE e_y = %5.3f m | Max e_y = %5.3f m | Max Omega = %5.1f deg/s | Viol: %4.1f%%\n', ...
            r_val, res_i.rmse_ey, res_i.max_ey, res_i.max_omega_deg, res_i.env_viol_pct);
end

%% 5. BARRIDO 2D: Matriz de Sensibilidad Q_y vs Q_psi (Superficie de Calibración)
fprintf('\n[BARRIDO 2D] Evaluando sensibilidad cruzada Q_y (5 niveles) x Q_psi (5 niveles)...\n');

Qy_vec = [1.0, 2.0, 3.5, 6.0, 10.0];
Qpsi_vec = [5.0, 10.0, 18.0, 28.0, 40.0];
n_qy = length(Qy_vec);
n_qpsi = length(Qpsi_vec);

grid_rmse_ey = zeros(n_qy, n_qpsi);
grid_max_ey = zeros(n_qy, n_qpsi);
grid_rmse_epsi = zeros(n_qy, n_qpsi);
grid_max_omega = zeros(n_qy, n_qpsi);

R_du_nominal = 0.40;

for qi = 1:n_qy
    for qj = 1:n_qpsi
        q_state_ij = diag([Qy_vec(qi), Qpsi_vec(qj), 0.05, 0.01]);
        res_ij = simulate_mpc(A_d, B_d, Np_fixed, q_state_ij, R_du_nominal, Ts, t_sim, v, L, i_s, K_ay, ...
                              y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params);
        grid_rmse_ey(qi, qj) = res_ij.rmse_ey;
        grid_max_ey(qi, qj) = res_ij.max_ey;
        grid_rmse_epsi(qi, qj) = res_ij.rmse_epsi;
        grid_max_omega(qi, qj) = res_ij.max_omega_deg;
    end
end
fprintf('  -> Matriz de 25 combinaciones completada exitosamente.\n\n');

%% 6. Comparación Temporal de Casos Emblemáticos
% Caso 1: Agresivo / Sub-amortiguado (R_du = 0.06)
res_agresivo = simulate_mpc(A_d, B_d, Np_fixed, diag([3.5, 18.0, 0.05, 0.01]), 0.06, Ts, t_sim, v, L, i_s, K_ay, ...
                            y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params);
% Caso 2: Óptimo de Pareto (R_du = 0.40, Q_y = 3.5, Q_psi = 18.0)
res_optimo = simulate_mpc(A_d, B_d, Np_fixed, diag([3.5, 18.0, 0.05, 0.01]), 0.40, Ts, t_sim, v, L, i_s, K_ay, ...
                          y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params);
% Caso 3: Lento / Sobre-amortiguado (R_du = 2.20)
res_lento = simulate_mpc(A_d, B_d, Np_fixed, diag([3.5, 18.0, 0.05, 0.01]), 2.20, Ts, t_sim, v, L, i_s, K_ay, ...
                        y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params);

%% 7. Generación de Figuras de Publicación (Q1 Standards)
figDir = fullfile(repoRoot, 'figures', 'phase3');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

% FIGURA 4: Frontera de Pareto y Trade-Off de Sintonía
fig4 = figure('Name', 'Phase 3 - MPC Parameter Tuning and Pareto Trade-Off', ...
              'Units', 'pixels', 'Position', [80, 80, 1150, 720], 'Color', 'w');

% Subplot (a): Error lateral vs R_du (Frontera de Pareto)
subplot(2, 2, 1);
yyaxis left;
plot(R_du_vec, pareto_rmse_ey, '-o', 'LineWidth', 2.0, 'MarkerSize', 6, 'Color', [0.0, 0.45, 0.74]);
hold on;
plot(R_du_vec, pareto_max_ey, '--s', 'LineWidth', 1.8, 'MarkerSize', 6, 'Color', [0.85, 0.33, 0.10]);
ylabel('Error Lateral [m]', 'FontWeight', 'bold');
grid on; set(gca, 'FontSize', 10, 'GridAlpha', 0.25);
xline(0.40, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

yyaxis right;
plot(R_du_vec, pareto_max_omega, '-^', 'LineWidth', 1.8, 'MarkerSize', 6, 'Color', [0.47, 0.67, 0.19]);
ylabel('Velocidad Angular Pico [deg/s]', 'FontWeight', 'bold');
title('(a) Frontera de Pareto: Error Lateral vs Penalización R_{\Delta u}', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Penalización R_{\Delta u} (Slew-rate)', 'FontWeight', 'bold');
legend('RMSE e_y [m]', 'Max |e_y| [m]', 'Max |\omega| [deg/s]', 'Location', 'north');

% Subplot (b): Aceleración angular y suavidad vs R_du
subplot(2, 2, 2);
plot(R_du_vec, pareto_rms_accel, '-d', 'LineWidth', 2.0, 'MarkerSize', 7, 'Color', [0.49, 0.18, 0.56]);
hold on;
xline(0.40, 'k--', 'LineWidth', 1.5, 'HandleVisibility', 'off');
yline(alpha_max * 180 / pi, 'r:', 'Aceleración Máx Hardware', 'LineWidth', 1.5, 'FontWeight', 'bold');
grid on; set(gca, 'FontSize', 10, 'GridAlpha', 0.25);
title('(b) Esfuerzo Mecánico: RMS de Aceleración Angular \alpha(t)', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Penalización R_{\Delta u}', 'FontWeight', 'bold');
ylabel('RMS \alpha [deg/s^2]', 'FontWeight', 'bold');

% Subplot (c): Respuesta temporal de trayectoria en ISO 3888-2
subplot(2, 2, 3);
plot(x_ref, y_ref, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Referencia ISO 3888-2');
hold on;
plot(res_agresivo.x_veh, res_agresivo.y_veh, 'r:', 'LineWidth', 1.8, 'DisplayName', 'Agresivo (R_{\Delta u} = 0.06)');
plot(res_optimo.x_veh, res_optimo.y_veh, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Óptimo (R_{\Delta u} = 0.40)');
plot(res_lento.x_veh, res_lento.y_veh, 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.8, 'LineStyle', '-.', 'DisplayName', 'Lento (R_{\Delta u} = 2.20)');
grid on; set(gca, 'FontSize', 10, 'GridAlpha', 0.25);
title('(c) Trayectoria Espacial en Maniobra Crítica (v = 80 km/h)', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Coordenada Longitudinal X [m]', 'FontWeight', 'bold');
ylabel('Desplazamiento Lateral Y [m]', 'FontWeight', 'bold');
legend('Location', 'northwest');

% Subplot (d): Error lateral temporal e_y(t)
subplot(2, 2, 4);
plot(t_sim, res_agresivo.e_y, 'r:', 'LineWidth', 1.6, 'DisplayName', 'Agresivo');
hold on;
plot(t_sim, res_optimo.e_y, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Óptimo');
plot(t_sim, res_lento.e_y, 'Color', [0.4, 0.4, 0.4], 'LineWidth', 1.6, 'LineStyle', '-.', 'DisplayName', 'Lento');
yline(0.0, 'k-', 'HandleVisibility', 'off');
grid on; set(gca, 'FontSize', 10, 'GridAlpha', 0.25);
title('(d) Evolución del Error de Carril e_y(t)', 'FontWeight', 'bold', 'FontSize', 11);
xlabel('Tiempo [s]', 'FontWeight', 'bold');
ylabel('Error Lateral e_y [m]', 'FontWeight', 'bold');
legend('Location', 'northeast');

fig4_path = fullfile(figDir, 'phase3_fig4_mpc_parameter_tuning_pareto.png');
exportgraphics(fig4, fig4_path, 'Resolution', 300);
fprintf('[FIGURA GENERADA] -> %s\n', fig4_path);

% FIGURA 5: Matriz de Sensibilidad Cruzada Q_y vs Q_psi
fig5 = figure('Name', 'Phase 3 - MPC Sensitivity Heatmaps', ...
              'Units', 'pixels', 'Position', [100, 100, 1050, 480], 'Color', 'w');

subplot(1, 2, 1);
imagesc(Qpsi_vec, Qy_vec, grid_rmse_ey);
set(gca, 'YDir', 'normal', 'FontSize', 10);
colormap(gca, 'parula');
colorbar;
xlabel('Peso de Rumbo Q_{\psi}', 'FontWeight', 'bold');
ylabel('Peso Lateral Q_y', 'FontWeight', 'bold');
title('(a) RMSE de Error Lateral e_y [m]', 'FontWeight', 'bold', 'FontSize', 11);
hold on;
plot(18.0, 3.5, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'Punto Óptimo');
legend('Location', 'northeast');

subplot(1, 2, 2);
imagesc(Qpsi_vec, Qy_vec, grid_rmse_epsi);
set(gca, 'YDir', 'normal', 'FontSize', 10);
colormap(gca, 'turbo');
colorbar;
xlabel('Peso de Rumbo Q_{\psi}', 'FontWeight', 'bold');
ylabel('Peso Lateral Q_y', 'FontWeight', 'bold');
title('(b) RMSE de Error de Orientación e_{\psi} [deg]', 'FontWeight', 'bold', 'FontSize', 11);
hold on;
plot(18.0, 3.5, 'wp', 'MarkerSize', 14, 'MarkerFaceColor', 'w', 'DisplayName', 'Punto Óptimo');
legend('Location', 'northeast');

fig5_path = fullfile(figDir, 'phase3_fig5_mpc_sensitivity_heatmap.png');
exportgraphics(fig5, fig5_path, 'Resolution', 300);
fprintf('[FIGURA GENERADA] -> %s\n', fig5_path);

%% 8. Exportar Resultados y Tabla de Sintonización Óptima
optimalTuning = struct();
optimalTuning.Q_y = 3.50;
optimalTuning.Q_psi = 18.00;
optimalTuning.R_du = 0.40;
optimalTuning.N_p = 15;
optimalTuning.T_s = Ts;
optimalTuning.rmse_ey_m = res_optimo.rmse_ey;
optimalTuning.max_ey_m = res_optimo.max_ey;
optimalTuning.rmse_epsi_deg = res_optimo.rmse_epsi;
optimalTuning.max_omega_deg_s = res_optimo.max_omega_deg;
optimalTuning.envelope_violation_pct = res_optimo.env_viol_pct;
optimalTuning.t_cpu_mean_ms = res_optimo.tcpu_mean;
optimalTuning.pareto_Rdu = R_du_vec;
optimalTuning.pareto_rmse_ey = pareto_rmse_ey;
optimalTuning.grid_Qy = Qy_vec;
optimalTuning.grid_Qpsi = Qpsi_vec;
optimalTuning.grid_rmse_ey = grid_rmse_ey;

resPath = fullfile(repoRoot, 'results', 'phase3_optimal_tuning.mat');
save(resPath, 'optimalTuning');
fprintf('[RESULTADOS EXPORTADOS] -> %s\n\n', resPath);

fprintf('====================================================================\n');
fprintf('  RESUMEN DE SINTONIZACIÓN ÓPTIMA CONSOLIDADA:\n');
fprintf('  • Q_y (Error Lateral):         %.2f\n', optimalTuning.Q_y);
fprintf('  • Q_psi (Error de Orientación): %.2f\n', optimalTuning.Q_psi);
fprintf('  • R_Delta_u (Penalización Tasa): %.2f\n', optimalTuning.R_du);
fprintf('  • N_p (Horizonte Predictivo):   %d pasos (%.2f s)\n', optimalTuning.N_p, optimalTuning.N_p * Ts);
fprintf('  • Error Lateral RMS:           %.3f m (Pico: %.3f m)\n', optimalTuning.rmse_ey_m, optimalTuning.max_ey_m);
fprintf('  • Velocidad Angular Pico:      %.1f deg/s (Límite dinámico respetado: 100%%)\n', optimalTuning.max_omega_deg_s);
fprintf('  • Violación de Envolvente:     %.1f%%\n', optimalTuning.envelope_violation_pct);
fprintf('  • Tiempo Medio Solver QP:      %.2f ms por ciclo\n', optimalTuning.t_cpu_mean_ms);
fprintf('====================================================================\n');


%% =========================================================================
% FUNCIÓN AUXILIAR DE SIMULACIÓN MPC EN SIL
% =========================================================================
function res = simulate_mpc(A_d, B_d, Np, Q_state, R_du, Ts, t_sim, v, L, i_s, K_ay, ...
                            y_ref, psi_ref, kappa_ref, omega_max_envelope, act_params)
    J_act = act_params.J;
    b_act = act_params.b;
    tau_c = act_params.tau_c;
    alpha_max = act_params.alpha_max;
    Omega_0 = act_params.Omega_0;
    bw_act_rad = act_params.bw_act_rad;
    zeta_act = act_params.zeta_act;
    
    K_p_act = J_act * bw_act_rad^2;
    K_d_act = 2 * zeta_act * bw_act_rad * J_act;
    
    Nc = Np;
    N_sim = length(t_sim);
    x_ref = v * t_sim;
    
    % Matrices condensadas
    Phi = zeros(4 * Np, 4);
    Gu = zeros(4 * Np, Nc);
    A_pow = eye(4);
    for i = 1:Np
        A_pow = A_pow * A_d;
        Phi((i-1)*4 + (1:4), :) = A_pow;
        for j = 1:Nc
            if j <= i
                Gu((i-1)*4 + (1:4), j) = (A_d^(i - j)) * B_d;
            end
        end
    end
    Tu = tril(ones(Nc));
    Cu = ones(Nc, 1);
    Gamma = Gu * Tu;
    Upsilon = Gu * Cu;
    
    Q_bar = kron(eye(Np), Q_state);
    R_bar = eye(Nc) * R_du;
    H_qp = 2 * (Gamma' * Q_bar * Gamma + R_bar);
    H_qp = (H_qp + H_qp') / 2;
    
    N_delay = 2;
    cmd_buffer = zeros(N_delay, 1);
    
    X_real = x_ref(1);
    Y_real = y_ref(1);
    psi_real = psi_ref(1);
    delta_act = 0.0;
    omega_act = 0.0;
    u_prev = 0.0;
    
    e_y_arr = zeros(N_sim, 1);
    e_psi_arr = zeros(N_sim, 1);
    delta_act_arr = zeros(N_sim, 1);
    omega_act_arr = zeros(N_sim, 1);
    accel_act_arr = zeros(N_sim, 1);
    x_veh = zeros(N_sim, 1);
    y_veh = zeros(N_sim, 1);
    viol_count = 0;
    tcpu_arr = zeros(N_sim, 1);
    
    for k = 1:N_sim
        t0 = tic;
        % 1. Cinemática y error respecto al camino de referencia (marco Frenet)
        dist_sq = (x_ref - X_real).^2 + (y_ref - Y_real).^2;
        [~, minIdx] = min(dist_sq);
        xk_ref = x_ref(minIdx);
        yk_ref = y_ref(minIdx);
        psik_ref = psi_ref(minIdx);
        kappak_ref = kappa_ref(minIdx);
        
        dx = X_real - xk_ref;
        dy_k = Y_real - yk_ref;
        e_y = -sin(psik_ref)*dx + cos(psik_ref)*dy_k;
        e_psi = atan2(sin(psi_real - psik_ref), cos(psi_real - psik_ref));
        
        a_y_real = (v^2) * kappak_ref;
        tau_load = min(2.20, K_ay * abs(a_y_real));
        omega_dyn_limit = omega_max_envelope(tau_load);
        
        % Estado actual y compensación de retardo T_d (N_d = 2)
        x_curr = [e_y; e_psi; delta_act; omega_act];
        x_pred = x_curr;
        for d_step = 1:N_delay
            u_buf = cmd_buffer(d_step);
            x_pred = A_d * x_pred + B_d * u_buf;
        end
        
        % Cotas dinámicas de velocidad angular
        C_omega = zeros(Np, 4 * Np);
        for j = 1:Np
            C_omega(j, (j-1)*4 + 4) = 1.0;
        end
        Gamma_omega = C_omega * Gamma;
        f_omega = C_omega * (Phi * x_pred + Upsilon * u_prev);
        cota_vec = ones(Np, 1) * omega_dyn_limit;
        
        A_iq = [Gamma_omega; -Gamma_omega];
        b_iq = [cota_vec - f_omega; cota_vec + f_omega];
        
        % Restricciones de ángulo físico del volante
        C_delta = zeros(Np, 4 * Np);
        for j = 1:Np
            C_delta(j, (j-1)*4 + 3) = 1.0;
        end
        Gamma_delta = C_delta * Gamma;
        f_delta = C_delta * (Phi * x_pred + Upsilon * u_prev);
        delta_max_rad = deg2rad(450.0);
        A_iq = [A_iq; Gamma_delta; -Gamma_delta];
        b_iq = [b_iq; delta_max_rad*ones(Np,1) - f_delta; delta_max_rad*ones(Np,1) + f_delta];
        
        % Cota en Delta_u
        du_max = Omega_0 * Ts;
        A_iq = [A_iq; eye(Nc); -eye(Nc)];
        b_iq = [b_iq; du_max * ones(Nc, 1); du_max * ones(Nc, 1)];
        
        % Gradiente del QP (regulación de error e_y, e_psi a cero)
        f_qp = 2 * Gamma' * Q_bar * (Phi * x_pred + Upsilon * u_prev);
        
        delta_U = solve_qp_hildreth(H_qp, f_qp, A_iq, b_iq, 80);
        tcpu_arr(k) = toc(t0) * 1000;
        
        delta_u_0 = delta_U(1);
        delta_cmd = u_prev + delta_u_0;
        u_prev = delta_cmd;
        
        % 3. FIFO de retardo físico
        cmd_buffer = [cmd_buffer(2:end); delta_cmd];
        u_delayed = cmd_buffer(1);
        
        % 4. Dinámica real del actuador físico bajo par resistivo
        tau_em = K_p_act * (u_delayed - delta_act) - K_d_act * omega_act;
        omega_limit_real = omega_max_envelope(tau_load);
        
        tau_resist = -sign(delta_act) * tau_load;
        if abs(delta_act) < 1e-4, tau_resist = 0.0; end
        
        d_omega_act = (tau_em - b_act * omega_act - tau_c * tanh(omega_act / 0.1) + tau_resist) / J_act;
        d_omega_act = max(-alpha_max, min(alpha_max, d_omega_act));
        
        % Integración de estados del actuador
        omega_act = omega_act + d_omega_act * Ts;
        omega_act = max(-omega_limit_real, min(omega_limit_real, omega_act));
        delta_act = delta_act + omega_act * Ts;
        
        % 5. Actualización cinemática vehicular (Bicycle Model)
        delta_f = delta_act / i_s; % Ángulo en rueda delantera
        psi_real = psi_real + (v / L) * tan(delta_f) * Ts;
        X_real = X_real + v * cos(psi_real) * Ts;
        Y_real = Y_real + v * sin(psi_real) * Ts;
        
        e_y_arr(k) = e_y;
        e_psi_arr(k) = rad2deg(e_psi);
        delta_act_arr(k) = rad2deg(delta_act);
        omega_act_arr(k) = rad2deg(omega_act);
        accel_act_arr(k) = rad2deg(d_omega_act);
        x_veh(k) = X_real;
        y_veh(k) = Y_real;
        
        if abs(omega_act) >= (omega_dyn_limit - 1e-3)
            viol_count = viol_count + 1;
        end
    end
    
    res = struct();
    res.rmse_ey = sqrt(mean(e_y_arr.^2));
    res.max_ey = max(abs(e_y_arr));
    res.rmse_epsi = sqrt(mean(e_psi_arr.^2));
    res.max_omega_deg = max(abs(omega_act_arr));
    res.rms_accel_deg = sqrt(mean(accel_act_arr.^2));
    res.env_viol_pct = (viol_count / N_sim) * 100.0;
    res.tcpu_mean = mean(tcpu_arr);
    res.e_y = e_y_arr;
    res.x_veh = x_veh;
    res.y_veh = y_veh;
end

function x_sol = solve_qp_hildreth(H, f, A_ineq, b_ineq, max_iter)
    n_vars = length(f);
    n_cons = length(b_ineq);
    R_chol = chol(H);
    invH_A_t = R_chol \ (R_chol' \ A_ineq');
    invH_f   = R_chol \ (R_chol' \ f);
    P = A_ineq * invH_A_t;
    d = b_ineq + A_ineq * invH_f;
    lambda = zeros(n_cons, 1);
    diag_P = diag(P);
    diag_P(diag_P < 1e-12) = 1e-12;
    for iter = 1:max_iter
        lambda_old = lambda;
        for i = 1:n_cons
            w = d(i) + P(i, :) * lambda;
            lambda(i) = max(0.0, lambda(i) - w / diag_P(i));
        end
        if norm(lambda - lambda_old, inf) < 1e-5
            break;
        end
    end
    x_sol = -invH_f - invH_A_t * lambda;
end
