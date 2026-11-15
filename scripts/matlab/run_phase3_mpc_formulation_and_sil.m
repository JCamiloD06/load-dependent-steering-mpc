%% =========================================================================
%% FASE 3: FORMULACIÓN DEL CONTROLADOR MPC CONSCIENTE DEL ACTUADOR
%%         Y VALIDACIÓN EN SIMULACIÓN EN LAZO CERRADO (SIL BENCHMARK)
%%
%% Proyecto: Experimental Characterization of Load-Dependent Steering
%%           Capability and Its Integration into Predictive Path Tracking Control
%% Objetivo Editorial: Vehicle System Dynamics (Taylor & Francis), Q1
%% Phase 3: Capacity-Aware Augmented QP MPC Formulation and SIL Benchmark
%% =========================================================================
clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  FASE 3: FORMULACIÓN MPC CONSCIENTE DEL ACTUADOR Y BENCHMARK SIL\n');
fprintf('====================================================================\n\n');

%% 1. Cargar Parámetros Físicos Validados de Fases 1 y 2
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
repoRoot = fileparts(fileparts(scriptDir));
if ~exist(fullfile(repoRoot, 'results'), 'dir'), repoRoot = pwd; end

p1Path = fullfile(repoRoot, 'results', 'phase1_actuator_params.mat');
p2Path = fullfile(repoRoot, 'results', 'phase2_capacity_envelope_constraints.mat');

if ~exist(p1Path, 'file') || ~exist(p2Path, 'file')
    error('Faltan archivos de Fase 1 o Fase 2. Ejecutar primero las fases previas.');
end

dataP1 = load(p1Path);
dataP2 = load(p2Path);

p1 = dataP1.actuatorParams;
p2 = dataP2.phase2Constraints;

% Parámetros identificados del actuador ODrive
J_act = p1.J;                           % [kg*m^2] Inercia efectiva (0.0131)
b_act = p1.b;                           % [N*m*s/rad] Amortiguamiento (0.0028)
tau_c = p1.tau_c;                       % [N*m] Fricción Coulomb (0.268)
T_d = p1.mean_Td_ms / 1000.0;           % [s] Retardo puro (0.0641 s)
Omega_0 = deg2rad(p1.Omega_max_noload_deg_s); % [rad/s] Velocidad en vacío (8.07 rad/s)
alpha_max = deg2rad(p1.alpha_max_deg_s2);     % [rad/s^2] Aceleración máxima (65.41 rad/s^2)
bw_act_rad = 2 * pi * p1.bw_small_signal_Hz;  % [rad/s] Ancho de banda (36.07 rad/s)
zeta_act = 0.85;                        % Coeficiente de amortiguamiento

% Parámetros de la envolvente de capacidad (Fase 2)
tau_stall = p2.tau_stall_Nm;            % [N*m] Par de calado térmico (2.45 Nm)
gamma_act = p2.gamma;                   % Exponente no lineal ajustado (4.50)

% Función de envolvente de capacidad dinámica (sin piso artificial, calado en tau_stall)
omega_max_envelope = @(tau) Omega_0 * max(0.0, (1.0 - (min(tau_stall, abs(tau)) / tau_stall).^gamma_act));

fprintf('[PASO 1] Parámetros validados cargados exitosamente:\n');
fprintf('  -> Inercia J:              %.4f kg*m^2\n', J_act);
fprintf('  -> Retardo puro T_d:       %.2f ms (N_d = %d pasos a 50 Hz)\n', T_d * 1000, round(T_d / 0.02));
fprintf('  -> Velocidad en vacío:     %.1f deg/s (%.2f rad/s)\n', rad2deg(Omega_0), Omega_0);
fprintf('  -> Par de Calado tau_s:    %.2f N*m (gamma = %.2f)\n\n', tau_stall, gamma_act);

%% 2. Parámetros del Vehículo y Espacio de Estados Aumentado
m_veh = 1450.0;            % [kg] Masa del vehículo
Iz_veh = 2500.0;           % [kg*m^2] Inercia de guiñada
Lf = 1.15;                 % [m] Distancia CG a eje delantero
Lr = 1.45;                 % [m] Distancia CG a eje trasero
L = Lf + Lr;               % [m] Batalla (2.60 m)
i_s = 15.0;                % Relación de desmultiplicación (volante a rueda)
K_ay = 0.30;               % [N*m / (m/s^2)] Coeficiente de par transferido
v_nom_kmh = 80.0;          % [km/h] Velocidad nominal del ensayo
v = v_nom_kmh / 3.6;       % [m/s] (~22.22 m/s)

% Tiempo de muestreo del controlador predictivo (50 Hz en SIL)
Ts = 0.02;                 % [s]
N_delay = round(T_d / Ts); % N_d = 3 pasos

% Modelo continuo aumentado: x = [e_y; e_psi; delta_act; omega_act]
A_c = [
    0,   v,              0,             0;
    0,   0,       v / (L * i_s),        0;
    0,   0,              0,             1;
    0,   0,    -bw_act_rad^2,  -2*zeta_act*bw_act_rad
];

B_c = [
    0;
    0;
    0;
    bw_act_rad^2
];

% Discretización exacta ZOH via Van Loan
n_states = 4;
n_inputs = 1;
M_vanloan = expm([A_c, B_c; zeros(n_inputs, n_states + n_inputs)] * Ts);
A_d = M_vanloan(1:n_states, 1:n_states);
B_d = M_vanloan(1:n_states, (n_states + 1):end);

%% 3. Formulación del Problema QP Convexo
Np = 15;      % Horizonte de predicción (15 pasos = 0.30 s)
Nc = 15;      % Horizonte de control

% Ponderaciones óptimas
Q_state = diag([3.5, 18.0, 0.05, 0.01]);
R_du = 0.40;

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

%% 4. Trayectoria de Maniobra Crítica (ISO 3888-2 Doble Cambio de Carril)
% Adhesión física en el límite vehicular: ay_max ~ 0.80 g (L_trans = 35 m)
L_trans = 35.0;
t_sim = 0:Ts:8.5;
N_sim = length(t_sim);
x_ref = v * t_sim;
y_ref = zeros(size(x_ref));

for k = 1:N_sim
    xk = x_ref(k);
    if xk >= 30.0 && xk < 30.0 + L_trans
        s = (xk - 30.0) / L_trans;
        y_ref(k) = 3.5 * (10*s^3 - 15*s^4 + 6*s^5);
    elseif xk >= 30.0 + L_trans && xk < 30.0 + L_trans + 25.0
        y_ref(k) = 3.5;
    elseif xk >= 30.0 + L_trans + 25.0 && xk < 30.0 + 2*L_trans + 25.0
        s = (xk - (30.0 + L_trans + 25.0)) / L_trans;
        y_ref(k) = 3.5 * (1 - (10*s^3 - 15*s^4 + 6*s^5));
    else
        y_ref(k) = 0.0;
    end
end

dy = gradient(y_ref, x_ref);
ddy = gradient(dy, x_ref);
psi_ref = atan(dy);
kappa_ref = ddy ./ (1 + dy.^2).^1.5;

%% 5. Bucle de Simulación SIL con Cuatro Controladores
controllers = {'Proposed_MPC', 'Actuator_MPC_Fixed_Bounds', 'Conventional_MPC', 'Pure_Pursuit'};
simResults = struct();

fprintf('[PASO 4] Iniciando simulación en lazo cerrado SIL (ISO 3888 @ 80 km/h)...\n');

for c_idx = 1:length(controllers)
    cname = controllers{c_idx};
    fprintf('  -> Ejecutando controlador: %-26s...', cname);
    
    cmd_buffer = zeros(max(1, N_delay), 1);
    X_real = x_ref(1);
    Y_real = y_ref(1);
    psi_real = psi_ref(1);
    delta_act = 0.0;
    omega_act = 0.0;
    u_prev = 0.0;
    
    X_hist = zeros(N_sim, 1);
    Y_hist = zeros(N_sim, 1);
    psi_hist = zeros(N_sim, 1);
    ey_hist = zeros(N_sim, 1);
    epsi_hist = zeros(N_sim, 1);
    delta_act_hist = zeros(N_sim, 1);
    omega_act_hist = zeros(N_sim, 1);
    delta_cmd_hist = zeros(N_sim, 1);
    tau_load_hist = zeros(N_sim, 1);
    omega_bound_hist = zeros(N_sim, 1);
    calc_time_ms = zeros(N_sim, 1);
    sat_count = 0;
    
    for k = 1:N_sim
        t_start = tic;
        
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
        
        a_y_real = (v^2) * abs(kappak_ref);
        tau_load = min(2.40, K_ay * a_y_real);
        omega_dyn_limit = omega_max_envelope(tau_load);
        
        switch cname
            case 'Proposed_MPC'
                x_curr = [e_y; e_psi; delta_act; omega_act];
                x_pred = x_curr;
                for d = 1:N_delay
                    x_pred = A_d * x_pred + B_d * cmd_buffer(d);
                end
                
                du_max = omega_dyn_limit * Ts;
                A_iq = [eye(Nc); -eye(Nc)];
                b_iq = [du_max * ones(Nc, 1); du_max * ones(Nc, 1)];
                
                f_qp = 2 * Gamma' * Q_bar * (Phi * x_pred + Upsilon * u_prev);
                R_chol = chol(H_qp);
                invH_A_t = R_chol \ (R_chol' \ A_iq');
                invH_f = R_chol \ (R_chol' \ f_qp);
                P = A_iq * invH_A_t; d_h = b_iq + A_iq * invH_f;
                lambda = zeros(length(b_iq), 1); diag_P = max(diag(P), 1e-12);
                for iter = 1:60
                    lam_old = lambda;
                    for i = 1:length(b_iq)
                        w = d_h(i) + P(i, :) * lambda;
                        lambda(i) = max(0.0, lambda(i) - w / diag_P(i));
                    end
                    if norm(lambda - lam_old, inf) < 1e-4, break; end
                end
                du = -invH_f - invH_A_t * lambda;
                delta_cmd = u_prev + du(1);
                omega_bound_eval = omega_dyn_limit;
                
            case 'Actuator_MPC_Fixed_Bounds'
                x_curr = [e_y; e_psi; delta_act; omega_act];
                x_pred = x_curr;
                for d = 1:N_delay
                    x_pred = A_d * x_pred + B_d * cmd_buffer(d);
                end
                
                du_max = Omega_0 * Ts;
                A_iq = [eye(Nc); -eye(Nc)];
                b_iq = [du_max * ones(Nc, 1); du_max * ones(Nc, 1)];
                
                f_qp = 2 * Gamma' * Q_bar * (Phi * x_pred + Upsilon * u_prev);
                R_chol = chol(H_qp);
                invH_A_t = R_chol \ (R_chol' \ A_iq');
                invH_f = R_chol \ (R_chol' \ f_qp);
                P = A_iq * invH_A_t; d_h = b_iq + A_iq * invH_f;
                lambda = zeros(length(b_iq), 1); diag_P = max(diag(P), 1e-12);
                for iter = 1:60
                    lam_old = lambda;
                    for i = 1:length(b_iq)
                        w = d_h(i) + P(i, :) * lambda;
                        lambda(i) = max(0.0, lambda(i) - w / diag_P(i));
                    end
                    if norm(lambda - lam_old, inf) < 1e-4, break; end
                end
                du = -invH_f - invH_A_t * lambda;
                delta_cmd = u_prev + du(1);
                omega_bound_eval = Omega_0;
                
            case 'Conventional_MPC'
                x_curr = [e_y; e_psi; delta_act; omega_act];
                x_pred = x_curr; % Ciego al retardo
                
                du_max = deg2rad(400.0) * Ts;
                A_iq = [eye(Nc); -eye(Nc)];
                b_iq = [du_max * ones(Nc, 1); du_max * ones(Nc, 1)];
                
                f_qp = 2 * Gamma' * Q_bar * (Phi * x_pred + Upsilon * u_prev);
                R_chol = chol(H_qp);
                invH_A_t = R_chol \ (R_chol' \ A_iq');
                invH_f = R_chol \ (R_chol' \ f_qp);
                P = A_iq * invH_A_t; d_h = b_iq + A_iq * invH_f;
                lambda = zeros(length(b_iq), 1); diag_P = max(diag(P), 1e-12);
                for iter = 1:60
                    lam_old = lambda;
                    for i = 1:length(b_iq)
                        w = d_h(i) + P(i, :) * lambda;
                        lambda(i) = max(0.0, lambda(i) - w / diag_P(i));
                    end
                    if norm(lambda - lam_old, inf) < 1e-4, break; end
                end
                du = -invH_f - invH_A_t * lambda;
                delta_cmd = u_prev + du(1);
                omega_bound_eval = deg2rad(400.0);
                
            case 'Pure_Pursuit'
                Ld = 8.0 + 0.30 * v;
                tx = X_real + Ld * cos(psi_real);
                ty = Y_real + Ld * sin(psi_real);
                [~, look_i] = min((x_ref - tx).^2 + (y_ref - ty).^2);
                alpha_pp = atan2(y_ref(look_i) - Y_real, x_ref(look_i) - X_real) - psi_real;
                delta_wheel = atan2(2 * L * sin(alpha_pp), Ld);
                delta_cmd = delta_wheel * i_s;
                omega_bound_eval = Omega_0;
        end
        
        calc_time_ms(k) = toc(t_start) * 1000.0;
        u_prev = delta_cmd;
        
        % Canal de retardo físico FIFO
        cmd_buffer = [cmd_buffer(2:end); delta_cmd];
        u_delayed = cmd_buffer(1);
        
        % Dinámica física del actuador bajo FOC a 8 kHz
        Kp_act = J_act * bw_act_rad^2;
        Kd_act = 2 * zeta_act * bw_act_rad * J_act;
        tau_em = Kp_act * (u_delayed - delta_act) - Kd_act * omega_act;
        d_omega = (tau_em - b_act * omega_act) / J_act;
        d_omega = max(-alpha_max, min(alpha_max, d_omega));
        
        omega_act = omega_act + d_omega * Ts;
        if abs(omega_act) >= omega_dyn_limit * 0.98
            sat_count = sat_count + 1;
        end
        omega_act = max(-omega_dyn_limit, min(omega_dyn_limit, omega_act));
        delta_act = delta_act + omega_act * Ts;
        
        % Cinemática vehicular
        delta_f = delta_act / i_s;
        psi_real = psi_real + (v / L) * tan(delta_f) * Ts;
        X_real = X_real + v * cos(psi_real) * Ts;
        Y_real = Y_real + v * sin(psi_real) * Ts;
        
        X_hist(k) = X_real;
        Y_hist(k) = Y_real;
        psi_hist(k) = psi_real;
        ey_hist(k) = e_y;
        epsi_hist(k) = e_psi;
        delta_act_hist(k) = delta_act;
        omega_act_hist(k) = omega_act;
        delta_cmd_hist(k) = delta_cmd;
        tau_load_hist(k) = tau_load;
        omega_bound_hist(k) = omega_bound_eval;
    end
    
    rms_ey = sqrt(mean(ey_hist.^2));
    max_ey = max(abs(ey_hist));
    pct_sat = (sat_count / N_sim) * 100.0;
    max_omega = max(abs(rad2deg(omega_act_hist)));
    mean_tcalc = mean(calc_time_ms);
    max_tcalc = max(calc_time_ms);
    
    simResults.(cname).X = X_hist;
    simResults.(cname).Y = Y_hist;
    simResults.(cname).psi = psi_hist;
    simResults.(cname).ey = ey_hist;
    simResults.(cname).epsi = epsi_hist;
    simResults.(cname).delta_act_deg = rad2deg(delta_act_hist);
    simResults.(cname).omega_act_dps = rad2deg(omega_act_hist);
    simResults.(cname).delta_cmd_deg = rad2deg(delta_cmd_hist);
    simResults.(cname).tau_load = tau_load_hist;
    simResults.(cname).omega_bound_dps = rad2deg(omega_bound_hist);
    simResults.(cname).calc_time_ms = calc_time_ms;
    simResults.(cname).rms_ey = rms_ey;
    simResults.(cname).max_ey = max_ey;
    simResults.(cname).pct_sat = pct_sat;
    simResults.(cname).sat_steps = sat_count;
    simResults.(cname).max_omega_dps = max_omega;
    simResults.(cname).mean_tcalc = mean_tcalc;
    simResults.(cname).max_tcalc = max_tcalc;
    
    fprintf('Completado! RMS: %5.2f cm | Max: %5.2f cm | Sat: %4.1f%% | Max Omega: %5.1f deg/s | T_calc: %4.2f ms\n', ...
        rms_ey * 100, max_ey * 100, pct_sat, max_omega, mean_tcalc);
end

%% 6. Generación de Figuras Científicas (Fase 3)
figDir = fullfile(repoRoot, 'figures', 'phase3');
if ~exist(figDir, 'dir'), mkdir(figDir); end

% FIGURA 1: Comparación Espacial de Trayectorias
fprintf('\n[PASO 5] Generando Figura 1: Trayectoria en Doble Cambio de Carril...\n');
fig1 = figure('Name', 'Phase3_Fig1_DLC_Trajectory', 'Color', 'w', 'Position', [100, 100, 950, 480]);

plot(x_ref, y_ref, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Referencia ISO 3888-2'); hold on;
plot(simResults.Proposed_MPC.X, simResults.Proposed_MPC.Y, 'b-', 'LineWidth', 2.2, ...
    'DisplayName', sprintf('Propuesto: Actuator-Aware (RMS = %.1f cm)', simResults.Proposed_MPC.rms_ey * 100));
plot(simResults.Actuator_MPC_Fixed_Bounds.X, simResults.Actuator_MPC_Fixed_Bounds.Y, 'm--', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Baseline 3: Cotas Fijas \\Omega_0 (RMS = %.1f cm)', simResults.Actuator_MPC_Fixed_Bounds.rms_ey * 100));
plot(simResults.Conventional_MPC.X, simResults.Conventional_MPC.Y, 'r-.', 'LineWidth', 1.8, ...
    'DisplayName', sprintf('Baseline 1: MPC Convencional Ciego (RMS = %.1f cm)', simResults.Conventional_MPC.rms_ey * 100));
plot(simResults.Pure_Pursuit.X, simResults.Pure_Pursuit.Y, 'g:', 'LineWidth', 2.0, ...
    'DisplayName', sprintf('Baseline 2: Pure Pursuit (RMS = %.1f cm)', simResults.Pure_Pursuit.rms_ey * 100));

yline(3.5 + 1.75, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(3.5 - 1.75, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(1.75, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-1.75, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');

xlabel('Posición Longitudinal X [m]', 'FontWeight', 'bold');
ylabel('Desplazamiento Lateral Y [m]', 'FontWeight', 'bold');
title('Comparación de Seguimiento de Trayectoria en Maniobra Severa ISO 3888-2 @ 80 km/h', ...
    'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 9);
grid on; xlim([0, max(x_ref)]); ylim([-1.5, 5.5]);

fig1Path = fullfile(figDir, 'phase3_fig1_dlc_trajectory_comparison.png');
exportgraphics(fig1, fig1Path, 'Resolution', 300);
fprintf('  -> Guardado: %s\n', fig1Path);

% FIGURA 2: Dinámica de Giro y Respeto de la Envolvente
fprintf('\n[PASO 6] Generando Figura 2: Dinámica de Giro y Respeto de Envolvente...\n');
fig2 = figure('Name', 'Phase3_Fig2_Steering_Dynamics', 'Color', 'w', 'Position', [120, 120, 950, 620]);

subplot(2, 1, 1);
plot(t_sim, simResults.Proposed_MPC.delta_act_deg, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Propuesto (Actuator-Aware)'); hold on;
plot(t_sim, simResults.Actuator_MPC_Fixed_Bounds.delta_act_deg, 'm--', 'LineWidth', 1.8, 'DisplayName', 'Baseline 3: Cotas Fijas');
plot(t_sim, simResults.Conventional_MPC.delta_act_deg, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Baseline 1: Convencional Ciego');
plot(t_sim, simResults.Pure_Pursuit.delta_act_deg, 'g:', 'LineWidth', 1.6, 'DisplayName', 'Baseline 2: Pure Pursuit');
ylabel('Ángulo de Volante \delta [deg]', 'FontWeight', 'bold');
title('Dinámica de Dirección y Saturación de Capacidad durante la Maniobra', 'FontSize', 11);
legend('Location', 'northeast', 'FontSize', 8.5);
grid on; xlim([0, max(t_sim)]);

subplot(2, 1, 2);
omega_dyn_plot = arrayfun(@(tau) rad2deg(omega_max_envelope(tau)), simResults.Proposed_MPC.tau_load);
plot(t_sim, abs(simResults.Proposed_MPC.omega_act_dps), 'b-', 'LineWidth', 2.0, 'DisplayName', 'Tasa Real (Propuesto)'); hold on;
plot(t_sim, abs(simResults.Actuator_MPC_Fixed_Bounds.omega_act_dps), 'm--', 'LineWidth', 1.8, 'DisplayName', 'Tasa Real (Cotas Fijas)');
plot(t_sim, abs(simResults.Conventional_MPC.omega_act_dps), 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Tasa Real (Convencional)');
plot(t_sim, omega_dyn_plot, 'k-', 'LineWidth', 2.2, 'DisplayName', 'Envolvente Dinámica \Omega_{max}(\tau_{load})');
yline(rad2deg(Omega_0), 'm--', 'LineWidth', 1.5, 'DisplayName', 'Cota Fija Catálogo \Omega_0 (462.5 deg/s)');

ylabel('Velocidad Angular |\omega| [deg/s]', 'FontWeight', 'bold');
xlabel('Tiempo [s]', 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 8.5);
grid on; xlim([0, max(t_sim)]); ylim([0, 500]);

fig2Path = fullfile(figDir, 'phase3_fig2_steering_dynamics_and_envelope_violation.png');
exportgraphics(fig2, fig2Path, 'Resolution', 300);
fprintf('  -> Guardado: %s\n', fig2Path);

% FIGURA 3: Errores de Seguimiento y Tiempo de Cómputo
fprintf('\n[PASO 7] Generando Figura 3: Errores de Seguimiento y Tiempo de Cómputo...\n');
fig3 = figure('Name', 'Phase3_Fig3_Tracking_Errors_Compute_Time', 'Color', 'w', 'Position', [140, 140, 950, 560]);

subplot(2, 2, [1, 2]);
plot(t_sim, simResults.Proposed_MPC.ey, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Propuesto: Actuator-Aware'); hold on;
plot(t_sim, simResults.Actuator_MPC_Fixed_Bounds.ey, 'm--', 'LineWidth', 1.8, 'DisplayName', 'Baseline 3: Cotas Fijas');
plot(t_sim, simResults.Conventional_MPC.ey, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Baseline 1: Convencional');
plot(t_sim, simResults.Pure_Pursuit.ey, 'g:', 'LineWidth', 1.6, 'DisplayName', 'Baseline 2: Pure Pursuit');
ylabel('Error Lateral e_y [m]', 'FontWeight', 'bold');
title('Evolución Temporal del Error Lateral', 'FontSize', 11);
legend('Location', 'southeast', 'FontSize', 8.5);
grid on; xlim([0, max(t_sim)]);

subplot(2, 2, 3);
histogram(simResults.Proposed_MPC.calc_time_ms, 20, 'FaceColor', [0.12, 0.47, 0.71], 'EdgeColor', 'w');
xline(20.0, 'r--', 'LineWidth', 2.0, 'DisplayName', 'Límite Tiempo Real (50 Hz = 20 ms)');
xlabel('Tiempo de Cálculo por Ciclo [ms]', 'FontWeight', 'bold');
ylabel('Frecuencia (Ciclos)', 'FontWeight', 'bold');
title(sprintf('Viabilidad Tiempo Real (Media = %.2f ms)', simResults.Proposed_MPC.mean_tcalc), 'FontSize', 10);
grid on;

subplot(2, 2, 4);
metric_data = [
    simResults.Proposed_MPC.rms_ey * 100, simResults.Proposed_MPC.max_ey * 100;
    simResults.Actuator_MPC_Fixed_Bounds.rms_ey * 100, simResults.Actuator_MPC_Fixed_Bounds.max_ey * 100;
    simResults.Conventional_MPC.rms_ey * 100, simResults.Conventional_MPC.max_ey * 100;
    simResults.Pure_Pursuit.rms_ey * 100, simResults.Pure_Pursuit.max_ey * 100
];
b_bar = bar(metric_data);
b_bar(1).FaceColor = [0.2, 0.4, 0.8];
b_bar(2).FaceColor = [0.8, 0.3, 0.3];
set(gca, 'XTickLabel', {'Propuesto', 'Act. Fijo', 'Convencional', 'Pure Pursuit'});
ylabel('Error Lateral [cm]', 'FontWeight', 'bold');
legend({'RMS e_y [cm]', 'Max |e_y| [cm]'}, 'Location', 'northeast', 'FontSize', 8.5);
title('Resumen Cuantitativo de Desempeño', 'FontSize', 10);
grid on;

fig3Path = fullfile(figDir, 'phase3_fig3_tracking_errors_and_compute_time.png');
exportgraphics(fig3, fig3Path, 'Resolution', 300);
fprintf('  -> Guardado: %s\n', fig3Path);

%% 7. Consolidación y Exportación
resPath = fullfile(repoRoot, 'results', 'phase3_mpc_sil_results.mat');
save(resPath, 'simResults', 'A_d', 'B_d', 'H_qp', 'Q_state', 'R_du');
fprintf('\n[PASO 8] Resultados de Fase 3 exportados exitosamente a: %s\n', resPath);

fprintf('\n====================================================================\n');
fprintf('  FASE 3 FINALIZADA CON ÉXITO Y TOTAL TRAZABILIDAD\n');
fprintf('====================================================================\n');
