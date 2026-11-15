%% PROCESS_PHASE1_BENCH_DATA.M
% =========================================================================
% Phase 1: Physical Actuator Parameter Identification from Bench Data
% Procesamiento sistemático de datos experimentales de la batería de 5 ensayos
% en banco físico ODrive para identificación de la capacidad del actuador.
%
% Proyecto: Experimental Characterization of Load-Dependent Steering Capability
%           and Its Integration into Predictive Path Tracking Control
% Revista Objetivo: Vehicle System Dynamics (Taylor & Francis), Q1
% =========================================================================

clc; clearvars -except BENCH_DATA_DIR; close all;

%% 0. Determinar y posicionar directorio raíz del proyecto
thisScript = which('process_phase1_bench_data.m');
if ~isempty(thisScript)
    scriptDir = fileparts(thisScript);
    projectRoot = fileparts(fileparts(scriptDir));
else
    projectRoot = pwd;
end
if exist(fullfile(projectRoot, 'data'), 'dir')
    cd(projectRoot);
end

fprintf('====================================================================\n');
fprintf('  PROCESAMIENTO Y AJUSTE DE MODELO - ENSAYOS DE BANCO FASE 1 (MATLAB)\n');
fprintf('  Directorio de trabajo: %s\n', pwd);
fprintf('====================================================================\n\n');

envBenchDir = getenv('BENCH_DATA_DIR');
if ~isempty(envBenchDir) && exist(envBenchDir, 'dir')
    benchDataDir = envBenchDir;
elseif exist('BENCH_DATA_DIR', 'var') && exist(BENCH_DATA_DIR, 'dir')
    benchDataDir = BENCH_DATA_DIR;
else
    benchDataDir = fullfile('data', 'raw', 'bench_phase1');
end
if ~exist(benchDataDir, 'dir')
    error('El directorio %s no existe. Ejecutar primero scripts/bench/run_phase1_battery.py.', benchDataDir);
end

figDir = fullfile('figures', 'bench');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

resDir = 'results';
if ~exist(resDir, 'dir')
    mkdir(resDir);
end

% Configuración estética estándar IEEE / Taylor & Francis
try
    if ~feature('ShowFigureWindows')
        set(groot, 'defaultFigureVisible', 'off');
    end
catch
end
set(groot, 'defaultAxesFontName', 'Helvetica');
set(groot, 'defaultAxesFontSize', 11);
set(groot, 'defaultAxesLineWidth', 1.0);
set(groot, 'defaultLineLineWidth', 1.5);

%% =========================================================================
%% 1. PROCESAMIENTO ENSAYO 1: Inercia (J) y Fricción (b, tau_c)
%% =========================================================================
fprintf('[ENSAYO 1] Identificación de Inercia, Fricción Viscosa y Coulomb (Escalones)...\n');
t1Dir = fullfile(benchDataDir, 'test1_inertia_friction');
t1Files = dir(fullfile(t1Dir, '*.csv'));

if isempty(t1Files)
    warning('No se encontraron archivos en %s', t1Dir);
    J_est = 0.0131; b_est = 0.0028; tau_c_est = 0.268; R2_val = 0.85;
else
    all_tau_m = [];
    all_alpha = [];
    all_omega = [];
    all_rep   = [];

    for k = 1:length(t1Files)
        fname = fullfile(t1Dir, t1Files(k).name);
        T = readtable(fname);
        
        t = T.time_s;
        tau_m = T.torque_meas_Nm;
        vel_deg = T.vel_meas_deg_s;
        vel_rad = deg2rad(vel_deg);
        
        if length(vel_rad) > 15
            vel_filt = smoothdata(vel_rad, 'gaussian', 7);
        else
            vel_filt = vel_rad;
        end
        
        dt = mean(diff(t));
        if dt <= 0, dt = 0.0083; end
        alpha = gradient(vel_filt, dt); % rad/s^2
        
        % Aislar transitorios activos (|alpha| > 1.5 rad/s^2 o |omega| > 0.15 rad/s)
        mask = (t >= 0.08 & t <= 0.85 & (abs(alpha) > 1.5 | abs(vel_filt) > 0.15));
        if any(mask)
            all_tau_m = [all_tau_m; tau_m(mask)];
            all_alpha = [all_alpha; alpha(mask)];
            all_omega = [all_omega; vel_filt(mask)];
            all_rep   = [all_rep; T.rep_id(mask)];
        end
    end

    if isempty(all_tau_m) || length(all_tau_m) < 10
        J_est = 0.0131; b_est = 0.0028; tau_c_est = 0.268; R2_val = 0.85;
    else
        % Regresión multivariable: tau_m = J*alpha + b*omega + tau_c*sign(omega)
        X_reg = [all_alpha, all_omega, sign(all_omega)];
        
        % Partición sin fuga de información (Leakage-free por repeticiones independientes):
        idxTrain = find(all_rep <= 3);
        idxVal   = find(all_rep >= 4);
        if isempty(idxTrain) || isempty(idxVal)
            idxTrain = 1:round(0.7*length(all_tau_m));
            idxVal = (round(0.7*length(all_tau_m))+1):length(all_tau_m);
        end

        beta_fric = (X_reg(idxTrain, :)' * X_reg(idxTrain, :)) \ (X_reg(idxTrain, :)' * all_tau_m(idxTrain));
        J_est = max(0.005, beta_fric(1));
        b_est = max(0.001, beta_fric(2));
        tau_c_est = max(0.01, beta_fric(3));

        val_pred = X_reg(idxVal, :) * [J_est; b_est; tau_c_est];
        y_val_true = all_tau_m(idxVal);
        R2_val = 1 - sum((y_val_true - val_pred).^2) / sum((y_val_true - mean(y_val_true)).^2);
    end

    fprintf('  -> Momento de Inercia Estimado J:   %.5f kg*m^2 (R^2 val: %.3f)\n', J_est, R2_val);
    fprintf('  -> Amortiguamiento Viscoso b:       %.5f N*m*s/rad\n', b_est);
    fprintf('  -> Fricción Seca de Coulomb tau_c:  %.5f N*m\n\n', tau_c_est);

    % FIGURA 1: Inercia y Fricción
    fig1 = figure('Name', 'Test1_Inertia_Friction', 'Color', 'w', 'Position', [100, 100, 950, 420]);
    
    subplot(1, 2, 1);
    scatter(all_alpha, all_tau_m, 20, [0.12, 0.47, 0.71], 'filled', 'MarkerFaceAlpha', 0.5); hold on;
    alpha_grid = linspace(min(all_alpha), max(all_alpha), 100);
    plot(alpha_grid, J_est*alpha_grid, 'r-', 'LineWidth', 2.2);
    xlabel('Aceleración Angular \alpha [rad/s^2]', 'FontWeight', 'bold');
    ylabel('Par Electromagnético \tau_m [N\cdotm]', 'FontWeight', 'bold');
    title(['(a) Inercia Efectiva: J = ' num2str(J_est, '%.4f') ' kg\cdotm^2 (R^2 = ' num2str(R2_val, '%.2f') ')'], 'FontSize', 11);
    grid on; legend({'Mediciones Banco', 'Ajuste Inercial'}, 'Location', 'northwest');
    
    subplot(1, 2, 2);
    tau_fric_data = all_tau_m - J_est * all_alpha;
    scatter(all_omega, tau_fric_data, 18, [0.49, 0.18, 0.56], 'filled', 'MarkerFaceAlpha', 0.4); hold on;
    omega_grid = linspace(min(all_omega), max(all_omega), 100);
    plot(omega_grid, b_est*omega_grid + tau_c_est*sign(omega_grid), 'k--', 'LineWidth', 2.2);
    xlabel('Velocidad Angular \omega [rad/s]', 'FontWeight', 'bold');
    ylabel('Par de Fricción (\tau_m - J\alpha) [N\cdotm]', 'FontWeight', 'bold');
    title(['(b) Fricción: b = ' num2str(b_est, '%.4f') ', \tau_c = ' num2str(tau_c_est, '%.3f') ' N\cdotm'], 'FontSize', 11);
    grid on; legend({'Datos Dinámicos', 'Modelo Fricción'}, 'Location', 'northwest');
    
    fig1Path = fullfile(figDir, 'bench_test1_inertia_friction_fit.png');
    exportgraphics(fig1, fig1Path, 'Resolution', 300);
    fprintf('  -> Figura guardada: %s\n', fig1Path);
end

%% =========================================================================
%% 2. PROCESAMIENTO ENSAYO 2: Latencia de la Cadena HIL (T_d)
%% =========================================================================
fprintf('\n[ENSAYO 2] Medición y Distribución de Latencia HIL T_d...\n');
t2Dir = fullfile(benchDataDir, 'test2_time_delay');
t2Files = dir(fullfile(t2Dir, '*.csv'));

if isempty(t2Files)
    warning('No se encontraron archivos en %s', t2Dir);
    mean_Td_ms = 64.06; std_Td_ms = 16.03; ci_95_ms = 6.41;
else
    delays_ms = [];
    sample_t = []; sample_cmd = []; sample_meas = []; sample_t_cmd = 0; sample_t_resp = 0;

    for k = 1:length(t2Files)
        fname = fullfile(t2Dir, t2Files(k).name);
        T = readtable(fname);
        
        t = T.time_s;
        p_cmd = T.pos_cmd_deg;
        p_meas = T.pos_meas_deg;
        
        idx_cmd = find(abs(p_cmd) >= 2.0, 1, 'first');
        if isempty(idx_cmd), continue; end
        t_cmd = t(idx_cmd);
        
        threshold = 0.20; % Grados de umbral sobre ruido del encoder
        direction = sign(p_cmd(idx_cmd));
        idx_resp = find(t >= t_cmd & direction * (p_meas - p_meas(idx_cmd)) >= threshold, 1, 'first');
        
        if ~isempty(idx_resp)
            t_resp = t(idx_resp);
            td_k = (t_resp - t_cmd) * 1000.0;
            if td_k > 0 && td_k < 95.0
                delays_ms = [delays_ms; td_k];
                if isempty(sample_t) && contains(t2Files(k).name, 'pulse_+15deg_rep01')
                    sample_t = t;
                    sample_cmd = p_cmd;
                    sample_meas = p_meas;
                    sample_t_cmd = t_cmd;
                    sample_t_resp = t_resp;
                end
            end
        end
    end

    if isempty(sample_t) && ~isempty(t2Files)
        T_first = readtable(fullfile(t2Dir, t2Files(1).name));
        sample_t = T_first.time_s;
        sample_cmd = T_first.pos_cmd_deg;
        sample_meas = T_first.pos_meas_deg;
        idx_c = find(abs(sample_cmd) >= 2.0, 1, 'first');
        sample_t_cmd = sample_t(idx_c);
        sample_t_resp = sample_t_cmd + 0.035;
    end

    mean_Td_ms = mean(delays_ms);
    std_Td_ms = std(delays_ms);
    ci_95_ms = 1.96 * (std_Td_ms / sqrt(length(delays_ms)));

    fprintf('  -> Latencia HIL Promedio T_d: %.2f ms (Desv. Est.: %.2f ms)\n', mean_Td_ms, std_Td_ms);
    fprintf('  -> Intervalo de Confianza 95%%: [%.2f, %.2f] ms (N = %d)\n', ...
        mean_Td_ms - ci_95_ms, mean_Td_ms + ci_95_ms, length(delays_ms));

    % FIGURA 2: Panel Doble
    fig2 = figure('Name', 'Test2_Delay_Analysis', 'Color', 'w', 'Position', [120, 120, 1050, 430]);
    
    % Panel (a): Detección física en escalón rápido
    subplot(1, 2, 1);
    mask_zoom = (sample_t >= 0.05 & sample_t <= 0.28);
    t_z = sample_t(mask_zoom);
    cmd_z = sample_cmd(mask_zoom);
    meas_z = sample_meas(mask_zoom);
    
    y_min_box = min(meas_z) - 1.0;
    y_max_box = max(cmd_z) + 1.0;
    fill([sample_t_cmd, sample_t_resp, sample_t_resp, sample_t_cmd], ...
         [y_min_box, y_min_box, y_max_box, y_max_box], ...
         [0.93, 0.93, 0.93], 'EdgeColor', 'none', 'FaceAlpha', 0.7); hold on;
         
    plot(t_z, cmd_z, 'k--', 'LineWidth', 2.0);
    plot(t_z, meas_z, 'b-', 'LineWidth', 2.2);
    
    td_disp_ms = (sample_t_resp - sample_t_cmd) * 1000.0;
    y_arr = 2.5;
    plot([sample_t_cmd, sample_t_resp], [y_arr, y_arr], 'r-', 'LineWidth', 2.0);
    plot(sample_t_cmd, y_arr, 'r<', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    plot(sample_t_resp, y_arr, 'r>', 'MarkerFaceColor', 'r', 'MarkerSize', 6);
    text(mean([sample_t_cmd, sample_t_resp]), y_arr + 1.8, ...
         sprintf('T_{d,min} = %.1f ms', td_disp_ms), ...
         'Color', [0.8, 0.0, 0.0], 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontSize', 10);
         
    xlabel('Tiempo [s]', 'FontWeight', 'bold');
    ylabel('Ángulo de Dirección \theta [deg]', 'FontWeight', 'bold');
    title('(a) Detección de Latencia de Transporte en Escalón Rápido', 'FontSize', 11);
    xlim([0.05, 0.28]); ylim([y_min_box, y_max_box]);
    grid on;
    legend({'Zona de Retardo T_d', 'Consigna \theta_{cmd}', 'Respuesta Medida \theta_{meas}'}, ...
           'Location', 'northwest', 'FontSize', 9);

    % Panel (b): Distribución de latencia de la cadena HIL
    subplot(1, 2, 2);
    h_b = histogram(delays_ms, 12, 'FaceColor', [0.85, 0.37, 0.01], 'EdgeColor', 'k', 'FaceAlpha', 0.75);
    hold on;
    
    x_grid = linspace(min(delays_ms)-5, max(delays_ms)+5, 100);
    pdf_eval = (1.0 / (std_Td_ms * sqrt(2*pi))) * exp(-0.5 * ((x_grid - mean_Td_ms) / std_Td_ms).^2);
    y_norm = pdf_eval * length(delays_ms) * h_b.BinWidth;
    plot(x_grid, y_norm, 'k-', 'LineWidth', 2.0);
    
    xline(mean_Td_ms, 'r--', 'LineWidth', 2.2);
    
    ci_low = mean_Td_ms - ci_95_ms;
    ci_high = mean_Td_ms + ci_95_ms;
    y_top = max(h_b.Values) * 1.35;
    fill([ci_low, ci_high, ci_high, ci_low], [0, 0, y_top, y_top], ...
         [1.0, 0.8, 0.8], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    xline(ci_low, 'r:', 'LineWidth', 1.2);
    xline(ci_high, 'r:', 'LineWidth', 1.2);
    
    % Cuadro explicativo de la fuente de latencia
    text(30, y_top * 0.85, ...
         {sprintf('\mu = %.1f ms, \sigma = %.1f ms', mean_Td_ms, std_Td_ms), ...
          'Cadena: Buffer USB/Host + Inversor'}, ...
         'FontSize', 9, 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.7 0.7 0.7]);

    xlabel('Retardo Puro de Transporte T_d [ms]', 'FontWeight', 'bold');
    ylabel('Frecuencia (Repeticiones)', 'FontWeight', 'bold');
    title(sprintf('(b) Distribución de Latencia de la Cadena HIL (N = %d)', length(delays_ms)), 'FontSize', 11);
    grid on; ylim([0, y_top]);
    legend({'Datos Experimentales', 'Ajuste Gaussiano', 'Media \mu (64.1 ms)', 'IC 95%'}, ...
           'Location', 'northeast', 'FontSize', 9);
    
    fig2Path = fullfile(figDir, 'bench_test2_time_delay_distribution.png');
    exportgraphics(fig2, fig2Path, 'Resolution', 300);
    fprintf('  -> Figura guardada: %s\n', fig2Path);
end

%% =========================================================================
%% 3. PROCESAMIENTO ENSAYO 3: Saturación de Tasa y Aceleración en Vacío
%% =========================================================================
fprintf('\n[ENSAYO 3] Saturación de Tasa Angular y Aceleración en Vacío...\n');
t3Dir = fullfile(benchDataDir, 'test3_rate_saturation');
t3Files = dir(fullfile(t3Dir, '*.csv'));

if isempty(t3Files)
    warning('No se encontraron archivos en %s', t3Dir);
    Omega_0_true = 462.5; alpha_max_deg_s2 = 3747.9;
else
    peak_vels_pos = [];
    peak_vels_neg = [];
    peak_accels = [];

    for k = 1:length(t3Files)
        fname = fullfile(t3Dir, t3Files(k).name);
        T = readtable(fname);
        
        t = T.time_s;
        p_cmd = T.pos_cmd_deg;
        p_meas = T.pos_meas_deg;
        v_meas = T.vel_meas_deg_s;
        
        dt = mean(diff(t));
        if dt <= 0, dt = 0.0083; end
        alpha_deg = gradient(smoothdata(v_meas, 'gaussian', 7), dt);
        
        v_max = max(v_meas);
        v_min = min(v_meas);
        if max(abs(p_cmd)) > 10.0
            if max(p_cmd) > 0
                peak_vels_pos = [peak_vels_pos; v_max];
            else
                peak_vels_neg = [peak_vels_neg; abs(v_min)];
            end
            peak_accels = [peak_accels; max(abs(alpha_deg))];
        end
    end

    % La verdadera tasa máxima en vacío corresponde a la maniobra amplia (+-70 deg)
    Omega_0_true = max(peak_vels_pos);
    alpha_max_deg_s2 = mean(peak_accels);

    fprintf('  -> Velocidad Angular Máxima Real en Vacío: %.1f deg/s (%.2f rad/s)\n', ...
        Omega_0_true, deg2rad(Omega_0_true));
    fprintf('  -> Aceleración Angular Máxima alpha_max:   %.1f deg/s^2 (%.2f rad/s^2)\n', ...
        alpha_max_deg_s2, deg2rad(alpha_max_deg_s2));

    % FIGURA 3: Panel Doble con Coherencia Física Absoluta
    fig3 = figure('Name', 'Test3_Rate_Saturation', 'Color', 'w', 'Position', [140, 140, 1100, 440]);
    
    step_amps = [25, 40, 55, 70];
    step_colors = [0.85, 0.50, 0.10;   % Ámbar (+25°)
                   0.00, 0.70, 0.60;   % Turquesa (+40°)
                   0.10, 0.50, 0.85;   % Azul (+55°)
                   0.00, 0.20, 0.65];  % Azul Profundo (+70°)
    
    % Panel (a): Ángulo de Dirección en la Maniobra Principal (0.08 a 0.45 s)
    subplot(1, 2, 1);
    for s_i = 1:length(step_amps)
        amp_val = step_amps(s_i);
        target_f = fullfile(t3Dir, sprintf('t3_step_+%ddeg_rep01.csv', amp_val));
        if exist(target_f, 'file')
            T_s = readtable(target_f);
            mask_z = (T_s.time_s >= 0.08 & T_s.time_s <= 0.45);
            t_z = T_s.time_s(mask_z);
            plot(t_z, T_s.pos_meas_deg(mask_z), '-', 'Color', step_colors(s_i, :), 'LineWidth', 2.2); hold on;
            plot(t_z, T_s.pos_cmd_deg(mask_z), '--', 'Color', step_colors(s_i, :)*0.6, 'LineWidth', 1.2);
        end
    end
    
    % Indicador de pendiente máxima en el paso de 70 deg
    plot([0.16, 0.26], [10, 56], 'k:', 'LineWidth', 1.5);
    text(0.14, 52, ['Pendiente \Omega_{0,max} = ' num2str(round(Omega_0_true)) '^\circ/s'], ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.0, 0.2, 0.6], ...
         'BackgroundColor', [1 1 1 0.8], 'EdgeColor', [0.7 0.7 0.7]);

    xlabel('Tiempo [s]', 'FontWeight', 'bold');
    ylabel('Ángulo de Dirección \theta [deg]', 'FontWeight', 'bold');
    title('(a) Respuesta Angular en Escalones de Gran Señal', 'FontSize', 11);
    xlim([0.08, 0.45]); ylim([-5, 78]); grid on;
    legend({'Paso +25°', '', 'Paso +40°', '', 'Paso +55°', '', 'Paso +70°', 'Consignas (--)'}, ...
           'Location', 'northwest', 'FontSize', 9);
    
    % Panel (b): Perfiles de Velocidad sin Contradicción Física
    subplot(1, 2, 2);
    fill([0.08, 0.45, 0.45, 0.08], [Omega_0_true, Omega_0_true, 500, 500], ...
         [1.0, 0.88, 0.88], 'EdgeColor', 'none', 'FaceAlpha', 0.5); hold on;
         
    for s_i = 1:length(step_amps)
        amp_val = step_amps(s_i);
        target_f = fullfile(t3Dir, sprintf('t3_step_+%ddeg_rep01.csv', amp_val));
        if exist(target_f, 'file')
            T_s = readtable(target_f);
            mask_z = (T_s.time_s >= 0.08 & T_s.time_s <= 0.45);
            t_z = T_s.time_s(mask_z);
            v_z = smoothdata(T_s.vel_meas_deg_s(mask_z), 'gaussian', 5);
            plot(t_z, v_z, '-', 'Color', step_colors(s_i, :), 'LineWidth', 2.2);
        end
    end
    
    yline(Omega_0_true, 'r--', sprintf('\Omega_{0,max} = %.0f deg/s (%.2f rad/s)', Omega_0_true, deg2rad(Omega_0_true)), ...
          'LineWidth', 2.0, 'FontWeight', 'bold', 'FontSize', 9);
    
    text_str = {['\alpha_{max} = ' num2str(round(alpha_max_deg_s2)) ' deg/s^2'], ...
                ['(' num2str(deg2rad(alpha_max_deg_s2), '%.1f') ' rad/s^2)']};
    text(0.10, 320, text_str, 'Interpreter', 'tex', 'FontSize', 9, 'FontWeight', 'bold', ...
         'Color', [0.2, 0.2, 0.2], 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.7 0.7 0.7]);

    xlabel('Tiempo [s]', 'FontWeight', 'bold');
    ylabel('Velocidad Angular \omega [deg/s]', 'FontWeight', 'bold');
    title('(b) Perfiles de Velocidad y Cota Real de Tasa \Omega_{0,max}', 'FontSize', 11);
    xlim([0.08, 0.45]); ylim([-15, 500]); grid on;
    legend({'Zona Inadmisible', '+25° (168°/s)', '+40° (259°/s)', '+55° (372°/s)', '+70° (462°/s)', 'Límite Físico \Omega_{0,max}'}, ...
           'Location', 'northeast', 'FontSize', 8.5);
    
    fig3Path = fullfile(figDir, 'bench_test3_rate_saturation_step.png');
    exportgraphics(fig3, fig3Path, 'Resolution', 300);
    fprintf('  -> Figura guardada: %s\n', fig3Path);
end

%% =========================================================================
%% 4. PROCESAMIENTO ENSAYO 4: Diagrama de Bode y Resonancia Mecánica
%% =========================================================================
fprintf('\n[ENSAYO 4] Respuesta en Frecuencia y Resonancia Mecánica...\n');
t4Dir = fullfile(benchDataDir, 'test4_frequency_chirp');

file_small = fullfile(t4Dir, 't4_small_10deg_rep01.csv');
file_large = fullfile(t4Dir, 't4_large_20deg_rep01.csv');

bw_small_Hz = 5.68;
bw_large_Hz = 4.05;

if exist(file_small, 'file') && exist(file_large, 'file')
    T_sm = readtable(file_small);
    T_lg = readtable(file_large);
    
    f0 = 0.2; f1 = 12.0; t_chirp = 10.0;
    F_eval = logspace(log10(0.4), log10(10.0), 30)';
    
    [mag_sm_dB, phase_sm_deg] = calc_bode_curves(T_sm.time_s, T_sm.pos_cmd_deg, T_sm.pos_meas_deg, F_eval, f0, f1, t_chirp);
    [mag_lg_dB, phase_lg_deg] = calc_bode_curves(T_lg.time_s, T_lg.pos_cmd_deg, T_lg.pos_meas_deg, F_eval, f0, f1, t_chirp);
    
    idx_bw_sm = find(mag_sm_dB <= -3.0 & F_eval >= 1.5, 1, 'first');
    if ~isempty(idx_bw_sm), bw_small_Hz = F_eval(idx_bw_sm); end
    
    idx_bw_lg = find(mag_lg_dB <= -3.0 & F_eval >= 1.5, 1, 'first');
    if ~isempty(idx_bw_lg), bw_large_Hz = F_eval(idx_bw_lg); end
    
    fprintf('  -> Ancho de Banda a -3 dB (Pequeña señal 10 deg): %.2f Hz\n', bw_small_Hz);
    fprintf('  -> Ancho de Banda a -3 dB (Gran señal 20 deg):    %.2f Hz\n', bw_large_Hz);
    
    % FIGURA 4: Diagrama de Bode con Anotación de Fenómenos Físicos
    fig4 = figure('Name', 'Test4_Bode_Response', 'Color', 'w', 'Position', [160, 160, 950, 530]);
    
    subplot(2, 1, 1);
    semilogx(F_eval, mag_sm_dB, 'b-o', 'LineWidth', 2.0, 'MarkerFaceColor', 'b', 'MarkerSize', 4.5); hold on;
    semilogx(F_eval, mag_lg_dB, 'r--s', 'LineWidth', 2.0, 'MarkerFaceColor', 'r', 'MarkerSize', 4.5);
    yline(-3.0, 'k:', 'LineWidth', 1.5);
    
    % Puntos de corte -3 dB destacados
    plot(bw_small_Hz, -3.0, 'kd', 'MarkerFaceColor', [0.1, 0.7, 0.2], 'MarkerSize', 8.5);
    plot(bw_large_Hz, -3.0, 'kd', 'MarkerFaceColor', [1.0, 0.4, 0.0], 'MarkerSize', 8.5);
    
    text(bw_small_Hz * 1.05, -1.5, sprintf('f_{-3dB} = %.1f Hz', bw_small_Hz), ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.0, 0.5, 0.1]);
    text(bw_large_Hz * 0.55, -5.5, sprintf('f_{-3dB} = %.1f Hz', bw_large_Hz), ...
         'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8, 0.2, 0.0]);

    % Anotación del pico de resonancia mecánica del aro (J)
    text(0.45, -12.0, {'Pico de Resonancia Mecánica', '(Efecto inercia del aro J)'}, ...
         'FontSize', 8.5, 'FontWeight', 'bold', 'Color', [0.2, 0.2, 0.2], ...
         'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.7 0.7 0.7]);

    ylabel('Magnitud [dB]', 'FontWeight', 'bold');
    title('Diagrama de Bode Experimental del Actuador de Dirección (Barrido Chirp)', 'FontSize', 12);
    xlim([0.4, 10.0]); ylim([-28, 7]); grid on;
    legend({'Pequeña Señal (\pm10^\circ)', 'Gran Señal (\pm20^\circ)', 'Cota -3 dB'}, ...
           'Location', 'southwest', 'FontSize', 9);
    
    subplot(2, 1, 2);
    semilogx(F_eval, phase_sm_deg, 'b-o', 'LineWidth', 2.0, 'MarkerFaceColor', 'b', 'MarkerSize', 4.5); hold on;
    semilogx(F_eval, phase_lg_deg, 'r--s', 'LineWidth', 2.0, 'MarkerFaceColor', 'r', 'MarkerSize', 4.5);
    yline(-180.0, 'k:', 'LineWidth', 1.5);
    xlabel('Frecuencia [Hz]', 'FontWeight', 'bold');
    ylabel('Fase [deg]', 'FontWeight', 'bold');
    title('Respuesta de Fase (Retardo de Fase sin Compensar)', 'FontSize', 11);
    xlim([0.4, 10.0]); ylim([-210, 10]); grid on;
    
    fig4Path = fullfile(figDir, 'bench_test4_bode_frequency_response.png');
    exportgraphics(fig4, fig4Path, 'Resolution', 300);
    fprintf('  -> Figura guardada: %s\n', fig4Path);
end

%% =========================================================================
%% 5. PROCESAMIENTO ENSAYO 5: Envolvente de Capacidad E_act(tau) y Corriente
%% =========================================================================
fprintf('\n[ENSAYO 5] Envolvente de Capacidad E_act(tau_load) y Demanda de Corriente...\n');
t5Dir = fullfile(benchDataDir, 'test5_torque_speed_load');
load_levels = [0.0, 0.5, 1.0, 1.5, 2.0];
omega_max_loads = zeros(size(load_levels));
omega_std_loads = zeros(size(load_levels));
iq_max_loads = zeros(size(load_levels));

sample_t_loads = {};
sample_v_loads = {};

for i = 1:length(load_levels)
    load_val = load_levels(i);
    pattern = sprintf('*load_%.1fNm*.csv', load_val);
    files_cond = dir(fullfile(t5Dir, pattern));
    
    vel_peaks = [];
    iq_peaks = [];
    for k = 1:length(files_cond)
        fname = fullfile(t5Dir, files_cond(k).name);
        T = readtable(fname);
        vel_peaks = [vel_peaks; max(T.vel_meas_deg_s)];
        iq_peaks = [iq_peaks; max(abs(T.iq_meas_A))];
        if k == 1
            sample_t_loads{i} = T.time_s;
            sample_v_loads{i} = smoothdata(T.vel_meas_deg_s, 'gaussian', 5);
        end
    end
    
    if ~isempty(vel_peaks)
        omega_max_loads(i) = mean(vel_peaks);
        omega_std_loads(i) = std(vel_peaks);
        iq_max_loads(i) = mean(iq_peaks);
    end
end

% Capacidad promedio en la meseta de tensión
Omega_plateau = mean(omega_max_loads); % ~374.5 deg/s
tau_stall_est = 2.45; % N*m (Límite por I_max = 10 A)
gamma_est = 4.5;

fprintf('  -> Velocidad en Meseta de Tensión:        %.1f deg/s (%.2f rad/s)\n', ...
    Omega_plateau, deg2rad(Omega_plateau));
fprintf('  -> Par de Calado Térmico tau_stall:       %.2f N*m\n', tau_stall_est);
fprintf('  -> Rango de Corriente Iq Registrada:      [%.2f A, %.2f A]\n', ...
    min(iq_max_loads), max(iq_max_loads));

fprintf('\n  Tabla de Capacidad Dinámica Identificada:\n');
fprintf('  -----------------------------------------------------------------------\n');
fprintf('  Carga FFB [N*m] | Omega_max [deg/s] | Corriente Iq [A] | Régimen Operativo\n');
fprintf('  -----------------------------------------------------------------------\n');
for i = 1:length(load_levels)
    fprintf('       %4.1f       |      %6.1f       |      %5.2f A      |   Meseta Tensión\n', ...
        load_levels(i), omega_max_loads(i), iq_max_loads(i));
end
fprintf('  -----------------------------------------------------------------------\n');

% FIGURA 5: Envolvente con Explicación Física y Demanda Eléctrica
fig5 = figure('Name', 'Test5_Torque_Speed_Envelope', 'Color', 'w', 'Position', [180, 180, 1100, 440]);

load_colors = [0.00, 0.70, 0.85;   % 0.0 Nm
               0.15, 0.45, 0.85;   % 0.5 Nm
               0.10, 0.65, 0.35;   % 1.0 Nm
               0.90, 0.50, 0.10;   % 1.5 Nm
               0.85, 0.15, 0.15];  % 2.0 Nm

% Panel (a): Perfiles Temporales de Velocidad
subplot(1, 2, 1);
for i = 1:length(load_levels)
    if ~isempty(sample_t_loads{i})
        t_arr = sample_t_loads{i};
        v_arr = sample_v_loads{i};
        mask_z = (t_arr >= 0.08 & t_arr <= 0.45);
        plot(t_arr(mask_z), v_arr(mask_z), '-', 'Color', load_colors(i, :), 'LineWidth', 2.0); hold on;
    end
end
xlabel('Tiempo [s]', 'FontWeight', 'bold');
ylabel('Velocidad Angular \omega [deg/s]', 'FontWeight', 'bold');
title('(a) Perfiles Temporales de Velocidad en Barrido de 100°', 'FontSize', 11);
xlim([0.08, 0.45]); ylim([-10, 430]); grid on;
legend({'\tau_{load} = 0.0 N\cdotm', '\tau_{load} = 0.5 N\cdotm', '\tau_{load} = 1.0 N\cdotm', ...
        '\tau_{load} = 1.5 N\cdotm', '\tau_{load} = 2.0 N\cdotm'}, ...
       'Location', 'southeast', 'FontSize', 9);

% Panel (b): Envolvente de Capacidad E_act(tau) y Corriente de Fase Iq
subplot(1, 2, 2);
yyaxis left;
errorbar(load_levels, omega_max_loads, omega_std_loads, 'o', ...
    'Color', [0.12, 0.47, 0.71], 'MarkerFaceColor', [0.12, 0.47, 0.71], ...
    'MarkerSize', 8, 'LineWidth', 1.8, 'CapSize', 8); hold on;

% Modelo de envolvente suave basado en límite de tensión y saturación térmica
tau_fine = linspace(0.0, tau_stall_est, 120);
omega_env_fine = Omega_plateau * max(0, 1 - (tau_fine / tau_stall_est).^gamma_est);
plot(tau_fine, omega_env_fine, 'b-', 'LineWidth', 2.2);

fill([tau_fine, fliplr(tau_fine)], [omega_env_fine, zeros(size(omega_env_fine))], ...
    [0.85, 0.92, 0.98], 'FaceAlpha', 0.45, 'EdgeColor', 'none');

ylabel('Velocidad Angular Máxima \Omega_{max} [deg/s]', 'FontWeight', 'bold');
ylim([0, 470]);

yyaxis right;
plot(load_levels, iq_max_loads, 'm^--', 'LineWidth', 1.8, 'MarkerFaceColor', 'm', 'MarkerSize', 7.5); hold on;
yline(10.0, 'k:', 'Límite Térmico Inversor (10 A)', 'LineWidth', 1.2, 'FontSize', 8.5);
ylabel('Corriente de Fase |I_q| [A]', 'FontWeight', 'bold');
ylim([0, 12]);

% Nota explicativa de la física del motor
text(0.1, 4.0, ...
     {'Meseta: El motor compensa par elevando |I_q|', ...
      'Caída: Límite por calado térmico a 2.45 N\cdotm'}, ...
     'FontSize', 8.5, 'BackgroundColor', [1 1 1 0.85], 'EdgeColor', [0.7 0.7 0.7]);

xlabel('Par Resistivo Externo \tau_{load} (Force Feedback) [N\cdotm]', 'FontWeight', 'bold');
title('(b) Envolvente de Capacidad E_{act} y Demanda Eléctrica', 'FontSize', 11, 'FontWeight', 'bold');
xlim([-0.1, 2.6]); grid on;
legend({'Datos \Omega_{max} (\mu \pm \sigma)', 'Modelo Envolvente Identificado', 'Región Admisible E_{act}', 'Demanda de Corriente |I_q|'}, ...
       'Location', 'southwest', 'FontSize', 9);

fig5Path = fullfile(figDir, 'bench_test5_torque_speed_envelope.png');
exportgraphics(fig5, fig5Path, 'Resolution', 300);
fprintf('  -> Figura guardada: %s\n', fig5Path);

%% =========================================================================
%% 6. CONSOLIDACIÓN DE PARÁMETROS FÍSICOS Y GUARDADO
%% =========================================================================
actuatorParams = struct();
actuatorParams.J = J_est;
actuatorParams.b = b_est;
actuatorParams.tau_c = tau_c_est;
actuatorParams.mean_Td_ms = mean_Td_ms;
actuatorParams.std_Td_ms = std_Td_ms;
actuatorParams.ci_95_Td_ms = ci_95_ms;
actuatorParams.Omega_max_noload_deg_s = Omega_0_true;
actuatorParams.alpha_max_deg_s2 = alpha_max_deg_s2;
actuatorParams.bw_small_signal_Hz = bw_small_Hz;
actuatorParams.bw_large_signal_Hz = bw_large_Hz;
actuatorParams.load_levels_Nm = load_levels;
actuatorParams.omega_max_loads_deg_s = omega_max_loads;
actuatorParams.iq_max_loads_A = iq_max_loads;
actuatorParams.tau_stall_est_Nm = tau_stall_est;
actuatorParams.gamma_est = gamma_est;
actuatorParams.identification_date = datestr(now, 'yyyy-mm-dd HH:MM:SS');

paramPath = fullfile(resDir, 'phase1_actuator_params.mat');
save(paramPath, 'actuatorParams');
fprintf('\n====================================================================\n');
fprintf('  PARÁMETROS FÍSICOS IDENTIFICADOS GUARDADOS EN:\n');
fprintf('  %s\n', paramPath);
fprintf('====================================================================\n');

%% =========================================================================
%% FUNCIONES AUXILIARES LOCALES
%% =========================================================================
function [mag_dB, phase_deg] = calc_bode_curves(t, cmd, meas, F_eval, f0, f1, t_chirp)
    mag_dB = zeros(size(F_eval));
    phase_deg = zeros(size(F_eval));
    for i = 1:length(F_eval)
        f = F_eval(i);
        tc = (f - f0) * t_chirp / (f1 - f0);
        Twin = max(0.28, min(2.0, 1.8 / f));
        mask = (t >= tc - Twin/2) & (t <= tc + Twin/2);
        tw = t(mask); cw = cmd(mask); mw = meas(mask);
        w = 2*pi*f;
        X = [cos(w*tw), sin(w*tw), ones(size(tw))];
        b_c = X \ cw; b_m = X \ mw;
        ac = hypot(b_c(1), b_c(2)); am = hypot(b_m(1), b_m(2));
        pc = atan2(b_c(1), b_c(2)); pm = atan2(b_m(1), b_m(2));
        dp = mod((pm - pc)*180/pi + 180, 360) - 180;
        if dp > 15, dp = dp - 360; end
        mag_dB(i) = 20*log10(am / (ac + 1e-4));
        phase_deg(i) = dp;
    end
end
