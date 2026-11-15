%% =========================================================================
%% FASE 4: VALIDACIÓN EXPERIMENTAL HIL Y COMPARATIVA MULTI-PERFIL
%%
%% Proyecto: Experimental Characterization of Load-Dependent Steering
%%           Capability and Its Integration into Predictive Path Tracking Control
%% Objetivo: Vehicle System Dynamics (Taylor & Francis), Q1
%% Phase 4: Full-Lap Hardware-in-the-Loop Performance Processing
%% =========================================================================
clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  PROCESAMIENTO OFICIAL FASE 4: CAMPAÑA EXPERIMENTAL HIL (MONZA)\n');
fprintf('  Análisis de Corridas Independientes (N_laps), Multi-Perfil y Ablación\n');
fprintf('====================================================================\n\n');

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
repoRoot = fileparts(fileparts(scriptDir));

hilDataDir = fullfile(repoRoot, 'data', 'raw', 'hil_phase4');
resultsDir = fullfile(repoRoot, 'results');
figDir = fullfile(repoRoot, 'figures', 'phase4');
finalFigDir = fullfile(repoRoot, 'figures', 'final');

if ~exist(figDir, 'dir'), mkdir(figDir); end
if ~exist(finalFigDir, 'dir'), mkdir(finalFigDir); end

% Cargar envolvente de capacidad identificada en Fase 2
envPath = fullfile(resultsDir, 'phase2_capacity_envelope_constraints.mat');
dEnv = load(envPath);
Omega_0 = dEnv.phase2Constraints.Omega_0_deg_s;
tau_stall = dEnv.phase2Constraints.tau_stall_Nm;
gamma_env = dEnv.phase2Constraints.gamma;
omega_envelope_fun = @(tau) Omega_0 * max(0.0, 1 - (min(tau_stall, abs(tau)) ./ tau_stall).^gamma_env);

fprintf('[PASO 1] Escaneando directorio de telemetría HIL:\n  %s\n\n', hilDataDir);

%% 1. Inventario y Clasificación de Corridas
dirs = dir(fullfile(hilDataDir, 'HILL_prueba_s00_*'));
n_dirs = length(dirs);

laps = struct('folder', {}, 'ctrl', {}, 'perfil', {}, 'fecha', {}, 'pista', {}, 'L_track', {}, ...
              'completada', {}, 't_total_s', {}, 't_lap_s', {}, ...
              'n_cycles', {}, 'rms_ey_cm', {}, 'max_ey_cm', {}, ...
              'rms_epsi_deg', {}, 'max_epsi_deg', {}, ...
              't_cpu_ms_mean', {}, 't_cpu_ms_p95', {}, 't_cpu_ms_max', {}, ...
              'sat_envelope_pct', {}, 'T_tel', {}, 'T_esp', {}, 'T_elec', {});

for k = 1:n_dirs
    folderName = dirs(k).name;
    folderPath = fullfile(hilDataDir, folderName);
    telemPath = fullfile(folderPath, 'telemetria.csv');
    manifPath = fullfile(folderPath, 'manifiesto.json');
    espPath = fullfile(folderPath, 'odrive_espejo.csv');
    elecPath = fullfile(folderPath, 'odrive_electrico.csv');
    perfPath = fullfile(folderPath, 'perfil.csv');
    
    if ~exist(telemPath, 'file') || ~exist(manifPath, 'file')
        continue;
    end
    
    % Filtrar por fecha formal de campaña (desde 2026-11-10 13:41:06)
    tok = regexp(folderName, '_(\d{8})_(\d{6})$', 'tokens', 'once');
    if isempty(tok), continue; end
    dateStr = tok{1};
    timeStr = tok{2};
    if strcmp(dateStr, '20261110') && str2double(timeStr) < 134106
        continue;
    end
    if str2double(dateStr) < 20261110
        continue;
    end
    
    % Leer manifiesto
    mText = fileread(manifPath);
    mJson = jsondecode(mText);
    
    ctrl = mJson.meta.controlador;
    perfil = mJson.meta.perfil;
    
    % Identificar circuito según perfil.csv
    L_track = 5757.2;
    pista = 'Monza';
    if exist(perfPath, 'file')
        try
            optsP = detectImportOptions(perfPath);
            optsP.VariableNamingRule = 'preserve';
            TP = readtable(perfPath, optsP);
            if ismember('s_m', TP.Properties.VariableNames)
                L_track = max(TP.s_m);
            end
        catch
        end
    end
    if L_track < 3500
        pista = 'Magione';
    end
    
    % Leer telemetría
    opts = detectImportOptions(telemPath);
    opts.VariableNamingRule = 'preserve';
    T = readtable(telemPath, opts);
    
    if height(T) < 100
        continue; % Descartar arranques cancelados antes de calentar
    end
    
    % Filtrar a vuelta de medición
    if ismember('fase_vuelta', T.Properties.VariableNames)
        idx_med = strcmp(T.fase_vuelta, 'medida');
    else
        idx_med = true(height(T), 1);
    end
    
    if sum(idx_med) < 100
        continue;
    end
    
    T_med = T(idx_med, :);
    t_med = T_med.t_s;
    t_lap = t_med(end) - t_med(1);
    
    pos = T_med.posicion_normalizada;
    if strcmp(pista, 'Magione')
        completada = (min(pos) < 0.05 && max(pos) > 0.95 && t_lap > 115.0);
    else
        completada = (min(pos) < 0.05 && max(pos) > 0.95 && t_lap > 150.0);
    end
    
    ey_m = abs(T_med.e_y_cg_m);
    epsi_deg = abs(rad2deg(T_med.e_psi_cg_rad));
    
    rms_ey = sqrt(mean(ey_m.^2)) * 100.0; % cm
    max_ey = max(ey_m) * 100.0;           % cm
    rms_epsi = sqrt(mean(epsi_deg.^2));   % deg
    max_epsi = max(epsi_deg);             % deg
    
    % Tiempos de cálculo
    if ismember('tc_ms', T_med.Properties.VariableNames)
        tc = T_med.tc_ms(~isnan(T_med.tc_ms) & T_med.tc_ms > 0);
        t_mean = mean(tc);
        t_p95 = prctile(tc, 95);
        t_max = max(tc);
    else
        t_mean = NaN; t_p95 = NaN; t_max = NaN;
    end
    
    % Datos de espejo para envolvente
    sat_pct = 0.0;
    T_esp = table();
    if exist(espPath, 'file')
        try
            optsE = detectImportOptions(espPath);
            optsE.VariableNamingRule = 'preserve';
            T_esp = readtable(espPath, optsE);
            if ismember('omega_dps', T_esp.Properties.VariableNames) && ismember('tau_carga_nm', T_esp.Properties.VariableNames)
                w_meas = abs(T_esp.omega_dps);
                tau_c = abs(T_esp.tau_carga_nm);
                w_lim = omega_envelope_fun(tau_c);
                sat_pct = 100.0 * sum(w_meas >= (w_lim - 5.0)) / length(w_meas);
            end
        catch
        end
    end
    
    T_elec = table();
    if exist(elecPath, 'file')
        try
            optsEl = detectImportOptions(elecPath);
            optsEl.VariableNamingRule = 'preserve';
            T_elec = readtable(elecPath, optsEl);
        catch
        end
    end
    
    lapIdx = length(laps) + 1;
    laps(lapIdx).folder = folderName;
    laps(lapIdx).ctrl = ctrl;
    laps(lapIdx).perfil = perfil;
    laps(lapIdx).pista = pista;
    laps(lapIdx).L_track = L_track;
    laps(lapIdx).completada = completada;
    laps(lapIdx).t_total_s = T.t_s(end) - T.t_s(1);
    laps(lapIdx).t_lap_s = t_lap;
    laps(lapIdx).n_cycles = height(T_med);
    laps(lapIdx).rms_ey_cm = rms_ey;
    laps(lapIdx).max_ey_cm = max_ey;
    laps(lapIdx).rms_epsi_deg = rms_epsi;
    laps(lapIdx).max_epsi_deg = max_epsi;
    laps(lapIdx).t_cpu_ms_mean = t_mean;
    laps(lapIdx).t_cpu_ms_p95 = t_p95;
    laps(lapIdx).t_cpu_ms_max = t_max;
    laps(lapIdx).sat_envelope_pct = sat_pct;
    laps(lapIdx).T_tel = T_med;
    laps(lapIdx).T_esp = T_esp;
    laps(lapIdx).T_elec = T_elec;
end

fprintf('Total de corridas analizadas: %d\n\n', length(laps));

%% 2. Agregación Estadística por Controlador y Perfil (MONZA)
ctrlList = {'mpc_actuator_aware', 'mpc_actuator_fixed', 'mpc_cinematico'};
ctrlLabels = {'Propuesto (Actuator-Aware)', 'Baseline 3 (Cotas Fijas)', 'Baseline 1 (Convencional)'};
ctrlColors = {[0.12, 0.47, 0.71], [0.49, 0.18, 0.56], [0.85, 0.20, 0.20]};

perfList = {'conservador', 'nominal', 'rapido'};
perfLabels = {'Conservador (40 km/h chicane)', 'Nominal (60 km/h chicane)', 'Rápido (80 km/h chicane)'};

stats = struct();

fprintf('------------------------------------------------------------------------------------------------------------------\n');
fprintf('  CAMPAÑA OFICIAL MONZA (5.757 km) - CORRIDAS INDEPENDIENTES (N_laps):\n');
fprintf('  Controlador         | Perfil       | Intentos | Vueltas OK | T_lap [s] | e_y RMS [cm]   | e_y Max [cm]   | Sat [%]\n');
fprintf('------------------------------------------------------------------------------------------------------------------\n');

for c = 1:length(ctrlList)
    cId = ctrlList{c};
    for p = 1:length(perfList)
        pId = perfList{p};
        
        matchIdx = find(strcmp({laps.ctrl}, cId) & strcmp({laps.perfil}, pId) & strcmp({laps.pista}, 'Monza'));
        n_att = length(matchIdx);
        
        okIdx = matchIdx([laps(matchIdx).completada]);
        n_ok = length(okIdx);
        
        if n_ok > 0
            rms_vec = [laps(okIdx).rms_ey_cm];
            max_vec = [laps(okIdx).max_ey_cm];
            tlap_vec = [laps(okIdx).t_lap_s];
            sat_vec = [laps(okIdx).sat_envelope_pct];
            
            m_rms = mean(rms_vec);
            s_rms = std(rms_vec);
            m_max = mean(max_vec);
            s_max = std(max_vec);
            m_tlap = mean(tlap_vec);
            m_sat = mean(sat_vec);
        else
            m_rms = NaN; s_rms = NaN; m_max = NaN; s_max = NaN; m_tlap = NaN; m_sat = NaN;
        end
        
        stats.(cId).(pId).n_attempts = n_att;
        stats.(cId).(pId).n_completed = n_ok;
        stats.(cId).(pId).completion_rate = 100.0 * n_ok / max(1, n_att);
        stats.(cId).(pId).mean_lap_s = m_tlap;
        stats.(cId).(pId).rms_ey_cm_mean = m_rms;
        stats.(cId).(pId).rms_ey_cm_std = s_rms;
        stats.(cId).(pId).max_ey_cm_mean = m_max;
        stats.(cId).(pId).max_ey_cm_std = s_max;
        stats.(cId).(pId).sat_rate_pct = m_sat;
        
        if n_ok > 0
            fprintf('  %-19s | %-12s |    %2d    |     %2d     |   %5.1f   | %5.2f ± %4.2f | %5.2f ± %4.2f |  %4.1f%%\n', ...
                cId, pId, n_att, n_ok, m_tlap, m_rms, s_rms, m_max, s_max, m_sat);
        else
            fprintf('  %-19s | %-12s |    %2d    |     %2d     |    DNF    |      DNF       |      DNF       |   N/A\n', ...
                cId, pId, n_att, n_ok);
        end
    end
    fprintf('------------------------------------------------------------------------------------------------------------------\n');
end

%% 2b. VALIDACIÓN EXPLORATORIA CROSS-TRACK EN CIRCUITO DESCONOCIDO (MAGIONE, 2.455 km)
fprintf('\n==================================================================================================================\n');
fprintf('  VALIDACIÓN EXPLORATORIA CROSS-TRACK: AUTODROMO DI MAGIONE (2.455 km, CERO SOBREAJUSTE / ZERO-SHOT)\n');
fprintf('------------------------------------------------------------------------------------------------------------------\n');
fprintf('  Controlador         | Perfil       | Intentos | Vueltas OK | T_lap [s] | e_y RMS [cm]   | e_y Max [cm]   | Éxito\n');
fprintf('------------------------------------------------------------------------------------------------------------------\n');

stats_magione = struct();
for c = 1:length(ctrlList)
    cId = ctrlList{c};
    for p = 1:length(perfList)
        pId = perfList{p};
        mIdx = find(strcmp({laps.ctrl}, cId) & strcmp({laps.perfil}, pId) & strcmp({laps.pista}, 'Magione'));
        n_att = length(mIdx);
        okIdx = mIdx([laps(mIdx).completada]);
        n_ok = length(okIdx);
        if n_ok > 0
            m_rms = mean([laps(okIdx).rms_ey_cm]);
            m_max = max([laps(okIdx).max_ey_cm]);
            m_tlap = mean([laps(okIdx).t_lap_s]);
            fprintf('  %-19s | %-12s |    %2d    |     %2d     |   %5.1f   |     %5.2f      |     %5.2f      |  100%%\n', ...
                cId, pId, n_att, n_ok, m_tlap, m_rms, m_max);
        elseif n_att > 0
            fprintf('  %-19s | %-12s |    %2d    |     %2d     |    DNF    |      DNF       |      DNF       |   0%%\n', ...
                cId, pId, n_att, n_ok);
        end
        stats_magione.(cId).(pId).n_attempts = n_att;
        stats_magione.(cId).(pId).n_completed = n_ok;
        if n_ok > 0
            stats_magione.(cId).(pId).rms_ey_cm = m_rms;
            stats_magione.(cId).(pId).max_ey_cm = m_max;
            stats_magione.(cId).(pId).mean_lap_s = m_tlap;
        end
    end
end
fprintf('==================================================================================================================\n\n');

%% 3. FIGURA 1: Benchmark Comparativo Multi-Perfil (RMS, Pico y Tasa de Éxito)
fprintf('\n[PASO 2] Generando Figura 1: Comparativa Multi-Perfil (Q1 Standard)...\n');
fig1 = figure('Name', 'Phase4_Fig1_MultiProfile_Benchmark', 'Color', 'w', 'Position', [60, 60, 1100, 480]);

rms_mat = nan(length(perfList), length(ctrlList));
max_mat = nan(length(perfList), length(ctrlList));
comp_mat = zeros(length(perfList), length(ctrlList));

for p = 1:length(perfList)
    pId = perfList{p};
    for c = 1:length(ctrlList)
        cId = ctrlList{c};
        if stats.(cId).(pId).n_completed > 0
            rms_mat(p, c) = stats.(cId).(pId).rms_ey_cm_mean;
            max_mat(p, c) = stats.(cId).(pId).max_ey_cm_mean;
        end
        comp_mat(p, c) = stats.(cId).(pId).completion_rate;
    end
end

% Subplot 1: Error Lateral RMS
subplot(1, 3, 1);
b1 = bar(rms_mat, 'grouped');
for c = 1:length(ctrlList), b1(c).FaceColor = ctrlColors{c}; end
set(gca, 'XTickLabel', {'Conservador', 'Nominal', 'Rápido'}, 'FontSize', 10);
ylabel('Error Lateral RMS e_y [cm]', 'FontWeight', 'bold');
title('(a) Error Cuadrático Medio', 'FontSize', 11, 'FontWeight', 'bold');
legend(ctrlLabels, 'Location', 'northwest', 'FontSize', 8);
grid on; ylim([0, max(rms_mat(~isnan(rms_mat))) * 1.35]);

% Subplot 2: Error Lateral Pico
subplot(1, 3, 2);
b2 = bar(max_mat, 'grouped');
for c = 1:length(ctrlList), b2(c).FaceColor = ctrlColors{c}; end
set(gca, 'XTickLabel', {'Conservador', 'Nominal', 'Rápido'}, 'FontSize', 10);
ylabel('Error Lateral Máximo |e_y|_{max} [cm]', 'FontWeight', 'bold');
title('(b) Error Máximo Absoluto', 'FontSize', 11, 'FontWeight', 'bold');
grid on; ylim([0, max(max_mat(~isnan(max_mat))) * 1.35]);

% Subplot 3: Tasa de Finalización de Vuelta (%)
subplot(1, 3, 3);
b3 = bar(comp_mat, 'grouped');
for c = 1:length(ctrlList), b3(c).FaceColor = ctrlColors{c}; end
set(gca, 'XTickLabel', {'Conservador', 'Nominal', 'Rápido'}, 'FontSize', 10);
ylabel('Vueltas Completadas [%]', 'FontWeight', 'bold');
title('(c) Tasa de Éxito / Viabilidad Dinámica', 'FontSize', 11, 'FontWeight', 'bold');
grid on; ylim([0, 115]);

fig1Path = fullfile(figDir, 'phase4_fig1_hil_tracking_performance.png');
exportgraphics(fig1, fig1Path, 'Resolution', 300);
copyfile(fig1Path, fullfile(finalFigDir, 'Fig7_HIL_Tracking_Performance.png'));
fprintf('  -> Guardado: %s\n', fig1Path);

%% 4. FIGURA 2: Perfil Espacial de Error e_y(s) a lo Largo del Autódromo de Monza
fprintf('\n[PASO 3] Generando Figura 2: Perfil Espacial Continuo en Monza...\n');
fig2 = figure('Name', 'Phase4_Fig2_Spatial_Error_Profile', 'Color', 'w', 'Position', [80, 80, 1100, 600]);

% Seleccionar corridas representativas completadas en Nominal
idx_prop = find(strcmp({laps.ctrl}, 'mpc_actuator_aware') & strcmp({laps.perfil}, 'nominal') & [laps.completada], 1);
idx_fixed = find(strcmp({laps.ctrl}, 'mpc_actuator_fixed') & strcmp({laps.perfil}, 'nominal') & [laps.completada], 1);
idx_conv = find(strcmp({laps.ctrl}, 'mpc_cinematico') & strcmp({laps.perfil}, 'conservador') & [laps.completada], 1);

T_p = laps(idx_prop).T_tel;
T_f = laps(idx_fixed).T_tel;
T_c = laps(idx_conv).T_tel;

s_p = T_p.s_cg_m;
ey_p = T_p.e_y_cg_m * 100.0; % cm

s_f = T_f.s_cg_m;
ey_f = T_f.e_y_cg_m * 100.0; % cm

s_c = T_c.s_cg_m;
ey_c = T_c.e_y_cg_m * 100.0; % cm

subplot(2, 1, 1);
plot(s_p, ey_p, 'Color', ctrlColors{1}, 'LineWidth', 1.6, 'DisplayName', 'Propuesto (Actuator-Aware MPC)'); hold on;
plot(s_f, ey_f, 'Color', ctrlColors{2}, 'LineWidth', 1.3, 'LineStyle', '--', 'DisplayName', 'Baseline 3 (Cotas Fijas)');
plot(s_c, ey_c, 'Color', ctrlColors{3}, 'LineWidth', 1.1, 'LineStyle', ':', 'DisplayName', 'Baseline 1 (Convencional - Conservador)');
yline(0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
grid on; xlim([0, 5793]);
ylabel('Error Lateral e_y [cm]', 'FontWeight', 'bold');
title('(a) Desviación Lateral a lo Largo de los 5.793 km del Circuito de Monza', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 9);

% Marcar zonas críticas de Monza
xline(500, 'k--', 'Variante del Rettifilo', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
xline(1500, 'k--', 'Curva Grande', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
xline(2100, 'k--', 'Variante della Roggia', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
xline(3000, 'k--', 'Lesmo 1 & 2', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
xline(4400, 'k--', 'Variante Ascari', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);
xline(5300, 'k--', 'Curva Parabolica', 'LabelVerticalAlignment', 'bottom', 'FontSize', 8);

subplot(2, 1, 2);
plot(s_p, T_p.v_kmh, 'Color', [0.2, 0.2, 0.2], 'LineWidth', 1.5, 'DisplayName', 'Velocidad Longitudinal v_x'); hold on;
grid on; xlim([0, 5793]);
ylabel('Velocidad v_x [km/h]', 'FontWeight', 'bold');
xlabel('Distancia Recorrida s [m]', 'FontWeight', 'bold');
title('(b) Perfil de Velocidad Real HIL (Monza)', 'FontSize', 11, 'FontWeight', 'bold');

fig2Path = fullfile(figDir, 'phase4_fig2_full_trajectory_tracking_map.png');
exportgraphics(fig2, fig2Path, 'Resolution', 300);
copyfile(fig2Path, fullfile(finalFigDir, 'Fig8_HIL_Spatial_Tracking.png'));
fprintf('  -> Guardado: %s\n', fig2Path);

%% 5. FIGURA 3: Envolvente Dinámica de Capacidad en HIL (|tau_load| vs |omega|)
fprintf('\n[PASO 4] Generando Figura 3: Envolvente de Capacidad Operacional HIL...\n');
fig3 = figure('Name', 'Phase4_Fig3_Dynamic_Envelope_HIL', 'Color', 'w', 'Position', [100, 100, 850, 560]);

% Puntos experimentales de la corrida nominal propuesta
if ~isempty(laps(idx_prop).T_esp)
    E_p = laps(idx_prop).T_esp;
    tau_p = abs(E_p.tau_carga_nm);
    w_p = abs(E_p.omega_dps);
    
    scatter(tau_p, w_p, 8, [0.12, 0.47, 0.71], 'filled', 'MarkerFaceAlpha', 0.25, 'DisplayName', 'Operación HIL Propuesta');
    hold on;
end

% Puntos de corrida rápido (Fixed fallida vs Propuesto exitosa)
idx_prop_rap = find(strcmp({laps.ctrl}, 'mpc_actuator_aware') & strcmp({laps.perfil}, 'rapido') & [laps.completada], 1);
if ~isempty(idx_prop_rap) && ~isempty(laps(idx_prop_rap).T_esp)
    E_rap = laps(idx_prop_rap).T_esp;
    scatter(abs(E_rap.tau_carga_nm), abs(E_rap.omega_dps), 10, [0.17, 0.63, 0.17], 'filled', 'MarkerFaceAlpha', 0.35, ...
        'DisplayName', 'Operación HIL Rápido (Propuesto)');
end

% Curva analítica de la envolvente identificada en banco (Fase 2)
tau_grid = linspace(0, tau_stall, 300);
omega_bound = omega_envelope_fun(tau_grid);

plot(tau_grid, omega_bound, 'r-', 'LineWidth', 2.8, 'DisplayName', 'Envolvente Identificada \Omega_{max}(\tau)');
yline(Omega_0, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Cota Fija Convencional \Omega_0 (Ciega)');
xline(tau_stall, 'k:', 'LineWidth', 1.5, 'DisplayName', '\tau_{stall} = 2.45 N\cdotm');

grid on; box on;
xlim([0, 2.6]); ylim([0, 520]);
xlabel('Par de Carga Resistiva / FFB |\tau_{load}| [N\cdotm]', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Velocidad Angular de Dirección |\omega| [°/s]', 'FontWeight', 'bold', 'FontSize', 11);
title('Envolvente Dinámica de Capacidad y Puntos de Operación HIL', 'FontSize', 12, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9.5);

fig3Path = fullfile(figDir, 'phase4_fig3_actuator_envelope_in_hil.png');
exportgraphics(fig3, fig3Path, 'Resolution', 300);
copyfile(fig3Path, fullfile(finalFigDir, 'Fig9_HIL_Dynamic_Envelope.png'));
fprintf('  -> Guardado: %s\n', fig3Path);

%% 6. FIGURA 4: Detalle Focalizado de Chicanes Críticas (Rettifilo y Roggia)
fprintf('\n[PASO 5] Generando Figura 4: Respuesta Transitoria en Chicanes...\n');
fig4 = figure('Name', 'Phase4_Fig4_Chicane_Detail', 'Color', 'w', 'Position', [120, 120, 1050, 620]);

% Variante della Roggia (s entre 1900 m y 2300 m)
idx_rog_p = find(s_p >= 1950 & s_p <= 2350);
idx_rog_f = find(s_f >= 1950 & s_f <= 2350);

subplot(3, 1, 1);
plot(s_p(idx_rog_p), ey_p(idx_rog_p), 'Color', ctrlColors{1}, 'LineWidth', 2.0, 'DisplayName', 'Propuesto (Aware)'); hold on;
plot(s_f(idx_rog_f), ey_f(idx_rog_f), 'Color', ctrlColors{2}, 'LineWidth', 1.8, 'LineStyle', '--', 'DisplayName', 'Baseline 3 (Fixed)');
yline(0, 'k:', 'HandleVisibility', 'off');
grid on; xlim([1950, 2350]);
ylabel('Error Lateral e_y [cm]', 'FontWeight', 'bold');
title('Comportamiento Experimental en la Variante della Roggia (Monza)', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 9);

subplot(3, 1, 2);
plot(s_p(idx_rog_p), rad2deg(T_p.delta_pedido_rad(idx_rog_p)), 'Color', ctrlColors{1}, 'LineWidth', 1.8, 'DisplayName', '\delta_{cmd} (Propuesto)'); hold on;
plot(s_p(idx_rog_p), rad2deg(T_p.delta_aplicado_rad(idx_rog_p)), 'Color', [0.2, 0.4, 0.8], 'LineWidth', 1.5, 'LineStyle', ':', 'DisplayName', '\delta_{meas} (Propuesto)');
plot(s_f(idx_rog_f), rad2deg(T_f.delta_pedido_rad(idx_rog_f)), 'Color', ctrlColors{2}, 'LineWidth', 1.8, 'LineStyle', '--', 'DisplayName', '\delta_{cmd} (Fixed)');
plot(s_f(idx_rog_f), rad2deg(T_f.delta_aplicado_rad(idx_rog_f)), 'Color', [0.6, 0.2, 0.7], 'LineWidth', 1.5, 'LineStyle', '-.', 'DisplayName', '\delta_{meas} (Fixed)');
grid on; xlim([1950, 2350]);
ylabel('Ángulo de Rueda \delta [°]', 'FontWeight', 'bold');
title('Consigna Óptima vs Ángulo Físico Medido por el Encoder', 'FontSize', 10, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 8);

subplot(3, 1, 3);
plot(s_p(idx_rog_p), T_p.v_kmh(idx_rog_p), 'Color', [0.3, 0.3, 0.3], 'LineWidth', 1.5);
grid on; xlim([1950, 2350]);
ylabel('Velocidad [km/h]', 'FontWeight', 'bold');
xlabel('Posición en Pista s [m]', 'FontWeight', 'bold');
title('Velocidad Longitudinal en la Maniobra', 'FontSize', 10, 'FontWeight', 'bold');

fig4Path = fullfile(figDir, 'phase4_fig4_chicane_behavior_detailed.png');
exportgraphics(fig4, fig4Path, 'Resolution', 300);
copyfile(fig4Path, fullfile(finalFigDir, 'Fig10_HIL_Chicane_Transient.png'));
fprintf('  -> Guardado: %s\n', fig4Path);

%% 7. FIGURA 5: Distribución de Tiempos de Cómputo y Latencia HIL
fprintf('\n[PASO 6] Generando Figura 5: Latencia Computacional y WCET...\n');
fig5 = figure('Name', 'Phase4_Fig5_Timing_WCET', 'Color', 'w', 'Position', [140, 140, 850, 480]);

all_tc = [];
for k = 1:length(laps)
    if laps(k).completada && strcmp(laps(k).ctrl, 'mpc_actuator_aware')
        tc_k = laps(k).T_tel.tc_ms;
        all_tc = [all_tc; tc_k(~isnan(tc_k) & tc_k > 0)];
    end
end

histogram(all_tc, 60, 'FaceColor', [0.12, 0.47, 0.71], 'EdgeColor', 'none', 'Normalization', 'pdf');
hold on; grid on;

tc_mean = mean(all_tc);
tc_p95 = prctile(all_tc, 95);
tc_p99 = prctile(all_tc, 99);
tc_max = max(all_tc);

xline(tc_mean, 'k-', sprintf('Media = %.2f ms', tc_mean), 'LineWidth', 2.0, 'LabelVerticalAlignment', 'middle');
xline(tc_p95, 'm--', sprintf('P95 = %.2f ms', tc_p95), 'LineWidth', 1.8, 'LabelVerticalAlignment', 'top');
xline(tc_max, 'r-', sprintf('WCET = %.2f ms', tc_max), 'LineWidth', 2.0, 'LabelVerticalAlignment', 'middle');
xline(50.0, 'g--', 'Periodo MPC T_s = 50.0 ms (20 Hz)', 'LineWidth', 2.0, 'LabelVerticalAlignment', 'bottom');

xlim([0, 10.0]);
xlabel('Tiempo de Cálculo de la Optimización QP [ms]', 'FontWeight', 'bold', 'FontSize', 11);
ylabel('Densidad de Probabilidad', 'FontWeight', 'bold', 'FontSize', 11);
title(sprintf('Distribución Temporal del Solver QP en Banco HIL (N = %d ciclos)', length(all_tc)), ...
    'FontSize', 12, 'FontWeight', 'bold');

fig5Path = fullfile(figDir, 'phase4_fig5_timing_and_wcet.png');
exportgraphics(fig5, fig5Path, 'Resolution', 300);
copyfile(fig5Path, fullfile(finalFigDir, 'Fig11_HIL_Computational_Timing.png'));
fprintf('  -> Guardado: %s\n', fig5Path);

%% 8. Guardar Estructura Consolidada de Resultados
hilValidationResults = struct();
hilValidationResults.laps = laps;
hilValidationResults.stats_monza = stats;
hilValidationResults.stats_magione = stats_magione;
hilValidationResults.timing = struct('mean_ms', tc_mean, 'p95_ms', tc_p95, 'p99_ms', tc_p99, 'wcet_ms', tc_max, 'n_samples', length(all_tc));
hilValidationResults.fecha_procesamiento = datestr(now, 31);

save(fullfile(resultsDir, 'phase4_hil_validation_results.mat'), 'hilValidationResults');
fprintf('\n[PASO 7] Archivo oficial guardado: %s\n', fullfile(resultsDir, 'phase4_hil_validation_results.mat'));

fprintf('\n====================================================================\n');
fprintf('  PROCESAMIENTO OFICIAL FASE 4 COMPLETADO EXITOSAMENTE\n');
fprintf('====================================================================\n');
