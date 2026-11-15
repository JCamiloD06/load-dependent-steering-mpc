%% =========================================================================
%% FASE 2: CONSTRUCCIÓN DE LA ENVOLVENTE DE CAPACIDAD DINÁMICA
%%         Y MAPEO DE DEMANDA VEHICULAR EN CIRCUITOS REALES
%%
%% Proyecto: Experimental Characterization of Load-Dependent Steering
%%           Capability and Its Integration into Predictive Path Tracking Control
%% Objetivo Editorial: Vehicle System Dynamics (Taylor & Francis), Q1
%% =========================================================================
clear; clc; close all;

fprintf('====================================================================\n');
fprintf('  FASE 2: CONSTRUCCIÓN DE ENVOLVENTE Y ANÁLISIS DE DEMANDA VEHICULAR\n');
fprintf('====================================================================\n\n');

%% 1. Cargar Parámetros Físicos Identificados en Fase 1
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
repoRoot = fileparts(fileparts(scriptDir));
if ~exist(fullfile(repoRoot, 'results'), 'dir')
    repoRoot = pwd;
end

paramsPath = fullfile(repoRoot, 'results', 'phase1_actuator_params.mat');
if ~exist(paramsPath, 'file')
    paramsPath = fullfile('results', 'phase1_actuator_params.mat');
end
if ~exist(paramsPath, 'file')
    error('No se encuentra %s. Ejecutar primero scripts/matlab/process_phase1_bench_data.m', paramsPath);
end
dataParams = load(paramsPath);
p1 = dataParams.actuatorParams;

Omega_0_deg_s = p1.Omega_max_noload_deg_s;      % ~380 deg/s
alpha_max_deg_s2 = p1.alpha_max_deg_s2;         % ~6722 deg/s^2
T_d_ms = p1.mean_Td_ms;                         % ~40.7 ms
tau_stall_est = p1.tau_stall_est_Nm;           % ~2.30 Nm
gamma_est = p1.gamma_est;                       % Exponente no lineal

fprintf('[PASO 1] Parámetros identificados del actuador cargados:\n');
fprintf('  -> Omega_0 (vacío):        %.1f deg/s (%.2f rad/s)\n', Omega_0_deg_s, deg2rad(Omega_0_deg_s));
fprintf('  -> Par de Calado tau_s:    %.2f N*m\n', tau_stall_est);
fprintf('  -> Aceleración alpha_max:  %.1f deg/s^2\n', alpha_max_deg_s2);
fprintf('  -> Retardo puro T_d:       %.2f ms\n', T_d_ms);
fprintf('  -> Ancho de banda (-3dB):  %.2f Hz\n\n', p1.bw_small_signal_Hz);

% Física de actuadores síncronos BLDC / FOC (Vehicle System Dynamics):
% La envolvente par-velocidad sigue gamma entre 1.0 (ley lineal de par-tensión) y 1.8 (saturación de ciclo PWM).
% Modelo de envolvente par-velocidad identificado (gamma = 4.50)
% Representa la meseta de saturacion de tension y la caida termica hacia tau_stall = 2.45 Nm
gamma_phys = gamma_est;
omega_envelope_fun = @(tau) Omega_0_deg_s * max(0, 1 - (abs(tau) / tau_stall_est).^gamma_phys);

%% 2. Directorios de Datos y Figuras
figDir = fullfile(repoRoot, 'figures', 'phase2');
if ~exist(figDir, 'dir'), mkdir(figDir); end

resDir = fullfile(repoRoot, 'results');
if ~exist(resDir, 'dir'), mkdir(resDir); end

% Buscar carpeta de telemetrías HIL de TrialSteer
trialsteerDirs = {
    'C:\Users\Volante\Documents\Paper Q2 SIL-HIL FFB\data\raw\trialsteer', ...
    fullfile(repoRoot, '..', 'Paper Q2 SIL-HIL FFB', 'data', 'raw', 'trialsteer'), ...
    fullfile(repoRoot, 'data', 'raw', 'trialsteer')
};
tsDir = '';
for d = 1:length(trialsteerDirs)
    if exist(trialsteerDirs{d}, 'dir')
        tsDir = trialsteerDirs{d};
        break;
    end
end
if isempty(tsDir)
    error('No se encontró el directorio de telemetrías de TrialSteer.');
end
fprintf('[PASO 2] Directorio de telemetría encontrado: %s\n', tsDir);

%% 3. Cargar y Procesar Telemetría Real de los 4 Circuitos
% Mapeo de sesiones a circuitos
pistas = struct();
pistas.monza.prefix = 'HILL_campana_s01_mpc_actuator_aware_nominal*';
pistas.monza.name = 'Monza (Alta velocidad / Chicanes)';
pistas.monza.color = [0.12, 0.47, 0.71]; % Azul

pistas.magione.prefix = 'HILL_campana_s02_mpc_actuator_aware_nominal*';
pistas.magione.name = 'Magione (Sinuoso continuo)';
pistas.magione.color = [0.17, 0.63, 0.17]; % Verde

pistas.laguna_seca.prefix = 'HILL_campana_s03_mpc_actuator_aware_nominal*';
pistas.laguna_seca.name = 'Laguna Seca (Desnivel / Corkscrew)';
pistas.laguna_seca.color = [1.00, 0.50, 0.05]; % Naranja

pistas.vallelunga.prefix = 'HILL_campana_s04_mpc_actuator_aware_nominal*';
pistas.vallelunga.name = 'Vallelunga Club (Horquillas ultra-cerradas)';
pistas.vallelunga.color = [0.58, 0.40, 0.74]; % Morado

% Filtrado de circuito seleccionado desde la GUI (TrialSteer)
targetCircuit = getenv('TARGET_CIRCUIT');
cfgFile = fullfile(repoRoot, 'results', 'phase2_gui_config.json');
if exist(cfgFile, 'file')
    try
        fid = fopen(cfgFile, 'r');
        rawJson = fread(fid, inf, '*char')';
        fclose(fid);
        cfg = jsondecode(rawJson);
        if isfield(cfg, 'target_circuit') && ~isempty(cfg.target_circuit)
            targetCircuit = cfg.target_circuit;
        end
    catch
    end
end
if isempty(targetCircuit) && exist('TARGET_CIRCUIT', 'var')
    targetCircuit = TARGET_CIRCUIT;
end

if ~isempty(targetCircuit) && ~strcmpi(targetCircuit, 'todas')
    cClean = lower(strrep(targetCircuit, ' ', '_'));
    if strcmpi(cClean, 'vallelunga_club')
        cClean = 'vallelunga';
    end
    if isfield(pistas, cClean)
        circuitos = {cClean};
        fprintf('  [FILTRO GUI ACTIVO] Procesando únicamente el circuito seleccionado: %s\n', pistas.(cClean).name);
    else
        circuitos = fieldnames(pistas);
    end
else
    circuitos = fieldnames(pistas);
    fprintf('  [MODO GLOBAL] Procesando todos los circuitos disponibles (%d pistas)\n', length(circuitos));
end
demandData = struct();

fprintf('\n[PASO 3] Procesando telemetría física de circuitos HIL:\n');
fprintf('  ----------------------------------------------------------------------------------------\n');
fprintf('  Circuito         | Muestras | tau_max [N*m] | Omega_max [deg/s] | Puntos en Contracción\n');
fprintf('  ----------------------------------------------------------------------------------------\n');

all_tau = [];
all_omega = [];

for c = 1:length(circuitos)
    cname = circuitos{c};
    patt = pistas.(cname).prefix;
    matchDirs = dir(fullfile(tsDir, patt));
    
    if isempty(matchDirs)
        % Búsqueda alternativa por nombre si s0X no coincide
        matchDirs = dir(fullfile(tsDir, sprintf('*%s*nominal*', cname)));
    end
    
    if ~isempty(matchDirs)
        runPath = fullfile(tsDir, matchDirs(1).name);
        fileEspejo = fullfile(runPath, 'odrive_espejo.csv');
        fileTelem = fullfile(runPath, 'telemetria.csv');
        
        if exist(fileEspejo, 'file') && exist(fileTelem, 'file')
            T_esp = readtable(fileEspejo);
            T_tel = readtable(fileTelem);
            
            tau_c = abs(T_esp.tau_carga_nm);
            om_meas = abs(T_esp.omega_dps);
            
            % Filtrado de muestras estacionarias en boxes
            validIdx = T_esp.t_perf_s > (T_esp.t_perf_s(1) + 2.0);
            tau_c = tau_c(validIdx);
            om_meas = om_meas(validIdx);
            
            demandData.(cname).tau = tau_c;
            demandData.(cname).omega = om_meas;
            demandData.(cname).T_esp = T_esp;
            demandData.(cname).T_tel = T_tel;
            
            % Evaluar cuántos puntos exigen más del 60% de la capacidad
            cota_local = omega_envelope_fun(tau_c);
            ratio = om_meas ./ (cota_local + 1e-3);
            pct_high_demand = 100 * sum(ratio > 0.60) / length(ratio);
            
            fprintf('  %-16s |  %6d  |    %5.2f      |       %6.1f      |        %5.1f%%\n', ...
                cname, length(tau_c), max(tau_c), max(om_meas), pct_high_demand);
            
            all_tau = [all_tau; tau_c];
            all_omega = [all_omega; om_meas];
        end
    end
end
fprintf('  ----------------------------------------------------------------------------------------\n');

%% 4. FIGURA 1: Envolvente de Capacidad E_act(tau_load) vs Demanda de Conducción en Circuito
fprintf('\n[PASO 4] Generando Figura 1: Mapa de Capacidad vs Demanda de Conducción...\n');
fig1 = figure('Name', 'Phase2_Capacity_Envelope_vs_Circuit_Demand', 'Color', 'w', 'Position', [100, 100, 920, 560]);

% Curva analítica de la envolvente identificada
tau_grid = linspace(0.0, tau_stall_est, 300);
omega_env = omega_envelope_fun(tau_grid);

% Región admisible sombreada
fill([tau_grid, fliplr(tau_grid)], [omega_env, zeros(size(omega_env))], ...
    [0.85, 0.92, 0.98], 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'DisplayName', 'Región Admisible E_{act}(\tau_{load})');
hold on;

% Cota fija clásica del MPC cinemático (Baseline 1: ciego al par de carga)
omega_fixed = 400.0; % Cota simétrica típica de catálogo
yline(omega_fixed, 'k--', 'LineWidth', 2.0, 'DisplayName', 'Cota Fija Convencional (MPC Clásico: 400^\circ/s)');

% Superposición de nubes de dispersión de demanda por circuito (submuestreo para claridad)
for c = 1:length(circuitos)
    cname = circuitos{c};
    if isfield(demandData, cname)
        tau_sub = demandData.(cname).tau;
        om_sub = demandData.(cname).omega;
        step_sub = max(1, round(length(tau_sub) / 800));
        scatter(tau_sub(1:step_sub:end), om_sub(1:step_sub:end), 12, pistas.(cname).color, ...
            'filled', 'MarkerFaceAlpha', 0.4, 'DisplayName', sprintf('Demanda: %s', pistas.(cname).name));
    end
end

% Borde de la envolvente identificado en banco
plot(tau_grid, omega_env, 'r-', 'LineWidth', 2.8, 'DisplayName', 'Envolvente Identificada \Omega_{max}(\tau_{load})');

% Punto de calado
plot(tau_stall_est, 0, 'rx', 'MarkerSize', 12, 'LineWidth', 2.5, 'DisplayName', sprintf('Calado \\tau_{stall} = %.2f N\\cdotm', tau_stall_est));

% Zona de infracción/inviabilidad del MPC convencional
x_infr = [1.2, tau_stall_est, tau_stall_est, 1.2];
y_infr = [omega_envelope_fun(1.2), 0, omega_fixed, omega_fixed];
patch(x_infr, y_infr, [1.0, 0.8, 0.8], 'FaceAlpha', 0.35, 'EdgeColor', 'r', 'LineStyle', ':', ...
    'DisplayName', 'Zona de Inviabilidad Física del MPC Convencional');

xlabel('Par Resistivo Externo \tau_{load} (Auto-alineante del Neumático) [N\cdotm]', 'FontWeight', 'bold');
ylabel('Velocidad Angular de Dirección |\omega| [deg/s]', 'FontWeight', 'bold');
title('Envolvente Dinámica de Capacidad del Actuador vs Demanda en Circuitos HIL', 'FontSize', 12, 'FontWeight', 'bold');
xlim([0.0, 2.5]); ylim([0.0, 450.0]);
grid on;
legend('Location', 'northeast', 'FontSize', 8.5);

fig1Path = fullfile(figDir, 'phase2_fig1_capacity_envelope_vs_demand.png');
exportgraphics(fig1, fig1Path, 'Resolution', 300);
fprintf('  -> Guardado: %s\n', fig1Path);

%% 5. FIGURA 2: Serie Temporal Transitoria en Maniobra Crítica (Monza Variante Ascari)
fprintf('\n[PASO 5] Generando Figura 2: Dinámica Transitoria en Maniobra Crítica...\n');
fig2 = figure('Name', 'Phase2_Transient_Maneuver_Analysis', 'Color', 'w', 'Position', [120, 120, 950, 620]);

% Extraer tramo representativo de Monza (Chicane con inversión rápida de dirección)
if isfield(demandData, 'monza')
    T_esp_m = demandData.monza.T_esp;
    t_raw = T_esp_m.t_perf_s - T_esp_m.t_perf_s(1);
    
    % Seleccionar ventana de 12 segundos con alta dinámica
    % Buscamos el momento de máxima velocidad angular combinada con par
    metric_stress = abs(T_esp_m.omega_dps) .* abs(T_esp_m.tau_carga_nm);
    [~, maxIdx] = max(metric_stress);
    t_center = t_raw(maxIdx);
    idx_win = find(t_raw >= max(0, t_center - 6.0) & t_raw <= (t_center + 6.0));
    
    t_w = t_raw(idx_win) - t_raw(idx_win(1));
    theta_w = T_esp_m.theta_meas_deg(idx_win);
    omega_w = abs(T_esp_m.omega_dps(idx_win));
    tau_w = abs(T_esp_m.tau_carga_nm(idx_win));
    
    cota_adaptativa = zeros(size(tau_w));
    for i = 1:length(tau_w)
        cota_adaptativa(i) = omega_envelope_fun(tau_w(i));
    end
    cota_fija = 400.0 * ones(size(tau_w));
    margen_capacidad = cota_adaptativa - omega_w;
    
    % Subplot 1: Ángulo del Volante
    subplot(3, 1, 1);
    plot(t_w, theta_w, 'b-', 'LineWidth', 2.0); hold on;
    yline(0, 'k:');
    ylabel('Ángulo \delta [deg]', 'FontWeight', 'bold');
    title('Respuesta Transitoria del Sistema de Dirección en Maniobra Evasiva (Monza)', 'FontSize', 11);
    grid on; xlim([0, max(t_w)]);
    
    % Subplot 2: Tasa de Dirección Demandada vs Cotas
    subplot(3, 1, 2);
    plot(t_w, omega_w, 'k-', 'LineWidth', 1.8, 'DisplayName', 'Tasa Demandada en Pista |\omega|'); hold on;
    plot(t_w, cota_adaptativa, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Cota Dinámica \Omega_{max}(\tau_{load})');
    plot(t_w, cota_fija, 'b--', 'LineWidth', 1.6, 'DisplayName', 'Cota Fija Clásica (400^\circ/s)');
    ylabel('Tasa |\omega| [deg/s]', 'FontWeight', 'bold');
    grid on; xlim([0, max(t_w)]); ylim([0, 450]);
    legend('Location', 'northwest', 'FontSize', 8);
    
    % Subplot 3: Par Resistivo y Margen de Capacidad
    subplot(3, 1, 3);
    yyaxis left;
    plot(t_w, tau_w, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 1.8);
    ylabel('Par Resistivo \tau_{load} [N\cdotm]', 'FontWeight', 'bold');
    ylim([0, 2.5]);
    
    yyaxis right;
    area(t_w, max(0, margen_capacidad), 'FaceColor', [0.47, 0.67, 0.19], 'FaceAlpha', 0.4, 'EdgeColor', 'none');
    ylabel('Margen \Delta\Omega [deg/s]', 'FontWeight', 'bold');
    ylim([0, 400]);
    xlabel('Tiempo de Maniobra [s]', 'FontWeight', 'bold');
    grid on; xlim([0, max(t_w)]);
    
    fig2Path = fullfile(figDir, 'phase2_fig2_transient_steering_stress.png');
    exportgraphics(fig2, fig2Path, 'Resolution', 300);
    fprintf('  -> Guardado: %s\n', fig2Path);
end

%% 6. FIGURA 3: Mapa de Contracción de Capacidad vs Curvatura y Velocidad
fprintf('\n[PASO 6] Generando Figura 3: Mapa de Contracción vs Curvatura y Velocidad...\n');
fig3 = figure('Name', 'Phase2_Contraction_vs_Curvature_Speed', 'Color', 'w', 'Position', [140, 140, 850, 480]);

% Relación física entre curvatura kappa, velocidad v y par auto-alineante:
% La aceleración lateral es a_y = v^2 * kappa (limitada por adherencia del neumático a_y,max ~ 10 m/s^2)
v_kmh_grid = linspace(40, 180, 100);
kappa_grid = linspace(0.001, 0.05, 100); % Radios de 1000m a 20m
[V, K] = meshgrid(v_kmh_grid / 3.6, kappa_grid); % V en m/s

% Aceleración lateral con saturación por adherencia en asfalto seco
a_y = min(10.0, (V.^2) .* K);

% Par auto-alineante transmitido al actuador: tau_load = K_ay * a_y
% Calibrado para alcanzar ~1.8 Nm en el límite de adherencia (1.0 g)
K_ay = 0.18; % [N*m / (m/s^2)]
tau_estimado = min(2.20, K_ay * a_y);

% Porcentaje de reducción de capacidad de giro
omega_disponible = omega_envelope_fun(tau_estimado);
pct_reduccion = 100 * (1 - omega_disponible / Omega_0_deg_s);

contourf(v_kmh_grid, 1./kappa_grid, pct_reduccion, 20, 'LineColor', 'none');
colormap(flipud(parula));
c = colorbar;
c.Label.String = 'Reducción de Capacidad Dinámica \Delta\Omega [%]';
c.Label.FontWeight = 'bold';

hold on;
% Línea de contorno al 20% y 50% de contracción
[C20, h20] = contour(v_kmh_grid, 1./kappa_grid, pct_reduccion, [20, 20], 'w--', 'LineWidth', 2.0);
clabel(C20, h20, 'Color', 'w', 'FontWeight', 'bold');
[C50, h50] = contour(v_kmh_grid, 1./kappa_grid, pct_reduccion, [50, 50], 'r-', 'LineWidth', 2.2);
clabel(C50, h50, 'Color', 'r', 'FontWeight', 'bold');

xlabel('Velocidad del Vehículo v [km/h]', 'FontWeight', 'bold');
ylabel('Radio de Curva R = 1/\kappa [m]', 'FontWeight', 'bold');
title('Contracción de la Capacidad de Dirección por Carga Lateral en Maniobras', 'FontSize', 11, 'FontWeight', 'bold');
set(gca, 'YScale', 'log');
ylim([20, 500]); xlim([40, 180]);
grid on;

fig3Path = fullfile(figDir, 'phase2_fig3_curvature_vs_torque_contraction.png');
exportgraphics(fig3, fig3Path, 'Resolution', 300);
fprintf('  -> Guardado: %s\n', fig3Path);

%% 7. Consolidación de Restricciones para el MPC de Fase 3
fprintf('\n[PASO 7] Consolidando Restricciones y Tabla para el Optimizador QP (Fase 3)...\n');

% Construcción de la tabla de restricciones discretizadas para QP
tau_table = linspace(0.0, 2.30, 24); % Discretización de 0.1 Nm
omega_table_deg = omega_envelope_fun(tau_table);
omega_table_rad = deg2rad(omega_table_deg);

phase2Constraints = struct();
phase2Constraints.Omega_0_deg_s = Omega_0_deg_s;
phase2Constraints.Omega_0_rad_s = deg2rad(Omega_0_deg_s);
phase2Constraints.alpha_max_deg_s2 = alpha_max_deg_s2;
phase2Constraints.alpha_max_rad_s2 = deg2rad(alpha_max_deg_s2);
phase2Constraints.T_d_s = T_d_ms / 1000.0;
phase2Constraints.N_delay_steps = ceil((T_d_ms / 1000.0) / 0.02); % Pasos a 50 Hz
phase2Constraints.tau_stall_Nm = tau_stall_est;
phase2Constraints.gamma = gamma_phys;
phase2Constraints.tau_table_Nm = tau_table;
phase2Constraints.omega_table_rad_s = omega_table_rad;

savePath = fullfile(resDir, 'phase2_capacity_envelope_constraints.mat');
save(savePath, 'phase2Constraints');
fprintf('  -> Restricciones exportadas con éxito: %s\n', savePath);

fprintf('\n====================================================================\n');
fprintf('  FASE 2 FINALIZADA EXITOSAMENTE\n');
fprintf('  Resultados listos para alimentar formalmente el MPC de Fase 3.\n');
fprintf('====================================================================\n');
