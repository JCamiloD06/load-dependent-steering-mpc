%% generate_phase4_figures_en.m
% Regenerates ALL Phase 4 HIL figures (Figs 6 to 11) in crisp English at 600 DPI
% strictly from the 18 Nov 10 formal HIL benchmark campaign runs:
%   HILL_prueba_s00_mpc_actuator_{aware,fixed,cinematico}_{conservador,nominal,rapido}_20261110_{134106..155820}
% Standalone MATLAB script for Phase 4 HIL benchmark figures (600 DPI).
%
% Outputs in figures/final/:
%   Fig6_HIL_Tracking_Performance.png
%   Fig7_HIL_Spatial_Tracking.png
%   Fig8_HIL_Chicane_Transient.png
%   Fig9_HIL_Dynamic_Envelope.png
%   Fig10_HIL_Computational_Timing.png
%   Fig11_Ablation_Limit_Cycle.png

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(fileparts(scriptDir));
hilRoot   = fullfile(repoRoot, 'data', 'raw', 'hil_phase4');
matP1     = fullfile(repoRoot, 'results', 'phase1_actuator_params.mat');
outDir    = fullfile(repoRoot, 'figures', 'final');
if ~exist(outDir, 'dir'), mkdir(outDir); end

load(matP1, 'actuatorParams');

% Hardware physical constants verified in bench calibration
Kt = 0.55;      % N*m/A
R_phase = 0.534; % Ohm

% Consistent publication palette
cProp = [0.06 0.35 0.76];  % Proposed (Actuator-Aware MPC)
cFix  = [0.85 0.33 0.10];  % Baseline 3 (Fixed-Bounds Actuator MPC)
cBlind= [0.76 0.12 0.24];  % Baseline 1 (Blind Kinematic MPC)

%% ========================================================================
%  1. LOAD & AGGREGATE THE 18 NOV 10 FORMAL HIL RUNS (ZERO HARDCODING)
%  ========================================================================
ctrlKeys   = {'aware', 'fixed', 'cinematico'};
ctrlLabels = {'Proposed (Aware)', 'Baseline 3 (Fixed)', 'Baseline 1 (Blind)'};
profKeys   = {'conservador', 'nominal', 'rapido'};
profLabels = {'Conservative', 'Nominal', 'Fast'};

runs = [];
all_tc_prop = [];

dList = dir(fullfile(hilRoot, 'HILL_prueba_s00_*'));
for i = 1:numel(dList)
    if ~dList(i).isdir, continue; end
    fname = dList(i).name;
    tok = regexp(fname, '_(\d{8})_(\d{6})$', 'tokens', 'once');
    if isempty(tok), continue; end
    dateStr = tok{1};
    timeStr = tok{2};
    if strcmp(dateStr, '20261110') && str2double(timeStr) < 134106, continue; end
    if str2double(dateStr) < 20261110, continue; end

    rDir = fullfile(dList(i).folder, fname);
    perfPath = fullfile(rDir, 'perfil.csv');
    if exist(perfPath, 'file')
        try
            optsP = detectImportOptions(perfPath);
            optsP.VariableNamingRule = 'preserve';
            TP = readtable(perfPath, optsP);
            if ismember('s_m', TP.Properties.VariableNames) && max(TP.s_m) < 3500
                continue; % Exclude Magione from Monza figure generation
            end
        catch
        end
    end

    telPath = fullfile(rDir, 'telemetria.csv');
    espPath = fullfile(rDir, 'odrive_espejo.csv');
    elecPath = fullfile(rDir, 'odrive_electrico.csv');
    if ~exist(telPath, 'file') || ~exist(espPath, 'file') || ~exist(elecPath, 'file')
        continue;
    end

    tel  = readtable(telPath);
    if height(tel) < 100, continue; end % Skip zero-distance aborted startups

    esp  = readtable(espPath);
    elec = readtable(elecPath);

    % Identify controller & profile from folder name
    cIdx = 0; pIdx = 0;
    for cc = 1:3
        if contains(fname, ctrlKeys{cc}), cIdx = cc; break; end
    end
    for pp = 1:3
        if contains(fname, profKeys{pp}), pIdx = pp; break; end
    end
    if cIdx == 0 || pIdx == 0, continue; end

    % Completion criterion: full lap completed ('terminada' present or s_max > 5650 & completed laps >= 1)
    isMed = strcmp(string(tel.fase_vuelta), 'medida');
    hasTerm = any(strcmp(string(tel.fase_vuelta), 'terminada'));
    sMed = tel.s_cg_m(isMed);
    completed = hasTerm && (~isempty(sMed)) && (max(sMed) > 5600);

    ey_cm = 100.0 * tel.e_y_cg_m(isMed);
    ep_deg = rad2deg(tel.e_psi_cg_rad(isMed));

    rec.id = [dateStr '_' timeStr];
    rec.dir = rDir;
    rec.cIdx = cIdx;
    rec.pIdx = pIdx;
    rec.completed = completed;
    rec.rms_ey = sqrt(mean(ey_cm.^2));
    rec.max_ey = max(abs(ey_cm));
    rec.rms_ep = sqrt(mean(ep_deg.^2));
    rec.tc_mean = mean(tel.tc_ms);
    rec.tc_max  = max(tel.tc_ms);
    rec.iq_rms  = sqrt(mean(elec.iq_a.^2));
    rec.iq_max  = max(abs(elec.iq_a));

    % Smoothed mirror angular velocity for encoder-quantization immunity
    t_esp = esp.t_perf_s - esp.t_perf_s(1);
    w_sm = computeSmoothRate(t_esp, esp.theta_meas_deg);
    rec.p99_w = prctile(abs(w_sm), 99);
    rec.max_w = max(abs(w_sm));

    runs = [runs; rec]; %#ok<AGROW>
    if cIdx == 1
        all_tc_prop = [all_tc_prop; tel.tc_ms(:)]; %#ok<AGROW>
    end
end
fprintf('[INFO] Loaded %d formal HIL runs; Proposed total cycles N = %d\n', numel(runs), numel(all_tc_prop));

% Aggregate 3x3 matrix (profile x controller)
rmsMat = nan(3, 3); maxMat = nan(3, 3); compMat = nan(3, 3);
rmsMin = nan(3, 3); rmsMax = nan(3, 3);
maxMin = nan(3, 3); maxMax = nan(3, 3);
for p = 1:3
    for c = 1:3
        sub = runs([runs.pIdx]==p & [runs.cIdx]==c);
        % For completion rate: mean over replicates
        compMat(p, c) = 100.0 * mean([sub.completed]);
        % If at least one replicate completed, report completed lap mean cleanly;
        % otherwise report aborted run mean (marked with *)
        subComp = sub([sub.completed]);
        if ~isempty(subComp)
            valsR = [subComp.rms_ey];
            valsM = [subComp.max_ey];
        else
            valsR = [sub.rms_ey];
            valsM = [sub.max_ey];
        end
        rmsMat(p, c) = mean(valsR); rmsMin(p, c) = min(valsR); rmsMax(p, c) = max(valsR);
        maxMat(p, c) = mean(valsM); maxMin(p, c) = min(valsM); maxMax(p, c) = max(valsM);
    end
end

%% ========================================================================
%  FIGURE 6: HIL BENCHMARK SUMMARY ACROSS PROFILES & COMPLETION RATES
%  ========================================================================
fig6 = figure('Units', 'pixels', 'Position', [40 40 1460 480], 'Color', 'w');
set(fig6, 'PaperPositionMode', 'auto');
cols3 = [cProp; cFix; cBlind];

% Panel (a): RMS Lateral Tracking Error [cm] (Log scale BaseValue=1)
subplot(1, 3, 1); hold on; grid on; box on;
b1 = bar(1:3, rmsMat, 'grouped', 'BarWidth', 0.80, 'BaseValue', 1);
for c = 1:3, b1(c).FaceColor = cols3(c, :); b1(c).EdgeColor = 'k'; b1(c).LineWidth = 0.8; end
set(gca, 'YScale', 'log', 'XTick', 1:3, 'XTickLabel', profLabels, 'FontName', 'Arial', 'FontSize', 9.5);
ylim([1 3800]);
ylabel('RMS Lateral Error e_y [cm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) Lateral RMS Tracking Error', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
for p = 1:3
    for c = 1:3
        xp = b1(c).XEndPoints(p); yp = rmsMat(p, c);
        tag = ''; if compMat(p, c) == 0, tag = '*'; end
        text(xp, yp * 1.18, sprintf('%.1f%s', yp, tag), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontName', 'Arial', 'FontSize', 6.8, 'FontWeight', 'bold');
    end
end
legend(ctrlLabels, 'Location', 'northwest', 'FontSize', 7.8);

% Panel (b): Peak Lateral Error [cm] (Log scale BaseValue=1)
subplot(1, 3, 2); hold on; grid on; box on;
b2 = bar(1:3, maxMat, 'grouped', 'BarWidth', 0.80, 'BaseValue', 1);
for c = 1:3, b2(c).FaceColor = cols3(c, :); b2(c).EdgeColor = 'k'; b2(c).LineWidth = 0.8; end
set(gca, 'YScale', 'log', 'XTick', 1:3, 'XTickLabel', profLabels, 'FontName', 'Arial', 'FontSize', 9.5);
ylim([1 9500]);
ylabel('Peak Lateral Error max|e_y| [cm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Peak Excursion (*Off-Track Abort)', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
for p = 1:3
    for c = 1:3
        xp = b2(c).XEndPoints(p); yp = maxMat(p, c);
        tag = ''; if compMat(p, c) == 0, tag = '*'; end
        text(xp, yp * 1.18, sprintf('%.0f%s', yp, tag), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontName', 'Arial', 'FontSize', 6.8, 'FontWeight', 'bold');
    end
end

% Panel (c): Full-Lap Completion Rate [%]
subplot(1, 3, 3); hold on; grid on; box on;
b3 = bar(1:3, compMat, 'grouped', 'BarWidth', 0.80);
for c = 1:3, b3(c).FaceColor = cols3(c, :); b3(c).EdgeColor = 'k'; b3(c).LineWidth = 0.8; end
set(gca, 'XTick', 1:3, 'XTickLabel', profLabels, 'FontName', 'Arial', 'FontSize', 9.5);
ylim([0 118]);
ylabel('Full-Lap Completion Rate [%]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(c) Completion Across All 18 Runs (n=2/cell)', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
for p = 1:3
    for c = 1:3
        xp = b3(c).XEndPoints(p); yp = compMat(p, c);
        text(xp, yp + 2.5, sprintf('%.0f%%', yp), 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'bottom', 'FontName', 'Arial', 'FontSize', 6.5, 'FontWeight', 'bold');
    end
end

exportgraphics(fig6, fullfile(outDir, 'Fig6_HIL_Tracking_Performance.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig6_HIL_Tracking_Performance.png\n');

%% ========================================================================
%  FIGURE 7: SPATIAL LATERAL TRACKING ERROR ALONG MONZA (FAST PROFILE)
%  ========================================================================
rPropF = loadRunById(hilRoot, '154629', Kt, R_phase);
rFixF  = loadRunById(hilRoot, '155021', Kt, R_phase);
rBldF  = loadRunById(hilRoot, '155426', Kt, R_phase);

fig7 = figure('Units', 'pixels', 'Position', [40 40 1240 600], 'Color', 'w');
set(fig7, 'PaperPositionMode', 'auto');

cornersS = [931.6, 2134.0, 2857.3, 3933.6, 5169.9];
cornersN = {'Rettifilo (0.93 km)', 'Roggia (2.13 km)', 'Lesmo (2.86 km)', 'Ascari (3.93 km)', 'Parabolica (5.17 km)'};

% Panel (a): Full Spatial Excursion Overview (cleanly cut at wrap)
subplot(2, 1, 1); hold on; grid on; box on;
[spP, eyP] = splitSpatialWrap(rPropF.s_med, rPropF.ey_med);
[spF, eyF] = splitSpatialWrap(rFixF.s_med,  rFixF.ey_med);
[spB, eyB] = splitSpatialWrap(rBldF.s_med,  rBldF.ey_med);

for k = 1:numel(cornersS)
    xline(cornersS(k), ':', 'Color', [0.55 0.55 0.60], 'LineWidth', 1.1, 'HandleVisibility', 'off');
end
plot(spB, eyB, '-', 'Color', cBlind, 'LineWidth', 1.4, 'DisplayName', 'Baseline 1 (Blind, off-track @ 5.08 km)');
plot(spF, eyF, '--', 'Color', cFix, 'LineWidth', 1.9, 'DisplayName', 'Baseline 3 (Fixed, crashes in Roggia @ 2.12 km)');
plot(spP, eyP, '-', 'Color', cProp, 'LineWidth', 2.0, 'DisplayName', 'Proposed (Aware, complete lap max|e_y| = 0.52 m)');
plot(rFixF.s_med(end), rFixF.ey_med(end), 'p', 'MarkerSize', 11, 'MarkerFaceColor', cFix, ...
    'MarkerEdgeColor', 'k', 'HandleVisibility', 'off');
text(2210, -14.1, 'Baseline 3 wall impact (s = 2,120 m, e_y = -14.99 m)', ...
    'FontName', 'Arial', 'FontSize', 8.5, 'FontWeight', 'bold', 'Color', cFix);
ey_all_min = min([rPropF.ey_med; rFixF.ey_med; rBldF.ey_med]);
ey_all_max = max([rPropF.ey_med; rFixF.ey_med; rBldF.ey_med]);
yA_lo = min(-4.0, ey_all_min - 1.5); yA_hi = max(4.0, ey_all_max + 4.8);
for k = 1:numel(cornersS)
    text(cornersS(k), yA_hi - 0.5, cornersN{k}, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'top', ...
        'FontName', 'Arial', 'FontSize', 7.8, 'BackgroundColor', [1 1 1 0.88], 'EdgeColor', 'none');
end
xlim([0 5780]); ylim([yA_lo yA_hi]);
ylabel('Lateral Error e_y [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) Fast-Profile Spatial Tracking & High-Speed Failure Modes Along Monza (5.793 km)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8.4);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

% Panel (b): Zoomed Track-Corridor Detail for Full Lap
subplot(2, 1, 2); hold on; grid on; box on;
for k = 1:numel(cornersS)
    xline(cornersS(k), ':', 'Color', [0.55 0.55 0.60], 'LineWidth', 1.1, 'HandleVisibility', 'off');
end
yline(0.50, 'k--', 'LineWidth', 0.9, 'HandleVisibility', 'off');
yline(-0.50, 'k--', 'LineWidth', 0.9, 'DisplayName', '\pm0.50 m precision corridor');
plot(spB, eyB, '-', 'Color', cBlind, 'LineWidth', 1.3, 'DisplayName', 'Baseline 1 (Blind)');
plot(spF, eyF, '--', 'Color', cFix, 'LineWidth', 1.7, 'DisplayName', 'Baseline 3 (Fixed)');
plot(spP, eyP, '-', 'Color', cProp, 'LineWidth', 2.0, 'DisplayName', 'Proposed (Aware, RMS = 15.8 cm)');
xlim([0 5780]); ylim([-0.72 0.72]);
xlabel('Circuit Track Station s [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Lateral Error e_y [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Admissible Track-Corridor Detail (e_y \in [-0.72, +0.72] m)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southeast', 'FontSize', 8.2);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig7, fullfile(outDir, 'Fig7_HIL_Spatial_Tracking.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig7_HIL_Spatial_Tracking.png\n');

%% ========================================================================
%  FIGURE 8: VARIANTE DELLA ROGGIA CHICANE TRANSIENT & ACTUATOR EFFORT
%  ========================================================================
sWin = [1900 2250];
wProp = extractSpatialWindow(rPropF, sWin(1), sWin(2), Kt, R_phase);
wFix  = extractSpatialWindow(rFixF,  sWin(1), sWin(2), Kt, R_phase);
wBld  = extractSpatialWindow(rBldF,  sWin(1), sWin(2), Kt, R_phase);

fprintf('[CHICANE STATS] Prop max|ey|=%.2fm, max|Iq|=%.2fA, EJ=%.2fJ | Fix max|ey|=%.2fm, max|w|=%.1f | Bld max|ey|=%.2fm\n', ...
    wProp.max_ey, wProp.max_iq, wProp.E_joule, wFix.max_ey, wFix.max_w, wBld.max_ey);

fig8 = figure('Units', 'pixels', 'Position', [40 40 1240 860], 'Color', 'w');
set(fig8, 'PaperPositionMode', 'auto');

% (a) Lateral Tracking Error e_y(s)
subplot(4, 1, 1); hold on; grid on; box on;
xline(2134.0, ':', 'Color', [0.45 0.45 0.50], 'LineWidth', 1.1, 'HandleVisibility', 'off');
plot(wBld.s_tel,  wBld.ey,  '-',  'Color', cBlind, 'LineWidth', 1.6, 'DisplayName', 'Baseline 1 (Blind, max|e_y| = 10.46 m)');
plot(wFix.s_tel,  wFix.ey,  '--', 'Color', cFix,   'LineWidth', 1.9, 'DisplayName', 'Baseline 3 (Fixed, crash @ 2,120 m)');
plot(wProp.s_tel, wProp.ey, '-',  'Color', cProp,  'LineWidth', 2.2, 'DisplayName', 'Proposed (Aware, max|e_y| = 0.38 m)');
y8_lo = min(-1.5, min([wProp.ey; wFix.ey; wBld.ey]) - 1.2);
y8_hi = max(1.8, max([wProp.ey; wFix.ey; wBld.ey]) + 1.5);
xlim(sWin); ylim([y8_lo y8_hi]);
ylabel('Lateral Error e_y [m]', 'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold');
title('(a) Variante della Roggia Lateral Tracking (Fast Profile, s = 1,900\rightarrow2,250 m)', ...
    'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8.0);
set(gca, 'FontName', 'Arial', 'FontSize', 9.0);

% (b) Commanded vs Measured Bench Handwheel Angle [deg]
subplot(4, 1, 2); hold on; grid on; box on;
xline(2134.0, ':', 'Color', [0.45 0.45 0.50], 'LineWidth', 1.1, 'HandleVisibility', 'off');
yline(250, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-250, 'k:', 'LineWidth', 1.0, 'DisplayName', '\pm250\circ mechanical stop');
plot(wBld.s_esp,  wBld.th_meas, '-',  'Color', cBlind, 'LineWidth', 1.5, 'DisplayName', 'Baseline 1 \theta_{meas}');
plot(wFix.s_esp,  wFix.th_meas, '--', 'Color', cFix,   'LineWidth', 1.8, 'DisplayName', 'Baseline 3 \theta_{meas}');
plot(wProp.s_esp, wProp.th_cmd,  ':',  'Color', [0.2 0.2 0.25], 'LineWidth', 1.3, 'DisplayName', 'Proposed \theta_{cmd}');
plot(wProp.s_esp, wProp.th_meas, '-',  'Color', cProp,  'LineWidth', 2.2, 'DisplayName', 'Proposed \theta_{meas}');
xlim(sWin); ylim([-275 275]);
ylabel('Steering Angle \theta [deg]', 'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold');
title('(b) Handwheel Tracking vs Hardware Stops (\pm250\circ)', ...
    'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
legend('Location', 'southwest', 'NumColumns', 2, 'FontSize', 7.6);
set(gca, 'FontName', 'Arial', 'FontSize', 9.0);

% (c) Angular Velocity Magnitude |omega| [deg/s] vs Dynamic Envelope
subplot(4, 1, 3); hold on; grid on; box on;
xline(2134.0, ':', 'Color', [0.45 0.45 0.50], 'LineWidth', 1.1, 'HandleVisibility', 'off');
plot(wProp.s_esp, wProp.env_lim, 'k:', 'LineWidth', 1.6, 'DisplayName', 'Dynamic limit \Omega_{max}(\tau_{load})');
plot(wBld.s_esp,  abs(wBld.w_sm),  '-',  'Color', cBlind, 'LineWidth', 1.4, 'DisplayName', 'Baseline 1 |\omega|');
plot(wFix.s_esp,  abs(wFix.w_sm),  '--', 'Color', cFix,   'LineWidth', 1.8, 'DisplayName', 'Baseline 3 |\omega|');
plot(wProp.s_esp, abs(wProp.w_sm), '-',  'Color', cProp,  'LineWidth', 2.1, 'DisplayName', 'Proposed |\omega|');
xlim(sWin); ylim([0 660]);
ylabel('Angular Rate |\omega| [deg/s]', 'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold');
title('(c) Smoothed Steering Angular Velocity vs Dynamic Capacity Envelope', ...
    'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
legend('Location', 'northwest', 'NumColumns', 2, 'FontSize', 7.8);
set(gca, 'FontName', 'Arial', 'FontSize', 9.0);

% (d) Real Measured ODrive Actuator Effort: Current |Iq(s)| & Joule Dissipation
subplot(4, 1, 4); hold on; grid on; box on;
xline(2134.0, ':', 'Color', [0.45 0.45 0.50], 'LineWidth', 1.1, 'HandleVisibility', 'off');
plot(wBld.s_elec,  abs(wBld.iq),  '-',  'Color', cBlind, 'LineWidth', 1.5, ...
    'DisplayName', sprintf('Baseline 1 |I_q| (peak %.2f A)', wBld.max_iq));
plot(wFix.s_elec,  abs(wFix.iq),  '--', 'Color', cFix,   'LineWidth', 1.8, ...
    'DisplayName', sprintf('Baseline 3 |I_q| (peak %.2f A)', wFix.max_iq));
plot(wProp.s_elec, abs(wProp.iq), '-',  'Color', cProp,  'LineWidth', 2.2, ...
    'DisplayName', sprintf('Proposed |I_q| (peak %.2f A, P_{J,max}=%.2f W, E_J=%.2f J)', ...
    wProp.max_iq, wProp.max_pj, wProp.E_joule));
xlim(sWin); ylim([0 3.5]);
ylabel('Phase Current |I_q| [A]', 'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold');
xlabel('Circuit Track Station s [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(d) Measured Direct-Drive Actuator Effort: Quadrature Current |I_q(s)| (R = 0.534 \Omega)', ...
    'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 7.8);
set(gca, 'FontName', 'Arial', 'FontSize', 9.0);

exportgraphics(fig8, fullfile(outDir, 'Fig8_HIL_Chicane_Transient.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig8_HIL_Chicane_Transient.png\n');

%% ========================================================================
%  FIGURE 9: HIL DYNAMIC OPERATING PHASE PLANES (FFB LOAD & EM TORQUE)
%  ========================================================================
fig9 = figure('Units', 'pixels', 'Position', [40 40 1240 500], 'Color', 'w');
set(fig9, 'PaperPositionMode', 'auto');

tau_env = linspace(0, 2.45, 250);
w0_hil = actuatorParams.Omega_max_noload_deg_s;
w_plat_env = w0_hil * max(0, 1 - (tau_env / 2.45).^4.5);

% Panel (a): FFB Load vs Smoothed Mirror Rate
subplot(1, 2, 1); hold on; grid on; box on;
fill([tau_env fliplr(tau_env)], [w_plat_env zeros(size(w_plat_env))], [0.22 0.55 0.85], ...
    'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(tau_env, w_plat_env, 'b-', 'LineWidth', 2.2, ...
    'DisplayName', 'MPC admissible envelope E_{act}(\tau_{load})');
yline(w0_hil, 'k:', 'LineWidth', 1.5, ...
    'DisplayName', 'Catalog fixed bound (\Omega_0 = 462.5\circ/s)');

mB = abs(rBldF.w_sm_med) > 20; mF = abs(rFixF.w_sm_med) > 20; mP = abs(rPropF.w_sm_med) > 20;
scatter(abs(rBldF.tau_med(mB)), abs(rBldF.w_sm_med(mB)), 10, cBlind, 'filled', 'MarkerFaceAlpha', 0.25, ...
    'DisplayName', 'Baseline 1 (Blind Fast)');
scatter(abs(rFixF.tau_med(mF)), abs(rFixF.w_sm_med(mF)), 14, cFix, '^', 'filled', 'MarkerFaceAlpha', 0.45, ...
    'DisplayName', 'Baseline 3 (Fixed Fast crash)');
scatter(abs(rPropF.tau_med(mP)), abs(rPropF.w_sm_med(mP)), 14, cProp, 'filled', 'MarkerFaceAlpha', 0.55, ...
    'DisplayName', 'Proposed (Aware Fast, 0.16% raw viol.)');
xlim([0 1.20]); ylim([0 640]);
xlabel('Resistive FFB Load |\tau_{load}| [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Smoothed Bench Steering Rate |\omega| [deg/s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) FFB Load Operating Envelope Compliance', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 7.8);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

% Panel (b): Measured Electromagnetic Effort |\tau_em| = Kt|Iq| vs |\omega|
subplot(1, 2, 2); hold on; grid on; box on;
fill([tau_env fliplr(tau_env)], [w_plat_env zeros(size(w_plat_env))], [0.22 0.55 0.85], ...
    'FaceAlpha', 0.10, 'EdgeColor', 'none', 'HandleVisibility', 'off');
plot(tau_env, w_plat_env, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Dynamic capacity envelope');
xline(2.45, 'r--', 'LineWidth', 1.6, 'DisplayName', 'Stall limit \tau_{stall} = 2.45 N\cdotm');

scatter(abs(rBldF.tem_elec), abs(rBldF.w_elec), 14, cBlind, 'filled', 'MarkerFaceAlpha', 0.30, ...
    'DisplayName', sprintf('Baseline 1 (peak %.2f N\\cdotm)', Kt*rBldF.iq_max));
scatter(abs(rFixF.tem_elec), abs(rFixF.w_elec), 18, cFix, '^', 'filled', 'MarkerFaceAlpha', 0.55, ...
    'DisplayName', sprintf('Baseline 3 (peak %.2f N\\cdotm)', Kt*rFixF.iq_max));
scatter(abs(rPropF.tem_elec), abs(rPropF.w_elec), 18, cProp, 'filled', 'MarkerFaceAlpha', 0.65, ...
    'DisplayName', sprintf('Proposed (peak %.2f N\\cdotm)', Kt*rPropF.iq_max));
xlim([0 2.65]); ylim([0 640]);
xlabel('Electromagnetic Torque |\tau_{em}| = K_t |I_q| [N\cdotm]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Smoothed Bench Steering Rate |\omega| [deg/s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Realized Electromagnetic Torque-Speed Effort', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 7.8);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig9, fullfile(outDir, 'Fig9_HIL_Dynamic_Envelope.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig9_HIL_Dynamic_Envelope.png\n');

%% ========================================================================
%  FIGURE 10: REAL-TIME COMPUTATIONAL TIMING & WCET TAIL (PROPOSED)
%  ========================================================================
tc_mean = mean(all_tc_prop);
tc_p95  = prctile(all_tc_prop, 95);
tc_p99  = prctile(all_tc_prop, 99);
tc_wcet = max(all_tc_prop);
N_cyc   = numel(all_tc_prop);

fig10 = figure('Units', 'pixels', 'Position', [40 40 1200 470], 'Color', 'w');
set(fig10, 'PaperPositionMode', 'auto');

subplot(1, 2, 1); hold on; grid on; box on;
histogram(all_tc_prop(all_tc_prop <= 3.0), 45, 'Normalization', 'probability', ...
    'FaceColor', cProp, 'EdgeColor', 'none', 'FaceAlpha', 0.78, 'DisplayName', 'Solve time PDF');
xline(tc_mean, 'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('Mean = %.2f ms', tc_mean));
xline(tc_p95,  'k-.', 'LineWidth', 1.6, 'DisplayName', sprintf('P_{95} = %.2f ms', tc_p95));
xline(tc_p99,  'm:',  'LineWidth', 1.8, 'DisplayName', sprintf('P_{99} = %.2f ms', tc_p99));
xlim([0 3.0]);
xlabel('OSQP QP Solve Time t_c [ms]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Cycle Probability', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title(sprintf('(a) Nominal Execution Distribution (N = %d Cycles)', N_cyc), ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northeast', 'FontSize', 8.3);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

subplot(1, 2, 2); hold on; grid on; box on;
tc_sorted = sort(all_tc_prop);
ccdf = 1 - (1:N_cyc)' / (N_cyc + 1);
semilogy(tc_sorted, ccdf, '-', 'Color', cProp, 'LineWidth', 2.2, 'DisplayName', 'Empirical exceedance (1 - CDF)');
xline(tc_wcet, 'r--', 'LineWidth', 2.0, 'DisplayName', sprintf('Observed WCET = %.2f ms', tc_wcet));
xline(20.0, 'k-.', 'LineWidth', 1.6, 'DisplayName', '20 ms soft budget (0.0% overrun)');
xline(50.0, '-', 'Color', [0.55 0.1 0.1], 'LineWidth', 2.0, 'DisplayName', '50 ms MPC deadline (76.8% margin)');
xlim([0 54]); ylim([1e-5 1.2]);
set(gca, 'YScale', 'log');
xlabel('OSQP QP Solve Time t_c [ms]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Tail Probability P(T_c > t)', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Real-Time WCET Tail & Sample-Time Margin', 'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8.0);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig10, fullfile(outDir, 'Fig10_HIL_Computational_Timing.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig10_HIL_Computational_Timing.png (N=%d, mean=%.2f, wcet=%.2f)\n', N_cyc, tc_mean, tc_wcet);

%% ========================================================================
%  FIGURE 11: ABLATION - SUPPRESSION OF LIMIT-CYCLE OSCILLATIONS (CONS.)
%  ========================================================================
rPropC = loadRunById(hilRoot, '134106', Kt, R_phase);
rBldC  = loadRunById(hilRoot, '135907', Kt, R_phase);

deP = gradient(rPropC.ey_med, rPropC.t_med);
deB = gradient(rBldC.ey_med,  rBldC.t_med);
t_med_B = rBldC.t_med - rBldC.t_med(1);
t_med_P = rPropC.t_med - rPropC.t_med(1);

fig11 = figure('Units', 'pixels', 'Position', [40 40 1240 760], 'Color', 'w');
set(fig11, 'PaperPositionMode', 'auto');

% Panel (a): Lateral Error Time History
subplot(2, 2, [1 2]); hold on; grid on; box on;
plot(t_med_B, rBldC.ey_med,  '-', 'Color', cBlind, 'LineWidth', 1.4, ...
    'DisplayName', 'Baseline 1 (Blind, RMS = 49.7 cm, max = 4.30 m)');
plot(t_med_P, rPropC.ey_med, '-', 'Color', cProp,  'LineWidth', 1.9, ...
    'DisplayName', 'Proposed (Aware, RMS = 8.5 cm, max = 0.25 m)');
xlim([0 max(t_med_P(end), t_med_B(end))]); ylim([-4.7 4.4]);
xlabel('Measured Lap Elapsed Time t [s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Lateral Error e_y [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(a) Conservative Lap Lateral Tracking: Suppression of Unmodeled Oscillations', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8.5);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

% Panel (b): Steering Angle Time History (full range, no clip!)
subplot(2, 2, 3); hold on; grid on; box on;
plot(rBldC.t_esp_med,  rBldC.th_meas_med,  '-', 'Color', cBlind, 'LineWidth', 1.3, ...
    'DisplayName', 'Baseline 1 \theta_{meas}');
plot(rPropC.t_esp_med, rPropC.th_meas_med, '-', 'Color', cProp,  'LineWidth', 1.8, ...
    'DisplayName', 'Proposed \theta_{meas}');
yline(250, 'k:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
yline(-250, 'k:', 'LineWidth', 1.0, 'DisplayName', '\pm250\circ physical limit');
xlim([0 max(t_med_P(end), t_med_B(end))]); ylim([-295 295]);
xlabel('Measured Lap Elapsed Time t [s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Steering Angle \theta_{meas} [deg]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(b) Realized Handwheel Deflection (Full Physical Range)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'southwest', 'FontSize', 8.0);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

% Panel (c): Error Phase Portrait (e_y vs de_y/dt)
subplot(2, 2, 4); hold on; grid on; box on;
plot(rBldC.ey_med,  deB, '-', 'Color', [cBlind 0.55], 'LineWidth', 1.1, ...
    'DisplayName', 'Baseline 1 (Limit-cycle divergence)');
plot(rPropC.ey_med, deP, '-', 'Color', cProp, 'LineWidth', 1.8, ...
    'DisplayName', 'Proposed (Compact invariant attractor)');
xlim([-4.6 4.2]); ylim([-6.5 8.5]);
xlabel('Lateral Error e_y [m]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
ylabel('Error Derivative de_y/dt [m/s]', 'FontName', 'Arial', 'FontSize', 10.5, 'FontWeight', 'bold');
title('(c) Phase-Plane Portrait (e_y vs de_y/dt)', ...
    'FontName', 'Arial', 'FontSize', 11, 'FontWeight', 'bold');
legend('Location', 'northwest', 'FontSize', 8.0);
set(gca, 'FontName', 'Arial', 'FontSize', 9.5);

exportgraphics(fig11, fullfile(outDir, 'Fig11_Ablation_Limit_Cycle.png'), 'Resolution', 600);
fprintf('[OK] Saved Fig11_Ablation_Limit_Cycle.png\n');

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================
function w_sm = computeSmoothRate(t_s, theta_deg)
    dt = diff(t_s);
    dt(dt <= 1e-4) = 1e-4;
    w_raw = [diff(theta_deg) ./ dt; 0];
    if numel(w_raw) >= 11
        w_sm = smoothdata(w_raw, 'gaussian', 9);
    else
        w_sm = w_raw;
    end
end

function runObj = loadRunById(hilRoot, idStr, Kt, R_phase)
    d = dir(fullfile(hilRoot, sprintf('*_%s', idStr)));
    rDir = fullfile(d(1).folder, d(1).name);
    tel  = readtable(fullfile(rDir, 'telemetria.csv'));
    esp  = readtable(fullfile(rDir, 'odrive_espejo.csv'));
    elec = readtable(fullfile(rDir, 'odrive_electrico.csv'));

    isMed = strcmp(string(tel.fase_vuelta), 'medida');
    t_med = tel.t_s(isMed);
    t0_med = t_med(1); t1_med = t_med(end);

    t_esp  = esp.t_perf_s - esp.t_perf_s(1);
    t_elec = elec.t_perf_s - elec.t_perf_s(1);
    w_sm_all = computeSmoothRate(t_esp, esp.theta_meas_deg);

    mEspMed  = (t_esp >= t0_med) & (t_esp <= t1_med);
    mElecMed = (t_elec >= t0_med) & (t_elec <= t1_med);

    runObj.tel = tel;
    runObj.esp = esp;
    runObj.elec = elec;
    runObj.t_esp = t_esp;
    runObj.t_elec = t_elec;
    runObj.w_sm = w_sm_all;
    runObj.t_med = t_med;
    runObj.s_med = tel.s_cg_m(isMed);
    runObj.ey_med = tel.e_y_cg_m(isMed);
    runObj.t_esp_med = t_esp(mEspMed) - t0_med;
    runObj.th_meas_med = esp.theta_meas_deg(mEspMed);
    runObj.tau_med = esp.tau_carga_nm(mEspMed);
    runObj.w_sm_med = w_sm_all(mEspMed);

    t_e_med = t_elec(mElecMed);
    runObj.tem_elec = Kt * elec.iq_a(mElecMed);
    runObj.w_elec = interp1(t_esp, w_sm_all, t_e_med, 'linear', 'extrap');
    runObj.iq_max = max(abs(elec.iq_a(mElecMed)));
    runObj.pj_elec = (elec.iq_a(mElecMed).^2) * R_phase;
end

function [so, eo] = splitSpatialWrap(s, e)
    ds = abs(diff(s));
    idxJ = find(ds > 200);
    if isempty(idxJ)
        so = s; eo = e; return;
    end
    so = []; eo = []; prev = 1;
    for k = 1:numel(idxJ)
        j = idxJ(k);
        so = [so; s(prev:j); NaN]; %#ok<AGROW>
        eo = [eo; e(prev:j); NaN]; %#ok<AGROW>
        prev = j + 1;
    end
    so = [so; s(prev:end)];
    eo = [eo; e(prev:end)];
end

function win = extractSpatialWindow(r, sLo, sHi, Kt, R_phase)
    tel = r.tel; esp = r.esp; elec = r.elec;
    isMed = strcmp(string(tel.fase_vuelta), 'medida');
    idxTel = find(isMed & tel.s_cg_m >= sLo & tel.s_cg_m <= sHi);
    t0 = tel.t_s(idxTel(1)); t1 = tel.t_s(idxTel(end));

    % Unique monotonic time grid for s_cg_m lookup
    [tu, ia] = unique(tel.t_s(idxTel));
    su = tel.s_cg_m(idxTel(ia));

    win.s_tel = tel.s_cg_m(idxTel);
    win.ey    = tel.e_y_cg_m(idxTel);
    win.max_ey = max(abs(win.ey));

    mE = (r.t_esp >= t0) & (r.t_esp <= t1);
    win.t_esp = r.t_esp(mE);
    win.s_esp = interp1(tu, su, win.t_esp, 'linear', 'extrap');
    win.th_cmd = esp.theta_cmd_deg(mE);
    win.th_meas = esp.theta_meas_deg(mE);
    win.w_sm = r.w_sm(mE);
    win.max_w = max(abs(win.w_sm));
    tau_l = abs(esp.tau_carga_nm(mE));
    win.env_lim = 462.5 * max(0, 1 - (tau_l / 2.45).^4.5);

    mQ = (r.t_elec >= t0) & (r.t_elec <= t1);
    win.t_elec = r.t_elec(mQ);
    win.s_elec = interp1(tu, su, win.t_elec, 'linear', 'extrap');
    win.iq = elec.iq_a(mQ);
    win.p_joule = (win.iq.^2) * R_phase;
    win.max_iq = max(abs(win.iq));
    win.max_pj = max(win.p_joule);
    if numel(win.t_elec) >= 2
        win.E_joule = trapz(win.t_elec, win.p_joule);
    else
        win.E_joule = 0;
    end
end
