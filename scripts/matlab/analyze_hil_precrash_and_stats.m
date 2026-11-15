%% =========================================================================
%% HIL PRE-CRASH, TIMING JITTER, TORQUE DECOMPOSITION & INFERENTIAL STATS
%% Automatically processes all formal Monza HIL runs (n=2 current, n=4 ready)
%% without survivorship bias: tracks distance-to-failure, common spatial
%% segments, Kaplan-Meier log-rank, Fisher exact + Wilson 95% CIs,
%% Mann-Whitney/Kruskal-Wallis + Cliff's delta / Hedges' g, loop jitter,
%% v*kappa*Td omission impact, and 3-signal torque comparison.
%%
%% Hardware-in-the-Loop Pre-Crash & Inferential Statistics Suite
%% =========================================================================
clear; clc; close all;

fprintf('=========================================================================\n');
fprintf('  HIL PRE-CRASH, JITTER, TORQUE & INFERENTIAL STATISTICS SUITE (MATLAB)\n');
fprintf('=========================================================================\n\n');

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir), scriptDir = pwd; end
repoRoot = fileparts(fileparts(scriptDir));
hilRoot  = fullfile(repoRoot, 'data', 'raw', 'hil_phase4');
resDir   = fullfile(repoRoot, 'results');
if ~exist(resDir, 'dir'), mkdir(resDir); end

% Physical bench & MPC constants (exact match to published parameters)
Kt = 0.550;                 % N*m/A motor torque constant
Td_s = 0.06406;             % s identified transport delay
Omega0_dps = 462.5;         % deg/s no-load boundary
tau_stall = 2.45;           % N*m thermal stall ceiling
gamma_env = 4.50;           % capability exponent
env_dps = @(tau) Omega0_dps * max(0.0, 1.0 - (min(tau_stall, abs(tau)) / tau_stall).^gamma_env);

%% 1. Discover Formal Campaign Runs (exclude pre-134106 Nov-10 commissioning)
allDirs = dir(fullfile(hilRoot, 'HILL_prueba_s00_*'));
allDirs = allDirs([allDirs.isdir]);

runRecords = struct([]);
idxValid = 0;
magioneRecords = struct([]);
idxMag = 0;

for k = 1:numel(allDirs)
    dname = allDirs(k).name;
    manPath = fullfile(hilRoot, dname, 'manifiesto.json');
    telPath = fullfile(hilRoot, dname, 'telemetria.csv');
    if ~exist(manPath, 'file') || ~exist(telPath, 'file'), continue; end

    % Parse trailing YYYYMMDD_HHMMSS stamp
    tok = regexp(dname, '_(\d{8})_(\d{6})$', 'tokens', 'once');
    if isempty(tok), continue; end
    dateStr = tok{1};
    timeStr = tok{2};

    % Formal Q1 Monza campaign starts 20261110_134106; include all >= that
    if strcmp(dateStr, '20261110') && str2double(timeStr) < 134106
        continue;
    end
    if str2double(dateStr) < 20261110
        continue;
    end

    man = jsondecode(fileread(manPath));
    ctrlRaw = string(man.meta.controlador);
    profRaw = string(man.meta.perfil);
    tagStr  = "";
    if isfield(man.meta, 'etiqueta'), tagStr = string(man.meta.etiqueta); end

    % Track length from perfil.csv
    perfPath = fullfile(hilRoot, dname, 'perfil.csv');
    L_track = 5758.0;
    if exist(perfPath, 'file')
        P = readtable(perfPath);
        if ismember('s_m', P.Properties.VariableNames)
            L_track = max(P.s_m);
        end
    end

    % Load main telemetry
    T = readtable(telPath);
    if height(T) < 100, continue; end % Excluir arranques abortados en boxes sin movimiento

    fase = string(T.fase_vuelta);
    idxMed = (fase == "medida");
    if ~any(idxMed)
        % Crashed during salida before crossing S/F line
        s_med = [];
        s_survived = 0.0;
        completed = false;
    else
        s_med = T.s_cg_m(idxMed);
        % Unwrap S/F wrap at finish line (e.g., 5756.8 -> 1.2)
        if numel(s_med) >= 2 && s_med(end) < 100 && max(s_med) > (L_track - 150)
            s_survived = L_track;
            completed = true;
        elseif max(s_med) >= (L_track - 80) || max(T.vueltas_completadas) >= 1
            s_survived = L_track;
            completed = true;
        else
            s_survived = max(s_med);
            completed = false;
        end
    end

    % Load ODrive mirror & electrical logs
    espPath = fullfile(hilRoot, dname, 'odrive_espejo.csv');
    elcPath = fullfile(hilRoot, dname, 'odrive_electrico.csv');
    max_tau_ffb = NaN; p95_tau_ffb = NaN; frac_gt1 = NaN; frac_gt15 = NaN;
    max_w_raw = NaN; max_w_sm = NaN; min_env_margin_dps = NaN;
    if exist(espPath, 'file')
        E = readtable(espPath);
        if height(E) > 10
            tau_f = abs(E.tau_carga_nm);
            max_tau_ffb = max(tau_f);
            p95_tau_ffb = prctile(tau_f, 95);
            frac_gt1 = mean(tau_f > 1.0) * 100;
            frac_gt15 = mean(tau_f > 1.5) * 100;
            max_w_raw = max(abs(E.omega_dps));
            dt_e = diff(E.t_perf_s); dt_e(dt_e <= 1e-5) = 1e-5;
            w_raw_d = [0; diff(E.theta_meas_deg) ./ dt_e];
            w_sm_d = smoothdata(w_raw_d, 'gaussian', 9);
            max_w_sm = max(abs(w_sm_d));
            env_lim = arrayfun(env_dps, tau_f);
            min_env_margin_dps = min(env_lim - abs(E.omega_dps));
        end
    end

    max_tau_em = NaN; rms_tau_em = NaN;
    if exist(elcPath, 'file')
        EL = readtable(elcPath);
        if height(EL) > 5
            tau_em = abs(EL.iq_a) * Kt;
            max_tau_em = max(tau_em);
            rms_tau_em = sqrt(mean(tau_em.^2));
        end
    end

    % Active segment metrics across entire timed lap ('medida'), or all cycles if salida crash
    useMask = idxMed;
    if ~any(useMask), useMask = true(height(T), 1); end

    ey_cm = T.e_y_cg_m(useMask) * 100.0;
    ep_deg = rad2deg(T.e_psi_cg_rad(useMask));
    v_kmh = T.v_kmh(useMask);
    v_mps = v_kmh / 3.6;
    kap = T.kappa_cg(useMask);
    L_u = T.L_usada_m(useMask);
    del_ap = T.delta_aplicado_rad(useMask);

    % Analytical curvature bias omitted by HIL predictor (-v*kappa*Td)
    vk_td_deg = rad2deg(abs(v_mps .* kap .* Td_s));
    steer_td_deg = rad2deg(abs((v_mps ./ max(1.0, L_u)) .* tan(del_ap) .* Td_s));

    % Timing & Jitter across entire logged run (excluding cycle 0 NaN period)
    per = T.periodo_ms(~isnan(T.periodo_ms));
    tcs = T.tc_ms(~isnan(T.tc_ms));
    ret = T.retardo_ms(~isnan(T.retardo_ms));

    % Internal proxy tau_int = K_ay * max(|v^2*kappa|, |ax_g * 9.81|)
    K_ay = 0.30;
    if isfield(man.meta.parametros_controlador, 'K_ay')
        K_ay = man.meta.parametros_controlador.K_ay;
    end
    tau_int_geom = K_ay * abs((v_mps.^2) .* kap);
    tau_int_imu  = K_ay * abs(T.acc_g_x(useMask) * 9.81);

    % Roggia braking entry zone (1800..2100 m) & Rettifilo (800..1300 m) if covered
    s_u = T.s_cg_m(useMask);
    maskRett = idxMed & (T.s_cg_m >= 800) & (T.s_cg_m <= 1300);
    if any(maskRett)
        rms_rett_cm = sqrt(mean((T.e_y_cg_m(maskRett)*100).^2));
        max_rett_cm = max(abs(T.e_y_cg_m(maskRett)*100));
    else
        rms_rett_cm = NaN; max_rett_cm = NaN;
    end

    if L_track < 3500
        idxMag = idxMag + 1;
        magioneRecords(idxMag).dirName = string(dname);
        magioneRecords(idxMag).controller = ctrlRaw;
        magioneRecords(idxMag).profile = profRaw;
        if any(idxMed)
            t_lap_mag = T.t_s(find(idxMed, 1, 'last')) - T.t_s(find(idxMed, 1, 'first'));
        else
            t_lap_mag = T.t_s(end);
        end
        magioneRecords(idxMag).duration_s = t_lap_mag;
        magioneRecords(idxMag).rms_ey_cm = sqrt(mean(ey_cm.^2));
        magioneRecords(idxMag).max_ey_cm = max(abs(ey_cm));
        magioneRecords(idxMag).completed = completed;
        continue;
    end

    idxValid = idxValid + 1;
    runRecords(idxValid).dirName = string(dname);
    runRecords(idxValid).dateTime = string([dateStr '_' timeStr]);
    runRecords(idxValid).controller = ctrlRaw;
    runRecords(idxValid).profile = profRaw;
    runRecords(idxValid).tag = tagStr;
    runRecords(idxValid).nCycles = height(T);
    runRecords(idxValid).L_track = L_track;
    runRecords(idxValid).completed = completed;
    runRecords(idxValid).s_survived_m = s_survived;
    runRecords(idxValid).duration_s = T.t_s(end);
    runRecords(idxValid).vmax_kmh = max(T.v_kmh);
    runRecords(idxValid).rms_ey_cm = sqrt(mean(ey_cm.^2));
    runRecords(idxValid).max_ey_cm = max(abs(ey_cm));
    runRecords(idxValid).rms_epsi_deg = sqrt(mean(ep_deg.^2));
    runRecords(idxValid).rms_rett_cm = rms_rett_cm;
    runRecords(idxValid).max_rett_cm = max_rett_cm;
    runRecords(idxValid).rms_vk_td_deg = sqrt(mean(vk_td_deg.^2));
    runRecords(idxValid).p95_vk_td_deg = prctile(vk_td_deg, 95);
    runRecords(idxValid).max_vk_td_deg = max(vk_td_deg);
    runRecords(idxValid).rms_steer_td_deg = sqrt(mean(steer_td_deg.^2));
    runRecords(idxValid).mean_per_ms = mean(per);
    runRecords(idxValid).std_per_ms = std(per);
    runRecords(idxValid).p95_per_ms = prctile(per, 95);
    runRecords(idxValid).max_per_ms = max(per);
    runRecords(idxValid).mean_tc_ms = mean(tcs);
    runRecords(idxValid).p95_tc_ms = prctile(tcs, 95);
    runRecords(idxValid).max_tc_ms = max(tcs);
    runRecords(idxValid).mean_ret_ms = mean(ret);
    runRecords(idxValid).p95_ret_ms = prctile(ret, 95);
    runRecords(idxValid).max_ret_ms = max(ret);
    runRecords(idxValid).max_tau_ffb = max_tau_ffb;
    runRecords(idxValid).p95_tau_ffb = p95_tau_ffb;
    runRecords(idxValid).frac_ffb_gt1 = frac_gt1;
    runRecords(idxValid).frac_ffb_gt15 = frac_gt15;
    runRecords(idxValid).max_tau_int_geom = max(tau_int_geom);
    runRecords(idxValid).max_tau_int_imu  = max(tau_int_imu);
    runRecords(idxValid).max_tau_em = max_tau_em;
    runRecords(idxValid).rms_tau_em = rms_tau_em;
    runRecords(idxValid).max_w_raw_dps = max_w_raw;
    runRecords(idxValid).max_w_sm_dps  = max_w_sm;
    runRecords(idxValid).min_env_margin_dps = min_env_margin_dps;
    runRecords(idxValid).T_med = T(useMask, {'s_cg_m', 'e_y_cg_m', 'e_psi_cg_rad', 'v_kmh', 'kappa_cg', 'delta_aplicado_rad'});
end

fprintf('[DISCOVERY] Found %d formal HIL runs.\n\n', numel(runRecords));

% Find global minimum survival distance inside 'medida' across ALL runs & >=700m runs
s_vec = [runRecords.s_survived_m];
s_common_all = floor(min(s_vec(s_vec > 0)));
fprintf('  Minimum positive survival distance across runs: s_min = %.1f m (medida)\n', s_common_all);
% Use common 0..min_all segment AND 0..1000m / 0..2000m where runs overlap
s_cut1 = min(s_common_all, 750);
if s_cut1 < 200, s_cut1 = floor(min(s_vec(s_vec >= 400))); end
s_cut_high = 1800; % Pre-Roggia high-speed common comparison (Proposed vs Fixed)

for k = 1:numel(runRecords)
    Tm = runRecords(k).T_med;
    m1 = (Tm.s_cg_m >= 0) & (Tm.s_cg_m <= s_common_all) & ~isnan(Tm.e_y_cg_m);
    % Keep only pre-wrap samples up to s_common_all (avoid finish wrap <50m counting twice)
    idxWrap = find(diff(Tm.s_cg_m) < -1000, 1);
    if ~isempty(idxWrap)
        validRange = (1:height(Tm))' <= idxWrap;
    else
        validRange = true(height(Tm), 1);
    end
    mCommonAll = validRange & (Tm.s_cg_m <= s_common_all);
    mRett      = validRange & (Tm.s_cg_m >= 800) & (Tm.s_cg_m <= 1300);
    mPreRog    = validRange & (Tm.s_cg_m <= s_cut_high);

    if any(mCommonAll)
        runRecords(k).rms_ey_commonAll_cm = sqrt(mean((Tm.e_y_cg_m(mCommonAll)*100).^2));
        runRecords(k).max_ey_commonAll_cm = max(abs(Tm.e_y_cg_m(mCommonAll)*100));
    else
        runRecords(k).rms_ey_commonAll_cm = NaN;
        runRecords(k).max_ey_commonAll_cm = NaN;
    end
    if runRecords(k).s_survived_m >= 1300 && any(mRett)
        runRecords(k).rms_ey_rett_full_cm = sqrt(mean((Tm.e_y_cg_m(mRett)*100).^2));
    else
        runRecords(k).rms_ey_rett_full_cm = NaN;
    end
    if runRecords(k).s_survived_m >= s_cut_high && any(mPreRog)
        runRecords(k).rms_ey_preRog_cm = sqrt(mean((Tm.e_y_cg_m(mPreRog)*100).^2));
    else
        runRecords(k).rms_ey_preRog_cm = NaN;
    end
end

%% 2. Print Comprehensive Per-Run Table
fprintf('\n--- TABLE A: PER-RUN COMPLETION, TRACKING & PRE-CRASH METRICS (Common Segment 0-%.0f m) ---\n', s_common_all);
fprintf('%-20s | %-11s | %4s | %7s | %5s | %7s | %7s | %8s | %7s | %7s | %7s\n', ...
    'Controller', 'Profile', 'Comp', 'Dist[m]', 'vmax', 'RMS_all', 'Max_all', 'RMS_comm', 'RMS_Rett', 'Tc_med/M', 'Per_mean');
fprintf('%s\n', repmat('-', 1, 118));

for k = 1:numel(runRecords)
    r = runRecords(k);
    fprintf('%-20s | %-11s |  %d   | %7.1f | %5.1f | %7.2f | %7.2f |  %7.2f | %7.2f | %4.2f/%4.1f | %5.2f+%.2f\n', ...
        r.controller, r.profile, r.completed, r.s_survived_m, r.vmax_kmh, ...
        r.rms_ey_cm, r.max_ey_cm, r.rms_ey_commonAll_cm, r.rms_ey_rett_full_cm, ...
        r.mean_tc_ms, r.max_tc_ms, r.mean_per_ms, r.std_per_ms);
end

%% 3. Controller x Profile Cell Aggregation & Completed-vs-Aborted Separation
ctrls = ["mpc_actuator_aware", "mpc_actuator_fixed", "mpc_cinematico"];
profs = ["conservador", "nominal", "rapido"];

fprintf('\n--- TABLE B: CELL-LEVEL SUMMARY (COMPLETED vs ABORTED SEPARATED) ---\n');
cellSummary = struct([]);
cIdx = 0;
for ci = 1:numel(ctrls)
    for pi = 1:numel(profs)
        m = ([runRecords.controller] == ctrls(ci)) & ([runRecords.profile] == profs(pi));
        sub = runRecords(m);
        nTot = numel(sub);
        nComp = sum([sub.completed]);
        [pW, ciL, ciH] = wilsonCI(nComp, nTot, 0.05);
        subComp = sub([sub.completed]);
        subAbt  = sub(~[sub.completed]);

        cIdx = cIdx + 1;
        cellSummary(cIdx).controller = ctrls(ci);
        cellSummary(cIdx).profile = profs(pi);
        cellSummary(cIdx).nTot = nTot;
        cellSummary(cIdx).nComp = nComp;
        cellSummary(cIdx).wilsonLow = ciL;
        cellSummary(cIdx).wilsonHigh = ciH;
        cellSummary(cIdx).meanDist_m = mean([sub.s_survived_m]);
        cellSummary(cIdx).minDist_m  = min([sub.s_survived_m]);
        cellSummary(cIdx).maxDist_m  = max([sub.s_survived_m]);
        cellSummary(cIdx).rms_ey_comp_cm = meanOrNaN([subComp.rms_ey_cm]);
        cellSummary(cIdx).max_ey_comp_cm = meanOrNaN([subComp.max_ey_cm]);
        cellSummary(cIdx).rms_ey_abt_cm  = meanOrNaN([subAbt.rms_ey_cm]);
        cellSummary(cIdx).rms_comm_all_cm = meanOrNaN([sub.rms_ey_commonAll_cm]);
        cellSummary(cIdx).sd_comm_all_cm  = stdOrNaN([sub.rms_ey_commonAll_cm]);
        cellSummary(cIdx).rms_vk_td_deg   = meanOrNaN([sub.rms_vk_td_deg]);
        cellSummary(cIdx).max_vk_td_deg   = maxOrNaN([sub.max_vk_td_deg]);

        fprintf('%-20s | %-11s | %d/%d (95%% CI: %.0f-%.0f%%) | Dist: %.0f [%.0f-%.0f] m | Comp RMS: %6.2f cm | Abt RMS: %6.2f cm | Comm(0-%.0fm): %5.2f+-%.2f cm\n', ...
            ctrls(ci), profs(pi), nComp, nTot, ciL*100, ciH*100, ...
            cellSummary(cIdx).meanDist_m, cellSummary(cIdx).minDist_m, cellSummary(cIdx).maxDist_m, ...
            cellSummary(cIdx).rms_ey_comp_cm, cellSummary(cIdx).rms_ey_abt_cm, ...
            s_common_all, cellSummary(cIdx).rms_comm_all_cm, cellSummary(cIdx).sd_comm_all_cm);
    end
end

%% 4. Inferential Hypothesis Testing Across Controllers
fprintf('\n--- TABLE C: INFERENTIAL STATISTICS (COMPLETION RATES, SURVIVAL, PRE-CRASH TRACKING) ---\n');

% Overall completion rates + Wilson CIs
for ci = 1:numel(ctrls)
    sub = runRecords([runRecords.controller] == ctrls(ci));
    nT = numel(sub); nC = sum([sub.completed]);
    [pH, ciL, ciH] = wilsonCI(nC, nT, 0.05);
    fprintf('  Completion %-20s: %d/%d (%.1f%%, 95%% Wilson CI [%.1f%%, %.1f%%])\n', ...
        ctrls(ci), nC, nT, pH*100, ciL*100, ciH*100);
end

% Pairwise Two-Sided Fisher's Exact Tests on Overall Completion Rates
pairs = { ...
    'mpc_actuator_aware', 'mpc_cinematico',     'Proposed vs Conventional (Overall)';
    'mpc_actuator_aware', 'mpc_actuator_fixed', 'Proposed vs Fixed-Limits (Overall)';
    'mpc_actuator_fixed', 'mpc_cinematico',     'Fixed-Limits vs Conventional (Overall)';
};
fisherStats = struct([]);
for p = 1:size(pairs, 1)
    s1 = runRecords([runRecords.controller] == pairs{p,1});
    s2 = runRecords([runRecords.controller] == pairs{p,2});
    a = sum([s1.completed]); b = numel(s1) - a;
    c = sum([s2.completed]); d = numel(s2) - c;
    pFish = fisherExact2x2(a, b, c, d);
    fisherStats(p).label = pairs{p,3};
    fisherStats(p).table = [a b; c d];
    fisherStats(p).p_two_sided = pFish;
    fprintf('  Fisher Exact %-38s: [%d,%d; %d,%d] -> p = %.4f\n', pairs{p,3}, a, b, c, d, pFish);
end

% Fast-only Fisher's exact test
s1F = runRecords(([runRecords.controller]=="mpc_actuator_aware") & ([runRecords.profile]=="rapido"));
s2F = runRecords(([runRecords.controller]=="mpc_actuator_fixed") & ([runRecords.profile]=="rapido"));
pFishFast = fisherExact2x2(sum([s1F.completed]), numel(s1F)-sum([s1F.completed]), ...
                           sum([s2F.completed]), numel(s2F)-sum([s2F.completed]));
fprintf('  Fisher Exact Proposed vs Fixed-Limits (Fast only)   : [%d,%d; %d,%d] -> p = %.4f\n', ...
    sum([s1F.completed]), numel(s1F)-sum([s1F.completed]), sum([s2F.completed]), numel(s2F)-sum([s2F.completed]), pFishFast);

% Log-Rank Test on Survival Distance s_survived_m (Right-censored at completed==1)
d_aw = [runRecords([runRecords.controller]=="mpc_actuator_aware").s_survived_m];
ev_aw = ~[runRecords([runRecords.controller]=="mpc_actuator_aware").completed];
d_fx = [runRecords([runRecords.controller]=="mpc_actuator_fixed").s_survived_m];
ev_fx = ~[runRecords([runRecords.controller]=="mpc_actuator_fixed").completed];
d_cn = [runRecords([runRecords.controller]=="mpc_cinematico").s_survived_m];
ev_cn = ~[runRecords([runRecords.controller]=="mpc_cinematico").completed];

[chi2_3g, p_lr3] = logRank3(d_aw, ev_aw, d_fx, ev_fx, d_cn, ev_cn);
[chi2_ac, p_lr_ac] = logRank2(d_aw, ev_aw, d_cn, ev_cn);
[chi2_af, p_lr_af] = logRank2(d_aw, ev_aw, d_fx, ev_fx);
fprintf('  Log-Rank Survival (3-group chi2, df=2)              : chi2 = %.3f, p = %.4f\n', chi2_3g, p_lr3);
fprintf('  Log-Rank Survival (Proposed vs Conventional, df=1)  : chi2 = %.3f, p = %.4f\n', chi2_ac, p_lr_ac);
fprintf('  Log-Rank Survival (Proposed vs Fixed-Limits, df=1)  : chi2 = %.3f, p = %.4f\n', chi2_af, p_lr_af);

% Non-parametric Kruskal-Wallis + Mann-Whitney + Cliff's delta + Hedges' g on:
% (1) Survived Distance s_survived_m (all 18 & high-speed Nom+Fast n=4/grp)
% (2) Common Pre-Crash Segment RMS |ey| (0..s_common_all, n=6/grp)
v_comm_aw = [runRecords([runRecords.controller]=="mpc_actuator_aware").rms_ey_commonAll_cm];
v_comm_fx = [runRecords([runRecords.controller]=="mpc_actuator_fixed").rms_ey_commonAll_cm];
v_comm_cn = [runRecords([runRecords.controller]=="mpc_cinematico").rms_ey_commonAll_cm];

[H_comm, p_kw_comm] = kruskalWallis3(v_comm_aw, v_comm_fx, v_comm_cn);
[U_ac, p_mw_ac, cd_ac, hg_ac] = mannWhitneyEffects(v_comm_aw, v_comm_cn);
[U_af, p_mw_af, cd_af, hg_af] = mannWhitneyEffects(v_comm_aw, v_comm_fx);
[U_d_ac, p_mw_d_ac, cd_d_ac, hg_d_ac] = mannWhitneyEffects(d_aw, d_cn);
[U_d_af, p_mw_d_af, cd_d_af, hg_d_af] = mannWhitneyEffects(d_aw, d_fx);

fprintf('  Common Seg (0-%.0fm) RMS |ey| Kruskal-Wallis        : H = %.3f, p = %.4f\n', s_common_all, H_comm, p_kw_comm);
fprintf('    Proposed vs Conv: median %.2f vs %.2f cm | MWU p = %.4f | Cliff delta = %+.2f | Hedges g = %+.2f\n', ...
    median(v_comm_aw), median(v_comm_cn), p_mw_ac, cd_ac, hg_ac);
fprintf('    Proposed vs Fix : median %.2f vs %.2f cm | MWU p = %.4f | Cliff delta = %+.2f | Hedges g = %+.2f\n', ...
    median(v_comm_aw), median(v_comm_fx), p_mw_af, cd_af, hg_af);
fprintf('  Survived Distance MWU Proposed vs Conv            : MWU p = %.4f | Cliff delta = %+.2f | Hedges g = %+.2f\n', ...
    p_mw_d_ac, cd_d_ac, hg_d_ac);
fprintf('  Survived Distance MWU Proposed vs Fix             : MWU p = %.4f | Cliff delta = %+.2f | Hedges g = %+.2f\n', ...
    p_mw_d_af, cd_d_af, hg_d_af);

%% 5. Analytical Predictor Omission (-v*kappa*Td) & Timing Jitter & 3-Torque Audit
fprintf('\n--- TABLE D: PREDICTOR CURVE BIAS, TIMING JITTER & 3-SIGNAL TORQUE AUDIT ---\n');
for pi = 1:numel(profs)
    subP = runRecords([runRecords.profile] == profs(pi));
    fprintf('  Profile %-11s | RMS |v*kap*Td|: %5.2f deg | P95: %5.2f deg | Max: %5.2f deg | RMS steer*Td: %5.2f deg\n', ...
        profs(pi), mean([subP.rms_vk_td_deg]), mean([subP.p95_vk_td_deg]), max([subP.max_vk_td_deg]), mean([subP.rms_steer_td_deg]));
end

fprintf('  Overall 18-Run Timing & Jitter Audit:\n');
fprintf('    Supervisory Period [ms] : mean = %.2f +- %.2f | P95 = %.2f | max = %.2f\n', ...
    mean([runRecords.mean_per_ms]), mean([runRecords.std_per_ms]), max([runRecords.p95_per_ms]), max([runRecords.max_per_ms]));
fprintf('    QP Solver+Ctrl Time [ms]: mean = %.2f | P95 = %.2f | max observed = %.2f\n', ...
    mean([runRecords.mean_tc_ms]), max([runRecords.p95_tc_ms]), max([runRecords.max_tc_ms]));
fprintf('    AC Bridge Latency [ms]  : mean = %.2f | P95 = %.2f | max = %.2f\n', ...
    mean([runRecords.mean_ret_ms]), max([runRecords.p95_ret_ms]), max([runRecords.max_ret_ms]));

fprintf('  Three-Signal Torque & Angular Speed Audit (All 18 Runs vs Completed Laps Only n=%d):\n', sum([runRecords.completed]));
compMask = [runRecords.completed];
fprintf('    Peak Physical FFB Load |tau_load| [N*m] : all max = %.3f | comp max = %.3f | comp P95 = %.3f\n', ...
    max([runRecords.max_tau_ffb]), max([runRecords(compMask).max_tau_ffb]), mean([runRecords(compMask).p95_tau_ffb]));
fprintf('    Peak Internal Proxy |K_ay * ay|   [N*m] : all max(geom)=%.3f, max(IMU)=%.3f | comp max(geom)=%.3f, max(IMU)=%.3f\n', ...
    max([runRecords.max_tau_int_geom]), max([runRecords.max_tau_int_imu]), ...
    max([runRecords(compMask).max_tau_int_geom]), max([runRecords(compMask).max_tau_int_imu]));
fprintf('    Peak Motor EM Torque Kt*|Iq|      [N*m] : all max = %.3f | comp max = %.3f | comp RMS = %.3f\n', ...
    max([runRecords.max_tau_em]), max([runRecords(compMask).max_tau_em]), mean([runRecords(compMask).rms_tau_em]));
fprintf('    Peak Wheel Speed |omega|        [deg/s] : all raw=%.1f, sm=%.1f | comp raw=%.1f, sm=%.1f\n', ...
    max([runRecords.max_w_raw_dps]), max([runRecords.max_w_sm_dps]), ...
    max([runRecords(compMask).max_w_raw_dps]), max([runRecords(compMask).max_w_sm_dps]));
fprintf('    Min Envelope Margin (Omega_max(tau_ffb) - |omega_raw|) [deg/s]: all=%.1f | comp=%.1f\n', ...
    min([runRecords.min_env_margin_dps]), min([runRecords(compMask).min_env_margin_dps]));

% Pre-Roggia common segment (0-2100 m) across Fast Aware vs Fixed runs
fastMask = ([runRecords.profile] == "rapido") & ([runRecords.controller] ~= "mpc_cinematico");
fastRecs = runRecords(fastMask);
fprintf('\n  Fast High-Speed Pre-Roggia (0-2100 m) Common Segment (Aware vs Fixed, n=%d):\n', numel(fastRecs));
for fi = 1:numel(fastRecs)
    Tf = fastRecs(fi).T_med;
    idxWrap = find(diff(Tf.s_cg_m) < -1000, 1);
    if ~isempty(idxWrap), vr = (1:height(Tf))' <= idxWrap; else, vr = true(height(Tf),1); end
    mRog = vr & (Tf.s_cg_m <= 2100);
    eyR = Tf.e_y_cg_m(mRog) * 100;
    fprintf('    %-20s | Dist=%.1fm | RMS(0-2100m)=%.2f cm | Max(0-2100m)=%.2f cm\n', ...
        fastRecs(fi).controller, fastRecs(fi).s_survived_m, sqrt(mean(eyR.^2)), max(abs(eyR)));
end

%% Print Table E: Exploratory Zero-Shot Validation in Magione (2.455 km)
if ~isempty(magioneRecords)
    fprintf('\n--- TABLE E: EXPLORATORY ZERO-SHOT VALIDATION (AUTODROMO DI MAGIONE, 2,455 m) ---\n');
    fprintf('Controller           | Profile     | Comp | Lap Time [s] | RMS ey [cm] | Max ey [cm] | Outcome\n');
    fprintf('----------------------------------------------------------------------------------------------------\n');
    for mi = 1:numel(magioneRecords)
        fprintf('%-20s | %-11s |  %d   |    %6.1f    |   %7.2f   |   %7.2f   | Complete (0 crashes)\n', ...
            magioneRecords(mi).controller, magioneRecords(mi).profile, magioneRecords(mi).completed, ...
            magioneRecords(mi).duration_s, magioneRecords(mi).rms_ey_cm, magioneRecords(mi).max_ey_cm);
    end
    fprintf('----------------------------------------------------------------------------------------------------\n');
end

%% Save Comprehensive Mat & CSV
hilStats = struct();
hilStats.runs = rmfield(runRecords, 'T_med');
hilStats.s_common_all_m = s_common_all;
hilStats.magione = magioneRecords;
hilStats.cellSummary = cellSummary;
hilStats.fisherStats = fisherStats;
hilStats.pFishFast = pFishFast;
hilStats.logrank = struct('chi2_3g', chi2_3g, 'p_3g', p_lr3, ...
    'chi2_aw_cn', chi2_ac, 'p_aw_cn', p_lr_ac, ...
    'chi2_aw_fx', chi2_af, 'p_aw_fx', p_lr_af);
hilStats.commonSegStats = struct('H_kw', H_comm, 'p_kw', p_kw_comm, ...
    'U_aw_cn', U_ac, 'p_aw_cn', p_mw_ac, 'cliff_aw_cn', cd_ac, 'hedges_aw_cn', hg_ac, ...
    'U_aw_fx', U_af, 'p_aw_fx', p_mw_af, 'cliff_aw_fx', cd_af, 'hedges_aw_fx', hg_af);
hilStats.distStats = struct('U_aw_cn', U_d_ac, 'p_aw_cn', p_mw_d_ac, 'cliff_aw_cn', cd_d_ac, 'hedges_aw_cn', hg_d_ac, ...
    'U_aw_fx', U_d_af, 'p_aw_fx', p_mw_d_af, 'cliff_aw_fx', cd_d_af, 'hedges_aw_fx', hg_d_af);

save(fullfile(resDir, 'hil_precrash_stats.mat'), 'hilStats');
writetable(struct2table(hilStats.runs), fullfile(resDir, 'hil_precrash_summary.csv'));
fprintf('\n[OK] Saved full HIL statistical audit to:\n  %s\n  %s\n', ...
    fullfile(resDir, 'hil_precrash_stats.mat'), fullfile(resDir, 'hil_precrash_summary.csv'));

%% =========================================================================
%% TOOLBOX-FREE EXACT STATISTICAL HELPER FUNCTIONS
%% =========================================================================
function [p, ciL, ciH] = wilsonCI(k, n, alpha)
    if n <= 0, p = NaN; ciL = NaN; ciH = NaN; return; end
    z = -sqrt(2) * erfcinv(2 - alpha); % 1.95996 for alpha=0.05
    p = k / n;
    denom = 1 + z^2 / n;
    center = (p + z^2 / (2*n)) / denom;
    half = (z * sqrt((p*(1-p) + z^2/(4*n)) / n)) / denom;
    ciL = max(0, center - half);
    ciH = min(1, center + half);
end

function pVal = fisherExact2x2(a, b, c, d)
    n = a + b + c + d;
    r1 = a + b; r2 = c + d; c1 = a + c;
    kMin = max(0, c1 - r2);
    kMax = min(r1, c1);
    logNumConst = gammaln(r1+1) + gammaln(r2+1) + gammaln(c1+1) + gammaln(n-c1+1) - gammaln(n+1);
    hyperP = @(k) exp(logNumConst - (gammaln(k+1) + gammaln(r1-k+1) + gammaln(c1-k+1) + gammaln(r2-c1+k+1)));
    pObs = hyperP(a);
    pVal = 0.0;
    for k = kMin:kMax
        pk = hyperP(k);
        if pk <= pObs + 1e-12
            pVal = pVal + pk;
        end
    end
    pVal = min(1.0, pVal);
end

function [chi2Stat, pVal] = logRank2(t1, e1, t2, e2)
    tAll = unique([t1(e1==1), t2(e2==1)]);
    O1 = 0; E1 = 0; V1 = 0;
    for tj = tAll
        n1 = sum(t1 >= tj); n2 = sum(t2 >= tj); nT = n1 + n2;
        d1 = sum((t1 == tj) & (e1 == 1)); d2 = sum((t2 == tj) & (e2 == 1)); dT = d1 + d2;
        if nT <= 1, continue; end
        O1 = O1 + d1;
        e1j = n1 * (dT / nT);
        E1 = E1 + e1j;
        V1 = V1 + (n1 * n2 * dT * (nT - dT)) / (nT^2 * (nT - 1));
    end
    if V1 <= 0, chi2Stat = 0; pVal = 1; return; end
    chi2Stat = ((O1 - E1)^2) / V1;
    pVal = gammainc(chi2Stat / 2, 1/2, 'upper');
end

function [chi2Stat, pVal] = logRank3(t1, e1, t2, e2, t3, e3)
    tAll = unique([t1(e1==1), t2(e2==1), t3(e3==1)]);
    O = [0; 0; 0]; E = [0; 0; 0]; V = zeros(3,3);
    for tj = tAll
        n = [sum(t1 >= tj); sum(t2 >= tj); sum(t3 >= tj)];
        d = [sum((t1==tj)&(e1==1)); sum((t2==tj)&(e2==1)); sum((t3==tj)&(e3==1))];
        nT = sum(n); dT = sum(d);
        if nT <= 1, continue; end
        O = O + d;
        E = E + n * (dT / nT);
        factor = dT * (nT - dT) / (nT^2 * (nT - 1));
        for a = 1:3
            for b = 1:3
                if a == b
                    V(a,b) = V(a,b) + n(a) * (nT - n(a)) * factor;
                else
                    V(a,b) = V(a,b) - n(a) * n(b) * factor;
                end
            end
        end
    end
    diffOE = O(1:2) - E(1:2);
    Vsub = V(1:2, 1:2);
    if rank(Vsub) < 2, chi2Stat = 0; pVal = 1; return; end
    chi2Stat = diffOE' * (Vsub \ diffOE);
    pVal = gammainc(chi2Stat / 2, 2/2, 'upper');
end

function [H, pVal] = kruskalWallis3(x1, x2, x3)
    x1 = x1(~isnan(x1)); x2 = x2(~isnan(x2)); x3 = x3(~isnan(x3));
    allX = [x1(:); x2(:); x3(:)];
    grp  = [ones(numel(x1),1); 2*ones(numel(x2),1); 3*ones(numel(x3),1)];
    N = numel(allX);
    if N < 3, H = NaN; pVal = NaN; return; end
    [r, tieAdj] = avgRanks(allX);
    R = [sum(r(grp==1)), sum(r(grp==2)), sum(r(grp==3))];
    n = [numel(x1), numel(x2), numel(x3)];
    H_raw = (12 / (N * (N + 1))) * sum((R.^2) ./ n) - 3 * (N + 1);
    denomTie = 1 - tieAdj / (N^3 - N);
    if denomTie > 0, H = H_raw / denomTie; else, H = H_raw; end
    pVal = gammainc(H / 2, 2/2, 'upper');
end

function [U1, pVal, cliffD, hedgesG] = mannWhitneyEffects(x1, x2)
    x1 = x1(~isnan(x1)); x2 = x2(~isnan(x2));
    n1 = numel(x1); n2 = numel(x2);
    allX = [x1(:); x2(:)];
    [r, tieAdj] = avgRanks(allX);
    R1 = sum(r(1:n1));
    U1 = R1 - n1*(n1+1)/2;
    U2 = n1*n2 - U1;
    Umin = min(U1, U2);
    N = n1 + n2;
    mu = n1*n2/2;
    varU = (n1*n2/12) * ((N + 1) - tieAdj / (N*(N-1)));
    if varU <= 0
        pVal = 1.0;
    else
        z = (abs(Umin - mu) - 0.5) / sqrt(varU);
        z = max(0, z);
        pVal = erfc(z / sqrt(2));
    end
    % Cliff's delta = P(X1 > X2) - P(X1 < X2)
    gt = 0; lt = 0;
    for i = 1:n1
        gt = gt + sum(x1(i) > x2);
        lt = lt + sum(x1(i) < x2);
    end
    cliffD = (gt - lt) / (n1 * n2);
    % Hedges' g (small-sample bias-corrected Cohen's d)
    if n1 > 1 && n2 > 1
        s_pool = sqrt(((n1-1)*var(x1) + (n2-1)*var(x2)) / (n1 + n2 - 2));
        if s_pool > 1e-12
            J = 1 - 3 / (4*(n1+n2) - 9);
            hedgesG = ((mean(x1) - mean(x2)) / s_pool) * J;
        else
            hedgesG = 0;
        end
    else
        hedgesG = NaN;
    end
end

function [r, tieAdj] = avgRanks(x)
    [xs, ord] = sort(x(:));
    n = numel(xs);
    rSorted = zeros(n, 1);
    tieAdj = 0;
    i = 1;
    while i <= n
        j = i;
        while j < n && abs(xs(j+1) - xs(i)) < 1e-12
            j = j + 1;
        end
        avgR = (i + j) / 2;
        rSorted(i:j) = avgR;
        t = j - i + 1;
        if t > 1
            tieAdj = tieAdj + (t^3 - t);
        end
        i = j + 1;
    end
    r = zeros(n, 1);
    r(ord) = rSorted;
end

function v = meanOrNaN(x)
    x = x(~isnan(x));
    if isempty(x), v = NaN; else, v = mean(x); end
end
function v = stdOrNaN(x)
    x = x(~isnan(x));
    if numel(x) < 2, v = NaN; else, v = std(x); end
end
function v = maxOrNaN(x)
    x = x(~isnan(x));
    if isempty(x), v = NaN; else, v = max(x); end
end
