%% generate_fig1_architecture.m
% Generates Figure 1: Multi-rate Hardware-in-the-Loop (HIL) communication and
% physical bench architecture integrating actual bench photographs + hardware specs.
% Standalone MATLAB script for Figure 1 compilation (600 DPI).
% Output: figures/final/Fig1_HIL_Bench_Architecture.png (600 DPI).

clear; close all; clc;

scriptDir = fileparts(mfilename('fullpath'));
repoRoot  = fileparts(fileparts(scriptDir));
outDir    = fullfile(repoRoot, 'figures', 'final');
photoDir  = fullfile(repoRoot, 'figures', 'bench');
if ~exist(outDir, 'dir'), mkdir(outDir); end

imgOverPath  = fullfile(photoDir, 'bench_overview_hil_20261111.jpg');
imgClosePath = fullfile(photoDir, 'bench_closeup_wheel_20261111.jpg');
assert(exist(imgOverPath,  'file')==2, 'Missing overview photo: %s', imgOverPath);
assert(exist(imgClosePath, 'file')==2, 'Missing close-up photo: %s', imgClosePath);

figW = 1440; figH = 1200;
fig = figure('Units', 'pixels', 'Position', [40 40 figW figH], 'Color', 'w');
set(fig, 'PaperPositionMode', 'auto');

%% ========================================================================
%  PANEL (a): MULTI-RATE HIL ARCHITECTURE SCHEMATIC (TOP 58.5% HEIGHT)
%  ========================================================================
posA = [0.015 0.375 0.970 0.585];
axA = axes('Parent', fig, 'Position', posA, 'XLim', [0 1], 'YLim', [0 1], 'Visible', 'off');
hold(axA, 'on');
axAspectA = (posA(3) * figW) / (posA(4) * figH);

text(axA, 0.50, 1.022, '(a) Multi-Rate Hardware-in-the-Loop (HIL) Communication & Control Architecture', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Arial', 'FontSize', 12.5, 'FontWeight', 'bold', 'Color', [0.08 0.08 0.08]);

% --- LAYER BACKGROUNDS ---
% Layer 1 (Top): y = 0.700 .. 0.975
rectangle(axA, 'Position', [0.010 0.700 0.980 0.275], 'Curvature', 0.025, ...
    'FaceColor', [0.937 0.965 0.996], 'EdgeColor', [0.18 0.44 0.72], 'LineWidth', 1.4);
text(axA, 0.022, 0.956, '1. VEHICLE SIMULATION PLANT (WINDOWS HOST)', ...
    'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', [0.10 0.30 0.56]);

% Layer 2 (Middle): y = 0.345 .. 0.595
rectangle(axA, 'Position', [0.010 0.345 0.980 0.250], 'Curvature', 0.025, ...
    'FaceColor', [0.935 0.976 0.949], 'EdgeColor', [0.15 0.50 0.30], 'LineWidth', 1.4);
text(axA, 0.022, 0.577, '2. PREDICTIVE CONTROLLER & MULTI-RATE BRIDGE (PYTHON HOST)', ...
    'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', [0.10 0.38 0.22]);

% Layer 3 (Bottom): y = 0.018 .. 0.255
rectangle(axA, 'Position', [0.010 0.018 0.980 0.237], 'Curvature', 0.025, ...
    'FaceColor', [0.996 0.953 0.922], 'EdgeColor', [0.76 0.33 0.06], 'LineWidth', 1.4);
text(axA, 0.022, 0.237, '3. PHYSICAL DIRECT-DRIVE STEERING TESTBENCH (24.3 V DC / ODRIVE FOC @ 8 kHz)', ...
    'FontName', 'Arial', 'FontSize', 9.5, 'FontWeight', 'bold', 'Color', [0.58 0.22 0.03]);

% --- LAYER 1 MODULES ---
cB = [0.16 0.42 0.70];
drawModBox(axA, [0.030 0.718 0.285 0.155], cB, 'Assetto Corsa v1.16.4', { ...
    'Giulietta QV | Monza (5.79 km)', ...
    'DirectInput Steering (JOY=1, 900\circ)', ...
    'Vehicle Physics & Tire Solver'}, 'tex');
drawModBox(axA, [0.358 0.718 0.285 0.155], cB, 'Windows Shared Memory', { ...
    'acpmf\_physics / graphics / static', ...
    'Vehicle State (v, x, z, \psi, a_y)', ...
    'Native FFB Signal (finalFF @ 100 Hz)'}, 'tex');
drawModBox(axA, [0.685 0.718 0.285 0.155], cB, 'vJoy Device #1 (Virtual HID)', { ...
    'Axis X \leftarrow \theta_{meas} (~200 Hz)', ...
    'Axis Y, ZRot \leftarrow Gas / Brake (20 Hz)', ...
    'DirectInput loop without quantization'}, 'tex');

% --- LAYER 2 MODULES ---
cG = [0.12 0.47 0.28];
drawModBox(axA, [0.030 0.392 0.285 0.158], cG, 'Actuator-Aware MPC (20 Hz)', { ...
    'Augmented State [e_y, e_\psi, \delta_a, d\delta_a/dt]^T', ...
    'Smith Predictor (T_d = 64.1 ms)', ...
    'Envelope \Omega_{max}(\tau_{load}) | OSQP (N=15)'}, 'tex');
drawModBox(axA, [0.358 0.392 0.285 0.158], cG, 'Longitudinal & Load Estimator', { ...
    'Speed PID \rightarrow Gas/Brake (20 Hz)', ...
    'Load Est. (100 Hz, f_c = 10 Hz LPF)', ...
    'Clipped FFB Load |\tau_{load}| \leq 1.0 N\cdotm'}, 'tex');
drawModBox(axA, [0.685 0.392 0.285 0.158], cG, 'Real-Time Bench Mirror (~200 Hz)', { ...
    'T_m = 5 ms Serial / HID Worker', ...
    'Interpolated Setpoint (\theta_{cmd}, v_{ff})', ...
    'Logs Encoder (\theta_{meas}, \omega) & I_q (10 Hz)'}, 'tex');

% --- LAYER 3 MODULES ---
cO = [0.74 0.31 0.04];
drawModBox(axA, [0.030 0.046 0.255 0.155], cO, '24.3 V DC PSU & Shunt', { ...
    'Regulated Industrial DC Supply', ...
    'Active Regenerative Brake Shunt', ...
    'Bus Overvoltage Protection'}, 'tex');
drawModBox(axA, [0.345 0.046 0.305 0.155], cO, 'MKS XDrive Mini (ODrive v0.5.6)', { ...
    '8 kHz FOC Position & Current Control', ...
    'USB HID FFB Stack + ASCII CDC Port', ...
    'Additive torque\_ff Injection'}, 'tex');
drawModBox(axA, [0.708 0.046 0.262 0.155], cO, 'BLDC Motor + 2880 CPR Enc.', { ...
    'Direct-Drive (K_t = 0.55 N\cdotm/A)', ...
    'R = 0.534 \Omega | J = 0.0131 kg\cdotm^2', ...
    'Gear-Coupled Rotary (0.125\circ/count)'}, 'tex');

% --- ORTHOGONAL SIGNAL ARROWS (DEDICATED CHANNELS) ---
drawPolyArrow(axA, [0.315 0.358], [0.795 0.795], [0.12 0.35 0.62], 1.6, '', axAspectA);
drawPolyArrow(axA, [0.825 0.825 0.172 0.172], [0.873 0.910 0.910 0.873], [0.12 0.35 0.62], 1.6, '', axAspectA);
drawPill(axA, 0.500, 0.910, 'DirectInput JOY=1: \theta_{meas} (~200 Hz) & Pedals (20 Hz)', ...
    [0.08 0.28 0.54], [0.95 0.97 1.0], [0.18 0.44 0.72]);

drawPolyArrow(axA, [0.405 0.405 0.172 0.172], [0.718 0.665 0.665 0.550], cG, 1.6, '', axAspectA);
drawPill(axA, 0.290, 0.665, 'Vehicle State (20 Hz)', [0.06 0.34 0.18], [0.96 0.99 0.97], cG);

drawPolyArrow(axA, [0.500 0.500], [0.718 0.550], cG, 1.6, '', axAspectA);
drawPill(axA, 0.500, 0.630, 'finalFF (100 Hz)', [0.06 0.34 0.18], [0.96 0.99 0.97], cG);

drawPolyArrow(axA, [0.585 0.585 0.755 0.755], [0.550 0.630 0.630 0.718], cB, 1.6, '', axAspectA);
drawPill(axA, 0.675, 0.630, 'Pedals (20 Hz)', [0.08 0.28 0.54], [0.95 0.97 1.0], cB);

drawPolyArrow(axA, [0.878 0.878], [0.550 0.718], cB, 1.6, '', axAspectA);
drawPill(axA, 0.878, 0.645, 'Axis X (~200 Hz)', [0.08 0.28 0.54], [0.95 0.97 1.0], cB);

drawPolyArrow(axA, [0.172 0.172 0.785 0.785], [0.392 0.365 0.365 0.392], [0.06 0.34 0.18], 1.6, '', axAspectA);
drawPill(axA, 0.455, 0.365, '\theta_{cmd} = i_s \delta_{cmd} (i_s = 19.49, 20 Hz)', ...
    [0.05 0.30 0.16], [0.94 0.98 0.95], cG);

drawPolyArrow(axA, [0.440 0.440], [0.392 0.201], [0.64 0.14 0.14], 1.7, '', axAspectA);
drawPill(axA, 0.335, 0.300, 'USB HID #2 (SDL2): \tau_{load} (100 Hz)', ...
    [0.58 0.08 0.08], [1.0 0.96 0.96], [0.75 0.20 0.20]);

drawPolyArrow(axA, [0.855 0.855 0.575 0.575], [0.392 0.300 0.300 0.201], [0.58 0.24 0.04], 1.7, 'both', axAspectA);
drawPill(axA, 0.745, 0.300, 'USB CDC (COM3, 115200 bps): p/f (~200 Hz), I_q (10 Hz)', ...
    [0.52 0.20 0.02], [1.0 0.97 0.94], cO);

drawPolyArrow(axA, [0.285 0.345], [0.123 0.123], cO, 1.7, '', axAspectA);
text(axA, 0.315, 0.148, '24.3 V', 'HorizontalAlignment', 'center', ...
    'FontName', 'Arial', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', [0.55 0.22 0.03]);

drawPolyArrow(axA, [0.650 0.708], [0.148 0.148], cO, 1.6, '', axAspectA);
text(axA, 0.679, 0.172, 'I_{abc}', 'HorizontalAlignment', 'center', ...
    'FontName', 'Arial', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', [0.55 0.22 0.03]);

drawPolyArrow(axA, [0.708 0.650], [0.096 0.096], cO, 1.6, '', axAspectA);
text(axA, 0.679, 0.070, 'ABZ', 'HorizontalAlignment', 'center', ...
    'FontName', 'Arial', 'FontSize', 8.8, 'FontWeight', 'bold', 'Color', [0.55 0.22 0.03]);

%% ========================================================================
%  PANEL (b): CLOSED-LOOP HIL WORKSTATION PHOTO (BOTTOM LEFT)
%  ========================================================================
img1 = imread(imgOverPath);
% Gently crop left edge (sleeve/margin) while preserving full AC monitor, PC, wheel & enclosure
img1 = img1(5:762, 62:1020, :);
[h1, w1, ~] = size(img1);

axB = axes('Parent', fig, 'Position', [0.020 0.020 0.355 0.295]);
image(axB, img1);
axis(axB, 'image', 'off');
hold(axB, 'on');
% Clean dark frame around photograph
rectangle(axB, 'Position', [1 1 w1-1 h1-1], 'EdgeColor', [0.22 0.26 0.32], 'LineWidth', 1.6);

title(axB, '(b) Closed-Loop HIL Bench & Physics Display', ...
    'FontName', 'Arial', 'FontSize', 10.8, 'FontWeight', 'bold', 'Color', [0.08 0.08 0.08]);

%% ========================================================================
%  PANEL (c): DIRECT-DRIVE ACTUATOR & HANDWHEEL CLOSE-UP (BOTTOM MIDDLE)
%  ========================================================================
img2 = imread(imgClosePath);
% Crop out ceiling strip & lower cardboard floor, keeping full wheel + rotor + belt + enclosure
img2 = img2(225:895, 110:768, :);
[h2, w2, ~] = size(img2);

axC = axes('Parent', fig, 'Position', [0.390 0.020 0.245 0.295]);
image(axC, img2);
axis(axC, 'image', 'off');
hold(axC, 'on');
rectangle(axC, 'Position', [1 1 w2-1 h2-1], 'EdgeColor', [0.22 0.26 0.32], 'LineWidth', 1.6);

title(axC, '(c) Actuator, Encoder & Wheel Close-Up', ...
    'FontName', 'Arial', 'FontSize', 10.8, 'FontWeight', 'bold', 'Color', [0.08 0.08 0.08]);

%% ========================================================================
%  PANEL (d): VERIFIED HARDWARE SPECIFICATIONS CARD (BOTTOM RIGHT)
%  ========================================================================
axD = axes('Parent', fig, 'Position', [0.650 0.020 0.335 0.295], ...
    'XLim', [0 1], 'YLim', [0 1], 'Visible', 'off');
hold(axD, 'on');

text(axD, 0.50, 1.032, '(d) Verified Bench & Control Specifications', ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Arial', 'FontSize', 10.8, 'FontWeight', 'bold', 'Color', [0.08 0.08 0.08]);

rectangle(axD, 'Position', [0.01 0.01 0.98 0.97], 'Curvature', 0.04, ...
    'FaceColor', [0.953 0.973 0.996], 'EdgeColor', [0.18 0.44 0.72], 'LineWidth', 1.4);

specs = {
    '[1] Motor:',     'Direct-drive BLDC, K_t = 0.550 N\cdotm/A, R = 0.534 \Omega';
    '[2] Driver:',    'MKS XDrive Mini (ODrive v0.5.6, 8 kHz FOC)';
    '[3] Encoder:',   'Gear-driven rotary, 2,880 CPR (0.125\circ/count)';
    '[4] Power Bus:', '24.3 V DC regulated supply + brake shunt';
    '[5] USB Buses:', 'COM3 (ASCII 115200 bps) + ODrive-Wheel HID';
    '[6] Dynamics:',  'J = 0.0131 kg\cdotm^2, \Omega_0 = 462.5\circ/s, T_d = 64.1 ms';
    '[7] Steering:',  'Mechanical stops \pm250\circ, i_s = 19.49, MPC 20 Hz'
};
ys = linspace(0.86, 0.13, size(specs, 1));
for k = 1:size(specs, 1)
    if mod(k, 2) == 0
        rectangle(axD, 'Position', [0.025 ys(k)-0.058 0.950 0.116], 'Curvature', 0.02, ...
            'FaceColor', [0.915 0.948 0.988], 'EdgeColor', 'none');
    end
    text(axD, 0.045, ys(k), specs{k, 1}, 'FontName', 'Arial', 'FontSize', 9.0, ...
        'FontWeight', 'bold', 'Color', [0.10 0.25 0.48], 'VerticalAlignment', 'middle');
    text(axD, 0.285, ys(k), specs{k, 2}, 'FontName', 'Arial', 'FontSize', 8.8, ...
        'Color', [0.12 0.14 0.18], 'Interpreter', 'tex', 'VerticalAlignment', 'middle');
end

outPath = fullfile(outDir, 'Fig1_HIL_Bench_Architecture.png');
exportgraphics(fig, outPath, 'Resolution', 600);
fprintf('[OK] Saved Figure 1 architecture + bench photos: %s\n', outPath);

%% ========================================================================
%  HELPER FUNCTIONS
%  ========================================================================
function drawModBox(ax, pos, hdrCol, titleStr, linesCell, interpMode)
    x = pos(1); y = pos(2); w = pos(3); h = pos(4);
    hdrH = 0.038;
    rectangle(ax, 'Position', [x y w h], 'Curvature', 0.08, ...
        'FaceColor', [1 1 1], 'EdgeColor', hdrCol, 'LineWidth', 1.4);
    rectangle(ax, 'Position', [x y+h-hdrH w hdrH], 'Curvature', 0.06, ...
        'FaceColor', hdrCol, 'EdgeColor', hdrCol, 'LineWidth', 1.0);
    rectangle(ax, 'Position', [x y+h-hdrH w hdrH*0.45], ...
        'FaceColor', hdrCol, 'EdgeColor', 'none');
    text(ax, x + w/2, y + h - hdrH/2, titleStr, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontName', 'Arial', 'FontSize', 9.0, ...
        'FontWeight', 'bold', 'Color', 'w', 'Interpreter', 'none');
    nL = numel(linesCell);
    bodyTop = y + h - hdrH - 0.015;
    bodyBot = y + 0.014;
    if nL == 1
        yPos = (bodyTop + bodyBot)/2;
    else
        yPos = linspace(bodyTop - 0.008, bodyBot + 0.008, nL);
    end
    for i = 1:nL
        text(ax, x + w/2, yPos(i), linesCell{i}, 'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', 'FontName', 'Arial', 'FontSize', 8.1, ...
            'Color', [0.14 0.16 0.20], 'Interpreter', interpMode);
    end
end

function drawPolyArrow(ax, xs, ys, col, lw, mode, axAspect)
    plot(ax, xs, ys, '-', 'Color', col, 'LineWidth', lw);
    addArrowHead(ax, xs(end-1), ys(end-1), xs(end), ys(end), col, axAspect);
    if strcmpi(mode, 'both')
        addArrowHead(ax, xs(2), ys(2), xs(1), ys(1), col, axAspect);
    end
end

function addArrowHead(ax, x1, y1, x2, y2, col, axAspect)
    dx = (x2 - x1) * axAspect;
    dy = (y2 - y1);
    L = hypot(dx, dy);
    if L < 1e-9, return; end
    ux = dx / L; uy = dy / L;
    vx = -uy; vy = ux;
    headLen = 0.016; headW = 0.0075;
    px = [x2, x2 - (headLen*ux - headW*vx)/axAspect, x2 - (headLen*ux + headW*vx)/axAspect];
    py = [y2, y2 - (headLen*uy - headW*vy),           y2 - (headLen*uy + headW*vy)];
    patch(ax, px, py, col, 'EdgeColor', col);
end

function drawPill(ax, x, y, txt, txtCol, bgCol, edgeCol)
    text(ax, x, y, txt, 'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', 'FontName', 'Arial', 'FontSize', 7.8, ...
        'FontWeight', 'bold', 'Color', txtCol, 'BackgroundColor', bgCol, ...
        'EdgeColor', edgeCol, 'LineWidth', 0.8, 'Margin', 2.2, 'Interpreter', 'tex');
end
