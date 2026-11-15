# MATLAB Data Analysis & Benchmark Reproduction Routines

This directory contains the standalone MATLAB (`.m`) routines to process raw experimental data, perform analytical system identification, solve the convex QP MPC formulation in simulation, and execute statistical hypothesis testing on Hardware-in-the-Loop (HIL) telemetry datasets.

---

## 1. Core Reproduction Pipeline

To reproduce the four methodological phases of the paper, execute the scripts sequentially in MATLAB (R2023b or newer):

| Phase | Script Name | Input Data Source | Output / Artifacts Generated |
| :---: | :--- | :--- | :--- |
| **Phase 1** | `process_phase1_bench_data.m` | `data/raw/bench_phase1/` (135 CSV logs from physical bench tests 1 to 5) | `results/phase1_actuator_params.mat`<br>• Inertia & friction ($J, \tau_c, b$)<br>• Transport delay ($\bar{T}_d$)<br>• Dynamic torque constant ($K_t$)<br>• Small-signal frequency response ($\omega_n, \zeta$) |
| **Phase 2** | `process_phase2_capacity_envelope.m` | `results/phase1_actuator_params.mat` + Test 5 load-speed logs | `results/phase2_envelope_model.mat`<br>• Parameter fit for voltage-plateau power-law ($\gamma = 4.50, \tau_{\mathrm{stall}} = 2.45\text{ N}\cdot\text{m}$, $R^2_{\mathrm{val}} = 0.9521$) |
| **Phase 3** | `run_phase3_mpc_formulation_and_sil.m` | Synthetic double-lane-change reference trajectory ($v_x = 25\text{ m/s}$) | `results/phase3_mpc_sil_results.mat`<br>• Comparative ablation between unconstrained kinematic MPC, rate-saturated MPC, and proposed capacity-aware MPC (Table 3 in paper) |
| **Phase 4** | `process_phase4_hil_validation.m` | `data/raw/hil_phase4/` (Full-lap telemetries from Monza and Magione circuits) | `results/phase4_hil_validation_results.mat`<br>• Full-lap lateral tracking error ($e_y$ RMSE, MAE, max)<br>• Steering work ($W_{\delta}$) and actuator power depletion |
| **Phase 4** | `analyze_hil_precrash_and_stats.m` | `data/raw/hil_phase4/` | `results/hil_precrash_stats.mat`<br>• Common pre-crash evaluation window ($s \in [0, 480]\text{ m}$)<br>• Non-parametric tests (Kruskal–Wallis, Mann–Whitney $U$, Cliff's $\delta$) |

---

## 2. Auxiliary Visualization & Metric Scripts

| Script Name | Purpose / Function |
| :--- | :--- |
| `plot_tracking_error_and_energy_metrics.m` | Generates comparative boxplots and time-series histograms for lateral error and mechanical steering energy. |
| `tune_phase3_mpc_parameters.m` | Executes offline parameter grid sweeps for state weights ($q_y, q_\psi, r_\delta, r_{\Delta\delta}$) and slack penalties. |
| `analyze_phase4_hil_run.m` | Single-run deep dive visualizer plotting steering angle $\delta_{\mathrm{act}}$, commanded $\delta_{\mathrm{cmd}}$, load torque $\tau_a$, and lateral error $e_y$. |
| `generate_phase1_figures_en.m` | Compiles publication-ready figures for Phase 1 actuator identification at 600 DPI. |
| `generate_phase4_figures_en.m` | Compiles publication-ready figures for Phase 4 HIL benchmark and zero-shot transfer at 600 DPI. |

---

## 3. MATLAB Environment Requirements

- **MATLAB Version:** R2023b or later (compatible with R2022a+).
- **Required Toolboxes:**
  - Optimization Toolbox (`quadprog` or standalone OSQP interface)
  - Statistics and Machine Learning Toolbox (`kruskalwallis`, `ranksum`, boxplots)
  - Signal Processing Toolbox (`fft`, `butter`, `filtfilt`)
