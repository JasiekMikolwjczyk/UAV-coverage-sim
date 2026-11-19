# Scan simulator v2 (Paper 1)

Scan simulator v2 extends the v1 scan simulator (`tools/run_scan_simulation.py`, unchanged) with
literature-calibrated rate-based sensors, a sourced mine population, clutter, N screeners + M
confirmers, corridor screening, queue instrumentation, safety injection and an analytic model. The
SPEC document ("SPEC: scan simulator v2 for Paper 1", now v2.1) is the source of truth. This file documents what is
implemented, including every place where the implementation had to fill a gap in the SPEC (section 9).
Section 11 lists what changed in v2.1.

## 1. How to run

```bash
# one mission (natural run + injection pass); outputs under data/scan_v2_runs/<timestamp>_seed<k>/
python3 tools/run_scan_simulation_v2.py --config configs/scan_v2_paper1.json --seed 1 --preset nominal

# v1-equivalent run (legacy_logistic, reproduces v1 bit-for-bit)
python3 tools/run_scan_simulation_v2.py --config configs/scan_v2_sar_mag_v1equiv.json

# sweeps (SPEC 10), in the order the SPEC asks for
python3 tools/sweep_scan_v2.py --quick                                   # (1)
python3 tools/sweep_scan_v2.py --map-only                                # (2)
python3 tools/sweep_scan_v2.py --groups primary --no-plots               # (3) full grid, seeds 1-30
python3 tools/sweep_scan_v2.py --groups planner_compare                  # corridor vs cross_track (seeds 1-10)
python3 tools/sweep_scan_v2.py --groups projection,survey,imaging,regime,clutter,vehicle,policy   # (4)
python3 tools/sweep_scan_v2.py --groups map_validation                   # (5)
# (6) analytic vs simulation: analytic_vs_sim.csv + analytic_vs_sim_notes.txt in every sweep directory

# tests
python3 -m pytest tests/test_scan_sim_v2.py -q
```

`--workers` sets the process-pool size (default: all cores) and `--seeds 1-30` (or `1,5,9`) sets the seeds.
Sweep outputs go to `data/scan_v2_sweeps/<timestamp>/`, which is git-ignored.

Cost: one Paper 1 mission takes about 1–10 s (more when the confirmers are saturated, because more
simulated time passes). `--map-only` takes about 50 s on 12 cores and `--quick` (24 runs) a few minutes.
The full `primary` group is 11,664 configurations (presets × GPR presets × a × N × M × `on_reject` ×
investigation time) × 30 seeds, so plan it as a multi-day job or reduce `--seeds`.

## 2. Files

| File | Content |
|---|---|
| `tools/scan_sim_v2/model.py` | objects, legacy v1 generation/detection (copied verbatim), rate-based sensors, presets, fusion, power, unused-key warnings |
| `tools/scan_sim_v2/population.py` | residual / doctrinal / scatterable regimes, clutter, injection |
| `tools/scan_sim_v2/geometry.py` | lanes, `lane_abreast` and `corridor` planners, coverage grid, local scans |
| `tools/scan_sim_v2/policies.py` | dispatch policy interface + `priority`, `nearest`, `corridor_first` |
| `tools/scan_sim_v2/simulate.py` | main loop, metrics, queue log, injection pass |
| `tools/scan_sim_v2/analytic.py` | SPEC 9 analytic model and requirements map |
| `tools/scan_sim_v2/outputs.py` | metrics.json, mines.csv/json, queue_log.csv, overview plot |
| `tools/run_scan_simulation_v2.py` | CLI for one mission |
| `tools/sweep_scan_v2.py` | SPEC 10 sweep |
| `configs/sensors_v2.json` | every sensor/preset, every number as `{value, source}` |
| `configs/mine_types_v2.json` | SPEC 4.5.1 type table, depth models, mixes with sources |
| `configs/scan_v2_paper1.json` | Paper 1 base (residual regime) |
| `configs/scan_v2_doctrinal.json`, `configs/scan_v2_scatterable.json` | stress scenarios (the scatterable one has the thermal surface layer) |
| `configs/scan_v2_sar_mag_v1equiv.json` | v1 config in `legacy_logistic` mode (regression) |
| `tests/test_scan_sim_v2.py` | SPEC 11 tests |

## 3. Config reference

### Top level
| Key | Meaning | Default |
|---|---|---|
| `seed` | base seed for all RNG streams (section 6) | 1 |
| `out_dir` | CLI output root | `data/scan_v2_runs` |
| `detection_mode` | `rate_based` (Paper 1) or `legacy_logistic` (v1 reproduction) | `legacy_logistic` |
| `sensors_path`, `mine_types_path` | override library files | `configs/sensors_v2.json`, `configs/mine_types_v2.json` |
| `localization_model` | `{"kind": "static"}` or `{"kind": "zones", "zones": [{x0,x1,y0,y1,t0,t1,std_m}]}` (Paper 2 hook) | static |

### `field`
`x_min, x_max, y_min, y_max` (m); `lane_width` (m; lane centres from `y_min + lane_width/2`);
`grid_res` (coverage grid cell, m); `terrain_margin_m` (grid margin outside the field).

### `mission`
`dt` (s, step), `max_time_s` (time limit, stop reason `mission_time_limit`),
`start_hour` (time of day at t = 0, for the thermal crossover windows; default 12.0).

### `battery`
`reserve_percent`: a drone at or below this stops the mission (SPEC 6.3).
`enforce` (default `true`): when `false`, batteries never stop the mission but energy is still reported.
The Paper 1 configs use `false` (section 9, item 6).
`platforms.<sensor>`: overrides the platform blocks in `sensors_v2.json` (`capacity_Wh`,
`base_power_W`, `speed_quad_coeff`, `payload_power_W`). Power = base + coeff·v² + payload (if on).
Legacy mode reads `battery.<sar|magnetometer>` exactly as v1 does.

### `team`
| Key | Meaning |
|---|---|
| `screening_sensor` | `mag_uav`, `emi_uav`, `gpr_uav_survey`, `generic_screen` (rate_based); `sar` (legacy) |
| `confirming_sensor` | `gpr_uav_confirm`; `null` / `"none"` = survey only (M = 0, every detection is final) |
| `sensor_preset` | screener preset (`pessimistic`/`nominal`/`optimistic`; emi: `ground_reference`/`eth_flight`) |
| `confirmer_preset` | GPR preset; defaults to `sensor_preset` if the GPR has it, else `nominal` |
| `n_screeners`, `n_confirmers` | N, M |
| `screening_mode` | `cross_track` (Paper 1 default since v2.1), `corridor`, or `lane_abreast` (v1 geometry) |
| `segment_length_m` | `corridor` mode segment length (default 10 m) |
| `turn_time_s` | turn between passes/lines (default 3 s) |
| `confirmer_policy` | `priority`, `nearest`, `corridor_first`, or a dotted path `pkg.mod.func` / `pkg.mod:func` |
| `confirmer_idle_spacing_m` | lateral spacing of idle confirmers (M > 1) |
| `scan_size` | `small`/`medium`/`full` = local scan × 0.5/0.75/1.0 (Paper 1: `full`) |
| `screening_overrides` | per-run overrides of the screener (`speed_mps`, `swath_m`, `localization_std_m`, `pd.<class>`) used by sweeps |
| `confirming_overrides` | overrides of the GPR scan geometry or depth model |
| `surface_layer` | `{"sensor": "thermal_uav"|"rgb_uav", "preset": "nominal", "n": 1}` imaging layer flying the lane |

### `generic_screen` (requirements map, SPEC 4.6)
`pd.{metal,low_metal,non_metal}`, `false_alarms_per_m2`, `swath_m`, `speed_mps`, `localization_std_m`.
False alarms are generated as `false_alarm` objects at that density over the field (Pd 1 for this sensor).
Clutter objects are not generated for this sensor.

### `decision`
`confirmed_mine_threshold` (legacy only), `prior_mine_probability` (D2; default 0.5),
`investigation_time_s` (how long an object in the corridor keeps blocking after it was confirmed, or after it was
rejected under `investigate`; default 300 s in rate_based, 0 in legacy; assumption between [MACA] 108 s and
[GICHD06] 1200 s). `on_reject` (SPEC 4.4): `release` (default; a rejected suspicion is released, and a rejected mine
is a confirmation miss) or `investigate` (a rejected suspicion inside the corridor keeps blocking for
`investigation_time_s` and is then resolved on the ground: a mine becomes `found_by_investigation`; outside the
corridor it is released). The v2.0 key `excavation_time_s` is still accepted, mapped to `investigation_time_s` and
warned once (`DeprecatedConfigKeyWarning`).
`sar_suspicious_min` (legacy only). `sar_suspicious_max` and `cleared_threshold` are unused and warn once (D3).

### `tractor`
`width_m`, `safety_margin_m` (corridor W = width + 2·margin = 4.5 m), `safe_distance_m` (vehicle
target = frontier − safe_distance), `max_speed_mps`, `pid.{kp,ki,kd}` (as v1, integral clamp ±50),
`block_mode` (`corridor_ahead` = v1: stop if any unresolved suspicion lies in the corridor between the vehicle and the frontier;
`standoff` = rate_based default: advance up to `standoff_m` before the nearest unresolved or excavating suspicion ahead),
`standoff_m` (default 5 m).

### `clutter` (SPEC 5)
`density_per_m2`, `classes` (weights of `metal_clutter`, `nonmetal_clutter`), `depth_m` (`mean, std, min, max`, clipped normal).

### `population` (rate_based, SPEC 4.5)
| Key | Meaning |
|---|---|
| `regime` | `residual`, `doctrinal`, `scatterable` |
| `density_per_m2` | residual density ρ (kept mines; default 8.5e-7) |
| `mix` / `mix_weights` | residual type mix (`residual_mix`) or explicit weights |
| `hard_target_selection` | keep each candidate with probability 1 − Pd_clearance (EMI ground_reference) |
| `aged_depth_shift` | add U(0, 0.05) m to buried depths |
| `non_metal_fraction` | relabel this fraction of mines as `non_metal` (sensitivity case) |
| `class_filter` | restrict types to these classes (used for `emi_uav/eth_flight`, whose low-metal Pd is unknown) |
| `density_multiplier` | doctrinal: in-row spacing divided by this |
| `doctrinal.{n_ap_rows, at_row_spacing_m, at_in_row_spacing_m, ap_row_spacing_m, ap_in_row_spacing_m, jitter_std_m, start_min_m, start_max_m}` | doctrinal generator |
| `scatterable_type` | `PFM-1` or `POM-2` |
| `scatterable.{density_per_m2, n_footprints, length_m, width_m}` | cassette footprints |

### `injection` (SPEC 4.5.4)
`enabled` (default true for rate_based), `n_inject` (default 20).

### Legacy blocks
`sar`, `magnetometer`, `mine_distribution`, `mine_properties`: exactly as v1, read only in `legacy_logistic` mode.

## 4. Sensor model

`logit Pd = a[class] − k_depth·(depth − 0.05)/0.10`; Pd = 0 beyond `max_depth_m`. Detection is
Bernoulli(Pd) the first time an object lies within swath/2 of the screener's swept track. A
detection creates a suspicion at the true position + N(0, σ) per axis, clipped to the field.
Confirmation is Bernoulli(P(confirm | class, depth) · exp(−½(d/0.5)²)), with d the distance from the
true position to the nearest local-scan line.

**Local GPR scan (SPEC 4.3, v2.1).** In rate_based mode the confirmer scans a square of side
L = max(`min_scan_side_m`, 2·2.5·σ + `swath_m`) centred on the estimated position (σ = localisation std,
1.0 m with the defaults). Lines run along y at spacing `swath_m`·(1 − `transverse_overlap`), from
−L/2 + swath/2 to +L/2 − swath/2, so the swath covers the square exactly. The number of lines is
n = ceil((L − swath)/spacing) + 1 and the path length is n·L + (L − swath): 4.75 m, i.e. 9.5 s at 0.5 m/s, with
the defaults. `scan_size` factors scale L. `local_scan_area_m` and `line_length_m` (the v1 8 × 6 m lawnmower)
are used only in legacy_logistic mode.

**Screening planners (SPEC 6.1).**
* `lane_abreast`: v1 geometry, a formation abreast across the lane flying along it.
* `corridor`: k = ceil(W/C) lateral passes per segment of `segment_length_m`. For even k the formation
  flies an empty return leg, so going from N = 2 (k = 3) to N = 4 (k = 2) does not speed up screening.
* `cross_track` (Paper 1 default): lines run across the corridor (y from lane_y − W/2 to lane_y + W/2 and
  back), with the N screeners side by side along x at sep = swath·(1 − overlap). After each line plus turn
  the block advances N·sep. Frontier speed v_f = N·sep/(W/v + t_turn), proportional to N. The frontier
  is the x up to which the ground is screened (leading boundary of the last completed block). The
  half-disc end caps of each line screen π·r² (r = swath/2) beyond the corridor. The suspicions found
  there are queued and load the confirmers. The numbers and their sources are in
`configs/sensors_v2.json` (they reproduce SPEC 4.3) and in section 7.

## 5. Metrics (`metrics.json`)

**Legacy (v1 meaning):** `elapsed_s`, `mission_complete`, `stop_reason`
(`completed`, `mission_time_limit`, `screener_<i>_battery_reserve_reached`, `surface_<i>_…`, `confirmer_<j>_…`),
`found_mines`, `missed_mines`, `mine_count`, `sar_suspicious`, `magnetometer_verifications` (all over natural mines),
`scanned_area_m2` (coverage-grid cells inside the field), `sar_energy_used_Wh` (screener 0),
`mag_energy_used_Wh` (confirmer 0), `sar_/mag_battery_remaining_percent`, `battery_reserve_percent`.

**Team:** `detection_mode`, `screening_sensor`, `confirming_sensor`, `sensor_preset`, `confirmer_preset`,
`surface_layer`, `screening_mode`, `n_screeners`, `n_confirmers`, `confirmer_policy`,
`projection` (true for `emi_uav`), `area_rate_m2ps`, `screener_swath_m`, `screener_speed_mps`, `corridor_width_m`.

**Safety (natural mines whose true position is inside a lane corridor):** `recall`, `recall_<class>`,
`screening_misses[_<class>]`, `confirmation_misses[_<class>]`, `residual_mines_in_corridor` (not found by the end).
These are `null` when there are no such mines, which is the normal case for `residual`.

**Injection:** `inj_n`, `inj_p_detect` with `_lo`/`_hi` (Wilson 95 %), and per class `inj_n_<c>`,
`inj_detected_<c>`, `inj_p_detect_<c>[_lo|_hi]`, `inj_unscreened_<c>` (never passed under a screener, e.g. the mission stopped),
`inj_screen_miss_<c>` (passed under, not detected), `inj_confirm_miss_<c>` (detected, not confirmed),
`inj_late_<c>` (found after the vehicle reached it), `inj_found_by_investigation_<c>`, plus `inj_elapsed_s`, `inj_mean_vehicle_speed_mps`,
`inj_fleet_energy_Wh`, `inj_n_suspicions`, `inj_stop_reason` from the injection pass.
`residual_risk_per_km` = ρ · W · 1000 · (1 − `inj_p_detect`). "Detected before vehicle arrival" means
found as a mine (confirmed, or at the end of its investigation) at a time `t_found` ≤ the time the vehicle
first reached the mine's x in its lane.

**Population:** `population_regime`, `population_density_per_m2`, `n_mines_natural`, `mines_per_type`,
`mines_per_class`, `mean_depth_per_class`, `n_clutter`.

**Workload:** `n_suspicions` (natural objects), `n_false_alarms` (clutter suspicions),
`false_confirmations`, `precision` = TP/(TP + false confirmations), `false_alarms_per_m2` (per scanned m²),
`n_investigated_rejections` (rejected in-corridor suspicions investigated under `investigate`),
`n_found_by_investigation` (natural mines found that way), `on_reject`, `investigation_time_s`.

**Throughput:** `vehicle_distance_m`, `vehicle_time_s` (time until the last lane ends, i.e. without the
final drain), `mean_vehicle_speed_mps` = distance / vehicle time, `frontier_speed_mps` (primary-layer
cleared length / its active time), `coverage_efficiency` η = v_f · W / (N · swath · speed), and the time
split `tractor_moving_s`, `tractor_wait_confirmation_s`, `tractor_wait_frontier_s`,
`tractor_wait_investigation_confirmed_s` (blocked by a confirmed object under investigation),
`tractor_wait_investigation_s` (blocked by a rejected object under investigation, `investigate` only) and
`tractor_done_waiting_s` (includes the final drain), which sums to `elapsed_s` within dt.
`tractor_wait_excavation_s` is a deprecated alias of `tractor_wait_investigation_confirmed_s` (kept for
one release, not part of the sum).

**Energy:** `screener_energy_Wh`, `surface_energy_Wh`, `confirmer_energy_Wh` (per drone),
`fleet_energy_screening_Wh`, `fleet_energy_confirming_Wh`, `fleet_energy_Wh`,
`energy_per_m2_cleared_Wh` = fleet / (vehicle distance · W).

**Queue:** `confirmer_busy_s` (per confirmer), `confirmer_utilization` = Σ busy / (M · elapsed),
`mean_queue_wait_s` (assignment − arrival), `max_queue_length`, `mean_service_time_s` (travel + scan),
`scv_service_time`, `n_queued`, `arrival_rate_per_s` = queued / primary screening time.

**`calibration`:** for every sensor actually used, the preset and every parameter with its `source`, plus population sources.

**Files:** `queue_log.csv` (one row per suspicion: `object_idx, is_mine, class, depth_m, t_arrival,
t_assigned, t_done, confirmer_id, travel_s, scan_s, in_corridor, confirmed, posterior`), `mines.csv` /
`mines.json` (v1 columns + `is_mine, class, mine_type, injected, screen_detected, confirm_result, outcome,
t_vehicle_reach`; all objects, clutter included), `injected_mines.csv`, `coverage.npy`, `config.json`, `scan_result.png`.
`outcome` ∈ {`true_positive`, `found_by_investigation`, `screening_miss`, `confirmation_miss`, `unresolved`, `false_confirmation`,
`rejected`, `undetected_clutter`}. `posterior` is reporting only: prior ρ_mine / (ρ_mine + ρ_clutter_detected),
updated with P(confirm | mine) (class mix of the population) and P(confirm | clutter).

**Sweep files:** `runs.csv` (one row per run: spec columns + scalar metrics), `summary.csv` (per
configuration: mean, std and 95 % t-interval of recall per class, injected P(detect) per class, vehicle
speed, fleet energy, energy per m², false confirmations, utilisation and more; `projection` column),
`queue_log_all.csv`, `analytic_vs_sim.csv` (analytic value, simulated value and relative difference for
frontier speed, vehicle speed, λ, E[S], utilisation and injected P(detect); cells over 15 % are flagged),
`analytic_vs_sim_notes.txt` (every flagged cell with the model terms involved), `requirements_map.csv`,
`measured_points.csv`, `sweep_manifest.json` (grid, seeds, git commit, dirty flag, base config) and `plots/`.

## 6. RNG streams

| Stream | Use |
|---|---|
| `seed` | natural mines (legacy: v1 order; rate_based: positions first, then type/depth per mine) |
| `seed + 1000` | localisation noise of natural mines (x then y, in detection order; v1 stream) |
| `seed + 2000` | clutter (count, positions, classes, depths) |
| `seed + 2001` | clutter localisation noise |
| `seed + 3000` | natural mines: pre-drawn uniforms (screen, surface layer, confirm) |
| `seed + 3001` | clutter: pre-drawn uniforms |
| `seed + 4000` / `4001` / `4002` | injected mines / their uniforms / their localisation noise |
| `seed + 5000` / `5001` | non-metal relabelling (natural / injected) |

Detection and confirmation use pre-drawn per-object uniforms (`u < p`), so an object's outcome does not
depend on how many other objects exist (SPEC 11.8).

## 7. Literature evidence (SPEC section 3)

### 7.1 Subsurface detection, buried 0–20 cm [OK26]
| Sensor | Ferrous | Non-ferrous metal | Plastic |
|---|---|---|---|
| Handheld EMI (Minelab CTX 3030) | 98.0 % | 72.2 % | 2.3 % |
| Handheld ferrous detector | 95.9 % | 19.5 % | 9.7 % |
| UAV magnetometer (0.2–2 m; 50–100 g items never detected; Pd falls with height) | 40.8 % | 14.0 % | 7.6 % |
| Cart-based GPR | 50.0 % | 80.6 % | 55.6 % |
| UAV GPR (Radarteam Cobra, 0.5 m) | 12.1 % | 11.8 % | 18.2 % |
| UAV thermal (only fresh burials detected) | 0.7 % | 1.4 % | 0.0 % |
| UAV SAR (OneSAR) | no ordnance detected | | |

Other UAV subsurface measurements: UAV magnetometer below 1 m, 100 % on M15, M16 and POMZ [RS24]; UAV metal
detector, 17/18 flush metal targets at 0.068 m²/s [ETH22]. By depth [OK26, Table 8]: shallow metallic
(1–20 cm) handheld EMI 96.2 %, UAV magnetometry 32.3 %; deeper metallic (> 20 cm) 50 % and 25 %; shallow
non-metallic cart GPR 50 %, UAV GPR 25 %; deeper non-metallic: no UAV capability. Subsurface sensors need
< 2 m altitude, ideally < 1 m.

### 7.2 Surface detection [OK26, Table 4]
RGB 94.8 % (±6.9), thermal 83.7 % (±12.2), multispectral 64.0 %, LiDAR 9.6 %.

### 7.3 Dual sensor and false alarms [ITEP04]
| Category | Metal detector Pd | HSTAMIDS alert Pd | HSTAMIDS correct classification Pd |
|---|---|---|---|
| AT-M | 1.00 | 1.00 | 1.00 |
| AP-M | 0.98 | 1.00 | 0.95 |
| AT-LM | 0.29 | 1.00 | 0.93 |
| AP-LM | 0.96 | 0.99 | 0.93 |

False alarms per m²: metal detector 1.023 total (0.598 from indigenous clutter); HSTAMIDS alert 0.884;
HSTAMIDS after GPR classification 0.201. Target mix: AT-M 7 %, AT-LM 31 %, AP-M 35 %, AP-LM 27 %.
Detection counted within 15 cm of the target edge. Other: handheld GPR 97.9 % (1560/1593 mines) at
0.23–0.28 false alarms per m² [GPRREV]; GPR on prepared sites Pd ≈ 100 % at PFA < 0.01 %, in normal ground
Pd ≈ 50 % at PFA ≈ 10 %; buried plastic mines Pd 0.9 at PFA 0.002 [DAN]; dual sensors cut false alarms by up
to 7:1 against a metal detector in live minefields [DANUXO]; in metal-contaminated fields 100–1000 false
alarms per real mine, and minimum-metal mines carry ~1 g of metal, down to 0.1 g [BRU].

### 7.4 Thermal physics [KV02, IRAT]
Buried targets are visible mainly around sunrise and sunset; mines at moderate depth produce no direct
signature; surface roughness creates false spots [KV02]. The contrast of AT mines is large through most
of the day except for the thermal crossover after sunrise and sunset [IRAT].

### 7.5 Mine populations
Residual: 7.1 million m² of post-clearance inspection found ~6 mines [GICHD12], i.e. ~0.85 per km².
Clearance is seldom 100 % (90 % in an easy test field [GICHD05]). Flails throw mines out of position
[MCL06]. Floods and mass movements displace mines [BAS23]. Doctrinal minefields follow FM3342/FM2032
(rows, spacings, outlays); US example density 0.005 per m². POM-2: 464 mines over 4,100–4,200 × 35–65 m
(~0.002 per m²) [POM2]. PFM-1 is surface-laid and scatterable [PFM1]. Types in use in Ukraine: [HRW23].
Test depths: AP 5 cm, AT 10 cm [ITEP04]; seeded field 0–20 cm [OK26].
### 7.6 Investigation time (v2.1)
Time to resolve one alarm on the ground: [MACA] 108 s and [GICHD06] 1200 s per investigated alarm. The default
`investigation_time_s` = 300 s lies between the two (assumption); the sweep uses {108, 300, 1200}. Full
citations for both keys are in SPEC v2.1 section 3.

Full source keys and URLs: SPEC section 3; short forms in `configs/sensors_v2.json` and `configs/mine_types_v2.json`.

## 8. Known v1 defects and how v2 handles them (D1–D10)

| # | Defect | v2 |
|---|---|---|
| D1 | `sar_energy_used_Wh` is one screener only | per-drone energy lists and `fleet_energy_*`; legacy keys keep v1 meaning |
| D2 | odds fusion assumes prior 0.5 | `decision.prior_mine_probability`; posterior odds = odds(p_s)·odds(p_c)/odds(prior); identical to v1 at 0.5 |
| D3 | `sar_suspicious_max`, `cleared_threshold` unused | not implemented; one warning per key |
| D4 | `mine_distribution.mean[1]/std[1]` ignored | kept; one warning per key |
| D5 | no false alarms | clutter process (SPEC 5) |
| D6 | deterministic detection | `rate_based` Bernoulli draws |
| D7 | one "plastic" class | `metal`, `low_metal`, `non_metal` |
| D8 | SAR sees buried mines, 12 m swaths | SAR only in legacy mode; subsurface sensors at ≤ 1 m with ≤ 1 m swath |
| D9 | screening never limiting | `corridor` mode with k lateral passes |
| D10 | unsourced population | sourced regimes and type table |

## 9. Implementation decisions where the SPEC was silent or ambiguous

These are modelling choices. Each one is `assumption`-grade and can be changed in config where noted.

1. **Stream isolation.** The SPEC names `seed + 3000` for detection draws and `seed + 1000` for
   localisation. Natural mines use exactly those streams. Clutter and injected objects get their own
   streams (section 6), so adding clutter does not shift the mines' draws (SPEC 11.8).
2. **Detection geometry (rate_based).** An object is "under" a screener if it lies within swath/2 of the
   segment flown in that step, not of the end-of-step point. With a 1 m swath at 1 m/s, a disc check at
   1 m spacing would leave diamond-shaped holes. Legacy mode keeps v1's disc check.
3. **Corridor planner with even k.** k passes over a segment end where the segment started. The formation
   then flies one extra transit leg of `segment_length` (payload off) to the next segment. For odd k the
   frontier speed is exactly L/(k(L/v + t_turn)) as in the SPEC. For even k it is
   L/(k(L/v + t_turn) + L/v). `geometry.frontier_speed_formula` and the tests use this. This empty leg is why
   v2.1 adds the `cross_track` planner.
4. **Vehicle blocking (`tractor.block_mode`).** v1 stops the vehicle if any unresolved suspicion lies
   anywhere in the corridor between the vehicle and the screeners. Paper 1 configs use `standoff`: the
   vehicle may advance until it is `standoff_m` (5 m, assumption) before the nearest unresolved or
   excavating suspicion in its corridor. This follows the SPEC 0.1 wording ("may only advance onto ground
   that has been screened and in which every suspicion has been resolved"). Blocking uses estimated
   positions in rate_based mode and true positions in legacy mode (as in v1).
5. **End of lane.** In rate_based mode, once every screening layer has finished a lane, the vehicle's
   target becomes the lane end. In v1 the vehicle covers the last `safe_distance` only through PID integral
   wind-up. Without this change the vehicle stalls 20 m before the end when the integral has bled off.
6. **Batteries.** Platform blocks are assumption-only (SPEC 6.3), and the 150 Wh GPR confirmer runs flat in
   saturated Paper 1 missions. The Paper 1 configs set `battery.enforce: false`, i.e. battery swaps are
   assumed and energy is reported as a relative comparison. `enforce: true` restores the SPEC 6.3 stop.
7. **Survey mode (M = 0).** Every detection is final: it is recorded as confirmed at detection time and,
   if it is in the corridor, it blocks for `investigation_time_s`.
8. **Drain.** v1 drains only in-flight confirmations at the end. Rate_based mode also serves the
   suspicions still queued (all of them outside the corridor) so that every suspicion has an outcome.
   This time is counted in `tractor_done_waiting_s`, not in `vehicle_time_s`.
9. **Suspicions outside the corridor.** A screening pass wider than the corridor (C > W) also screens and
   queues objects outside it, as SPEC 6.2 (`corridor_first`) assumes. These suspicions load the
   confirmers but do not block the vehicle.
10. **`emi_uav/eth_flight`.** Low-metal and non-metal Pd are unknown, so the sensor raises an error for
    those classes. Sweeps restrict the population with `class_filter: ["metal"]`. Its clutter Pd uses
    `clutter_a` (metal clutter like ETH22 metal targets; non-metal clutter as OK26 handheld EMI 2.3 %),
    both marked assumption.
11. **`gpr_uav_survey`.** The SPEC gives Pd values; the config stores their logits (logit 0.12, 0.18, 0.227).
12. **GPR local scan geometry.** v2.0 reused the v1 8 m × 6 m lawnmower (about 110 s per alarm). v2.1 ties
    the scan to the localisation error (section 4); the numbers are SPEC 4.3 assumptions.
13. **Residual mix.** The TM-62 series takes 0.5 of the mix (split 0.6/0.4 between TM-62M and TM-62P3) and
    the three buried AP types share the remaining 0.5 equally (assumption; HRW23 only says TM-62 is the
    most frequent). Hard-target selection keeps the configured density ρ as the density of kept mines:
    the count is Poisson(ρ·A), and each mine's type and depth are drawn by rejection with keep
    probability 1 − Pd_clearance.
14. **Doctrinal generator.** The AT block starts at x ~ U(10, 40) m. There are 3 AP rows (FM3342 says
    "several"). Rows run across the lanes. The row orientation angle applies to the row direction. Mines
    outside the field are dropped. Injection in the doctrinal regime draws from AT and AP types with
    weight 0.5 each. All of these are assumptions.
15. **Scatterable generator.** The ellipse length is U(150, 300) m and its orientation U(0, π), both
    assumptions; POM-2's 4 km length would not fit a 200 m field. Mines outside the field are dropped.
16. **Imaging layer.** One drone flies each lane ahead of the primary screeners. The vehicle frontier is
    the minimum over layers. Clutter at depth ≤ 1 cm is seen like surface mines (worst case, assumption).
    Localisation σ is 0.3 m (assumption). `start_hour` sets the time of day for the thermal crossover.
17. **Priority score (rate_based).** All suspicions get the screener's metal Pd at 5 cm (no class
    leakage). Ties break by arrival time, then index, which makes the policy FIFO. Legacy mode uses v1's
    (−p, idx) order.
18. **Recall** is computed over natural mines whose true position lies inside a lane corridor.
19. **Injection outcomes** add `inj_unscreened_<class>` so that mines the screeners never reached (the
    mission stopped) are not counted as screening misses.
20. **Sweep design.** The full Cartesian product of every SPEC 10 factor (presets × GPR presets × a × N ×
    M × regimes × clutter × vehicle × policies × 30 seeds) is far too large. The sweep runs the full
    primary grid (presets × GPR presets × a × N × M) and varies the other factors one at a time around it
    (`regime`, `clutter`, `vehicle`, `policy` groups at N = 4, M = 2, nominal). Edit `build_specs` for other
    designs. `sweep_manifest.json` records the exact specs. `planner_compare` runs corridor vs cross_track at
    N ∈ {2, 3, 4, 5}, nominal presets, a = 0.25, M = 1 (the spec default; the SPEC does not fix M) and always seeds 1–10.
21. **Cross-track frontier.** The SPEC says "rear edge of the last completed block". The implementation
    uses the boundary up to which all ground is screened (block start + N·sep). It differs from the rear
    edge by one block (≤ N·sep), not in speed.
22. **Investigation under `investigate`** applies to rejected suspicions whose *estimated* position is in
    the corridor (the vehicle only knows estimates). A mine whose estimate falls outside the corridor is
    released and can still be a confirmation miss.

## 10. Analytic model (SPEC 9)

`tools/scan_sim_v2/analytic.py` gives frontier speed (exact for the planner), η, N_req, λ (Pd averaged
over the depth and class distribution of kept mines and over the clutter depth distribution), E[S]
(fixed-seed Monte Carlo integration of the travel distance: from the previous scan end with the Erlang C
probability that work is waiting, else from the SPEC 6.2 idle point, which trails the frontier by the
vehicle's lag), ρ, M*, Allen–Cunneen W_q and the predicted vehicle speed. Arrivals use the screened
width (cross-track adds the line end caps, a single wide pass adds its overhang). With `investigate`,
P(detect) is the screening Pd and every in-corridor suspicion blocks for the investigation time. The
speed model is per lane:

T_lane = max(L/v0, L·n_m·E[S]/M) + max(0, R − B)

where R is the resolution lag (confirmation, plus investigation if a blocking object is likely in the lag
window) and B = (safe_distance − standoff)/v0. All approximations are listed in the module docstring.

v2.1 check (3 seeds per cell, a = 0.25, both planners, both `on_reject` values, N 2–5, M 1–8): frontier
speed within 2 %, λ within 1 %, E[S] within 5 % for M ≤ 2 (every `--quick` cell within 2 %), vehicle speed
within 13 %. Known weak spot: with 6–8 confirmers E[S] is underestimated by 10–12 % (idle-position model).
`analytic_vs_sim_notes.txt` ends with a per-cell 5 % check of frontier speed and E[S]. Small-seed runs still
flag injected P(detect) because 60 injected mines give a Wilson interval of about ±0.09.

The requirements map (`requirements_map.csv`) is analytic over the SPEC 4.6 grid with M ∈ {M* − 1, M*, M* + 1},
for both `on_reject` values (column `on_reject`; plots `requirements_map.png` and `requirements_map_investigate.png`).
`measured_points.csv` places the measured sensors on the same axes: Pd per class averaged over the
residual population, detections per m² from clutter, and area rate.

## 11. Changes in v2.1

| Change | Where |
|---|---|
| Local GPR scan tied to σ: square of side max(1 m, 5σ + 0.25 m), 0.25 m swath at 0.5 m/s (~9.5 s per alarm instead of ~110 s) | `sensors_v2.json`, `geometry.local_scan_side` / `local_square_segments`, `simulate.execute_task`, `analytic._scan_template` |
| `decision.on_reject` ∈ {`release`, `investigate`} | `simulate.complete`, outcomes, metrics, analytic |
| `excavation_time_s` → `investigation_time_s` (old key mapped, warned) | configs, `simulate.investigation_time`, analytic |
| `cross_track` planner (Paper 1 default) | `geometry.CrossTrackPlanner`, `make_planner`, `frontier_speed_formula` |
| Analytic: screened width with end caps, Erlang C / idle-point travel model, parallel map | `analytic.py`, `sweep_scan_v2.py` |
| Sweep: `on_reject` × investigation time {108, 300, 1200} in `primary`; `planner_compare`; map per `on_reject`; 5 % check in notes | `sweep_scan_v2.py` |
| Tests 17–19 (+ default/deprecation/investigate checks) | `tests/test_scan_sim_v2.py` |

Test 18 compares `on_reject: release` against `tests/data/v2_0_release_reference.json`. That fixture was
recorded from the code *after* the v2.1 scan-geometry change and *before* the rejection-policy change, with
the quick config pinned to `corridor` mode (N ∈ {2, 4}, M ∈ {1, 2}, seeds 1–3, a = 0.25). The scan geometry
change alters every rate_based result, so the fixture checks that adding `on_reject`, the rename and the
cross-track planner left the `release` behaviour bit-identical. It does not compare against the v2.0
commit itself.
