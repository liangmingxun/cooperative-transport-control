# Experimental conditions

The scene settings cover the five experimental comparisons in the manuscript. The standalone examples evaluate these settings on deterministic inputs; integrating them with a physical platform requires that platform's measured states and execution interface.

Section numbers refer to the accompanying English manuscript.

## Shared setup

| Quantity | Setting |
| --- | --- |
| Path | Clockwise circle with entry/exit transitions and an exit straight |
| Circle radius | 0.70 m |
| Support formation | Equilateral triangle, side length 0.30 m |
| Nominal progress velocity | 0.100 m/s |
| Geometry and execution configuration | `config/geometry` |

### Method interfaces

CR-A uses the same `M2a + R1` configuration in both the derating and compound-disturbance conditions. Reference-velocity bounds are fixed at their nominal initial values; local candidate generation, common-reference fusion and the proposed lower-layer controller are retained. Both scene interfaces obtain this configuration through `crAForScenario`.

All cooperative LR-NM conditions use `M2bController` with the parameters in `config/methods/lr_nm.json`. Its local reference generator and nonlinear-mapping tracking law produce the raw progress-channel input. Local candidates are averaged arithmetically for the load reference. The shared-reference lower-layer condition uses `PaperNM`.

For Proposed, CR-A, LR-NM and VP-PD, the controller output passes through acceleration/deceleration limiting, measured-velocity Euler command construction and execution-interval projection. Available limits are supplied by the caller using the same platform rules for each compared method. A prescribed capacity change is applied through these platform inputs and the wheel limiter. LR-NM's algorithmic reference and mapping intervals remain fixed as configured; execution bounds are separate inputs. A one-point example uses the nominal limit values, not a reconstructed physical trajectory.

The planar settings in `config/geometry/execution.json` apply to all methods: robot-specific longitudinal/lateral/heading feedback, relative-pose gains 0.25/0.85/0.70, positive yaw-feedforward scales [0.780, 0.780810219, 0.850009371], negative scales [0.940, 0.951221879, 0.940], and preview times [0.05, 0, 0.05] s in both directions. These settings belong to the common platform execution layer. Final wheel-speed and acceleration/deceleration limits use `WheelCommandLimiter`. Startup reference trajectories and initial states are supplied by the calling platform; the example programs are not hardware experiment runners.

## Section 5.2: Normal-condition comparison

The proposed method, LR-NM and NS-B use the common circular path, nominal velocity 0.100 m/s and 0.30 m support formation without imposed derating or compound disturbances. The normal-condition progress-velocity state bounds are `[-0.15, 0.58] m/s`. Method parameters are stored in `config/methods`.

NS-B uses uniform nominal progress velocity followed by planar support tracking. Its quintic startup ramp lasts 2.5 s. The baseline uses the shared planar calibration. Its fleet command-scale rule has a 0.150 m/s wheel-rim command threshold and a minimum allowed scale of 0.70; the same scale is applied to all six wheel demands and the reference-progress update. The baseline motion and fleet-scaling settings are in `config/methods/ns_b.json`; planar settings come from `config/geometry/execution.json`. The platform nominal wheel capability is 0.160 m/s in the common scene settings.

`scenario_example normal_conditions` calls the proposed upper/candidate/lower interfaces, complete cooperative LR-NM and `NominalSpeedBaseline` at a supplied steady-motion example state. It prints the reference velocities and wheel demands. This entry requires the geometry-enabled build.

## Section 5.3.1: Mechanism and performance under single-robot derating and recovery

The `derating_recovery` entry selects the 0.85 profile in `config/scenarios/derating_recovery.json`.

| Quantity | Setting |
| --- | --- |
| Affected robot | Robot2 |
| Capacity ratio | 0.85 |
| Nominal / derated wheel-rim velocity limit | 0.160 / 0.136 m/s |
| Derating / recovery trigger | Actual load progress 1.50 / 3.00 m |
| Derating / recovery transition duration | 1.00 / 1.00 s |
| Acceleration / deceleration limit ratios during derating | 0.70 / 0.70 |

This experiment examines the proposed method's regulation mechanism and compares the Proposed, CR-A and LR-NM responses. The entry provides the limit schedule and controller settings. CR-A uses the shared `M2a + R1` interface.

## Section 5.3.2: Completion and failure under severe single-robot derating

The scenario emulates protective speed limiting or drive derating associated with low battery charge or increased temperature. Robot2's wheel-rim velocity limits are reduced from their nominal magnitude of 0.160 m/s to 0.120 m/s (ratio 0.75) or 0.128 m/s (ratio 0.80). The displayed proposed-method condition uses 0.75; the displayed CR-A and LR-NM conditions use 0.80. The 0.85 profile is supplied for the related derating/recovery condition.

Derating is triggered at actual load progress 1.50 m, and recovery is triggered at 3.00 m. Both transitions take 1.00 s. Acceleration and deceleration limit ratios are 0.70 during derating. `DeratingScenario` latches activation and restoration, and `DeratingProfile` interpolates the limit ratios.

The scene contains the condition and limit schedule. It does not implement a physical load-drop detector.

## Section 5.4: Local compound execution disturbances

The condition emulates increased local rolling resistance and unequal left/right wheel effects on a flat floor. Each robot's trigger coordinate is its support-point projection onto the common path centerline. The spatial envelope acts over 2.00-2.80 m and uses 0.05 m entry/exit transitions.

| Quantity | Setting |
| --- | --- |
| Base effectiveness | 0.85 |
| Longitudinal disturbance amplitude | 0.020 m/s |
| Yaw-rate disturbance amplitude | 0.2625 rad/s |
| Angular frequency | 1.0 rad/s |
| Longitudinal / yaw phase | 0 / pi/2 |
| Severity factors, Robots 1-3 | 1.00, 1.15, 0.85 |

The commanded longitudinal velocity is multiplied by the effectiveness factor, then the longitudinal disturbance is added. The yaw disturbance is added to the native yaw-rate command. The resulting chassis command is converted to wheel-rim velocity demand, followed by final wheel-speed/acceleration limiting. Robot2 has the strongest disturbance and Robot3 the weakest. The offline example explicitly retains this injection-before-limiting order.

The mathematical disturbance definitions are given in the "Local compound-disturbance settings" section, Eqs. (S3)-(S5), of `Experimental_Settings.md` in the supplementary dataset.

## Section 5.5: Hardware comparison of lower-layer controllers

The proposed lower controller, LR-NM lower controller and VP-PD use the same common reference and progress-velocity state interval `[-0.05, 0.14] m/s`. The control period is 0.01 s and the nominal reference velocity is 0.100 m/s. No derating or compound disturbance is applied in this condition. VP-PD uses the selected constraint-approach penalty coefficient 0.012.

The library accepts reference position, velocity and acceleration; actual position and velocity; state-velocity bounds; and available acceleration/deceleration magnitudes. `algorithm_example` supplies an interior point to demonstrate each controller call. The coupled cooperative LR-NM controller is provided separately for the cooperative experiments.

