# Cooperative transportation control

This repository provides the core control algorithms and experimental settings for robust-margin-aware cooperative transportation by three mobile robots.

The C++17 library is accompanied by offline examples illustrating controller calls and scenario evaluation with explicitly supplied example inputs. These examples do not reproduce physical robot trajectories or manuscript statistics. A ROS master and hardware drivers are not required.

## Experiments

| Experiment | Manuscript section | Included settings |
| --- | --- | --- |
| Normal-condition comparison | Section 5.2 | Proposed, LR-NM and NS-B on the common path without imposed derating or disturbances |
| Single-robot derating and recovery | Section 5.3.1 | The 0.85 Robot2 profile and the Proposed/CR-A/LR-NM method settings |
| Severe single-robot derating | Section 5.3.2 | The 0.75 and 0.80 Robot2 capacity-reduction profiles |
| Local compound execution disturbances | Section 5.4 | Spatial effectiveness loss, longitudinal velocity disturbance and yaw-rate disturbance |
| Lower-layer controller comparison | Section 5.5 | A common reference and the state-velocity interval `[-0.05, 0.14] m/s` |

Section numbers refer to the accompanying English manuscript.

## Methods

| Manuscript method | Code identifiers | Role | Parameter file |
| --- | --- | --- | --- |
| Proposed | `M1` upper layer, `R1` lower layer | Margin-aware dynamic reference regulation and adaptive constrained tracking | `config/methods/proposed.json` |
| CR-A | `M2a` upper layer and `R1` lower layer in all cooperative experiments | Fixed nominal initial reference bounds, local candidates, common-reference fusion and the shared lower controller | `config/methods/cr_a.json` |
| LR-NM | `M2b` in cooperative experiments; `PaperNM` in the shared-reference lower-layer comparison | Complete local-reference/nonlinear-mapping method, or its lower-layer control law under a common reference | `config/methods/lr_nm.json` |
| VP-PD | `PDPenalty` | PD tracking with a velocity-bound penalty term | `config/methods/vp_pd.json` |
| NS-B | `NominalSpeedBaseline` | Uniform nominal progress velocity, planar tracking and common fleet command scaling | `config/methods/ns_b.json` |

`M1` identifies the proposed upper reference generator and `R1` the proposed lower controller. In the lower-layer comparison, the proposed method uses `R1` under the supplied common reference. Use the complete identifiers `M2a` and `M2b` to distinguish CR-A from LR-NM.

Both derating and compound-disturbance examples select CR-A through `crAForScenario`, which returns the same `M2a + R1` configuration. Its reference-velocity bounds stay at their nominal initial values. `crALower()` shares the proposed `R1` controller parameters.

The core entry points are `UpperReferenceGenerator`, `stepDistributedReference`, `LowerChannelController` and `M2bController`. The supplied configurations and examples select the methods in the table.

Normal conditions, derating/recovery, severe derating and compound disturbances use the cooperative LR-NM reference generator and tracking law in `M2bController`. Its load reference is the arithmetic mean of the local candidates. The shared-reference comparison uses the LR-NM tracking law through `PaperNM`. The reference generator, nonlinear mapping and control gains remain method-specific.

All acceleration-based controllers limit `input_raw` to the supplied available acceleration/deceleration range, form `velocity_actual + dt * input_limited`, and project that command onto the supplied platform execution interval. The input, limit flags and executed command are exposed separately. Execution limits do not modify LR-NM's reference-generator states or nonlinear-mapping bounds. Supply the same platform-limit rules and values to every method within a comparison condition; limits may differ between robots and change with the prescribed derating schedule.

`LowerChannelInput.execution_velocity_lower_bound/upper_bound` specify the command interval independently of the tracking-law bounds. If both are NaN, the tracking-state bounds are used. `M2bInput` carries robot-specific execution intervals and available acceleration/deceleration values. The examples explicitly supply common values from the scene settings. With a capability mapper, the caller can form the execution interval by intersecting the state interval with the mapped capacity interval.

All methods use the calibrated planar settings in `config/geometry/execution.json`, including relative-pose feedback, directional yaw-feedforward scaling and progress preview. Physical wheel-speed and acceleration/deceleration limits are applied downstream using `WheelCommandLimiter`. NS-B is a velocity-reference baseline: its nominal-speed and fleet-scaling rule is described in the experiment documentation, and its planar calibration is shared with the other methods.

## Build and dependencies

| Dependency | Requirement |
| --- | --- |
| C++ compiler | C++17 support |
| CMake | Version 3.20 or later recommended |
| Python | Version 3.8 or later; standard library only |
| Eigen3 | Headers required for geometry modules |

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel 2
```

If Eigen is outside the standard search paths, add `-DEIGEN3_INCLUDE_DIR=/path/to/eigen` when configuring. A core-only build is available with `-DPAPER_BUILD_GEOMETRY=OFF`. On Windows, use the executable paths under `build`, or the selected configuration subdirectory for a multi-configuration generator.

## Run the examples

```sh
./build/algorithm_example
./build/scenario_example normal_conditions
./build/scenario_example derating_recovery
./build/scenario_example severe_derating
./build/scenario_example compound_disturbances
./build/scenario_example lower_layer_comparison
```

`algorithm_example` demonstrates the proposed upper layer, CR-A, cooperative LR-NM and the three shared-reference lower controllers. `scenario_example normal_conditions` evaluates the Proposed, LR-NM and NS-B interfaces at an explicitly supplied steady-motion state. The other scene entries evaluate the selected derating/disturbance schedules or display the shared-reference controller settings. No vehicle commands are transmitted.

The normal-condition entry uses the geometry modules and requires the default geometry-enabled build. `derating_recovery` provides the 0.85 derating/recovery schedule in Section 5.3.1. All entries are offline examples; their printed values are not experimental performance statistics.

## Repository layout

- `config/methods`: parameters for the five methods.
- `config/scenarios`: normal conditions, 0.85 derating/recovery, severe derating, compound disturbances and lower-layer comparison.
- `config/geometry`: the common path, support layout and planar execution parameters.
- `include` and `src`: algorithm interfaces and implementations.
- `examples`: offline usage examples.
- `tools/generate_parameters.py`: converts JSON configuration into a typed C++ parameter header during the CMake configuration step.

## Parameters and interfaces

Edit the JSON files, then reconfigure and rebuild to apply parameter changes. Inputs use SI units. Feedback fields named `previous_*` refer to the most recently completed lower-layer cycle. Capability timestamps use the transport-neutral `SourceStamp` metadata structure.

## Documentation

| Document | Contents |
| --- | --- |
| [Experimental conditions](docs/EXPERIMENTS.md) | Experiment design, parameters, units and manuscript sections |
| [References](docs/references.md) | Comparison-method attribution and bibliographic citations |
