"""Generate typed C++ parameters from the repository's JSON configuration."""
import argparse
import json
from pathlib import Path


def literal(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, (int, float)):
        return repr(value)
    raise TypeError(value)


def array(value):
    return '{{' + ', '.join(literal(x) for x in value) + '}}'


def generate(root):
    read = lambda name: json.loads((root / (name + '.json')).read_text(encoding='utf-8'))
    proposed = read('methods/proposed')
    cra = read('methods/cr_a')
    lrnm = read('methods/lr_nm')
    vppd = read('methods/vp_pd')
    nsb = read('methods/ns_b')
    severe = read('scenarios/severe_derating')
    normal = read('scenarios/normal_conditions')
    recovery = read('scenarios/derating_recovery')
    if normal['derating_enabled'] or normal['compound_disturbances_enabled']:
        raise ValueError('The normal-condition entry uses no imposed derating or compound disturbances')
    if severe['profiles'][recovery['profile']]['target_speed_ratio_left'] != recovery['capacity_ratio']:
        raise ValueError('Derating/recovery ratio does not match its selected profile')
    compound_scene = read('scenarios/compound_disturbances')
    compound = compound_scene['disturbance']
    lower_scene = read('scenarios/lower_layer_comparison')
    execution = read('geometry/execution')
    path = read('geometry/circle_path')
    geometry = read('geometry/support_formation')
    lines = [
        '// Generated from config/*.json; edit configuration, then reconfigure CMake.',
        '#pragma once', '#include <cmath>', '#include <stdexcept>', '#include <string>',
        '#include "multi_agv_control/upper_reference_generator.hpp"',
        '#include "multi_agv_control/lower_channel_controller.hpp"',
        '#include "multi_agv_control/m2b_controller.hpp"',
        '#include "multi_agv_control/capability_mapper.hpp"',
        '#include "multi_agv_control/candidate_b_spatial_composite_disturbance.hpp"',
        '#include "chassis_controller/derating_profile.hpp"',
        '#include "paper_control/margin_feedback.hpp"',
        '#ifdef PAPER_WITH_GEOMETRY',
        '#include "multi_agv_control/planar_support_tracker.hpp"',
        '#include "paper_control/nominal_speed_baseline.hpp"', '#endif',
        'namespace paper_control {',
    ]
    if cra['upper']['mode'] != 'M2a' or cra['lower_controller'] != 'R1':
        raise ValueError('CR-A uses the shared M2a upper layer and R1 lower controller')
    if cra['upper'].get('hard_capability_envelope', {}).get('enabled', False):
        raise ValueError('CR-A reference bounds are fixed at their nominal initial values')
    for function, value in [('proposedUpper', proposed['upper']),
                            ('crAUpper', cra['upper'])]:
        lines += [f'inline multi_agv_control::UpperReferenceConfig {function}() {{',
                  'multi_agv_control::UpperReferenceConfig c;',
                  'c.mode = multi_agv_control::upperModeFromString(' + literal(value['mode']) + ');',
                  'c.controller_ticks_per_update = ' + literal(value['controller_ticks_per_update']) + ';']
        hard = value.get('hard_capability_envelope', {})
        lines += ['c.hard_capability_envelope_enabled = ' + literal(hard.get('enabled', False)) + ';',
                  'c.hard_capability_reserve = ' + literal(hard.get('capability_reserve', .005)) + ';']
        for i in range(3):
            agent = value['agents']
            for field in ['physical_lower', 'physical_upper', 'physical_margin_lower',
                          'physical_margin_upper', 'zero_margin', 'minimum_width']:
                lines.append(f'c.agents[{i}].admissible_set.{field} = {literal(agent[field][i])};')
            for role in ['initial', 'nominal']:
                lines.append(f'c.agents[{i}].{role}_boundary = ' + '{' +
                             f'{literal(agent[role + "_lower"][i])}, {literal(agent[role + "_upper"][i])}' + '};')
            for field in ['inner_margin', 'restore_gain_lower', 'restore_gain_upper',
                          'risk_gain_lower', 'risk_gain_upper', 'risk_scale', 'risk_decay']:
                if field in agent:
                    lines.append(f'c.agents[{i}].{field} = {literal(agent[field][i])};')
        lines += ['return c;', '}']
    for function, value in [('proposedLower', proposed['lower']),
                            ('lrNmLower', lrnm['shared_reference_lower']),
                            ('vpPdLower', vppd['lower'])]:
        lines += [f'inline multi_agv_control::LowerChannelConfig {function}() {{',
                  'multi_agv_control::LowerChannelConfig c;',
                  'c.mode = multi_agv_control::lowerModeFromString(' + literal(value['mode']) + ');']
        for field in ['composite_gain', 'feedback_gain', 'adaptation_gain',
                      'disturbance_adaptation_gain', 'parameter_leakage', 'disturbance_leakage',
                      'robust_boundary_layer', 'parameter_min', 'parameter_max',
                      'disturbance_estimate_max', 'constraint_margin', 'sustained_saturation_seconds']:
            if field in value:
                item = value[field]
                lines.append(f'c.{field} = ' + (array(item) if isinstance(item, list) else literal(item)) + ';')
        for source, target in [('paper_nm', 'paper_nm_'), ('pd_penalty', 'pd_penalty_')]:
            for key, item in value.get(source, {}).items():
                lines.append(f'c.{target}{key} = {literal(item)};')
        lines += ['return c;', '}']
    lines += ['inline multi_agv_control::LowerChannelConfig crALower() { return proposedLower(); }',
              'inline multi_agv_control::M2bConfig cooperativeLrNm() {',
              'multi_agv_control::M2bConfig c;']
    for key, item in lrnm['cooperative'].items():
        if key in ['alpha', 'beta', 'l0', 'l1', 'l2', 'l3', 'h', 'k1', 'k2',
                   'generator_guard', 'mapping_guard', 'generator_substeps', 'auxiliary_boundary_layer']:
            lines.append(f'c.{key} = ' + (array(item) if isinstance(item, list) else literal(item)) + ';')
    lines += ['return c;', '}', 'inline MarginScales marginScales() {', 'MarginScales c;']
    for key in ['capability_reserve', 'capability_safe_margin', 'input_safe_margin',
                'wheel_safe_margin', 'position_limit', 'position_safe_margin']:
        lines.append(f'c.{key} = {literal(execution["risk"][key])};')
    lines += ['return c;', '}', 'inline multi_agv_control::CapabilityReserve capabilityReserve() {',
              'multi_agv_control::CapabilityReserve c;']
    for key, item in execution['capability_reserve'].items():
        lines.append(f'c.{key} = {literal(item)};')
    lines += ['return c;', '}']
    for name, value in [
        ('lowerDt', lower_scene['dt_seconds']),
        ('nominalVelocity', lower_scene['nominal_reference_velocity_mps']),
        ('lowerVelocityBound', lower_scene['state']['velocity_lower_bound']),
        ('upperVelocityBound', lower_scene['state']['velocity_upper_bound']),
        ('initialDisturbanceEstimate', lower_scene['state']['initial_disturbance_estimate']),
        ('nominalWheelSpeed', severe['nominal_wheel_rim_speed_mps']),
        ('nominalWheelAcceleration', severe['nominal_wheel_acceleration_mps2']),
        ('nominalWheelDeceleration', severe['nominal_wheel_deceleration_mps2']),
        ('normalDt', normal['dt_seconds']),
        ('normalVelocity', normal['nominal_reference_velocity_mps']),
        ('normalLowerVelocityBound', normal['state']['velocity_lower_bound']),
        ('normalUpperVelocityBound', normal['state']['velocity_upper_bound']),
        ('normalInitialDisturbanceEstimate', normal['state']['initial_disturbance_estimate']),
        ('deratingRecoveryRatio', recovery['capacity_ratio']),
    ]:
        lines.append(f'inline constexpr double {name} = {literal(value)};')
    for name, scene in [('normalSection', normal), ('deratingRecoverySection', recovery),
                        ('severeDeratingSection', severe), ('compoundDisturbanceSection', compound_scene),
                        ('lowerComparisonSection', lower_scene)]:
        lines.append(f'inline constexpr const char* {name} = {literal(scene["paper_section"])};')
    lines += ['inline std::array<std::string, 3> normalMethods() { return ' + array(normal['methods']) + '; }',
              'inline std::array<std::string, 3> recoveryMethods() { return ' + array(recovery['methods']) + '; }']
    lines += [
        'struct DeratingSceneConfig { double activation_progress, restoration_progress, ramp_down, ramp_up;',
        'chassis_controller::DeratingRatios target; };',
        'inline DeratingSceneConfig deratingScene(double ratio) {',
    ]
    for value in severe['profiles'].values():
        ratio = value['target_speed_ratio_left']
        target = [value['target_' + key + '_ratio_' + side]
                  for key in ['speed', 'acceleration', 'deceleration'] for side in ['left', 'right']]
        numbers = [value['activation_progress'], value['restoration_progress'],
                   value['ramp_down_time'], value['ramp_up_time']]
        lines.append(f'if (std::abs(ratio - {literal(ratio)}) < 1e-12) return ' + '{' +
                     ', '.join(literal(x) for x in numbers) + ', {' +
                     ', '.join(literal(x) for x in target) + '}};')
    lines += ['throw std::invalid_argument("supported derating ratios: 0.75, 0.80, 0.85");', '}']
    lines += ['inline multi_agv_control::CandidateBSpatialCompositeConfig compoundDisturbance() {',
              'multi_agv_control::CandidateBSpatialCompositeConfig c;',
              'c.enabled = true;  // Explicit offline evaluation, no hardware authorization.']
    for key, item in compound.items():
        lines.append(f'c.{key} = ' + (array(item) if isinstance(item, list) else literal(item)) + ';')
    lines += ['c.wheel_separation = ' + array(execution['tracker']['wheel_separation']) + ';',
              'return c;', '}', '#ifdef PAPER_WITH_GEOMETRY',
              'inline multi_agv_control::SCurveConfig circlePath() {', 'multi_agv_control::SCurveConfig c;']
    for key in ['amplitude', 'longitudinal_length', 'lookup_samples', 'model', 'circle_radius',
                'entry_straight_length', 'curvature_ramp_length', 'circle_direction', 'exit_straight_length']:
        if key in path:
            lines.append(f'c.{key} = {literal(path[key])};')
    lines += ['return c;', '}', 'inline multi_agv_control::SupportGeometryConfig supportFormation() {',
              'multi_agv_control::SupportGeometryConfig c;']
    for key in ['minimum_nondegeneracy', 'minimum_speed_scale', 'validation_samples']:
        lines.append(f'c.{key} = {literal(geometry[key])};')
    for robot in geometry['supports']:
        lines.append('c.offsets.push_back({' + literal(robot['q_tangent']) + ', ' + literal(robot['q_normal']) + '});')
    lines += ['return c;', '}']
    for name, tracker in [('planarTracker', execution['tracker']), ('nsBTracker', execution['tracker'])]:
        lines += [f'inline multi_agv_control::PlanarTrackerConfig {name}() {{',
                  'multi_agv_control::PlanarTrackerConfig c;']
        for key in ['longitudinal_gain', 'lateral_gain', 'heading_gain', 'chassis_reference_samples']:
            lines.append(f'c.{key} = {literal(tracker[key])};')
        lines.append('c.wheel_separation = ' + array(tracker['wheel_separation']) + ';')
        for i, offset in enumerate(tracker['base_to_support']):
            lines.append(f'c.base_to_support[{i}] = Eigen::Vector2d(' + literal(offset['x']) + ', ' + literal(offset['y']) + ');')
        lines += ['return c;', '}']
    for name, tracker in [('executionAdapter', execution['tracker']), ('nsBExecutionAdapter', execution['tracker'])]:
        lines += [f'inline multi_agv_control::FleetPlanarExecutionConfig {name}() {{',
                  'multi_agv_control::FleetPlanarExecutionConfig c;']
        for key in ['wheel_separation', 'angular_feedforward_scale_positive', 'angular_feedforward_scale_negative',
                    'curvature_preview_seconds_positive', 'curvature_preview_seconds_negative',
                    'formation_longitudinal_gain', 'formation_lateral_gain', 'formation_heading_gain']:
            item = tracker[key]
            lines.append(f'c.{key} = ' + (array(item) if isinstance(item, list) else literal(item)) + ';')
        for key in ['longitudinal_gain', 'lateral_gain', 'heading_gain']:
            lines.append(f'c.{key} = ' + array(tracker[key + '_per_robot']) + ';')
        lines += ['return c;', '}']
    lines += ['inline NominalSpeedBaselineConfig nominalSpeedBaselineConfig() {',
              'NominalSpeedBaselineConfig c;']
    for target, source in [('nominal_velocity', 'load_path_speed'), ('target_progress', 'target_progress'),
                           ('startup_ramp_seconds', 'startup_ramp_seconds'), ('startup_profile', 'startup_profile'),
                           ('maximum_dt_seconds', 'maximum_integration_step')]:
        lines.append(f'c.{target} = {literal(nsb["motion"][source])};')
    lines += ['c.maximum_wheel_command = ' + literal(nsb['command_limits']['maximum_wheel_linear_velocity']) + ';',
              'c.minimum_command_scale = ' + literal(nsb['command_limits']['minimum_wheel_command_scale']) + ';',
              'return c;', '}', '#endif', '}  // namespace paper_control', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    text = generate(args.config)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(text, encoding='utf-8')


if __name__ == '__main__':
    main()

