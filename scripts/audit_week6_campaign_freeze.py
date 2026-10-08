#!/usr/bin/env python3
"""Run the v7 pre-campaign freeze audit and preserve its evidence."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
import runpy
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
CAMPAIGN_RELATIVE = Path('results/week6/final_campaign_v7')
CAMPAIGN = WORKSPACE / CAMPAIGN_RELATIVE
V6_AUDIT = WORKSPACE / 'results/week6/final_campaign_v6/freeze_audit.json'
TEST_FILES = (
    'src/ares_reliability/test/test_week6_runtime_isolation.py',
    'src/ares_reliability/test/test_week6_nav2_startup_barrier.py',
    'src/ares_reliability/test/test_week6_navigation_config.py',
    'src/ares_reliability/test/test_week6_campaign_runner.py',
    'src/ares_reliability/test/test_week6_campaign_analyzer.py',
    'src/ares_reliability/test/test_week4_analysis.py',
    'src/ares_reliability/test/test_week5_analysis.py',
    'src/ares_reliability/test/test_week5_fusion_config.py',
    'src/ares_reliability/test/test_recovery_manager.py',
)
CORE_IDEA_CRITERIA = {
    'C1_HEALTHY_BEHAVIOR': (
        'All healthy baseline/unprotected/protected runs complete every '
        'waypoint within 0.30 m; protected healthy runs have zero ARES gates '
        'and zero ARES probation entries.'),
    'C2_FAULT_DETECTION': (
        'All three protected gnss_step_5m runs record fault onset, at least '
        'one pre-fusion rejection, and GNSS trust transitions through '
        'DEGRADED and UNTRUSTED.'),
    'C3_PREFUSION_ISOLATION': (
        'Protected guard is enabled only for protected mode and all three '
        'protected fault runs reject bad GNSS before fusion and enter GATED.'),
    'C4_NAVIGATION_DURING_FAULT': (
        'All protected fault runs complete the mission and have at least one '
        'waypoint goal active during the configured 15-45 s mission-relative '
        'fault interval.'),
    'C5_PROTECTED_BENEFIT': (
        'Protected fault completion exceeds unprotected completion and the '
        'matched-seed mean of run-maximum localization errors is lower for '
        'protected runs.'),
    'C6_RECOVERY': (
        'All protected fault runs progress through GATED, PROBATION, NORMAL, '
        'and end with GNSS trust HEALTHY.'),
    'C7_NO_ORACLE_DEPENDENCY': (
        'All 15 runs record readiness_ground_truth_used=false; ground truth '
        'is only recorded for evaluation and is absent from the readiness '
        'decision path.'),
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree_hash(path: Path) -> str:
    digest = hashlib.sha256()
    for item in sorted(path.rglob('*')):
        if item.is_file():
            digest.update(item.relative_to(path).as_posix().encode())
            digest.update(b'\0')
            digest.update(item.read_bytes())
            digest.update(b'\0')
    return digest.hexdigest()


def yaml_file(relative: str) -> dict[str, Any]:
    return yaml.safe_load((WORKSPACE / relative).read_text(
        encoding='utf-8'))


def check_v7_target_unused() -> bool:
    return not CAMPAIGN.exists()


def nav2_scientific_settings_unchanged() -> bool:
    production = yaml_file(
        'src/ares_localization/config/nav2_params.yaml')
    week6 = yaml_file(
        'src/ares_reliability/config/nav2_week6_navigation.yaml')
    week6 = copy.deepcopy(week6)
    week6['controller_server']['ros__parameters'][
        'goal_checker']['stateful'] = True
    week6['bt_navigator']['ros__parameters'].pop(
        'wait_for_service_timeout', None)
    behavior = week6['behavior_server']['ros__parameters']
    for plugin in behavior.get('behavior_plugins', []):
        behavior.pop(plugin, None)
    behavior.pop('behavior_plugins', None)
    return week6 == production


def run_targeted_tests(environment: dict[str, str]) -> dict[str, Any]:
    command = [
        sys.executable, '-m', 'pytest', '-q',
        *[str(WORKSPACE / path) for path in TEST_FILES],
    ]
    result = subprocess.run(
        command, cwd=WORKSPACE, env=environment,
        capture_output=True, text=True, check=False)
    return {
        'command': command,
        'exit_code': result.returncode,
        'passed': result.returncode == 0,
        'stdout': result.stdout,
        'stderr': result.stderr,
    }


def resolved_mode_readiness_profiles() -> dict[str, Any]:
    """Exercise the installed manifest and launch helper for every mode."""
    from ament_index_python.packages import get_package_share_directory
    from nav2_msgs.action import (
        BackUp,
        ComputePathThroughPoses,
        ComputePathToPose,
        DriveOnHeading,
        FollowPath,
        Spin,
        Wait,
    )
    from nav2_msgs.srv import ClearEntireCostmap, IsPathValid

    from ares_reliability import week6_nav2_startup_barrier as barrier_module

    expected_modes = ('baseline', 'unprotected', 'protected')
    expected_lifecycle = {
        'baseline': (
            'map_server', 'planner_server', 'controller_server',
            'behavior_server', 'amcl'),
        'unprotected': (
            'map_server', 'planner_server', 'controller_server',
            'behavior_server'),
        'protected': (
            'map_server', 'planner_server', 'controller_server',
            'behavior_server'),
    }
    expected_actions = (
        ('/compute_path_to_pose', ComputePathToPose),
        ('/compute_path_through_poses', ComputePathThroughPoses),
        ('/follow_path', FollowPath),
        ('/spin', Spin),
        ('/wait', Wait),
        ('/backup', BackUp),
        ('/drive_on_heading', DriveOnHeading),
    )
    expected_services = (
        ('/is_path_valid', IsPathValid),
        ('/global_costmap/clear_entirely_global_costmap',
         ClearEntireCostmap),
        ('/local_costmap/clear_entirely_local_costmap',
         ClearEntireCostmap),
    )

    launch_path = (
        WORKSPACE / 'src/ares_reliability/launch/week6_navigation.launch.py')
    spec = importlib.util.spec_from_file_location(
        'week6_navigation_freeze_audit', launch_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'cannot load Week 6 launch module {launch_path}')
    launch_module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(launch_module)

    def find_barrier_node(actions: list[Any]) -> Any | None:
        pending = list(actions)
        while pending:
            action = pending.pop(0)
            nested = getattr(action, '_TimerAction__actions', None)
            if nested is not None:
                pending.extend(nested)
            if (
                    getattr(action, '_Node__package', None) ==
                    'ares_reliability' and
                    getattr(action, '_Node__node_executable', None) ==
                    'week6_nav2_startup_barrier'):
                return action
        return None

    profiles: dict[str, Any] = {}
    installed_share = Path(get_package_share_directory('ares_reliability'))
    source_launch = launch_path
    installed_launch = (
        installed_share / 'launch' / 'week6_navigation.launch.py')
    source_barrier = (
        WORKSPACE / 'src/ares_reliability/ares_reliability/'
        'week6_nav2_startup_barrier.py')
    installed_barrier = Path(barrier_module.__file__).resolve()
    source_install_match = (
        source_launch.read_bytes() == installed_launch.read_bytes() and
        source_barrier.read_bytes() == installed_barrier.read_bytes())

    for mode in expected_modes:
        manifest = barrier_module.MODE_READINESS_MANIFESTS.get(mode)
        actions = launch_module._week6_startup_barrier_actions(
            True, start_upstream=(mode != 'baseline'),
            navigation_mode=mode)
        barrier_node = find_barrier_node(actions)
        arguments = (
            getattr(barrier_node, '_Node__arguments', [])
            if barrier_node is not None else [])
        try:
            mode_index = arguments.index('--navigation-mode')
            selected_mode = arguments[mode_index + 1]
        except (ValueError, IndexError):
            selected_mode = None
        valid = (
            manifest is not None and
            tuple(manifest['required_lifecycle_nodes']) ==
            expected_lifecycle[mode] and
            tuple(manifest['action_endpoints']) == expected_actions and
            tuple(manifest['service_endpoints']) == expected_services and
            selected_mode == mode and barrier_node is not None)
        profiles[mode] = {
            'valid': valid,
            'selected_mode': selected_mode,
            'required_lifecycle_nodes': (
                list(manifest['required_lifecycle_nodes'])
                if manifest is not None else None),
            'required_actions': (
                [name for name, _ in manifest['action_endpoints']]
                if manifest is not None else None),
            'required_services': (
                [name for name, _ in manifest['service_endpoints']]
                if manifest is not None else None),
            'barrier_node_enabled': barrier_node is not None,
            'source_launch_sha256': sha256_file(source_launch),
            'installed_launch_sha256': sha256_file(installed_launch),
            'source_barrier_sha256': sha256_file(source_barrier),
            'installed_barrier_sha256': sha256_file(installed_barrier),
            'source_install_match': source_install_match,
        }
    return {
        'profiles': profiles,
        'all_profiles_valid': all(
            profile['valid'] for profile in profiles.values()),
        'source_install_match': source_install_match,
    }


def create_audit() -> dict[str, Any]:
    if CAMPAIGN.exists():
        allowed_existing = {
            'freeze_audit.json',
            'pre_campaign_runtime_isolation.json',
        }
        existing = {
            path.relative_to(CAMPAIGN).as_posix()
            for path in CAMPAIGN.rglob('*') if path.is_file()
        }
        if existing - allowed_existing or any(
                CAMPAIGN.rglob('result.json')):
            raise FileExistsError(
                f'{CAMPAIGN} contains evidence beyond the freeze audit; '
                'refusing to overwrite it')

    environment = os.environ.copy()
    environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    tests = run_targeted_tests(environment)

    CAMPAIGN.mkdir(parents=True, exist_ok=True)
    isolation_path = CAMPAIGN / 'pre_campaign_runtime_isolation.json'
    isolation_report: dict[str, Any]
    try:
        sys.path.insert(0, str(WORKSPACE / 'scripts'))
        from week6_runtime_isolation import establish_world_exclusivity

        isolation_report = establish_world_exclusivity(
            isolation_path, environment=environment)
    except (RuntimeError, OSError) as error:
        isolation_report = {
            'world_exclusivity_ready': False,
            'error': str(error),
        }
    mode_readiness = resolved_mode_readiness_profiles()

    previous = json.loads(V6_AUDIT.read_text(encoding='utf-8'))
    mission = yaml_file('src/ares_reliability/config/week6_mission.yaml')
    fault = mission['faults']['gnss_step_5m']
    waypoints = mission['waypoints']
    baseline_launch = (
        WORKSPACE / 'src/ares_simulation/launch/ares_baseline.launch.py'
    ).read_text(encoding='utf-8')
    week6_launch = (
        WORKSPACE / 'src/ares_reliability/launch/week6_navigation.launch.py'
    ).read_text(encoding='utf-8')
    barrier = (
        WORKSPACE /
        'src/ares_reliability/ares_reliability/'
        'week6_nav2_startup_barrier.py'
    ).read_text(encoding='utf-8')
    runner = (WORKSPACE / 'scripts/run_week6_navigation.py').read_text(
        encoding='utf-8')
    mission_source = (
        WORKSPACE /
        'src/ares_reliability/ares_reliability/week6_navigation_mission.py'
    ).read_text(encoding='utf-8')
    analyzer_hash = sha256_file(
        WORKSPACE / 'scripts/analyze_week6_campaign.py')
    previous_analyzer_hash = previous['implementation_sha256'][
        'scripts/analyze_week6_campaign.py']

    evidence_directories = (
        'results/week6/final_campaign_v2',
        'results/week6/final_campaign_v3',
        'results/week6/final_campaign_v4',
        'results/week6/final_campaign_v5',
        'results/week6/final_campaign_v6',
        'results/week6/diagnostics',
        'results/week6/runtime_stabilization',
        'results/week6/post_v4_diagnostics',
        'results/week6/post_v5_runtime_diagnostics',
        'results/week6/post_v6_runtime_diagnostics',
    )
    preserved_hashes = {
        path: tree_hash(WORKSPACE / path)
        for path in evidence_directories if (WORKSPACE / path).is_dir()
    }
    preserved_complete = all(
        (WORKSPACE / path).is_dir() for path in evidence_directories)

    production_nav2 = yaml_file(
        'src/ares_localization/config/nav2_params.yaml')
    week6_nav2 = yaml_file(
        'src/ares_reliability/config/nav2_week6_navigation.yaml')
    week6_behavior_plugins = week6_nav2[
        'behavior_server']['ros__parameters']
    expected_behavior_plugins = {
        'spin': 'nav2_behaviors::Spin',
        'backup': 'nav2_behaviors::BackUp',
        'drive_on_heading': 'nav2_behaviors::DriveOnHeading',
        'wait': 'nav2_behaviors::Wait',
    }

    checks: dict[str, bool] = {
        'v7_destination_unused_before_audit': True,
        'pre_campaign_world_exclusivity_passed':
            bool(isolation_report.get('world_exclusivity_ready')),
        'no_ares_world_or_stale_week6_processes':
            not isolation_report.get(
                'checks_after_cleanup', {}).get('owned_processes') and
            not isolation_report.get(
                'checks_after_cleanup', {}).get(
                    'gazebo_ares_world_services'),
        'no_stale_critical_ros_nodes':
            not isolation_report.get(
                'checks_after_cleanup', {}).get('critical_ros_nodes'),
        'clock_publishers_clean':
            isolation_report.get('clock_publisher_count') == 0,
        'raw_clock_publishers_clean':
            isolation_report.get('raw_clock_publisher_count') == 0,
        'fast_dds_enabled':
            environment['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp',
        'week6_clock_boundary_100_hz':
            "WEEK6_CLOCK_RATE_HZ = 100.0" in week6_launch and
            "'output_rate_hz': WEEK6_CLOCK_RATE_HZ" in week6_launch,
        'mode_specific_readiness_enabled':
            mode_readiness['all_profiles_valid'],
        'readiness_source_install_match':
            mode_readiness['source_install_match'],
        'behavior_action_readiness_enabled':
            all(
                plugin in barrier
                for plugin in (
                    "'/spin'", "'/backup'", "'/drive_on_heading'", "'/wait'")
            ) and
            'behavior_actions_ready_before_planner_services' in barrier,
        'behavior_plugins_explicit_and_typed':
            list(week6_behavior_plugins.get('behavior_plugins', [])) == [
                'spin', 'backup', 'drive_on_heading', 'wait'] and
            {
                name: week6_behavior_plugins[name]['plugin']
                for name in expected_behavior_plugins
            } == expected_behavior_plugins,
        'baseline_bt_navigator_enabled':
            "('nav2_bt_navigator', 'bt_navigator')" in baseline_launch and
            "if stage == 'nav2' or executable != 'bt_navigator':"
            in baseline_launch,
        'analyzer_unchanged_from_v6':
            analyzer_hash == previous_analyzer_hash,
        'analyzer_acceptance_logic_unchanged':
            "check('frozen_goal_threshold_satisfied'" in
            (WORKSPACE / 'scripts/analyze_week6_campaign.py').read_text(
                encoding='utf-8'),
        'scientific_waypoint_threshold_0_30_m':
            mission['waypoint_xy_tolerance_m'] == 0.30 and
            previous['frozen_scientific_values'][
                'waypoint_xy_tolerance_m'] == 0.30,
        'mission_geometry_unchanged':
            waypoints == previous['frozen_scientific_values']['waypoints'],
        'fault_profile_unchanged':
            fault['profile'] == 'gnss_step_5m' and
            fault['parameters']['east_offset_m'] == 5.0 and
            fault['onset_sim_sec'] == 15.0 and
            fault['duration_sim_sec'] == 30.0 and
            fault['parameters']['mode'] == 'step',
        'trust_and_prefusion_values_unchanged':
            "gnss_healthy_threshold_m': '1.5'" in week6_launch and
            "gnss_fault_threshold_m': '4.0'" in week6_launch and
            "'prefusion_threshold_m': '4.0'" in week6_launch and
            "'prefusion_window_sec': '2.0'" in week6_launch,
        'recovery_logic_unchanged':
            "gnss_recovery_persistence': '10'" in week6_launch and
            "probation_after_ungated_degraded': 'false'" in week6_launch,
        'ekf_scientific_configuration_unchanged':
            sha256_file(
                WORKSPACE /
                'src/ares_reliability/config/ekf_week6_navigation.yaml') ==
            previous['implementation_sha256'][
                'src/ares_reliability/config/ekf_week6_navigation.yaml'],
        'navsat_scientific_configuration_unchanged':
            sha256_file(
                WORKSPACE /
                'src/ares_reliability/config/navsat_week6_navigation.yaml') ==
            previous['implementation_sha256'][
                'src/ares_reliability/config/navsat_week6_navigation.yaml'],
        'planner_controller_scientific_tuning_unchanged':
            nav2_scientific_settings_unchanged(),
        'map_unchanged':
            sha256_file(
                WORKSPACE /
                'src/ares_simulation/maps/ares_warehouse.yaml') ==
            previous['implementation_sha256'][
                'src/ares_simulation/maps/ares_warehouse.yaml'] and
            sha256_file(
                WORKSPACE /
                'src/ares_simulation/maps/ares_warehouse.pgm') ==
            previous['implementation_sha256'][
                'src/ares_simulation/maps/ares_warehouse.pgm'],
        'robot_geometry_unchanged':
            sha256_file(
                WORKSPACE /
                'src/ares_jackal_description/urdf/jackal.urdf.xacro') ==
            previous['implementation_sha256'][
                'src/ares_jackal_description/urdf/jackal.urdf.xacro'] and
            sha256_file(
                WORKSPACE /
                'src/ares_simulation/models/ares_jackal/model.sdf') ==
            previous['implementation_sha256'][
                'src/ares_simulation/models/ares_jackal/model.sdf'],
        'v2_v3_v4_v5_v6_and_diagnostics_preserved':
            preserved_complete,
        'targeted_regression_suite_passed': tests['passed'],
    }

    failures = sorted(name for name, passed in checks.items() if not passed)
    audit = {
        'schema_version': 1,
        'audit_verdict': 'PASS' if not failures else 'FAIL',
        'completed_utc': datetime.now(timezone.utc).isoformat(),
        'workspace': str(WORKSPACE),
        'campaign_directory': str(CAMPAIGN),
        'scientific_runs_created_before_audit_pass': 0,
        'checks': checks,
        'failed_checks': failures,
        'core_idea_criteria_frozen_before_campaign': CORE_IDEA_CRITERIA,
        'scientific_values': {
            'waypoint_xy_tolerance_m':
                mission['waypoint_xy_tolerance_m'],
            'mission_id': mission['mission_id'],
            'waypoints': waypoints,
            'fault_profile': fault['profile'],
            'fault_east_offset_m':
                fault['parameters']['east_offset_m'],
            'fault_onset_mission_sec': fault['onset_sim_sec'],
            'fault_duration_sec': fault['duration_sim_sec'],
            'fault_parameters': fault['parameters'],
            'gnss_healthy_threshold_m': 1.5,
            'gnss_fault_threshold_m': 4.0,
            'pre_fusion_threshold_m': 4.0,
            'pre_fusion_window_sec': 2.0,
            'startup_barrier_timeout_wall_sec': 60.0,
        },
        'runtime_values': {
            'rmw_implementation': environment['RMW_IMPLEMENTATION'],
            'clock_raw_topic': '/ares/week6/raw_clock',
            'clock_output_topic': '/clock',
            'clock_rate_hz': 100.0,
            'mode_specific_readiness': True,
            'behavior_action_types': expected_behavior_plugins,
        },
        'analyzer_sha256': analyzer_hash,
        'freeze_audit_script_sha256': sha256_file(Path(__file__)),
        'implementation_sha256': {
            relative: sha256_file(WORKSPACE / relative)
            for relative in runpy.run_path(
                str(WORKSPACE / 'scripts/run_week6_navigation.py')
            )['IMPLEMENTATION_FILES']
        },
        'preserved_evidence_tree_sha256': preserved_hashes,
        'pre_campaign_runtime_isolation': isolation_report,
        'mode_readiness_profile_resolution': mode_readiness,
        'targeted_regression_tests': tests,
        'campaign_matrix': [
            {
                'seed': seed,
                'scenario': scenario,
                'navigation_mode': mode,
            }
            for seed in (2530, 2531, 2532)
            for mode, scenario in (
                ('baseline', 'healthy'),
                ('unprotected', 'healthy'),
                ('protected', 'healthy'),
                ('unprotected', 'gnss_step_5m'),
                ('protected', 'gnss_step_5m'),
            )
        ],
    }
    (CAMPAIGN / 'freeze_audit.json').write_text(
        json.dumps(audit, indent=2, sort_keys=True) + '\n',
        encoding='utf-8')
    if not failures:
        print('V7_FREEZE_AUDIT: PASS', flush=True)
    else:
        print('V7_FREEZE_AUDIT: FAIL', flush=True)
        print('FAILED_CHECKS: ' + ', '.join(failures), flush=True)
    return audit


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    audit = create_audit()
    raise SystemExit(0 if audit['audit_verdict'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
