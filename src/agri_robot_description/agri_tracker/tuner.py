#!/usr/bin/env python3

import argparse
import json
import random
import signal
import subprocess
import time
from math import hypot
from pathlib import Path
from statistics import mean
from typing import Any, Dict, Iterable, List, Sequence, Tuple

from ament_index_python.packages import get_package_prefix


DEFAULT_WAYPOINTS: List[Tuple[float, float]] = [
    (0.0, 0.0), 
    (5.0, 0.0),
    (10.0, 5.0),
    (15.0, 5.0),
]


def load_log(path: str) -> Dict[str, Any]:
    log_path = Path(path)
    if not log_path.exists():
        raise FileNotFoundError(f'Log file not found: {path}')
    with log_path.open('r', encoding='utf-8') as f:
        return json.load(f)


def mean_cross_track_error(error_list: Sequence[float]) -> float:
    if not error_list:
        return 0.0
    return float(mean(error_list))


def mean_abs_angular_speed(control_list: Sequence[Sequence[float]]) -> float:
    if not control_list:
        return 0.0
    angular_values = [abs(float(ctrl[1])) for ctrl in control_list if len(ctrl) > 1]
    return float(mean(angular_values)) if angular_values else 0.0


def steering_oscillation_penalty(control_list: Sequence[Sequence[float]]) -> float:
    if len(control_list) < 2:
        return 0.0
    omega_values = [float(ctrl[1]) for ctrl in control_list]
    diffs = [abs(omega_values[i] - omega_values[i - 1]) for i in range(1, len(omega_values))]
    return float(mean(diffs)) if diffs else 0.0


def score_log(
    log: Dict[str, Any],
    angular_weight: float = 0.2,
    final_error_weight: float = 1.0,
    oscillation_weight: float = 0.3,
) -> float:
    error_list = log.get('error', [])
    mean_error = mean_cross_track_error(error_list)
    final_error = float(error_list[-1]) if error_list else 0.0
    angular_speed = mean_abs_angular_speed(log.get('control', []))
    oscillation = steering_oscillation_penalty(log.get('control', []))
    return -(
        mean_error
        + final_error_weight * final_error
        + angular_weight * angular_speed
        + oscillation_weight * oscillation
    )


def estimate_min_run_duration(linear_speed: float, safety_factor: float = 2.2) -> float:
    if linear_speed <= 0.0:
        return 30.0

    path_length = 0.0
    for i in range(len(DEFAULT_WAYPOINTS) - 1):
        x1, y1 = DEFAULT_WAYPOINTS[i]
        x2, y2 = DEFAULT_WAYPOINTS[i + 1]
        path_length += hypot(x2 - x1, y2 - y1)

    return max(12.0, (path_length / linear_speed) * safety_factor)


def _start_process(cmd: List[str]) -> subprocess.Popen:
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _package_executable(package: str, executable: str) -> str:
    prefix = Path(get_package_prefix(package))
    exec_path = prefix / 'lib' / package / executable
    if not exec_path.exists():
        raise FileNotFoundError(f'Executable not found: {exec_path}')
    return str(exec_path)


def launch_trial_processes(
    lookahead_distance: float,
    linear_speed: float,
    max_steering: float,
    off_track_speed_scale: float,
    off_track_error_threshold: float,
    odom_source: str,
    log_path: str,
) -> List[subprocess.Popen]:
    package = 'agri_robot_description'
    tracker_cmd = [
        _package_executable(package, 'pure_pursuit_tracker'),
        '--ros-args',
        '-p',
        f'lookahead_distance:={lookahead_distance}',
        '-p',
        f'linear_speed:={linear_speed}',
        '-p',
        f'max_steering:={max_steering}',
        '-p',
        f'off_track_speed_scale:={off_track_speed_scale}',
        '-p',
        f'off_track_error_threshold:={off_track_error_threshold}',
    ]
    collector_cmd = [
        _package_executable(package, 'data_collector'),
        '--ros-args',
        '-p',
        f'log_path:={log_path}',
    ]

    processes: List[subprocess.Popen] = []
    if odom_source == 'fake':
        fake_odom_cmd = [_package_executable(package, 'fake_odom_publisher')]
        processes.append(_start_process(fake_odom_cmd))
    processes.append(_start_process(tracker_cmd))
    processes.append(_start_process(collector_cmd))
    return processes


def terminate_processes(processes: Iterable[subprocess.Popen], timeout_seconds: float = 5.0) -> None:
    for proc in processes:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)

    deadline = time.time() + timeout_seconds
    for proc in processes:
        if proc.poll() is not None:
            continue
        remaining = max(0.0, deadline - time.time())
        try:
            proc.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            proc.terminate()

    for proc in processes:
        if proc.poll() is None:
            proc.kill()
            proc.wait()


def run_single_trial(
    lookahead_distance: float,
    linear_speed: float,
    max_steering: float,
    off_track_speed_scale: float,
    off_track_error_threshold: float,
    odom_source: str,
    trial_id: str,
    log_dir: Path,
    run_duration_s: float,
    init_delay_s: float,
    angular_weight: float,
    oscillation_weight: float,
) -> Dict[str, Any]:
    log_path = log_dir / (
        f'log_{trial_id}_L{lookahead_distance:.3f}_V{linear_speed:.3f}_S{max_steering:.3f}.json'
    )
    processes: List[subprocess.Popen] = []

    try:
        processes = launch_trial_processes(
            lookahead_distance,
            linear_speed,
            max_steering,
            off_track_speed_scale,
            off_track_error_threshold,
            odom_source,
            str(log_path),
        )
        time.sleep(init_delay_s)

        # Fail fast if any process dies during startup.
        for idx, proc in enumerate(processes):
            if proc.poll() is not None:
                out, err = proc.communicate(timeout=1.0)
                return {
                    'score': float('-inf'),
                    'mean_error': float('inf'),
                    'mean_abs_angular_speed': float('inf'),
                    'oscillation': float('inf'),
                    'log_path': str(log_path),
                    'status': f'process_{idx}_exited_early',
                    'stdout_tail': (out or '')[-300:],
                    'stderr_tail': (err or '')[-300:],
                }

        time.sleep(run_duration_s)
    finally:
        terminate_processes(processes)

    # Give collector a short grace period to finish writing JSON.
    deadline = time.time() + 2.0
    while not log_path.exists() and time.time() < deadline:
        time.sleep(0.1)

    if not log_path.exists():
        diagnostics = []
        for proc in processes:
            try:
                out, err = proc.communicate(timeout=0.5)
            except subprocess.TimeoutExpired:
                out, err = '', ''
            tail = ((out or '') + '\n' + (err or '')).strip()
            if tail:
                diagnostics.append(tail[-300:])
        return {
            'score': float('-inf'),
            'mean_error': float('inf'),
            'mean_abs_angular_speed': float('inf'),
            'oscillation': float('inf'),
            'log_path': str(log_path),
            'status': 'missing_log',
            'diagnostics': diagnostics,
        }

    log = load_log(str(log_path))
    score = score_log(log, angular_weight=angular_weight, oscillation_weight=oscillation_weight)
    mean_error = mean_cross_track_error(log.get('error', []))
    angular_speed = mean_abs_angular_speed(log.get('control', []))
    oscillation = steering_oscillation_penalty(log.get('control', []))
    return {
        'score': score,
        'mean_error': mean_error,
        'mean_abs_angular_speed': angular_speed,
        'oscillation': oscillation,
        'log_path': str(log_path),
        'status': 'ok',
    }


def parameter_combinations_grid(grid: Dict[str, List[float]]) -> List[Tuple[float, float, float]]:
    combos: List[Tuple[float, float, float]] = []
    for lookahead in grid['lookahead_distance']:
        for speed in grid['linear_speed']:
            for steering in grid['max_steering']:
                combos.append((lookahead, speed, steering))
    return combos


def parameter_combinations_random(
    lookahead_values: Sequence[float],
    speed_values: Sequence[float],
    steering_values: Sequence[float],
    samples: int,
    seed: int,
) -> List[Tuple[float, float, float]]:
    all_combos = [(l, v, s) for l in lookahead_values for v in speed_values for s in steering_values]
    rng = random.Random(seed)
    if samples >= len(all_combos):
        rng.shuffle(all_combos)
        return all_combos
    return rng.sample(all_combos, samples)


def tune_parameters(
    combinations: Sequence[Tuple[float, float, float]],
    repeats_per_combo: int,
    log_dir: Path,
    run_duration_s: float,
    init_delay_s: float,
    off_track_speed_scale: float,
    off_track_error_threshold: float,
    odom_source: str,
    angular_weight: float,
    oscillation_weight: float,
) -> Dict[str, Any]:
    trials: List[Dict[str, Any]] = []
    summary: List[Dict[str, Any]] = []

    for combo_idx, (lookahead, speed, max_steering) in enumerate(combinations, start=1):
        run_results: List[Dict[str, Any]] = []
        effective_run_duration = max(run_duration_s, estimate_min_run_duration(speed))
        print(f'[{combo_idx}/{len(combinations)}] L={lookahead:.3f}, V={speed:.3f}, S={max_steering:.3f}')
        if effective_run_duration > run_duration_s:
            print(
                f'  adjusted run_duration from {run_duration_s:.1f}s to '
                f'{effective_run_duration:.1f}s to allow full-path completion'
            )

        for rep in range(1, repeats_per_combo + 1):
            trial_id = f'c{combo_idx:03d}_r{rep:02d}'
            result = run_single_trial(
                lookahead_distance=lookahead,
                linear_speed=speed,
                max_steering=max_steering,
                off_track_speed_scale=off_track_speed_scale,
                off_track_error_threshold=off_track_error_threshold,
                odom_source=odom_source,
                trial_id=trial_id,
                log_dir=log_dir,
                run_duration_s=effective_run_duration,
                init_delay_s=init_delay_s,
                angular_weight=angular_weight,
                oscillation_weight=oscillation_weight,
            )
            run_results.append(result)
            trials.append(
                {
                    'trial_id': trial_id,
                    'lookahead_distance': lookahead,
                    'linear_speed': speed,
                    'max_steering': max_steering,
                    **result,
                }
            )
            print(
                f'  run {rep}/{repeats_per_combo}: score={result["score"]:.4f}, '
                f'error={result["mean_error"]:.4f}, angular={result["mean_abs_angular_speed"]:.4f}, '
                f'osc={result["oscillation"]:.4f}'
            )

        valid_scores = [r['score'] for r in run_results if r['status'] == 'ok']
        valid_errors = [r['mean_error'] for r in run_results if r['status'] == 'ok']
        valid_angular = [r['mean_abs_angular_speed'] for r in run_results if r['status'] == 'ok']
        valid_osc = [r['oscillation'] for r in run_results if r['status'] == 'ok']
        combo_summary = {
            'lookahead_distance': lookahead,
            'linear_speed': speed,
            'max_steering': max_steering,
            'avg_score': float(mean(valid_scores)) if valid_scores else float('-inf'),
            'avg_error': float(mean(valid_errors)) if valid_errors else float('inf'),
            'avg_abs_angular_speed': float(mean(valid_angular)) if valid_angular else float('inf'),
            'avg_oscillation': float(mean(valid_osc)) if valid_osc else float('inf'),
            'valid_runs': len(valid_scores),
            'total_runs': repeats_per_combo,
        }
        summary.append(combo_summary)

    best = max(summary, key=lambda s: s['avg_score']) if summary else {}
    return {'trials': trials, 'summary': summary, 'best': best}


def parse_float_list(value: str) -> List[float]:
    if not value.strip():
        return []
    return [float(v.strip()) for v in value.split(',') if v.strip()]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Automatic parameter tuner for Pure Pursuit tracker')
    parser.add_argument('--method', choices=['grid', 'random'], default='grid')
    parser.add_argument('--lookahead-values', default='0.5,1.0,1.5,2.0,2.5')
    parser.add_argument('--speed-values', default='0.15,0.25,0.35,0.5')
    parser.add_argument('--max-steering-values', default='0.45,0.55,0.70,0.85')
    parser.add_argument('--random-samples', type=int, default=10)
    parser.add_argument('--random-seed', type=int, default=42)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--run-duration', type=float, default=40.0)
    parser.add_argument('--init-delay', type=float, default=1.5)
    parser.add_argument(
        '--odom-source',
        choices=['fake', 'isaac'],
        default='fake',
        help='Odometry source for tuning trials: fake launches fake_odom_publisher, isaac expects external /odom',
    )
    parser.add_argument(
        '--off-track-speed-scale',
        type=float,
        default=0.35,
        help='Speed multiplier when deviation exceeds threshold (0.0, 1.0]',
    )
    parser.add_argument(
        '--off-track-error-threshold',
        type=float,
        default=0.1,
        help='Deviation threshold (meters) to trigger speed slowdown; set 0.05 for stricter recovery',
    )
    parser.add_argument('--angular-weight', type=float, default=0.3)
    parser.add_argument('--oscillation-weight', type=float, default=0.45)
    parser.add_argument('--log-dir', default='steering_tuning_logs')
    parser.add_argument('--output', default='steering_tuning_results.json')
    return parser


def main() -> None:
    args = build_parser().parse_args()

    lookahead_values = parse_float_list(args.lookahead_values)
    speed_values = parse_float_list(args.speed_values)
    steering_values = parse_float_list(args.max_steering_values)
    if not lookahead_values or not speed_values or not steering_values:
        raise ValueError('lookahead-values, speed-values, and max-steering-values must not be empty')
    if args.repeats <= 0:
        raise ValueError('--repeats must be > 0')
    if args.angular_weight < 0.0:
        raise ValueError('--angular-weight must be >= 0.0')
    if args.oscillation_weight < 0.0:
        raise ValueError('--oscillation-weight must be >= 0.0')
    if args.off_track_speed_scale <= 0.0 or args.off_track_speed_scale > 1.0:
        raise ValueError('--off-track-speed-scale must be in the range (0.0, 1.0]')
    if args.off_track_error_threshold <= 0.0:
        raise ValueError('--off-track-error-threshold must be > 0.0')

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    if args.method == 'grid':
        combinations = parameter_combinations_grid(
            {
                'lookahead_distance': lookahead_values,
                'linear_speed': speed_values,
                'max_steering': steering_values,
            }
        )
    else:
        combinations = parameter_combinations_random(
            lookahead_values,
            speed_values,
            steering_values,
            samples=args.random_samples,
            seed=args.random_seed,
        )

    print('=== Pure Pursuit Steering Tuner ===')
    print(f'Method: {args.method}')
    print(f'Odom source: {args.odom_source}')
    print(
        'Off-track recovery: '
        f'threshold={args.off_track_error_threshold:.3f} m, '
        f'speed_scale={args.off_track_speed_scale:.2f}'
    )
    print(f'Combinations: {len(combinations)}')
    print(f'Repeats per combination: {args.repeats}')

    result = tune_parameters(
        combinations=combinations,
        repeats_per_combo=args.repeats,
        log_dir=log_dir,
        run_duration_s=args.run_duration,
        init_delay_s=args.init_delay,
        off_track_speed_scale=args.off_track_speed_scale,
        off_track_error_threshold=args.off_track_error_threshold,
        odom_source=args.odom_source,
        angular_weight=args.angular_weight,
        oscillation_weight=args.oscillation_weight,
    )

    output_path = Path(args.output)
    with output_path.open('w', encoding='utf-8') as f:
        json.dump(result, f, indent=2)

    best = result.get('best', {})
    if best:
        print('\nBest parameters:')
        print(f"  lookahead_distance: {best['lookahead_distance']}")
        print(f"  linear_speed: {best['linear_speed']}")
        print(f"  max_steering: {best['max_steering']}")
        print(f"  average score: {best['avg_score']:.6f}")
    print(f'\nSaved detailed results to {output_path}')


if __name__ == '__main__':
    main()