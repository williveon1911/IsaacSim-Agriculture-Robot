#!/usr/bin/env python3

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Dict, Iterable, List, Sequence, Tuple


def load_log(path: str) -> Dict:
    """Load a JSON run log from disk."""
    log_path = Path(path)
    if not log_path.exists():
        raise FileNotFoundError(f'Log file not found: {path}')

    with log_path.open('r', encoding='utf-8') as f:
        return json.load(f)


def mean_cross_track_error(error_list: Sequence[float]) -> float:
    """Compute mean cross-track error (smaller is better)."""
    if not error_list:
        return 0.0
    return float(mean(error_list))


def steering_oscillation_penalty(control_list: Sequence[Sequence[float]]) -> float:
    """Compute steering oscillation from consecutive angular velocity deltas."""
    if len(control_list) < 2:
        return 0.0

    omega_values = [float(ctrl[1]) for ctrl in control_list]
    diffs = [abs(omega_values[i] - omega_values[i - 1]) for i in range(1, len(omega_values))]
    return float(mean(diffs)) if diffs else 0.0


def score_log(log: Dict, oscillation_weight: float = 0.3) -> float:
    """Compute run score: higher is better (less negative means better tracking)."""
    error_list = log.get('error', [])
    control_list = log.get('control', [])

    mean_error = mean_cross_track_error(error_list)
    oscillation = steering_oscillation_penalty(control_list)
    return -(mean_error + oscillation_weight * oscillation)


def evaluate_log(log: Dict, oscillation_weight: float = 0.3) -> Dict[str, float]:
    """Return metric dictionary for one log."""
    mean_error = mean_cross_track_error(log.get('error', []))
    oscillation = steering_oscillation_penalty(log.get('control', []))
    score = -(mean_error + oscillation_weight * oscillation)
    return {
        'mean_error': mean_error,
        'oscillation': oscillation,
        'score': score,
        'trajectory_points': len(log.get('trajectory', [])),
        'control_points': len(log.get('control', [])),
    }


def evaluate_multiple_logs(paths: Iterable[str], oscillation_weight: float = 0.3) -> Dict:
    """
    Evaluate multiple logs to support parameter comparison across simulations.

    Returns per-run metrics and aggregate means (recommended for fair comparison).
    """
    results: List[Dict] = []

    for path in paths:
        log = load_log(path)
        metrics = evaluate_log(log, oscillation_weight=oscillation_weight)
        results.append({'path': path, **metrics})

    if not results:
        return {'runs': [], 'aggregate': {}}

    aggregate = {
        'mean_error': float(mean([r['mean_error'] for r in results])),
        'oscillation': float(mean([r['oscillation'] for r in results])),
        'score': float(mean([r['score'] for r in results])),
        'trajectory_points': float(mean([r['trajectory_points'] for r in results])),
        'control_points': float(mean([r['control_points'] for r in results])),
        'num_runs': len(results),
    }

    return {'runs': results, 'aggregate': aggregate}


def _print_report(report: Dict) -> None:
    runs = report.get('runs', [])
    aggregate = report.get('aggregate', {})

    for idx, run in enumerate(runs, start=1):
        print(f'Run {idx}: {run["path"]}')
        print(f'  Mean cross-track error: {run["mean_error"]:.4f} m')
        print(f'  Steering oscillation:   {run["oscillation"]:.4f} rad/s')
        print(f'  Composite score:        {run["score"]:.4f}')
        print(f'  Trajectory points:      {run["trajectory_points"]}')
        print(f'  Control commands:       {run["control_points"]}')
        print('')

    if aggregate:
        print('Aggregate (mean across runs)')
        print(f'  Runs:                   {aggregate["num_runs"]}')
        print(f'  Mean cross-track error: {aggregate["mean_error"]:.4f} m')
        print(f'  Steering oscillation:   {aggregate["oscillation"]:.4f} rad/s')
        print(f'  Composite score:        {aggregate["score"]:.4f}')


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Evaluate Pure Pursuit simulation logs (single or multiple runs).'
    )
    parser.add_argument('logs', nargs='+', help='Path(s) to JSON log files from data_collector')
    parser.add_argument(
        '--oscillation-weight',
        type=float,
        default=0.3,
        help='Weight for oscillation term in score (default: 0.3)',
    )
    return parser


def main() -> None:
    parser = _build_arg_parser()
    args = parser.parse_args()

    if args.oscillation_weight < 0.0:
        raise ValueError('--oscillation-weight must be >= 0.0')

    report = evaluate_multiple_logs(args.logs, oscillation_weight=args.oscillation_weight)
    _print_report(report)


if __name__ == '__main__':
    main()