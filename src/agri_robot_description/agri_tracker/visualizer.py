#!/usr/bin/env python3

import argparse
import json
import os
import re
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import matplotlib
import numpy as np


def _select_matplotlib_backend(show_plots: bool) -> None:
    """Pick a safe backend before importing pyplot.

    When no display is available, force a non-interactive backend so ROS launch
    jobs can run in headless environments without hanging or crashing.
    """

    display_available = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    if not show_plots or not display_available:
        matplotlib.use('Agg', force=True)


_select_matplotlib_backend(show_plots=True)

import matplotlib.pyplot as plt
import matplotlib.animation as mpl_animation
import matplotlib.patches as patches


def load_log(path: str) -> Dict[str, Any]:
    with Path(path).open('r', encoding='utf-8') as f:
        return json.load(f)


def load_tuning_results(path: str) -> Dict[str, List[Dict[str, Any]]]:
    with Path(path).open('r', encoding='utf-8') as f:
        data = json.load(f)

    if isinstance(data, dict):
        if 'trials' in data or 'summary' in data:
            return {
                'trials': data.get('trials', []),
                'summary': data.get('summary', []),
            }

    if isinstance(data, list):
        # Legacy list-of-trials format.
        return {'trials': data, 'summary': []}

    raise ValueError('Unrecognized tuning results format')


def parse_log_filename(log_path: str) -> Optional[Dict[str, float]]:
    filename = Path(log_path).stem
    match = re.match(r'log_.*_L([\d.]+)_V([\d.]+)$', filename)
    if not match:
        match = re.match(r'log_L([\d.]+)_v([\d.]+)$', filename)
    if not match:
        return None
    return {
        'lookahead_distance': float(match.group(1)),
        'linear_speed': float(match.group(2)),
    }


def _title_suffix(lookahead_distance: Any, linear_speed: Any) -> str:
    return f'L={lookahead_distance}, v={linear_speed}'


def _set_axis_limits(ax: plt.Axes, waypoints: np.ndarray, trajectory: np.ndarray) -> None:
    points = np.vstack([arr for arr in (waypoints, trajectory) if arr.size]) if (waypoints.size or trajectory.size) else None
    if points is None:
        return

    x_min = float(np.min(points[:, 0]))
    x_max = float(np.max(points[:, 0]))
    y_min = float(np.min(points[:, 1]))
    y_max = float(np.max(points[:, 1]))

    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)
    padding = 0.08 * max(x_span, y_span)

    ax.set_xlim(x_min - padding, x_max + padding)
    ax.set_ylim(y_min - padding, y_max + padding)


def _animate_trajectory(
    fig: plt.Figure,
    ax: plt.Axes,
    trajectory: np.ndarray,
    trail_line: plt.Line2D,
    robot_point: plt.Line2D,
    time_text: plt.Text,
    title_prefix: str,
    interval_msec: int = 100,
) -> mpl_animation.FuncAnimation:
    def _init() -> Tuple[plt.Line2D, plt.Line2D, plt.Text]:
        trail_line.set_data([], [])
        robot_point.set_data([], [])
        time_text.set_text('')
        return trail_line, robot_point, time_text

    def _update(frame_index: int) -> Tuple[plt.Line2D, plt.Line2D, plt.Text]:
        current_path = trajectory[: frame_index + 1]
        trail_line.set_data(current_path[:, 0], current_path[:, 1])
        robot_point.set_data([trajectory[frame_index, 0]], [trajectory[frame_index, 1]])
        time_text.set_text(f'Step {frame_index + 1}/{len(trajectory)}')
        ax.set_title(f'{title_prefix} | Step {frame_index + 1}/{len(trajectory)}')
        return trail_line, robot_point, time_text

    animation = mpl_animation.FuncAnimation(
        fig,
        _update,
        init_func=_init,
        frames=len(trajectory),
        interval=interval_msec,
        blit=False,
        repeat=False,
    )
    fig._trajectory_animation = animation  # Keep a strong reference for plt.show().
    return animation


def _animate_replay(
    log: Dict[str, Any],
    lookahead_distance: Any,
    linear_speed: Any,
) -> Optional[mpl_animation.FuncAnimation]:
    waypoints = np.asarray(log.get('waypoints', []), dtype=float)
    trajectory = np.asarray(log.get('trajectory', []), dtype=float)
    control = np.asarray(log.get('control', []), dtype=float)

    if not trajectory.size:
        return None

    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(f'Robot Trajectory ({_title_suffix(lookahead_distance, linear_speed)})')

    ax_main = axes[0]
    ax_main.set_title('Trajectory')
    ax_main.set_xlabel('X (meters)')
    ax_main.set_ylabel('Y (meters)')
    ax_main.grid(True, alpha=0.3)
    ax_main.axis('equal')
    _set_axis_limits(ax_main, waypoints, trajectory)

    if waypoints.size:
        ax_main.plot(
            waypoints[:, 0],
            waypoints[:, 1],
            'b--',
            linewidth=1.5,
            alpha=0.5,
            label='Reference',
        )
        ax_main.plot(waypoints[0, 0], waypoints[0, 1], 'gs', markersize=8, label='Start')

    ax_main.plot(trajectory[-1, 0], trajectory[-1, 1], 'r*', markersize=12, label='End')
    (trail_line,) = ax_main.plot([], [], 'g-', linewidth=2.0, alpha=0.85, label='Actual')
    agent_circle = patches.Circle(
        (trajectory[0, 0], trajectory[0, 1]),
        radius=0.4,
        color='royalblue',
        zorder=5,
    )
    ax_main.add_patch(agent_circle)
    time_text = ax_main.text(0.02, 0.95, '', transform=ax_main.transAxes, fontsize=10, va='top')
    ax_main.legend(loc='upper right', fontsize=8)

    ax_steer = axes[1]
    ax_steer.set_title('Angular Velocity (rad/s)')
    ax_steer.set_xlabel('step')
    ax_steer.set_ylabel('angular (rad/s)')
    ax_steer.grid(True, alpha=0.3)
    steer_line, = ax_steer.plot([], [], 'r-', linewidth=1.5)

    ax_vel = axes[2]
    ax_vel.set_title('Linear Velocity (m/s)')
    ax_vel.set_xlabel('step')
    ax_vel.set_ylabel('velocity (m/s)')
    ax_vel.grid(True, alpha=0.3)
    vel_line, = ax_vel.plot([], [], 'b-', linewidth=1.5)

    if control.size:
        linear_values = control[:, 0]
        angular_values = control[:, 1]
    else:
        linear_values = np.zeros(len(trajectory), dtype=float)
        angular_values = np.zeros(len(trajectory), dtype=float)

    def _init() -> Tuple[Any, ...]:
        trail_line.set_data([], [])
        steer_line.set_data([], [])
        vel_line.set_data([], [])
        time_text.set_text('')
        agent_circle.center = (trajectory[0, 0], trajectory[0, 1])
        return trail_line, agent_circle, time_text, steer_line, vel_line

    def _update(frame_index: int) -> Tuple[Any, ...]:
        current_path = trajectory[: frame_index + 1]
        trail_line.set_data(current_path[:, 0], current_path[:, 1])
        agent_circle.center = (trajectory[frame_index, 0], trajectory[frame_index, 1])
        time_text.set_text(f'Step {frame_index + 1}/{len(trajectory)}')

        control_index = min(frame_index, len(linear_values) - 1)
        steps = np.arange(control_index + 1)
        steer_line.set_data(steps, angular_values[: control_index + 1])
        vel_line.set_data(steps, linear_values[: control_index + 1])

        ax_steer.set_xlim(0, max(frame_index + 1, 1))
        ax_vel.set_xlim(0, max(frame_index + 1, 1))

        if control_index >= 0:
            steer_seen = angular_values[: control_index + 1]
            steer_span = max(float(np.max(np.abs(steer_seen))) if steer_seen.size else 0.0, 0.2)
            ax_steer.set_ylim(-steer_span * 1.2, steer_span * 1.2)

            vel_seen = linear_values[: control_index + 1]
            vel_max = float(np.max(vel_seen)) if vel_seen.size else 0.0
            ax_vel.set_ylim(0.0, max(vel_max * 1.2, 1.0))

        return trail_line, agent_circle, time_text, steer_line, vel_line

    animation = mpl_animation.FuncAnimation(
        fig,
        _update,
        init_func=_init,
        frames=len(trajectory),
        interval=100,
        blit=False,
        repeat=False,
    )
    fig._trajectory_animation = animation  # Keep a strong reference for plt.show().
    plt.tight_layout()
    return animation


def plot_trajectory(
    log: Dict[str, Any],
    lookahead_distance: Any,
    linear_speed: Any,
    output_path: Optional[str] = None,
    show: bool = False,
) -> None:
    waypoints = np.asarray(log.get('waypoints', []), dtype=float)
    trajectory = np.asarray(log.get('trajectory', []), dtype=float)

    if show and trajectory.size:
        animation = _animate_replay(log, lookahead_distance, linear_speed)
        if animation is None:
            show = False
        else:
            if output_path:
                # Save the current frame snapshot for headless artifact workflows.
                plt.gcf().savefig(output_path, dpi=150, bbox_inches='tight')
                print(f'Saved: {output_path}')
            plt.show()
            return

    fig, ax = plt.subplots(figsize=(10, 8))

    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    _set_axis_limits(ax, waypoints, trajectory)

    if waypoints.size:
        ax.plot(
            waypoints[:, 0],
            waypoints[:, 1],
            'r--o',
            linewidth=2,
            markersize=6,
            label='Reference waypoints',
        )

    if trajectory.size:
        ax.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=9, label='Start', zorder=5)
        ax.plot(trajectory[-1, 0], trajectory[-1, 1], 'r*', markersize=12, label='End', zorder=5)

        trail_line, = ax.plot([], [], 'b-', linewidth=2.0, label='Robot path', zorder=4)
        robot_point, = ax.plot([], [], 'bo', markersize=7, zorder=6)
        time_text = ax.text(
            0.02,
            0.96,
            '',
            transform=ax.transAxes,
            fontsize=12,
            va='top',
            bbox={'facecolor': 'white', 'alpha': 0.75, 'edgecolor': 'none'},
        )

        ax.legend(loc='best')
        fig.tight_layout()

        if output_path:
            fig.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f'Saved: {output_path}')

        if show:
            _animate_trajectory(
                fig,
                ax,
                trajectory,
                trail_line,
                robot_point,
                time_text,
                f'Robot Trajectory ({_title_suffix(lookahead_distance, linear_speed)})',
            )
            plt.show()
        else:
            plt.close(fig)
        return

    ax.set_title(f'Robot Trajectory ({_title_suffix(lookahead_distance, linear_speed)})')
    ax.legend(loc='best')

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_error(
    log: Dict[str, Any],
    lookahead_distance: Any,
    linear_speed: Any,
    output_path: Optional[str] = None,
    show: bool = False,
) -> None:
    error_list = np.asarray(log.get('error', []), dtype=float)
    fig, ax = plt.subplots(figsize=(12, 6))

    if error_list.size:
        ax.plot(error_list, 'b-', linewidth=1.5, label='Cross-track error')
        mean_error = float(np.mean(error_list))
        std_error = float(np.std(error_list))
        ax.axhline(mean_error, color='r', linestyle='--', linewidth=2, label=f'Mean: {mean_error:.4f} m')
        ax.fill_between(
            np.arange(len(error_list)),
            mean_error - std_error,
            mean_error + std_error,
            alpha=0.2,
            color='orange',
            label=f'+/-1 sigma: {std_error:.4f} m',
        )

    ax.set_xlabel('Time step')
    ax.set_ylabel('Cross-track error (meters)')
    ax.set_title(f'Cross-Track Error ({_title_suffix(lookahead_distance, linear_speed)})')
    ax.grid(True, alpha=0.3)
    ax.legend()

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')

    if show:
        plt.show()
    else:
        plt.close(fig)


def plot_control(
    log: Dict[str, Any],
    lookahead_distance: Any,
    linear_speed: Any,
    output_path: Optional[str] = None,
    show: bool = False,
) -> None:
    control = np.asarray(log.get('control', []), dtype=float)
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))

    if control.size:
        linear = control[:, 0]
        angular = control[:, 1]

        ax1.plot(linear, 'g-', linewidth=1.5, label='Linear velocity')
        ax1.axhline(float(np.mean(linear)), color='darkgreen', linestyle='--', linewidth=1.5)
        ax1.legend()

        ax2.plot(angular, color='tab:purple', linewidth=1.5, label='Angular velocity')
        ax2.axhline(0.0, color='black', linewidth=0.8, alpha=0.5)
        osc = float(np.mean(np.abs(np.diff(angular)))) if len(angular) > 1 else 0.0
        ax2.axhline(osc, color='red', linestyle='--', linewidth=1.0, alpha=0.7, label=f'Oscillation: {osc:.4f}')
        ax2.axhline(-osc, color='red', linestyle='--', linewidth=1.0, alpha=0.7)
        ax2.legend()

    ax1.set_ylabel('Linear velocity (m/s)')
    ax1.set_title(f'Control Outputs ({_title_suffix(lookahead_distance, linear_speed)})')
    ax1.grid(True, alpha=0.3)
    ax2.set_xlabel('Time step')
    ax2.set_ylabel('Angular velocity (rad/s)')
    ax2.grid(True, alpha=0.3)
    fig.tight_layout()

    if output_path:
        fig.savefig(output_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {output_path}')

    if show:
        plt.show()
    else:
        plt.close(fig)


def _normalize_trial_record(trial: Dict[str, Any]) -> Dict[str, Any]:
    # Supports both legacy {'params': {...}, 'score': ...} and current tuner format.
    if 'params' in trial:
        p = trial.get('params', {})
        return {
            'lookahead_distance': p.get('lookahead_distance'),
            'linear_speed': p.get('linear_speed'),
            'score': trial.get('score', float('-inf')),
            'trial_id': trial.get('trial_id', ''),
        }
    return {
        'lookahead_distance': trial.get('lookahead_distance'),
        'linear_speed': trial.get('linear_speed'),
        'score': trial.get('score', float('-inf')),
        'trial_id': trial.get('trial_id', ''),
    }


def plot_tuning_results(
    trial_results: Sequence[Dict[str, Any]],
    summary_results: Sequence[Dict[str, Any]],
    output_dir: Path,
    show: bool = False,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)

    normalized_trials = [_normalize_trial_record(t) for t in trial_results]
    normalized_trials = [t for t in normalized_trials if np.isfinite(float(t['score']))]
    if not normalized_trials and not summary_results:
        print('No valid tuning results to plot.')
        return

    # Plot 1: trial score progression (visualize tuning process).
    if normalized_trials:
        fig1, ax1 = plt.subplots(figsize=(12, 5))
        scores = [float(t['score']) for t in normalized_trials]
        ax1.plot(range(1, len(scores) + 1), scores, marker='o', linewidth=1.3)
        best_idx = int(np.argmax(scores))
        ax1.scatter(best_idx + 1, scores[best_idx], color='green', s=90, label='Best trial')
        ax1.set_xlabel('Trial index')
        ax1.set_ylabel('Score')
        ax1.set_title('Tuning Process: Trial Score Progression')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        progression_path = output_dir / 'tuning_process_progression.png'
        fig1.savefig(progression_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {progression_path}')
        if show:
            plt.show()
        else:
            plt.close(fig1)

    # Plot 2: parameter effectiveness map.
    source = list(summary_results) if summary_results else normalized_trials
    xs: List[float] = []
    ys: List[float] = []
    cs: List[float] = []
    for row in source:
        lx = row.get('lookahead_distance')
        sv = row.get('linear_speed')
        sc = row.get('avg_score', row.get('score'))
        if lx is None or sv is None:
            continue
        if not np.isfinite(float(sc)):
            continue
        xs.append(float(lx))
        ys.append(float(sv))
        cs.append(float(sc))

    if xs:
        fig2, ax2 = plt.subplots(figsize=(10, 7))
        scatter = ax2.scatter(xs, ys, c=cs, cmap='viridis', s=220, edgecolors='black')
        best_idx2 = int(np.argmax(cs))
        ax2.scatter([xs[best_idx2]], [ys[best_idx2]], marker='*', s=350, color='red', label='Best')
        ax2.set_xlabel('Lookahead distance')
        ax2.set_ylabel('Linear speed')
        ax2.set_title('Parameter Effectiveness (color = score)')
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        cbar = fig2.colorbar(scatter, ax=ax2)
        cbar.set_label('Score')
        map_path = output_dir / 'tuning_parameter_effectiveness.png'
        fig2.savefig(map_path, dpi=150, bbox_inches='tight')
        print(f'Saved: {map_path}')
        if show:
            plt.show()
        else:
            plt.close(fig2)


def plot_single_run(log_path: str, output_dir: Path, show: bool = False) -> None:
    log = load_log(log_path)
    params = parse_log_filename(log_path)
    lookahead = params['lookahead_distance'] if params else 'unknown'
    speed = params['linear_speed'] if params else 'unknown'

    output_dir.mkdir(parents=True, exist_ok=True)
    run_id = Path(log_path).stem

    plot_trajectory(
        log,
        lookahead,
        speed,
        output_path=str(output_dir / f'trajectory_{run_id}.png'),
        show=show,
    )
    plot_error(
        log,
        lookahead,
        speed,
        output_path=str(output_dir / f'error_{run_id}.png'),
        show=show,
    )
    plot_control(
        log,
        lookahead,
        speed,
        output_path=str(output_dir / f'control_{run_id}.png'),
        show=show,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='Visualize tracker runs and tuning results')
    parser.add_argument('--run', type=str, help='Path to single run log JSON')
    parser.add_argument('--tuning', type=str, help='Path to tuning results JSON')
    parser.add_argument('--plots-dir', type=str, default='plots', help='Directory to save plot images')
    parser.add_argument('--show', action='store_true', help='Force interactive windows')
    parser.add_argument('--no-show', action='store_true', help='Disable interactive windows (save only)')
    return parser


def main() -> None:
    args = build_parser().parse_args()

    display_available = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
    show_plots = display_available and (args.show or not args.no_show)

    _select_matplotlib_backend(show_plots)

    if not args.run and not args.tuning:
        raise ValueError('Provide --run and/or --tuning')

    plots_dir = Path(args.plots_dir)

    if args.run:
        if not Path(args.run).exists():
            raise FileNotFoundError(f'Run log file not found: {args.run}')
        print(f'Plotting single run: {args.run}')
        plot_single_run(args.run, output_dir=plots_dir, show=show_plots)

    if args.tuning:
        if not Path(args.tuning).exists():
            raise FileNotFoundError(f'Tuning results file not found: {args.tuning}')
        tuning = load_tuning_results(args.tuning)
        print(f'Plotting tuning results: {args.tuning}')
        plot_tuning_results(
            trial_results=tuning.get('trials', []),
            summary_results=tuning.get('summary', []),
            output_dir=plots_dir,
            show=show_plots,
        )


if __name__ == '__main__':
    main()