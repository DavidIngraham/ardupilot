"""Run seeded native-SITL landing trials and matched no-flare controls."""
# AP_FLAKE8_CLEAN
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def configurations(count, seed):
    """Draw independent uniform uncertainties; these are engineering assumptions."""
    rng = np.random.default_rng(seed)
    trials = []
    for index in range(count):
        speed = rng.uniform(0, 2)
        direction = rng.uniform(-np.pi, np.pi)
        trials.append({
            'trial': index, 'flare_pct': 60,
            'wind_ned_m_s': [speed*np.cos(direction), speed*np.sin(direction), 0.],
            'gust_amplitude_m_s': rng.uniform(0, 0.45),
            'gust_period_s': rng.uniform(8, 20),
            'gust_phase_rad': rng.uniform(-np.pi, np.pi),
            'cross_track_error_m': rng.uniform(-10, 10),
            'height_error_m': rng.uniform(-5, 5),
            'heading_error_deg': rng.uniform(-15, 15),
            'speed_scale': rng.uniform(0.9, 1.1),
            'voltage_v': rng.uniform(20.4, 22.2),
            'brake_lift_scale': rng.uniform(0.6, 1.4),
            'brake_drag_scale': rng.uniform(0.6, 1.4),
            'actuator_rate_mm_s': rng.uniform(80, 220),
            'actuator_delay_s': rng.uniform(0, 0.12),
            'landing_datum_error_m': rng.uniform(-0.5, 0.5),
        })
    return trials


def main():
    """Run each trial in a fresh SITL process and preserve failures in the denominator."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--count', type=int, default=24)
    parser.add_argument('--pairs', type=int, default=8)
    parser.add_argument('--seed', type=int, default=20261013)
    parser.add_argument('--flare-aim', type=float, default=50)
    parser.add_argument('--flare-sink', type=float, default=0.25)
    parser.add_argument('--record-viewer', action='store_true')
    args = parser.parse_args()
    if args.count < 1 or not 0 <= args.pairs <= args.count:
        parser.error('require count >= 1 and 0 <= pairs <= count')
    if not 0 <= args.flare_aim <= 100 or not 0 <= args.flare_sink <= 2:
        parser.error('require flare aim in [0, 100] and flare sink in [0, 2] m/s')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = output / 'manifest.json'
    trials = configurations(args.count, args.seed)
    trials = [row | {'flare_aim_pct': args.flare_aim, 'flare_sink_m_s': args.flare_sink,
                     'record_viewer': args.record_viewer} for row in trials]
    jobs = [('moderate', row) for row in trials]
    jobs += [('no_flare', row | {'flare_pct': 0}) for row in trials[:args.pairs]]
    content = {'seed': args.seed, 'count': args.count, 'pairs': args.pairs,
               'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip(),
               'trials': trials}
    if manifest.exists():
        previous = json.loads(manifest.read_text())
        if previous['trials'] != trials or previous['pairs'] != args.pairs:
            raise RuntimeError('Existing manifest differs; choose another output directory')
    else:
        manifest.write_text(json.dumps(content, indent=2))
    results = []
    for kind, config in jobs:
        directory = output / ('%s-%02d' % (kind, config['trial']))
        directory.mkdir(exist_ok=True)
        record = directory / 'result.json'
        if record.exists():
            results.append(json.loads(record.read_text()))
            continue
        config_path = directory / 'config.json'
        config_path.write_text(json.dumps(config, indent=2))
        env = os.environ.copy()
        env.update(PG_LANDING_CASE='nominal', PG_LANDING_CONFIG=str(config_path), BUILDLOGS=str(directory))
        command = ['python', 'Tools/simulation/paraglider/landing_autotest.py',
                   '--no-check-parameter-leaks', 'test.Plane.ParagliderLandingStudy']
        start = time.monotonic()
        with (directory / 'autotest.log').open('w') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            try:
                returncode = process.wait(timeout=360)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait()
                returncode = -1
        result = {'kind': kind, 'trial': config['trial'], 'config': config,
                  'returncode': returncode, 'wall_seconds': time.monotonic()-start}
        summary = directory / 'landing.json'
        if summary.exists():
            result['landing'] = json.loads(summary.read_text())
        result['completed_contact'] = returncode == 0 and bool(result.get('landing', {}).get('contact'))
        record.write_text(json.dumps(result, indent=2))
        results.append(result)
        (output / 'results.json').write_text(json.dumps(results, indent=2))
        print(kind, config['trial'], 'contact' if result['completed_contact'] else 'FAILED',
              result.get('landing', {}).get('radial_error_m'), flush=True)
    (output / 'results.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
