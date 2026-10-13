"""Check landing control invariants and plot the recorded SITL comparisons."""
# AP_FLAKE8_CLEAN
import argparse
import json
from pathlib import Path

import matplotlib

import numpy as np

from pymavlink import mavutil

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402,I100


def messages(path):
    """Read only the messages needed for flare checks."""
    result = {name: [] for name in ['PGFL', 'PGYR', 'RCOU', 'LAND', 'MSG']}
    log = mavutil.mavlink_connection(str(path))
    while True:
        msg = log.recv_match(type=list(result))
        if msg is None:
            break
        result[msg.get_type()].append(msg.to_dict())
    return result


def main():
    """Validate allocation/reset/throttle behavior and save comparable plots."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    summaries = []
    for directory in args.directories:
        summary = json.loads((directory / 'landing.json').read_text())
        case = summary['case']
        logs = messages(directory / 'flight.BIN')
        flare = [row for row in logs['PGFL'] if row['Active']]
        expected = case not in ('disabled', 'abort')
        assert bool(flare) == expected, (case, 'flare activation')
        for row in logs['PGFL']:
            assert 0 <= row['Left'] <= 100 and 0 <= row['Right'] <= 100, (case, row)
            assert 0 <= row['Common'] <= (80 if case == 'reserve' else 60), (case, row)
            if row['Active']:
                assert abs(min(row['Left'], row['Right']) - row['Common']) < 1e-4, (case, row)
            else:
                assert row['Common'] == 0, (case, 'stale common brake')
        times = [row['TimeUS'] for row in logs['PGFL']]
        for row in logs['PGYR']:
            index = int(np.searchsorted(times, row['TimeUS'], side='left')) - 1
            if index >= 0 and logs['PGFL'][index]['Active']:
                limit = 1 - logs['PGFL'][index]['Common'] * 0.01
                assert abs(row['Out']) <= limit + 1e-4, (case, 'steering exceeds allocated travel')
        for before, after in zip(flare, flare[1:]):
            dt = (after['TimeUS'] - before['TimeUS']) * 1e-6
            assert after['Common'] - before['Common'] <= 30 * dt + 0.01, (case, 'ramp rate')
        if flare:
            t0 = flare[0]['TimeUS'] * 1e-6
            t1 = flare[-1]['TimeUS'] * 1e-6
            # This study uses 100 percent/second throttle slew; retain that
            # existing actuator ramp and require idle after one full-scale second.
            throttle = [row['C1'] for row in logs['RCOU'] if t0 + 1.0 < row['TimeUS']*1e-6 < t1]
            assert throttle and max(throttle) <= 1001, (case, 'flare throttle not cut')
            summary['flare_start_s'] = t0
            summary['max_common_pct'] = max(row['Common'] for row in flare)
            if case == 'reserve':
                assert abs(summary['max_common_pct'] - 80) < 1e-4, 'reserve cap not exercised'
            if case == 'takeover':
                cleared = [row for row in logs['PGFL'] if row['TimeUS'] > flare[-1]['TimeUS']]
                assert cleared and all(row['Common'] == 0 for row in cleared), 'takeover reset'
        else:
            t0 = None
        contact = summary['contact']
        if contact:
            summary['contact_sink_m_s'] = contact['velocity'][2]
            summary['contact_groundspeed_m_s'] = float(np.linalg.norm(contact['velocity'][:2]))
        summary['allocation_checks_passed'] = True
        summaries.append(summary)
        if case in ('nominal', 'disabled', 'weak', 'strong', 'crosswind', 'timed', 'reserve'):
            data = np.genfromtxt(directory / 'landing.csv', delimiter=',', names=True)
            end = contact['time']
            time = data['time'] - end
            selected = (time >= -12) & (time <= 0)
            line, = axes[0].plot(time[selected], data['height'][selected], label=case)
            axes[1].plot(time[selected], data['vd'][selected], color=line.get_color())
            if flare:
                ft = np.array([row['TimeUS']*1e-6-end for row in logs['PGFL']])
                common = [row['Common'] for row in logs['PGFL']]
                axes[2].plot(ft, common, color=line.get_color())
    axes[0].set_ylabel('Payload height (m)')
    axes[1].set_ylabel('Downward speed (m/s)')
    axes[2].set_ylabel('Common brake (%)')
    axes[2].set_xlabel('Time before first ground-plane crossing (s)')
    axes[0].legend(ncol=4)
    axes[2].set_xlim(-12, 0)
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.suptitle('Native AUTO landing: uncertain flare aerodynamics; no impact model')
    fig.tight_layout()
    fig.savefig(args.output / 'landing-comparison.png', dpi=170)
    (args.output / 'landing-results.json').write_text(json.dumps(summaries, indent=2))
    rows = ''.join(
        '<tr><td>{}</td><td>{}</td><td>{}</td></tr>'.format(
            row['case'],
            '{:.2f}'.format(row['contact_sink_m_s']) if 'contact_sink_m_s' in row else '—',
            '{:.2f}'.format(row['contact_groundspeed_m_s']) if 'contact_groundspeed_m_s' in row else '—')
        for row in summaries)
    report = (
        '<!doctype html><meta charset="utf-8"><title>Micro AGU landing study</title>'
        '<style>body{font:17px system-ui;max-width:1050px;margin:40px auto;padding:0 20px;line-height:1.5}'
        'img{width:100%}td,th{padding:8px 20px;text-align:left;border-bottom:1px solid #ddd}</style>'
        '<h1>Landing through the existing ArduPilot flare state</h1>'
        '<p>The approach uses the existing throttle-only controller. AP_Landing triggers flare; '
        'a bounded common-brake ramp consumes part of the available steering travel. '
        'The lateral controller accounts for that limit, and TECS clears throttle integration '
        'while existing landing logic suppresses thrust. Automatic brake flare defaults to disabled.</p>'
        '<p>All recorded cases passed the applicable activation, allocation, ramp, steering-limit, '
        'reset and throttle checks. The separate native SITL landing test passed flare, idle '
        'throttle, ground handling and automatic disarm.</p>'
        '<img src="landing-comparison.png" alt="Height, sink speed and common brake before contact">'
        '<table><tr><th>Case</th><th>Contact sink (m/s)</th><th>Ground speed (m/s)</th></tr>' + rows + '</table>'
        '<p>Weak/strong scale brake lift and drag coefficients by 0.5/1.5. Crosswind is 1.5 m/s. '
        'Reserve uses an earlier 8 m flare and reaches the 80% common-brake cap. '
        'Takeover intentionally releases automatic brake near the ground; it is a reset test, '
        'not a landing-performance candidate. Abort commands go-around before flare.</p>'
        '<p>The two-body runs end at a payload-CG crossing of a flat ground plane. '
        'Impact, canopy collapse and high-brake stall response are not validated. '
        'The results show a useful first implementation, not flight-qualified flare settings. '
        'Next compare trigger timing and brake schedules, and measure response and steering margin aloft.</p>')
    (args.output / 'report.html').write_text(report)
    print(json.dumps(summaries, indent=2))


if __name__ == '__main__':
    main()
