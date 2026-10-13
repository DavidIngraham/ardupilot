"""Summarize exploratory landing Monte Carlo trials without hiding incomplete runs."""
# AP_FLAKE8_CLEAN
import argparse
import json
from pathlib import Path

from landing_report import messages

import matplotlib

import numpy as np

matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402,I100


def main():
    """Plot the footprint and report empirical spot fractions and paired effects."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    root = args.directory
    rows = json.loads((root / 'results.json').read_text())
    manifest = json.loads((root / 'manifest.json').read_text())
    audits = []
    for row in rows:
        if not row['completed_contact']:
            continue
        directory = root / ('%s-%02d' % (row['kind'], row['trial']))
        logged = messages(directory / 'flight.BIN')
        flare = [item for item in logged['PGFL'] if item['Active']]
        expected = row['config']['flare_pct'] > 0
        assert bool(flare) == expected, (row['kind'], row['trial'], 'flare activation mismatch')
        for item in flare:
            assert 0 <= item['Common'] <= 60.001, (row['trial'], 'common brake bound')
            assert 0 <= item['Left'] <= 100 and 0 <= item['Right'] <= 100, (row['trial'], 'allocation')
        audits.append({'kind': row['kind'], 'trial': row['trial'], 'flare_activation_correct': True})
    moderate = [row for row in rows if row['kind'] == 'moderate']
    completed = [row for row in moderate if row['completed_contact']]
    controls = {row['trial']: row for row in rows if row['kind'] == 'no_flare' and row['completed_contact']}
    if not completed:
        raise RuntimeError('No completed moderate-flare contacts yet')
    landing = [row['landing'] for row in completed]
    radial = np.array([row['radial_error_m'] for row in landing])
    along = np.array([row['along_track_error_m'] for row in landing])
    cross = np.array([row['cross_track_error_m'] for row in landing])
    sink = np.array([row['contact']['velocity'][2] for row in landing])
    speed = np.array([np.linalg.norm(row['contact']['velocity'][:2]) for row in landing])
    wind = np.array([row['config']['wind_ned_m_s'][0] for row in completed])
    pairs = []
    for row in completed:
        if row['trial'] not in controls:
            continue
        control = controls[row['trial']]['landing']
        trial = row['landing']
        pairs.append({
            'trial': row['trial'],
            'radial_change_m': trial['radial_error_m'] - control['radial_error_m'],
            'sink_change_m_s': trial['contact']['velocity'][2] - control['contact']['velocity'][2],
            'groundspeed_change_m_s': float(np.linalg.norm(trial['contact']['velocity'][:2]) -
                                            np.linalg.norm(control['contact']['velocity'][:2])),
        })
    fractions = {str(radius): int(np.sum(radial <= radius)) for radius in (5, 10, 20)}
    summary = {
        'seed': manifest['seed'], 'planned_moderate': manifest['count'],
        'attempted_moderate': len(moderate), 'completed_contacts': len(completed),
        'incomplete_trials': [row['trial'] for row in moderate if not row['completed_contact']],
        'inside_radius_counts': fractions,
        'median_radial_m': float(np.median(radial)), 'p90_radial_m': float(np.percentile(radial, 90)),
        'worst_radial_m': float(np.max(radial)), 'mean_along_m': float(np.mean(along)),
        'mean_cross_m': float(np.mean(cross)),
        'p90_abs_cross_m': float(np.percentile(abs(cross), 90)),
        'p90_radius_about_sample_mean_m': float(np.percentile(
            np.hypot(along-np.mean(along), cross-np.mean(cross)), 90)),
        'median_sink_m_s': float(np.median(sink)),
        'p90_sink_m_s': float(np.percentile(sink, 90)), 'median_groundspeed_m_s': float(np.median(speed)),
        'matched_pairs': pairs, 'controller_checks': audits,
    }
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    footprint, cdf, energy, bias = axes.flat
    for radius in (5, 10, 20):
        footprint.add_patch(plt.Circle((0, 0), radius, fill=False, color='0.7', linestyle='--'))
        footprint.text(radius, 0, '%d m' % radius, fontsize=8)
    points = footprint.scatter(cross, along, c=wind, cmap='coolwarm', vmin=-2, vmax=2,
                               edgecolors='black', linewidths=0.5, zorder=3)
    footprint.scatter([0], [0], marker='+', s=150, color='black', label='Mission target')
    footprint.scatter([np.mean(cross)], [np.mean(along)], marker='*', s=130, color='gold',
                      edgecolors='black', label='Sample mean', zorder=4)
    footprint.legend(loc='upper left', fontsize=8)
    for row in completed:
        if row['trial'] in controls:
            target = controls[row['trial']]['landing']
            footprint.plot([row['landing']['cross_track_error_m'], target['cross_track_error_m']],
                           [row['landing']['along_track_error_m'], target['along_track_error_m']],
                           color='0.65', linewidth=0.8)
            footprint.scatter(target['cross_track_error_m'], target['along_track_error_m'],
                              marker='x', color='0.4', s=35)
    footprint.set(xlabel='Cross-track error (m)', ylabel='Along-track error (m; negative = short)',
                  title='Touchdown footprint; gray crosses = matched no-flare runs')
    footprint.set_aspect('equal', adjustable='datalim')
    fig.colorbar(points, ax=footprint, label='Mean along-track wind (m/s; positive = tailwind)')
    cdf.step(np.r_[0, np.sort(radial), max(20, max(radial))*1.05],
             np.r_[0, np.arange(1, len(radial)+1)/len(moderate), len(radial)/len(moderate)], where='post')
    for radius in (5, 10, 20):
        cdf.axvline(radius, color='0.7', linestyle='--')
    cdf.set(xlabel='Distance from mission target (m)', ylabel='Fraction of attempted moderate trials',
            ylim=(0, 1.05), title='Empirical spot-landing fraction')
    energies = energy.scatter(radial, sink, c=speed, cmap='viridis', edgecolors='black', linewidths=0.5)
    fig.colorbar(energies, ax=energy, label='Contact ground speed (m/s)')
    energy.set(xlabel='Distance from target (m)', ylabel='Contact sink speed (m/s)',
               title='Position accuracy and sink speed are separate outcomes')
    bias.scatter(wind, along)
    bias.axhline(0, color='0.5')
    bias.set(xlabel='Mean along-track wind (m/s; positive = tailwind)',
             ylabel='Along-track error (m)', title='Descriptive wind relationship; other factors also vary')
    for ax in axes.flat:
        ax.grid(alpha=0.2)
    fig.suptitle('Moderate flare: 60%% brake, 30%%/s, 3 m trigger — %d/%d contacts' %
                 (len(completed), len(moderate)))
    fig.tight_layout()
    fig.savefig(root / 'monte-carlo.png', dpi=160)
    (root / 'summary.json').write_text(json.dumps(summary, indent=2))
    counts = ', '.join('%s/%s inside %s m' % (fractions[str(r)], len(moderate), r) for r in (5, 10, 20))
    pair_text = 'Matched no-flare comparisons are still running.'
    if pairs:
        pair_text = ('Across %d matched pairs, moderate flare changed radial error by %.2f m, '
                     'sink speed by %.2f m/s and ground speed by %.2f m/s on average. '
                     'Negative values mean reductions.' % (
                         len(pairs), np.mean([p['radial_change_m'] for p in pairs]),
                         np.mean([p['sink_change_m_s'] for p in pairs]),
                         np.mean([p['groundspeed_change_m_s'] for p in pairs])))
    table = ''.join('<tr><td>%d</td><td>%.1f</td><td>%.1f</td><td>%.1f</td><td>%.2f</td></tr>' % (
        row['trial'], row['landing']['radial_error_m'], row['landing']['along_track_error_m'],
        row['landing']['cross_track_error_m'], row['landing']['contact']['velocity'][2]) for row in completed)
    html = (
        '<!doctype html><meta charset="utf-8"><title>Micro AGU landing Monte Carlo</title>'
        '<style>body{font:17px system-ui;max-width:1200px;margin:40px auto;padding:0 20px;line-height:1.5}'
        'img{width:100%%}td,th{padding:6px 16px;text-align:right;border-bottom:1px solid #ddd}</style>'
        '<h1>Can the moderate flare spot-land?</h1>'
        '<p>Seed %d. %d of %d planned moderate-flare trials attempted; %d reached the ground plane. '
        'Incomplete trial IDs: %s.</p>' % (
            manifest['seed'], len(moderate), manifest['count'], len(completed), summary['incomplete_trials']) +
        '<p><strong>' + counts + '.</strong> Median radial error %.1f m; 90th percentile %.1f m. '
        'Mean along-track error %.1f m (negative means short). Median sink speed %.2f m/s.</p>' %
        (summary['median_radial_m'], summary['p90_radial_m'], summary['mean_along_m'], summary['median_sink_m_s']) +
        '<p>' + pair_text + '</p><img src="monte-carlo.png" alt="Landing footprint and accuracy distributions">'
        '<h2>What varied</h2><p>Independent uniform draws: mean wind magnitude 0–2 m/s with any direction; '
        'horizontal smooth-gust amplitude 0–0.45 m/s (vertical amplitude one quarter), periods 8–20 s; '
        'approach cross-track offset ±10 m, height ±5 m, heading ±15 degrees, initial speed 0.9–1.1 times trim; '
        'battery 20.4–22.2 V; independent brake lift/drag multipliers 0.6–1.4; '
        'actuator travel rate 80–220 mm/s and command delay 0–0.12 s; landing-datum error ±0.5 m. '
        'Mass and rigging use the Trout Lake profile. No gains or flare settings were optimized on these draws.</p>'
        '<h2>A likely source of short landings</h2><p>The tested setup retains LAND_FLARE_AIM=50 and '
        'TECS_LAND_SINK=0.25 m/s. AP_Landing estimates flare duration from a blend of approach sink and that '
        'configured flare sink, then moves the approach aim point upstream by ground speed times duration. '
        'The observed higher contact sink suggests that this can overestimate the available flare distance. '
        'That is a mechanism to test, not a causal conclusion from this Monte Carlo. '
        'A next experiment can adjust the existing landing aim/sink assumptions on matched cases, '
        'then assess fresh held-out cases; no new spot-landing controller is implied.</p>'
        '<h2>How to interpret this</h2><p>These are exploratory frequencies under chosen engineering assumptions, '
        'not real-world success probabilities. The sample is small and cannot resolve rare failures. '
        'Full native ArduPilot controllers and AP_Landing run in SITL, with the existing two-body physics. '
        'Contact means the payload CG crosses a flat ground plane, interpolated within the integration frame. '
        'No ground contact, canopy collapse or validated high-brake stall response is modeled. '
        'The gust signal is smooth and deterministic, not a validated turbulence spectrum. '
        'The landing-datum offset exercises existing landing-height logic; there is no new height estimator. '
        'Incomplete runs remain in the attempted-trial denominator; pending runs are not counted.</p>'
        '<h2>Individual completed moderate-flare trials</h2><table><tr><th>Trial</th><th>Radial (m)</th>'
        '<th>Along (m)</th><th>Cross (m)</th><th>Sink (m/s)</th></tr>' + table + '</table>')
    (root / 'report.html').write_text(html)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
