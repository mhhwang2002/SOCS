"""Plot speed and battery energy from the DEVS motion collector."""

from argparse import ArgumentParser
from json import loads
from math import ceil
from pathlib import Path

from ..motion import ExtendedProfile, MovingProfile
from .run_crossing_energy_comparison import CASES, DEFAULT_CONFIG, collect_motion, cumulative_energy


def profile_breakpoints(profile):
    if isinstance(profile, ExtendedProfile):
        prefix = (t for t in profile_breakpoints(profile.original) if t < profile.switch_time)
        return (*prefix, *(profile.switch_time + t for t in profile_breakpoints(profile.tail)))
    acceleration_end = (profile.acceleration_time if isinstance(profile, MovingProfile)
                        else profile.peak / profile.acceleration)
    return 0., acceleration_end, profile.brake_start, profile.duration


def speed_points(collector, name, horizon):
    points = [(0., 0.)]
    for start, end, origin, profile in collector.motion_intervals(name):
        times = {start, end}
        times.update(origin + t for t in profile_breakpoints(profile) if start < origin + t < end)
        points.extend((t, profile.velocity_at(t - origin)) for t in sorted(times))
    return [*points, (horizon, 0.)]


def save_plot(output, horizon, y_max, title, axis_label, series,
              subtitle='Rolling passage: Forbidden | Stopping in intersection: Allowed | Rack-carrying robots',
              *, font_scale=1):
    from reportlab.lib.colors import HexColor
    from reportlab.pdfgen.canvas import Canvas

    x_max = 5 * ceil(horizon / 5)
    left, bottom, width, height = (82, 60, 513, 178) if font_scale == 2 else (62, 53, 533, 185)
    x = lambda t: left + t * width / x_max
    y = lambda v: bottom + v * height / y_max
    pdf = Canvas(str(output), pagesize=(620, 355 if font_scale == 2 else 325))
    pdf.setTitle(title)
    pdf.setFillColor(HexColor('#152938'))
    pdf.setFont('Helvetica-Bold', 16 * font_scale)
    pdf.drawString(left, 316 if font_scale == 2 else 295, title)
    pdf.setFont('Helvetica', 10 * font_scale)
    if font_scale == 2:
        policy, stopping, load = subtitle.split(' | ')
        pdf.drawString(left, 290, f'{policy} | {load}')
        pdf.drawString(left, 268, stopping)
    else:
        pdf.drawString(left, 276, subtitle)
    pdf.setFont('Helvetica', 9 * font_scale)
    pdf.setLineWidth(.5)
    pdf.setStrokeColor(HexColor('#DFE5E9'))
    for tick in range(0, x_max + 1, 5):
        pdf.line(x(tick), bottom, x(tick), bottom + height)
        pdf.drawCentredString(x(tick), bottom - (20 if font_scale == 2 else 15), str(tick))
    for i in range(5):
        tick = y_max * i / 4
        pdf.line(left, y(tick), left + width, y(tick))
        pdf.drawRightString(left - 8, y(tick) - 3, f'{tick:g}')
    pdf.setStrokeColor(HexColor('#526575'))
    pdf.line(left, bottom, left + width, bottom)
    pdf.line(left, bottom, left, bottom + height)
    pdf.drawCentredString(left + width / 2, bottom - (43 if font_scale == 2 else 32), 'Simulation time (s)')
    pdf.saveState()
    pdf.translate(20, bottom + height / 2)
    pdf.rotate(90)
    pdf.drawCentredString(0, 0, axis_label)
    pdf.restoreState()
    for index, (label, color, dash, points) in enumerate(series):
        legend_x = left + index * min(138, width / len(series))
        pdf.setStrokeColor(HexColor(color))
        pdf.setDash(dash)
        pdf.setLineWidth(2)
        pdf.line(legend_x, 249 if font_scale == 2 else 256, legend_x + 24, 249 if font_scale == 2 else 256)
        pdf.drawString(legend_x + 31, 244 if font_scale == 2 else 253, label)
        line = pdf.beginPath()
        line.moveTo(x(points[0][0]), y(points[0][1]))
        for time, speed in points[1:]:
            line.lineTo(x(time), y(speed))
        pdf.drawPath(line)
    pdf.save()


def energy_points(config, collector, arrivals, horizon):
    mass = config['robot_mass_kg'] + config['rack_mass_kg']
    intervals = {name: tuple(collector.motion_intervals(name)) for name in arrivals}
    # Sampling approximates the drawn curve only. Each plotted value uses
    # the exact energy integral, with every motion boundary also plotted.
    samples = ceil(horizon / .05)
    times = {horizon * k / samples for k in range(samples + 1)}
    for runs in intervals.values():
        for start, end, origin, profile in runs:
            times.update((start, end))
            times.update(origin + t for t in profile_breakpoints(profile) if start < origin + t < end)
    times = sorted(times)
    traces = {name: [(t, cumulative_energy(config, mass, intervals[name], arrivals[name], t)) for t in times]
              for name in arrivals}
    return traces


def plot_paper_alternatives(config, output):
    styles = (('#BF302F', ()), ('#007E55', (6, 3)), ('#284DB8', (3, 2)),
              ('#A75400', (8, 2, 2, 2)), ('#007594', (9, 4)))
    output.mkdir(parents=True, exist_ok=True)
    for case, prefix in zip(CASES, ('one_crossing', 'two_crossings')):
        runs = []
        for push_pull in (False, True):
            for allow_stops in (True, False):
                collector, paths = collect_motion(config, case, True, push_pull, allow_stops, .01)
                arrivals = {name: collector.arrival_time(name, route[-1]) for name, route in paths.items()}
                runs.append((push_pull, allow_stops, collector, arrivals))
        horizon = max(max(arrivals.values()) for _, _, _, arrivals in runs)
        plots = []
        for push_pull, allow_stops, collector, arrivals in runs:
            energy = energy_points(config, collector, arrivals, horizon)
            total = [(points[0][0], sum(value for _, value in points))
                     for points in zip(*energy.values())]
            plots.append((push_pull, allow_stops, collector, arrivals, energy, total))
        y_energy_max = ceil(max(total[-1][1] for *_, total in plots))
        for push_pull, allow_stops, collector, arrivals, energy, total in plots:
            mode = 'push_pull' if push_pull else 'stop_first'
            rule = 'allow' if allow_stops else 'forbid'
            stem = f'{prefix}_{mode}_{rule}'
            rolling_passage = 'Allowed' if push_pull else 'Forbidden'
            intersection_stopping = 'Allowed' if allow_stops else 'Forbidden'
            subtitle = (f'Rolling passage: {rolling_passage} | '
                        f'Stopping in intersection: {intersection_stopping} | Rack-carrying robots')
            speed_series = [(name, color, dash, speed_points(collector, name, horizon))
                            for name, (color, dash) in zip(arrivals, styles)]
            energy_series = [(name, color, dash, energy[name])
                             for name, (color, dash) in zip(arrivals, styles)]
            save_plot(output / f'{stem}_speed.pdf', horizon, config['with_rack']['speed'],
                      'Robot speed profiles', 'Speed (m/s)', speed_series, subtitle, font_scale=2)
            save_plot(output / f'{stem}_energy.pdf', horizon, y_energy_max,
                      'Cumulative battery energy', 'Cumulative energy (Wh)',
                      [*energy_series, ('Fleet total', '#182B45', (), total)], subtitle, font_scale=2)
            print(f'{stem}: {max(arrivals.values()):.3f} s, {total[-1][1]:.3f} Wh')


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--output', type=Path, required=True,
                        help='output PDF path, or output directory with --all-alternatives')
    parser.add_argument('--all-alternatives', action='store_true',
                        help='generate both graphs for all eight rack-carrying paper cases')
    parser.add_argument('--quantity', choices=('speed', 'energy'), default='speed')
    args = parser.parse_args()
    config = loads(args.config.read_text(encoding='utf-8-sig'))
    if args.all_alternatives:
        plot_paper_alternatives(config, args.output)
        return
    collector, paths = collect_motion(config, CASES[0], True, False, True, .01)
    arrivals = {name: collector.arrival_time(name, paths[name][-1]) for name in ('R1', 'R2')}
    horizon = max(arrivals.values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.quantity == 'energy':
        traces = energy_points(config, collector, arrivals, horizon)
        combined = [(t, a + b) for (t, a), (_, b) in zip(traces['R1'], traces['R2'])]
        save_plot(args.output, horizon, ceil(combined[-1][1] * 2) / 2,
                  'One crossing: cumulative battery energy', 'Cumulative energy (Wh)', (
                      ('R1 - horizontal', '#BF302F', (), traces['R1']),
                      ('R2 - vertical', '#007E55', (6, 3), traces['R2']),
                      ('R1 + R2', '#182B45', (), combined),
                  ))
        print(', '.join(f'{name}: {points[-1][1]:.6f} Wh' for name, points in
                        (*traces.items(), ('R1 + R2', combined))))
    else:
        save_plot(args.output, horizon, max(2., config['with_rack']['speed']),
                  'One crossing: speed profiles', 'Speed (m/s)', (
                      ('R1 - horizontal', '#BF302F', (), speed_points(collector, 'R1', horizon)),
                      ('R2 - vertical', '#007E55', (6, 3), speed_points(collector, 'R2', horizon)),
                  ))
    print(args.output)


if __name__ == '__main__':
    main()
