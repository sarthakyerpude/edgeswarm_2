#!/usr/bin/env python3
"""Summarize paired proposed-vs-baseline EdgeSwarm experiment runs.

Input CSV columns: pair_id,mode,elapsed_s,completed_tasks,expected_tasks,collisions.
Each pair_id must have exactly one proposed and one baseline row from the
same task workload/seed. Collision counts are total inter-robot contacts for
that run, supplied by the launch's experiment recorder. Acceptance also
requires a preserved contact-commissioning event log covering each robot link.
"""

import argparse
import csv
import math
import re
import statistics
import sys
from collections import defaultdict


MODES = {'proposed', 'baseline'}
CONTACT_SENSORS = ('contacts', 'contacts/wheel_left',
                   'contacts/wheel_right', 'contacts/caster')


def load_runs(path):
    pairs = defaultdict(dict)
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        expected = {'pair_id', 'mode', 'elapsed_s', 'completed_tasks',
                    'expected_tasks', 'collisions'}
        missing = expected - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(sorted(missing))}")
        for line, row in enumerate(reader, start=2):
            mode = row['mode'].strip().lower()
            if mode not in MODES:
                raise ValueError(f'line {line}: mode must be proposed or baseline')
            pair_id = row['pair_id'].strip()
            if not pair_id or mode in pairs[pair_id]:
                raise ValueError(f'line {line}: empty or duplicate pair_id/mode')
            run = {
                'elapsed_s': float(row['elapsed_s']),
                'completed_tasks': int(row['completed_tasks']),
                'expected_tasks': int(row['expected_tasks']),
                'collisions': int(row['collisions']),
            }
            if (not math.isfinite(run['elapsed_s']) or
                    run['elapsed_s'] <= 0 or
                    min(run['completed_tasks'], run['expected_tasks'],
                        run['collisions']) < 0):
                raise ValueError(f'line {line}: metrics must be nonnegative and time > 0')
            if run['expected_tasks'] == 0:
                raise ValueError(f'line {line}: expected_tasks must be greater than zero')
            if run['completed_tasks'] != run['expected_tasks']:
                raise ValueError(f'line {line}: incomplete workload; exclude and investigate')
            pairs[pair_id][mode] = run
    if not pairs:
        raise ValueError('CSV contains no runs')
    workload_sizes = set()
    for pair_id, modes in pairs.items():
        if set(modes) != MODES:
            raise ValueError(f'{pair_id}: needs one run for each mode')
        if modes['proposed']['completed_tasks'] != modes['baseline']['completed_tasks']:
            raise ValueError(f'{pair_id}: proposed and baseline task counts differ')
        if modes['proposed']['expected_tasks'] != modes['baseline']['expected_tasks']:
            raise ValueError(f'{pair_id}: proposed and baseline expected task counts differ')
        workload_sizes.add(modes['proposed']['expected_tasks'])
    if len(workload_sizes) != 1:
        raise ValueError('all paired workloads must use the same expected task count')
    return pairs


def load_contact_commissioning(path, robot_ids):
    expected = {(robot_id, sensor) for robot_id in robot_ids
                for sensor in CONTACT_SENSORS}
    seen = set()
    with open(path, newline='', encoding='utf-8') as stream:
        reader = csv.DictReader(stream)
        required = {'robot_sensor', 'link_sensor', 'collision1', 'collision2'}
        missing_columns = required - set(reader.fieldnames or [])
        if missing_columns:
            raise ValueError(
                'contact commissioning CSV missing columns: '
                + ', '.join(sorted(missing_columns)))
        for line, row in enumerate(reader, start=2):
            robot_id = (row.get('robot_sensor') or '').strip()
            sensor = (row.get('link_sensor') or '').strip()
            if (robot_id, sensor) not in expected:
                continue
            collision1 = (row.get('collision1') or '').strip()
            collision2 = (row.get('collision2') or '').strip()
            if not collision1 or not collision2:
                raise ValueError(
                    f'contact commissioning line {line}: empty collision name')
            model_name = rf'(?<![A-Za-z0-9_]){re.escape(robot_id)}::'
            if (re.search(model_name, collision1) is None and
                    re.search(model_name, collision2) is None):
                raise ValueError(
                    f'contact commissioning line {line}: collision names do not '
                    f'identify sensor owner {robot_id}')
            seen.add((robot_id, sensor))
    missing = expected - seen
    if missing:
        formatted = ', '.join(f'{robot}/{sensor}'
                              for robot, sensor in sorted(missing))
        raise ValueError(f'contact commissioning incomplete; no observed event for: {formatted}')
    return len(seen), len(expected)


def summarize(pairs, minimum_pairs=3, contact_coverage=None):
    improvements = []
    proposed_times = []
    baseline_times = []
    proposed_collisions = 0
    proposed_tasks = 0
    baseline_collisions = 0
    for pair in pairs.values():
        proposed = pair['proposed']
        baseline = pair['baseline']
        improvements.append((baseline['elapsed_s'] - proposed['elapsed_s'])
                            / baseline['elapsed_s'])
        proposed_times.append(proposed['elapsed_s'])
        baseline_times.append(baseline['elapsed_s'])
        proposed_collisions += proposed['collisions']
        proposed_tasks += proposed['completed_tasks']
        baseline_collisions += baseline['collisions']

    mean_gain = statistics.mean(improvements)
    total_proposed_s = sum(proposed_times)
    total_baseline_s = sum(baseline_times)
    total_gain = ((total_baseline_s - total_proposed_s) / total_baseline_s)
    safe = proposed_collisions == 0
    commissioned = contact_coverage is not None
    enough_pairs = len(pairs) >= minimum_pairs
    faster = total_gain >= 0.20
    print(f'Paired workloads: {len(pairs)}')
    print(f'Proposed mean task time: {statistics.mean(proposed_times):.2f} s')
    print(f'Proposed median task time: {statistics.median(proposed_times):.2f} s')
    print(f'Baseline mean task time: {statistics.mean(baseline_times):.2f} s')
    print(f'Baseline median task time: {statistics.median(baseline_times):.2f} s')
    print(f'Mean paired time reduction: {100.0 * mean_gain:.2f}% (descriptive)')
    print(f'Total paired completion-time reduction: {100.0 * total_gain:.2f}%')
    print(f'Proposed inter-robot collisions: {proposed_collisions}')
    print(f'Baseline inter-robot collisions: {baseline_collisions}')
    print(f'Proposed tasks completed: {proposed_tasks}')
    if commissioned:
        print(f'Contact instrumentation commissioning: '
              f'{contact_coverage[0]}/{contact_coverage[1]} sensor streams PASS')
    else:
        print('Contact instrumentation commissioning: FAIL (not provided)')
    print(f'Run-count criterion (at least {minimum_pairs} pairs): '
          f'{"PASS" if enough_pairs else "FAIL"}')
    print(f'Safety criterion: {"PASS" if safe else "FAIL"}')
    print(f'20% total-time criterion: {"PASS" if faster else "FAIL"}')
    return enough_pairs and safe and faster and commissioned


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', help='paired run metrics CSV')
    parser.add_argument('--minimum-pairs', type=int, default=3,
                        help='minimum complete workload pairs required to pass (default: 3)')
    parser.add_argument('--contact-commissioning', required=True,
                        help='preserved contact_sensor_events.csv from the unscored sensor commissioning run')
    parser.add_argument('--robot-ids', nargs='+',
                        default=['robot_1', 'robot_2', 'robot_3'],
                        help='robot IDs whose chassis, wheels, and casters were commissioned')
    args = parser.parse_args()
    if args.minimum_pairs < 1:
        parser.error('--minimum-pairs must be at least 1')
    if len(args.robot_ids) < 3 or len(set(args.robot_ids)) != len(args.robot_ids):
        parser.error('--robot-ids must contain at least three unique IDs')
    try:
        pairs = load_runs(args.csv)
        contact_coverage = load_contact_commissioning(
            args.contact_commissioning, args.robot_ids)
        # The default acceptance command requires repeated workloads; a user
        # lowering the threshold can still label an exploratory result.
        success = summarize(pairs, minimum_pairs=args.minimum_pairs,
                            contact_coverage=contact_coverage)
    except (OSError, ValueError, TypeError, csv.Error) as exc:
        print(f'error: {exc}', file=sys.stderr)
        return 2
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
