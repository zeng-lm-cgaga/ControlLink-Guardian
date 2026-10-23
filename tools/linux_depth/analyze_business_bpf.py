#!/usr/bin/env python3
"""真实业务的同实例/TID/MONOTONIC相交证据和短窗BPF开销；不推断独占根因。"""
import argparse
import bisect
from collections import Counter, defaultdict
import csv
import hashlib
import io
import json
from pathlib import Path
import re

from analyze import analyze, distribution, encoded, load_inputs, timing
from analyze_bpf import analyze_kernel
from business_trace import publish

ROLES = ('gateway', 'can_adapter')
PHASES = {'source_receive', 'source_result', 'source_select', 'canonical_publish',
          'mailbox_write', 'can_write', 'can_receive', 'feedback_validate', 'feedback_publish', 'gateway_feedback'}


def resource_metrics(rows, begin, end, hz):
    grouped = defaultdict(list)
    for row in rows:
        if begin <= row['monotonic_ns'] < end:
            grouped[row['role']].append(row)
    result = {}
    for role, samples in grouped.items():
        samples.sort(key=lambda row: row['monotonic_ns'])
        cpu, seconds, ticks = [], 0.0, 0
        for before, after in zip(samples, samples[1:]):
            if (before['pid'], before['starttime']) != (after['pid'], after['starttime']):
                raise ValueError('resource instance mismatch')
            elapsed = (after['monotonic_ns'] - before['monotonic_ns']) / 1e9
            delta = after['cpu_ticks'] - before['cpu_ticks']
            if elapsed <= 0 or delta < 0:
                raise ValueError('invalid CPU interval')
            seconds += elapsed
            ticks += delta
            cpu.append(100 * delta / hz / elapsed)
        result[role] = {'samples': len(samples), 'sampled_interval_s': seconds,
                        'cpu_mean_one_core_pct': 100 * ticks / hz / seconds if seconds else None,
                        'cpu_max_one_core_pct': max(cpu) if cpu else None,
                        'rss_kib': distribution([row['rss_kib'] for row in samples])}
    return result


def correlate(content, log, app, target, observations, window):
    kernel = analyze_kernel(content, log, include_intervals=True)
    errors = []
    begin, end = window['start_ns'], window['end_ns']
    header = kernel['target_header']
    if (header.get('target_tgid'), header.get('start_ticks'), header.get('hz'), header.get('target_tid')) != (target['pid'], target['starttime'], target['clk_tck'], 0):
        errors.append('target TGID/starttime/CLK_TCK mismatch')
    starts = re.findall(rb'# attached=5 start_monotonic_ns=(\d+)', content)
    if len(starts) != 1 or int(starts[0]) > begin or kernel['quality']['end_ns'] < end:
        errors.append('kernel capture does not enclose the fixed measurement window')
    threads = defaultdict(set)
    for tid, fact in target['threads'].items():
        threads[tid].add(fact['starttime'])
    for observation in observations:
        for tid, starttime in observation['threads'].items():
            threads[tid].add(starttime)
    for pid, tid, started in kernel['task_instances']:
        expected = threads.get(str(tid), set())
        if pid != target['pid'] or len(expected) != 1 or not any(
                start * 1000000000 // target['clk_tck'] <= started < (start + 1) * 1000000000 // target['clk_tck'] for start in expected):
            errors.append('unknown/reused kernel thread instance')
            break
    raw = list(csv.DictReader(io.StringIO('\n'.join(line for line in content.decode().splitlines() if not line.startswith('#')))))
    kinds = Counter(int(row['kind']) for row in raw if begin <= int(row['at_ns']) < end)
    if not kernel['population_complete'] or any(not kinds[kind] for kind in (1, 2, 3, 4, 5)):
        errors.append('empty/lossy or missing sched/syscall events inside the fixed window')
    indexed = defaultdict(list)
    for interval in kernel.pop('kernel_intervals'):
        indexed[(interval['pid'], interval['tid'])].append(interval)
    lookup = {}
    for key, intervals in indexed.items():
        intervals.sort(key=lambda row: row['begin_ns'])
        maxima, maximum = [], 0
        for interval in intervals:
            maximum = max(maximum, interval['end_ns'])
            maxima.append(maximum)
        lookup[key] = (intervals, [row['begin_ns'] for row in intervals], maxima)
    counts, missing, evidence = Counter(), Counter(), []
    for index, row in enumerate(app):
        if row['phase'] not in PHASES or not (begin <= row['begin_ns'] <= row['end_ns'] < end):
            continue
        counts[row['phase']] += 1
        if row['pid'] != target['pid'] or len(threads.get(str(row['tid']), set())) != 1:
            missing['unknown_application_instance'] += 1
            continue
        intervals, beginnings, maxima = lookup.get((row['pid'], row['tid']), ([], [], []))
        overlaps = []
        for interval in intervals[bisect.bisect_right(maxima, row['begin_ns']):bisect.bisect_left(beginnings, row['end_ns'])]:
            length = min(row['end_ns'], interval['end_ns']) - max(row['begin_ns'], interval['begin_ns'])
            if length > 0:
                overlaps.append({**interval, 'intersection_ns': length})
        if overlaps:
            evidence.append({'application_event_index_zero_based': index,
                             **{key: row[key] for key in ('phase', 'pid', 'tid', 'source', 'gid', 'sequence',
                                  'source_stamp_ns', 'frame_id', 'result', 'begin_ns', 'end_ns')},
                             'kernel_intersections': overlaps})
        else:
            missing['no_observed_interval_intersection'] += 1
    command = [row for row in evidence if row['sequence'] > 0 and row['source'] == 'planning' and row['phase'] not in ('feedback_validate', 'feedback_publish', 'gateway_feedback', 'can_receive')]
    if not command:
        errors.append('no full-identity real command phase correlated')
    if not any(interval['kind'].startswith('syscall_') for row in evidence for interval in row['kernel_intersections']):
        errors.append('no actual syscall interval correlated to an application phase')
    return {'accepted': not errors, 'errors': errors, 'kernel': kernel, 'window_event_kinds': dict(kinds),
            'candidate_phase_counts': dict(counts), 'unmatched_counts': dict(missing),
            'matched_phase_counts': dict(Counter(row['phase'] for row in evidence)),
            'matched_unique_command_identities': len({(row['source'], row['gid'], row['sequence'], row['source_stamp_ns']) for row in command}),
            'evidence': evidence, 'limits': 'same task instance/TID and MONOTONIC interval intersection, not exclusive causal attribution; '
            'mailbox_consume is cross-thread logical residence and excluded; feedback frames are not per-command ACK; '
            'GID is a ROS publisher generation, not authentication or a network session; RAW scheduled_ns is never subtracted'}


def read_case(case):
    index = json.loads((case / 'capture-freeze.json').read_bytes())
    required = {'broker.json', 'scene/events.json', 'scene/frozen-inputs.json', 'bpf/resource.jsonl'}
    if not required.issubset(index['files']):
        raise ValueError('missing frozen capture inputs')
    for name, fact in index['files'].items():
        content = (case / name).read_bytes()
        if len(content) != fact['size'] or hashlib.sha256(content).hexdigest() != fact['sha256']:
            raise ValueError('capture drift: ' + name)
    inputs = load_inputs(case / 'scene')
    events = json.loads(inputs['events.json'])
    broker = json.loads((case / 'broker.json').read_bytes())
    if events['euid'] == 0 or events['euid'] != broker['business_uid'] or broker['errors']:
        raise ValueError('wrong business privileges or broker failure')
    if events['errors'] or any(row['exit'] or row.get('timeout') for row in events['exits'].values()):
        raise ValueError('business peer error/nonzero/forced shutdown')
    if 'business' not in broker['exits'] or any(row['exit'] or row['forced'] for row in broker['exits'].values()):
        raise ValueError('incomplete/forced process exit')
    return inputs, events, broker


def compare_cases(cases):
    before, after = cases['off'], cases['on']
    business_roles = ('gateway', 'can_adapter', 'planning_ingress', 'simulator', 'idle_teleop', 'fixture_observer')
    for key in ('config_sha256', 'driver_sha256', 'clk_tck', 'vcpus', 'kernel'):
        if before['events'][key] != after['events'][key]:
            raise ValueError('off/on environment/configuration mismatch: ' + key)
    for role, fact in before['events']['preflight'].items():
        if fact['sha256'] != after['events']['preflight'].get(role, {}).get('sha256'):
            raise ValueError('off/on executable or runtime closure mismatch: ' + role)
    for key in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'RMW_FASTRTPS_USE_QOS_FROM_XML', 'FASTRTPS_DEFAULT_PROFILES_FILE'):
        if before['events']['environment'].get(key) != after['events']['environment'].get(key):
            raise ValueError('off/on ROS/RMW configuration mismatch: ' + key)
    for role in business_roles:
        if before['events']['identities'][role]['mapped_sha256'] != after['events']['identities'][role]['mapped_sha256']:
            raise ValueError('off/on actually loaded code mismatch: ' + role)
    for case in cases.values():
        if not case['events']['arguments']['timing'] or case['events']['window']['duration_s'] != 20 or case['events']['window']['warmup_s'] != 10:
            raise ValueError('off/on application instrumentation or fixed window mismatch')
        if case['events']['arguments']['cpu_load'] or case['events']['arguments']['pause_source']:
            raise ValueError('not the normal fixed workload')
    delta = {}
    for role in business_roles:
        original = before['summary']['resource'].get(role)
        traced = after['summary']['resource'].get(role)
        if (not original or not traced or original['samples'] < 2 or traced['samples'] < 2 or
                original['cpu_mean_one_core_pct'] is None or traced['cpu_mean_one_core_pct'] is None):
            raise ValueError('insufficient process resource intervals: ' + role)
        delta[role] = {'cpu_percentage_point_delta': traced['cpu_mean_one_core_pct'] - original['cpu_mean_one_core_pct'],
                       'rss_mean_kib_delta': traced['rss_kib']['mean'] - original['rss_kib']['mean']}
    phase_deltas = {}
    for phase, original in before['summary']['latencies_ms'].items():
        traced = after['summary']['latencies_ms'].get(phase)
        if traced and original['samples'] and traced['samples']:
            phase_deltas[phase] = {'off_samples': original['samples'], 'on_samples': traced['samples'],
                                  'mean_ms_delta': traced['mean'] - original['mean'],
                                  'p99_ms_delta': traced['p99'] - original['p99'] if min(original['samples'], traced['samples']) >= 100 else None}
    excluded = {role: {'off_samples': before['summary']['resource'].get(role, {}).get('samples', 0),
                       'on_samples': after['summary']['resource'].get(role, {}).get('samples', 0),
                       'reason': 'not a fixed-window business/observer role'}
                for role in sorted((set(before['summary']['resource']) | set(after['summary']['resource'])) - set(business_roles))}
    return {'business_process_deltas': delta, 'latency_observation_deltas': phase_deltas,
            'excluded_resource_roles': excluded,
            'off_tracer': 'not started; zero BPF loader processes',
            'limits': 'one nonrandomized short VM pair; no stable improvement/overhead percentage claim; '
            'CPU delta(utime+stime)/CLK_TCK/monotonic, 100% is one logical CPU; RSS sampled resident KiB, not system peak; '
            'both sides retain identical application recorders/bounded buffers/background output and observer logging'}


def generate(base, output):
    output.mkdir(exist_ok=False)
    cases, correlations = {}, {}
    for mode in ('off', 'on'):
        case = base / mode
        inputs, events, broker = read_case(case)
        summary = analyze(case / 'scene')
        window = events['window']
        if events['trace_gate']['reply']['mode'] != mode:
            raise ValueError('BPF mode mismatch')
        expected_exits = {'business', *ROLES} if mode == 'on' else {'business'}
        if set(broker['exits']) != expected_exits or set(broker['tracers']) != (set(ROLES) if mode == 'on' else set()):
            raise ValueError('missing tracer exit/identity or BPF-off contamination')
        if mode == 'on' and not {f'bpf/{role}/{name}' for role in ROLES for name in ('kernel.csv', 'loader.log')}.issubset(
                json.loads((case / 'capture-freeze.json').read_bytes())['files']):
            raise ValueError('BPF input not frozen')
        if not summary['planning_accepted_delta'] or not summary['can_counts'].get('active_nonzero') or not summary['first_matches'].get('vcan'):
            raise ValueError('no accepted/ACTIVE/nonzero valid CAN business population')
        rows = [json.loads(line) for line in (case / 'bpf/resource.jsonl').read_bytes().splitlines()]
        auxiliary = resource_metrics(rows, window['start_ns'], window['end_ns'], events['clk_tck'])
        cases[mode] = {'events': events, 'summary': summary, 'observer_and_tracer_resources': auxiliary}
        if mode == 'on':
            for role in ROLES:
                app, quality = timing(inputs[role + '-timing.csv'], inputs[role + '-timing.csv.status.json'])
                correlations[role] = correlate((case / 'bpf' / role / 'kernel.csv').read_bytes(),
                    (case / 'bpf' / role / 'loader.log').read_bytes(), app, broker['targets'][role],
                    broker['thread_observations'][role], window)
                correlations[role]['application_recorder_quality'] = quality
                correlations[role]['population_claim'] = 'observed records only; recorder losses retained, not full-population proof'
        publish(output / (mode + '-summary.json'), {'business': summary, 'auxiliary_resources': auxiliary})
    publish(output / 'correlations.json', correlations)
    publish(output / 'comparison.json', compare_cases(cases))
    accepted = all(result['accepted'] for result in correlations.values())
    publish(output / 'acceptance.json', {'business_bpf_association': accepted, 'status': 'LOCAL_OBSERVATION' if accepted else 'INCOMPLETE'})
    if not accepted:
        raise ValueError('business BPF coverage incomplete; see preserved correlations.json')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('base', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    generate(args.base, args.output)
