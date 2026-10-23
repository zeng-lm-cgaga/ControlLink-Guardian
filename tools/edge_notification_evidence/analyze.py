#!/usr/bin/env python3
"""复用冻结输入和 CAN 解码；仅判定本次 Edge 受影响路径，不重实现业务规则。"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/linux_depth'))
from analyze import encoded, freeze, load_inputs
import judge


def command_key(message):
    stamp = message['source_stamp']
    return (message['source_id'], message['source_sequence'], stamp['sec'], stamp['nanosec'])


def analyze(scene):
    inputs = load_inputs(scene)
    events = json.loads(inputs['events.json'])
    if events['result'] or events['errors'] or events['stop_reason'] != 'fixed_window_complete':
        raise ValueError('scene did not complete its fixed window')
    exits = list(events['exits'].values())
    actions = events.get('edge_actions', {})
    if actions:
        exits.append(actions['feedback_replace']['stop'])
    if any(not item['exit_confirmed'] or item['exit'] or item.get('timeout') or
           item.get('forced') or item.get('errors') for item in exits):
        raise ValueError('owned process cleanup not cleanly closed')
    if events['flush_done_ns'] <= events['cutoff_ns']:
        raise ValueError('capture not flushed after cutoff')
    streams = {name: [json.loads(line) for line in inputs[name + '.jsonl'].splitlines()]
               for name in ('gateway', 'status', 'vehicle', 'canonical', 'can',
                            'ingress_observation', 'edge_client')}
    mapping = judge.load_can_map(inputs['config-adas-can_signal_map.yaml'])
    start, end = (events['window'][key] for key in ('start_ns', 'end_ns'))

    def window(name, begin, cutoff):
        return [row for row in streams[name] if begin <= row['monotonic_ns'] < cutoff]

    def population(begin, cutoff):
        status = [row for row in window('status', begin, cutoff)
                  if row['message']['source_id'] == 'planning']
        gateways = window('gateway', begin, cutoff)
        facts = Counter(judge.control_frame_fact(row['frame'], mapping)[0]
                        for row in window('can', begin, cutoff) if row['frame']['can_id'] == 0x180)
        ingress = {command_key(row['message']) for row in window('ingress_observation', begin, cutoff)}
        matched = [row for row in window('canonical', begin, cutoff)
                   if row['message']['mode'] == 0 and command_key(row['message']) in ingress]
        active = [row for row in gateways if row['message']['state'] == 1 and
                  row['message']['active_source_id'] == 'planning']
        vehicles = [row for row in window('vehicle', begin, cutoff)
                    if row['message']['state'] == 1 and row['message']['fault_code'] == 0 and
                    row['message']['linear_velocity_mps'] > 0]
        growth = status[-1]['message']['accepted_count'] - status[0]['message']['accepted_count'] if status else 0
        return {'accepted_delta_observed': growth,
                'active_planning_observations': len(active),
                'canonical_matching_exact_ingress_identity': len(matched),
                'healthy_moving_vehicle_observations': len(vehicles), 'control_can': dict(facts),
                'source_gids': sorted({bytes(row['message']['endpoint_publisher_gid']).hex() for row in status
                                       if row['message']['endpoint_generation_valid']}),
                'states': dict(Counter(str(row['message']['state']) for row in gateways)),
                'observed_path_present': bool(growth > 0 and active and matched and vehicles and
                                              facts['active_nonzero'] > 0 and facts['invalid'] == 0)}

    segments = {'whole_measurement': population(start, end)}
    checks = {'whole_path': segments['whole_measurement']['observed_path_present'],
              'ordinary_user': events['uid'] == events['euid'] == 1000}
    if actions:
        pause, resume, replace = (actions[key]['monotonic_ns'] for key in ('pause', 'resume', 'command_replace'))
        bridge = actions['feedback_replace']
        bridge_stop = bridge['stop']['requested_ns']
        for label, begin, cutoff in [('before_pause', start, pause), ('after_resume', resume, replace),
                                     ('after_command_replace', replace, bridge_stop),
                                     ('after_feedback_replace', bridge['started_ns'], end)]:
            segments[label] = population(begin, cutoff)
            checks[label] = segments[label]['observed_path_present']
        paused = population(pause + 200_000_000, resume)
        paused['submit_count'] = sum(row['event'] == 'submit' for row in window('edge_client', pause + 200_000_000, resume))
        paused['ingress_count'] = len(window('ingress_observation', pause + 200_000_000, resume))
        paused['feedback_count'] = sum(row['event'] == 'feedback' for row in window('edge_client', pause + 200_000_000, resume))
        segments['heartbeat_only_command_pause'] = paused
        checks['command_pause_fails_closed'] = bool(paused['submit_count'] == paused['ingress_count'] == 0 and
            paused['feedback_count'] > 0 and paused['control_can'].get('hold', 0) > 0 and
            paused['control_can'].get('active_nonzero', 0) == 0 and paused['states'].get('3', 0) > 0)
        client = streams['edge_client']
        checks['old_target_rejected'] = any(row['event'] == 'old_target_rejected' and row['code'] == 'StaleHandle' for row in client)
        checks['new_bridge_instance'] = bridge['before']['pid'] != bridge['after']['pid']
        checks['new_source_gid'] = bool(set(segments['after_feedback_replace']['source_gids']) -
                                         set(segments['before_pause']['source_gids']))
        closed = [row for row in client if row['event'] == 'closed']
        checks['feedback_notification_reregistered'] = bool(len(closed) == 1 and closed[0]['registrations'] >= 2 and
            closed[0]['registration_failures'] == 0 and closed[0]['command_notification_errno'] == 0)
    if not all(checks.values()):
        raise ValueError('affected path evidence incomplete: ' + repr(checks))
    return {'schema': 1, 'mode': events['edge_mode'], 'window': events['window'],
            'checks': checks, 'segments': segments, 'exits': events['exits'],
            'boundaries': [
                'VM_ONLY; observed population, not a latency or performance comparison',
                'CAN has no full business sequence/session: segment-level proof only',
                'status/Gateway/canonical are independent observations, not an atomic snapshot',
                'heartbeat-only producer loop is evidenced by fixture source; feedback samples are recorded',
                'cooperative SDK control guarantees require explicit context capability; normal measured cleanup is not an arbitrary kernel-blocking hard bound']}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('scene', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--freeze', action='store_true')
    args = parser.parse_args()
    if args.freeze:
        events = json.loads((args.scene / 'events.json').read_bytes())
        if any(not row.get('exit_confirmed') for row in events['exits'].values()):
            raise ValueError('cannot freeze before owner exit confirmation')
        freeze(args.scene)
    with args.output.open('xb') as handle:
        handle.write(encoded(analyze(args.scene)))
