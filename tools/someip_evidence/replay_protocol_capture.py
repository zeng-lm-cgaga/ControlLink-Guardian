#!/usr/bin/env python3
"""只读旧捕获，在新目录重放收紧后的分段判据；不改写历史产物身份。"""

import argparse
from pathlib import Path

import judge
from run_protocol_scene import evaluate_phases


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input-report", required=True)
    parser.add_argument("--output-report", required=True)
    parser.add_argument("--root", required=True)
    arguments = parser.parse_args()
    source = Path(arguments.input_report).resolve()
    destination = Path(arguments.output_report).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    events = judge.parse_json((source / "events.json").read_bytes())
    if events.get("exit_code") != 0 or events.get("observer_wait_exit") != 0:
        raise judge.InvalidEvidence("input capture was not a successful cooperative observation")
    records = judge.parse_json((source / "protocol-records.json").read_bytes())
    facts = evaluate_phases(destination, records, source / "capture",
                            Path(arguments.root).resolve(), events["observer_wait_exit"])
    judge.publish_json(destination / "summary.json", {
        "input_report": str(source), "scope": "replay of unchanged historical capture",
        "phases": facts})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
