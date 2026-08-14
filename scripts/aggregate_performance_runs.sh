#!/usr/bin/env bash

set -Eeuo pipefail

OUTPUT_PATH=""
declare -a RUN_DIRS=()

usage()
{
	printf '%s\n' \
		"Usage: $0 --output <aggregate.json> <run-dir-1> <run-dir-2> <run-dir-3>"
}

fail()
{
	printf 'ERROR: %s\n' "$1" >&2
	exit "${2:-2}"
}

while (($# > 0)); do
	case "$1" in
	--output)
		(($# >= 2)) || fail "--output requires a value"
		OUTPUT_PATH="$2"
		shift 2
		;;
	-h|--help)
		usage
		exit 0
		;;
	--*)
		fail "unknown argument: $1"
		;;
	*)
		RUN_DIRS+=("$1")
		shift
		;;
	esac
done

[[ -n "${OUTPUT_PATH}" ]] || fail "--output is required"
((${#RUN_DIRS[@]} == 3)) || fail "exactly three run directories are required"
command -v jq >/dev/null || fail "jq is required" 3
command -v python3 >/dev/null || fail "python3 is required" 3
command -v sha256sum >/dev/null || fail "sha256sum is required" 3
python3 -c 'import yaml' >/dev/null 2>&1 || fail "Python PyYAML is required" 3

OUTPUT_PARENT="$(dirname -- "${OUTPUT_PATH}")"
[[ -d "${OUTPUT_PARENT}" ]] || fail "output parent does not exist: ${OUTPUT_PARENT}"
OUTPUT_PATH="$(realpath -m -- "${OUTPUT_PATH}")"
[[ ! -e "${OUTPUT_PATH}" ]] || fail "output already exists: ${OUTPUT_PATH}"

TEMP_DIR="$(mktemp -d)"
trap 'rm -rf -- "${TEMP_DIR}"' EXIT

declare -A SEEN_RUN_DIRS=()
EXPECTED_COMMIT=""
index=0

for requested_run_dir in "${RUN_DIRS[@]}"; do
	RUN_DIR="$(realpath -e -- "${requested_run_dir}")" ||
		fail "run directory does not exist: ${requested_run_dir}"
	[[ -d "${RUN_DIR}" ]] || fail "run path is not a directory: ${RUN_DIR}"
	[[ -z "${SEEN_RUN_DIRS[${RUN_DIR}]:-}" ]] ||
		fail "run directory is duplicated: ${RUN_DIR}"
	SEEN_RUN_DIRS["${RUN_DIR}"]=1

	MANIFEST_PATH="${RUN_DIR}/run_manifest.yaml"
	SUMMARY_PATH="${RUN_DIR}/performance/summary.json"
	HASH_PATH="${RUN_DIR}/artifact_sha256.txt"
	LAUNCH_LOG_PATH="${RUN_DIR}/logs/launch.log"
	OUTPUT_CSV_PATH="${RUN_DIR}/performance/output_ticks.csv"
	CAN_CSV_PATH="${RUN_DIR}/performance/can_round_trip.csv"
	SWITCH_CSV_PATH="${RUN_DIR}/performance/source_switch.csv"
	for required_path in \
		"${MANIFEST_PATH}" \
		"${SUMMARY_PATH}" \
		"${HASH_PATH}" \
		"${LAUNCH_LOG_PATH}" \
		"${OUTPUT_CSV_PATH}" \
		"${CAN_CSV_PATH}" \
		"${SWITCH_CSV_PATH}"; do
		[[ -f "${required_path}" ]] || fail "required run artifact is missing: ${required_path}" 7
	done

	HASH_LINE_COUNT=0
	while IFS= read -r hash_line || [[ -n "${hash_line}" ]]; do
		((HASH_LINE_COUNT += 1))
		artifact_path="${hash_line#*  }"
		[[ "${artifact_path}" != "${hash_line}" ]] ||
			fail "malformed SHA-256 line in ${HASH_PATH}" 7
		case "${artifact_path}" in
		contract_snapshot/*|environment.json|performance/*.csv|performance/summary.json)
			;;
		*)
			fail "unexpected path in SHA-256 manifest: ${artifact_path}" 7
			;;
		esac
		[[ "${artifact_path}" != /* && "${artifact_path}" != *".."* ]] ||
			fail "unsafe path in SHA-256 manifest: ${artifact_path}" 7
	done < "${HASH_PATH}"
	((HASH_LINE_COUNT == 12)) ||
		fail "unexpected SHA-256 artifact count in ${RUN_DIR}: ${HASH_LINE_COUNT}" 7
	(
		cd -- "${RUN_DIR}"
		sha256sum --strict -c artifact_sha256.txt >/dev/null
	) || fail "SHA-256 verification failed for ${RUN_DIR}" 7

	MANIFEST_JSON="${TEMP_DIR}/$(printf '%02d' "${index}").manifest.json"
	python3 - "${MANIFEST_PATH}" "${MANIFEST_JSON}" <<'PY'
import json
import pathlib
import sys

import yaml

source_path = pathlib.Path(sys.argv[1])
output_path = pathlib.Path(sys.argv[2])
with source_path.open("r", encoding="utf-8") as source:
	manifest = yaml.safe_load(source)
if not isinstance(manifest, dict):
	raise SystemExit("run manifest root must be a map: " + str(source_path))
with output_path.open("x", encoding="utf-8") as output:
	json.dump(manifest, output, ensure_ascii=True, sort_keys=True, default=str)
PY

	jq -e '
		.schema_version == 1 and
		.profile == "adas" and
		.status == "PASS" and
		.scope == "VM_ONLY_ADAS_EXTERNAL_OBSERVER" and
		.git_dirty == false and
		.build_type == "RelWithDebInfo" and
		.rmw == "rmw_fastrtps_cpp" and
		(.git_commit | type == "string" and length == 40) and
		(.ros_domain_id | type == "number" and . >= 0 and . <= 232) and
		.warmup_seconds == 30 and
		.measurement_seconds == 300 and
		.minimum_output_ticks == 10000 and
		.source_switch_scenario == true and
		.measurement_exit_code == 0
	' "${MANIFEST_JSON}" >/dev/null ||
		fail "manifest protocol validation failed for ${RUN_DIR}" 7

	RUN_COMMIT="$(jq -r '.git_commit' "${MANIFEST_JSON}")"
	if [[ -z "${EXPECTED_COMMIT}" ]]; then
		EXPECTED_COMMIT="${RUN_COMMIT}"
	elif [[ "${RUN_COMMIT}" != "${EXPECTED_COMMIT}" ]]; then
		fail "runs use different Git commits: ${EXPECTED_COMMIT} and ${RUN_COMMIT}" 7
	fi

	jq -e '
		.schema_version == 1 and
		.completed == true and
		.scope == "VM_ONLY_ADAS_EXTERNAL_OBSERVER" and
		.protocol.protocol_conformant_parameters == true and
		.protocol.require_source_switch == true and
		.protocol.warmup_seconds == 30 and
		.protocol.minimum_duration_seconds == 300 and
		.protocol.minimum_output_ticks == 10000 and
		.protocol.observed_output_ticks >= 10000 and
		.protocol.observed_callback_samples >= 2500 and
		.protocol.observed_can_round_trip_samples >= 5000 and
		.protocol.observed_source_switch_samples >= 1 and
		(.measurement_end_steady_ns - .measurement_start_steady_ns) >= 300000000000 and
		.excluded.invalid_control_frames == 0 and
		.excluded.invalid_state_frames == 0 and
		([.resources[] |
			(.cpu_percent.count >= 150 and .rss_kib.count >= 150)] | all)
	' "${SUMMARY_PATH}" >/dev/null ||
		fail "summary protocol validation failed for ${RUN_DIR}" 7

	awk -F, 'NR > 1 && $8 != "0" {exit 1}' "${OUTPUT_CSV_PATH}" ||
		fail "canonical output entered a non-NORMAL mode during ${RUN_DIR}" 7
	SWITCH_ROWS="$(( $(wc -l < "${SWITCH_CSV_PATH}") - 1 ))"
	((SWITCH_ROWS >= 1)) || fail "source-switch CSV has no data row: ${RUN_DIR}" 7

	MEASUREMENT_END_NS="$(jq -r '.measurement_end_steady_ns' "${SUMMARY_PATH}")"
	LAST_OUTPUT_NS="$(tail -n 1 "${OUTPUT_CSV_PATH}" | cut -d, -f1)"
	LAST_CAN_ECHO_NS="$(tail -n 1 "${CAN_CSV_PATH}" | cut -d, -f2)"
	[[ "${MEASUREMENT_END_NS}" =~ ^[0-9]+$ && "${LAST_OUTPUT_NS}" =~ ^[0-9]+$ &&
		"${LAST_CAN_ECHO_NS}" =~ ^[0-9]+$ ]] ||
		fail "invalid steady timestamp in ${RUN_DIR}" 7
	OUTPUT_END_GAP_NS="$((MEASUREMENT_END_NS - LAST_OUTPUT_NS))"
	CAN_END_GAP_NS="$((MEASUREMENT_END_NS - LAST_CAN_ECHO_NS))"
	((OUTPUT_END_GAP_NS >= 0 && OUTPUT_END_GAP_NS <= 100000000)) ||
		fail "canonical output did not continue to measurement end: ${RUN_DIR}" 7
	((CAN_END_GAP_NS >= 0 && CAN_END_GAP_NS <= 100000000)) ||
		fail "CAN echo did not continue to measurement end: ${RUN_DIR}" 7

	for process_name in \
		vcan_vehicle_simulator \
		socketcan_adapter_node \
		mock_control_source_node \
		control_link_gateway_node; do
		grep -Eq "\[${process_name}-[0-9]+\]: process has finished cleanly" \
			"${LAUNCH_LOG_PATH}" ||
			fail "missing clean process exit for ${process_name} in ${RUN_DIR}" 7
	done
	if grep -Eq 'process has died|Traceback \(most recent call last\)' "${LAUNCH_LOG_PATH}"; then
		fail "launch log contains a process failure: ${RUN_DIR}" 7
	fi

	RUN_JSON="${TEMP_DIR}/$(printf '%02d' "${index}").run.json"
	jq -n \
		--slurpfile manifest "${MANIFEST_JSON}" \
		--slurpfile summary "${SUMMARY_PATH}" \
		--arg run_directory "${RUN_DIR}" \
		--arg manifest_sha256 "$(sha256sum "${MANIFEST_PATH}" | cut -d' ' -f1)" \
		--arg artifact_manifest_sha256 "$(sha256sum "${HASH_PATH}" | cut -d' ' -f1)" \
		--argjson output_end_gap_ms "$(jq -n "${OUTPUT_END_GAP_NS} / 1000000")" \
		--argjson can_end_gap_ms "$(jq -n "${CAN_END_GAP_NS} / 1000000")" \
		'{
			run_id: $manifest[0].run_id,
			run_directory: $run_directory,
			ros_domain_id: $manifest[0].ros_domain_id,
			git_commit: $manifest[0].git_commit,
			manifest_sha256: $manifest_sha256,
			artifact_manifest_sha256: $artifact_manifest_sha256,
			measurement_duration_seconds:
				(($summary[0].measurement_end_steady_ns -
				$summary[0].measurement_start_steady_ns) / 1000000000),
			output_end_gap_ms: $output_end_gap_ms,
			can_end_gap_ms: $can_end_gap_ms,
			protocol: $summary[0].protocol,
			metrics: $summary[0].metrics,
			resources: $summary[0].resources,
			excluded: $summary[0].excluded
		}' > "${RUN_JSON}"
	((index += 1))
done

GENERATED_AT="$(date --iso-8601=seconds)"
AGGREGATE_TEMP_PATH="${TEMP_DIR}/aggregate.json"
jq -s \
	--arg generated_at "${GENERATED_AT}" \
	--arg git_commit "${EXPECTED_COMMIT}" '
	def range($values):
		{min: ($values | min), max: ($values | max)};
	def metric_envelope($runs; $metric):
		($runs | map(.metrics[$metric])) as $values |
		{
			count_total: ($values | map(.count) | add),
			mean_ms: range($values | map(.mean)),
			p50_ms: range($values | map(.p50)),
			p95_ms: range($values | map(.p95)),
			p99_ms: range($values | map(.p99)),
			max_ms: range($values | map(.max))
		};
	def resource_envelope($runs; $resource_name):
		{
			cpu_mean_percent: range($runs | map(.resources[$resource_name].cpu_percent.mean)),
			cpu_max_percent: range($runs | map(.resources[$resource_name].cpu_percent.max)),
			rss_max_kib: range($runs | map(.resources[$resource_name].rss_kib.max))
		};
	. as $runs |
	{
		schema_version: 1,
		validated: true,
		generated_at: $generated_at,
		scope: "VM_ONLY_ADAS_EXTERNAL_OBSERVER",
		git_commit: $git_commit,
		run_count: ($runs | length),
		aggregation_method:
			"range_across_per_run_summaries_raw_samples_are_not_pooled",
		protocol: {
			warmup_seconds: 30,
			measurement_seconds: 300,
			minimum_output_ticks: 10000,
			require_source_switch: true
		},
		runs: $runs,
		metric_envelopes: {
			canonical_delivery_interval_ms:
				metric_envelope($runs; "canonical_delivery_interval_ms"),
			canonical_delivery_abs_jitter_ms:
				metric_envelope($runs; "canonical_delivery_abs_jitter_ms"),
			observer_callback_to_output_ms:
				metric_envelope($runs; "observer_callback_to_output_ms"),
			observer_source_switch_ms:
				metric_envelope($runs; "observer_source_switch_ms"),
			can_observer_round_trip_ms:
				metric_envelope($runs; "can_observer_round_trip_ms")
		},
		resource_envelopes: {
			gateway: resource_envelope($runs; "gateway"),
			adapter: resource_envelope($runs; "adapter"),
			simulator: resource_envelope($runs; "simulator"),
			source: resource_envelope($runs; "source"),
			observer: resource_envelope($runs; "observer")
		}
	}' "${TEMP_DIR}"/*.run.json > "${AGGREGATE_TEMP_PATH}" ||
	fail "aggregate jq generation failed" 7

jq -e '.validated == true and .run_count == 3' "${AGGREGATE_TEMP_PATH}" >/dev/null ||
	fail "aggregate summary self-check failed" 7
mv -- "${AGGREGATE_TEMP_PATH}" "${OUTPUT_PATH}"
printf 'Validated aggregate summary: %s\n' "${OUTPUT_PATH}"
