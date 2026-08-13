#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
WORKSPACE_DIR="${ROOT_DIR}/ros2_ws"
OUTPUT_ROOT="${ROOT_DIR}/reports"
RUN_ID=""
ROS_DOMAIN_ID_VALUE=81
CAN_INTERFACE="vcan0"
WARMUP_SECONDS=30
MEASUREMENT_SECONDS=300
MIN_OUTPUT_TICKS=10000
MAX_EXTRA_WAIT_SECONDS=60
SOURCE_SWITCH_SCENARIO=false
LAUNCH_PID=""
LAUNCH_PGID=""
MEASUREMENT_PID=""
MEASUREMENT_PGID=""
SECONDARY_SOURCE_PID=""
SECONDARY_SOURCE_PGID=""

usage()
{
	printf '%s\n' \
		"Usage: $0 --run-id <id> [options]" \
		"" \
		"Options:" \
		"  --output-root <path>" \
		"  --domain-id <0..232>" \
		"  --can-interface <vcan*>" \
		"  --warmup-seconds <integer>" \
		"  --measurement-seconds <integer>" \
		"  --min-output-ticks <integer>" \
		"  --max-extra-wait-seconds <integer>" \
		"  --source-switch"
}

fail()
{
	printf 'ERROR: %s\n' "$1" >&2
	exit "${2:-2}"
}

require_unsigned()
{
	local value="$1"
	local field="$2"
	[[ "${value}" =~ ^[0-9]+$ ]] || fail "${field} must be an unsigned integer"
}

while (($# > 0)); do
	case "$1" in
		--run-id)
			(($# >= 2)) || fail "--run-id requires a value"
			RUN_ID="$2"
			shift 2
			;;
		--output-root)
			(($# >= 2)) || fail "--output-root requires a value"
			OUTPUT_ROOT="$2"
			shift 2
			;;
		--domain-id)
			(($# >= 2)) || fail "--domain-id requires a value"
			ROS_DOMAIN_ID_VALUE="$2"
			shift 2
			;;
		--can-interface)
			(($# >= 2)) || fail "--can-interface requires a value"
			CAN_INTERFACE="$2"
			shift 2
			;;
		--warmup-seconds)
			(($# >= 2)) || fail "--warmup-seconds requires a value"
			WARMUP_SECONDS="$2"
			shift 2
			;;
		--measurement-seconds)
			(($# >= 2)) || fail "--measurement-seconds requires a value"
			MEASUREMENT_SECONDS="$2"
			shift 2
			;;
		--min-output-ticks)
			(($# >= 2)) || fail "--min-output-ticks requires a value"
			MIN_OUTPUT_TICKS="$2"
			shift 2
			;;
		--max-extra-wait-seconds)
			(($# >= 2)) || fail "--max-extra-wait-seconds requires a value"
			MAX_EXTRA_WAIT_SECONDS="$2"
			shift 2
			;;
		--source-switch)
			SOURCE_SWITCH_SCENARIO=true
			shift
			;;
		-h|--help)
			usage
			exit 0
			;;
		*)
			fail "unknown argument: $1"
			;;
	esac
done

[[ "${RUN_ID}" =~ ^[A-Za-z0-9._-]+$ ]] ||
	fail "run-id must match [A-Za-z0-9._-]+"
require_unsigned "${ROS_DOMAIN_ID_VALUE}" "domain-id"
((ROS_DOMAIN_ID_VALUE <= 232)) || fail "domain-id must be at most 232"
require_unsigned "${WARMUP_SECONDS}" "warmup-seconds"
require_unsigned "${MEASUREMENT_SECONDS}" "measurement-seconds"
require_unsigned "${MIN_OUTPUT_TICKS}" "min-output-ticks"
require_unsigned "${MAX_EXTRA_WAIT_SECONDS}" "max-extra-wait-seconds"
((MEASUREMENT_SECONDS > 0)) || fail "measurement-seconds must be positive"
((MIN_OUTPUT_TICKS > 0)) || fail "min-output-ticks must be positive"
((MAX_EXTRA_WAIT_SECONDS > 0)) || fail "max-extra-wait-seconds must be positive"
[[ "${CAN_INTERFACE}" == vcan* ]] ||
	fail "performance baseline only accepts an explicit vcan* interface" 3

[[ -f /opt/ros/humble/setup.bash ]] || fail "ROS2 Humble setup is missing" 3
[[ -f "${WORKSPACE_DIR}/install/setup.bash" ]] ||
	fail "workspace install/setup.bash is missing; build the workspace first" 3
# shellcheck disable=SC1091
set +u
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WORKSPACE_DIR}/install/setup.bash"
set -u

command -v ros2 >/dev/null || fail "ros2 is not available" 3
command -v jq >/dev/null || fail "jq is not available" 3
command -v setsid >/dev/null || fail "setsid is not available" 3
command -v sha256sum >/dev/null || fail "sha256sum is not available" 3
ip link show "${CAN_INTERFACE}" >/dev/null 2>&1 ||
	fail "SocketCAN interface does not exist: ${CAN_INTERFACE}" 3
ip -details link show "${CAN_INTERFACE}" | grep -q '<[^>]*UP[^>]*>' ||
	fail "SocketCAN interface is not UP: ${CAN_INTERFACE}" 3

mkdir -p -- "${OUTPUT_ROOT}"
OUTPUT_ROOT="$(realpath -- "${OUTPUT_ROOT}")"
RUN_DIR="${OUTPUT_ROOT}/${RUN_ID}"
[[ ! -e "${RUN_DIR}" ]] || fail "run directory already exists: ${RUN_DIR}"
mkdir -p -- \
	"${RUN_DIR}/contract_snapshot" \
	"${RUN_DIR}/logs" \
	"${RUN_DIR}/performance"
: > "${RUN_DIR}/logs/secondary_source.log"

PROFILE_PATH="${ROOT_DIR}/config/adas/adas_profile.yaml"
CONFIG_ROOT="${ROOT_DIR}/config"
CONTRACT_PATH="${ROOT_DIR}/config/common/gateway_contract.yaml"
SOURCE_POLICY_PATH="${ROOT_DIR}/config/common/source_policy.yaml"
FAST_DDS_PATH="${ROOT_DIR}/config/common/fastdds_default_profiles.xml"
CAN_MAP_PATH="${ROOT_DIR}/config/adas/can_signal_map.yaml"

for file in \
	"${PROFILE_PATH}" \
	"${CONTRACT_PATH}" \
	"${SOURCE_POLICY_PATH}" \
	"${FAST_DDS_PATH}" \
	"${CAN_MAP_PATH}"; do
	[[ -f "${file}" ]] || fail "required configuration is missing: ${file}" 3
done

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export RMW_FASTRTPS_USE_QOS_FROM_XML="0"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FAST_DDS_PATH}"

cp -- "${PROFILE_PATH}" "${RUN_DIR}/contract_snapshot/adas_profile.yaml"
cp -- "${CONTRACT_PATH}" "${RUN_DIR}/contract_snapshot/gateway_contract.yaml"
cp -- "${SOURCE_POLICY_PATH}" "${RUN_DIR}/contract_snapshot/source_policy.yaml"
cp -- "${FAST_DDS_PATH}" "${RUN_DIR}/contract_snapshot/fastdds_default_profiles.xml"
cp -- "${CAN_MAP_PATH}" "${RUN_DIR}/contract_snapshot/can_signal_map.yaml"

START_SYSTEM_TIME="$(date --iso-8601=seconds)"
START_MONOTONIC_SECONDS="$(cut -d' ' -f1 /proc/uptime)"
GIT_COMMIT="$(git -C "${ROOT_DIR}" rev-parse HEAD)"
if [[ -n "$(git -C "${ROOT_DIR}" status --porcelain)" ]]; then
	GIT_DIRTY=true
else
	GIT_DIRTY=false
fi

jq -n \
	--arg recorded_at "${START_SYSTEM_TIME}" \
	--arg os "$(. /etc/os-release && printf '%s' "${PRETTY_NAME}")" \
	--arg kernel "$(uname -srvm)" \
	--arg architecture "$(uname -m)" \
	--arg virtualization "$(systemd-detect-virt 2>/dev/null || printf 'unknown')" \
	--arg cpu_model "$(lscpu | awk -F: '/Model name/ {sub(/^[[:space:]]+/, "", $2); print $2; exit}')" \
	--arg cpu_governor "$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null || printf 'unavailable')" \
	--arg ros_distro "${ROS_DISTRO:-unknown}" \
	--arg rmw "${RMW_IMPLEMENTATION}" \
	--arg fastdds "$(dpkg-query -W -f='${Version}' ros-humble-fastrtps 2>/dev/null || printf 'unknown')" \
	--arg rmw_fastrtps "$(dpkg-query -W -f='${Version}' ros-humble-rmw-fastrtps-cpp 2>/dev/null || printf 'unknown')" \
	--arg build_type "RelWithDebInfo" \
	--arg scope "VM_ONLY" \
	--argjson logical_cpus "$(nproc)" \
	--argjson memory_kib "$(awk '/MemTotal/ {print $2}' /proc/meminfo)" \
	'{
		recorded_at: $recorded_at,
		os: $os,
		kernel: $kernel,
		architecture: $architecture,
		virtualization: $virtualization,
		cpu_model: $cpu_model,
		logical_cpus: $logical_cpus,
		cpu_governor: $cpu_governor,
		memory_kib: $memory_kib,
		ros_distro: $ros_distro,
		rmw: $rmw,
		fastdds_package: $fastdds,
		rmw_fastrtps_package: $rmw_fastrtps,
		build_type: $build_type,
		scope: $scope
	}' > "${RUN_DIR}/environment.json"

cleanup_launch()
{
	if [[ -z "${LAUNCH_PGID}" ]] ||
		! process_group_exists "${LAUNCH_PGID}"; then
		if [[ -n "${LAUNCH_PID}" ]]; then
			wait "${LAUNCH_PID}" 2>/dev/null || true
		fi
		LAUNCH_PGID=""
		return
	fi
	printf 'Stopping launch process group %s with SIGINT\n' "${LAUNCH_PGID}"
	/bin/kill -INT -- "-${LAUNCH_PGID}" 2>/dev/null || true
	for _ in $(seq 1 40); do
		if ! process_group_exists "${LAUNCH_PGID}"; then
			wait "${LAUNCH_PID}" 2>/dev/null || true
			LAUNCH_PGID=""
			return
		fi
		sleep 0.25
	done
	/bin/kill -TERM -- "-${LAUNCH_PGID}" 2>/dev/null || true
	for _ in $(seq 1 20); do
		if ! process_group_exists "${LAUNCH_PGID}"; then
			wait "${LAUNCH_PID}" 2>/dev/null || true
			LAUNCH_PGID=""
			return
		fi
		sleep 0.25
	done
	/bin/kill -KILL -- "-${LAUNCH_PGID}" 2>/dev/null || true
	wait "${LAUNCH_PID}" 2>/dev/null || true
	LAUNCH_PGID=""
}

cleanup_measurement()
{
	if [[ -z "${MEASUREMENT_PGID}" ]] ||
		! process_group_exists "${MEASUREMENT_PGID}"; then
		if [[ -n "${MEASUREMENT_PID}" ]]; then
			wait "${MEASUREMENT_PID}" 2>/dev/null || true
		fi
		MEASUREMENT_PGID=""
		return
	fi
	printf 'Stopping measurement process group %s with SIGINT\n' "${MEASUREMENT_PGID}"
	/bin/kill -INT -- "-${MEASUREMENT_PGID}" 2>/dev/null || true
	for _ in $(seq 1 20); do
		if ! process_group_exists "${MEASUREMENT_PGID}"; then
			wait "${MEASUREMENT_PID}" 2>/dev/null || true
			MEASUREMENT_PGID=""
			return
		fi
		sleep 0.25
	done
	/bin/kill -TERM -- "-${MEASUREMENT_PGID}" 2>/dev/null || true
	wait "${MEASUREMENT_PID}" 2>/dev/null || true
	MEASUREMENT_PGID=""
}

cleanup_secondary_source()
{
	if [[ -z "${SECONDARY_SOURCE_PGID}" ]] ||
		! process_group_exists "${SECONDARY_SOURCE_PGID}"; then
		if [[ -n "${SECONDARY_SOURCE_PID}" ]]; then
			wait "${SECONDARY_SOURCE_PID}" 2>/dev/null || true
		fi
		SECONDARY_SOURCE_PGID=""
		return
	fi
	printf 'Stopping secondary source process group %s with SIGINT\n' "${SECONDARY_SOURCE_PGID}"
	/bin/kill -INT -- "-${SECONDARY_SOURCE_PGID}" 2>/dev/null || true
	for _ in $(seq 1 20); do
		if ! process_group_exists "${SECONDARY_SOURCE_PGID}"; then
			wait "${SECONDARY_SOURCE_PID}" 2>/dev/null || true
			SECONDARY_SOURCE_PGID=""
			return
		fi
		sleep 0.25
	done
	/bin/kill -TERM -- "-${SECONDARY_SOURCE_PGID}" 2>/dev/null || true
	wait "${SECONDARY_SOURCE_PID}" 2>/dev/null || true
	SECONDARY_SOURCE_PGID=""
}

process_group_exists()
{
	local group="$1"
	ps -eo pgid= | awk -v expected="${group}" '
		$1 == expected {found = 1; exit}
		END {exit !found}'
}

cleanup_all()
{
	cleanup_measurement
	cleanup_secondary_source
	cleanup_launch
}

on_exit()
{
	local status=$?
	trap - EXIT INT TERM
	cleanup_all
	exit "${status}"
}

trap on_exit EXIT
trap 'exit 130' INT TERM

LAUNCH_COMMAND=(
	ros2 launch control_link_bringup adas_vcan_demo.launch.py
	"can_interface:=${CAN_INTERFACE}"
)
printf 'Launch command:'
printf ' %q' "${LAUNCH_COMMAND[@]}"
printf '\n'
printf -v LAUNCH_COMMAND_TEXT '%q ' "${LAUNCH_COMMAND[@]}"

setsid "${LAUNCH_COMMAND[@]}" > "${RUN_DIR}/logs/launch.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID="${LAUNCH_PID}"

READY=false
for _ in $(seq 1 90); do
	if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
		fail "ADAS launch exited before Lifecycle ACTIVE; see logs/launch.log" 6
	fi
	LIFECYCLE_STATE="$(
		timeout 3s ros2 lifecycle get --no-daemon --spin-time 1 \
			/control_link/gateway 2>/dev/null || true)"
	if grep -Eq 'active[[:space:]]*\[3\]' <<< "${LIFECYCLE_STATE}"; then
		READY=true
		break
	fi
	sleep 0.5
done
[[ "${READY}" == true ]] || fail "Gateway did not become Lifecycle ACTIVE" 6

find_group_pid()
{
	local executable="$1"
	ps -eo pid=,pgid=,args= | awk \
		-v group="${LAUNCH_PGID}" \
		-v executable="${executable}" \
		'$2 == group && index($0, executable) != 0 {print $1; exit}'
}

GATEWAY_PID="$(find_group_pid control_link_gateway_node)"
ADAPTER_PID="$(find_group_pid socketcan_adapter_node)"
SIMULATOR_PID="$(find_group_pid vcan_vehicle_simulator)"
SOURCE_PID="$(find_group_pid mock_control_source_node)"
for pair in \
	"gateway:${GATEWAY_PID}" \
	"adapter:${ADAPTER_PID}" \
	"simulator:${SIMULATOR_PID}" \
	"source:${SOURCE_PID}"; do
	label="${pair%%:*}"
	pid="${pair#*:}"
	[[ "${pid}" =~ ^[1-9][0-9]*$ ]] ||
		fail "could not resolve ${label} PID inside launch process group" 6
	[[ -r "/proc/${pid}/stat" ]] || fail "${label} PID disappeared before sampling" 6
done

printf 'Target PIDs: gateway=%s adapter=%s simulator=%s source=%s\n' \
	"${GATEWAY_PID}" "${ADAPTER_PID}" "${SIMULATOR_PID}" "${SOURCE_PID}"

MEASUREMENT_COMMAND=(
	ros2 run control_link_bringup measure_runtime --ros-args
	-p "profile_path:=${PROFILE_PATH}"
	-p "config_root:=${CONFIG_ROOT}"
	-p "output_dir:=${RUN_DIR}/performance"
	-p "can_interface:=${CAN_INTERFACE}"
	-p "warmup_seconds:=${WARMUP_SECONDS}"
	-p "measurement_seconds:=${MEASUREMENT_SECONDS}"
	-p "min_output_ticks:=${MIN_OUTPUT_TICKS}"
	-p "max_extra_wait_seconds:=${MAX_EXTRA_WAIT_SECONDS}"
	-p "target_pids:=[${GATEWAY_PID},${ADAPTER_PID},${SIMULATOR_PID},${SOURCE_PID}]"
	-p "target_labels:=[gateway,adapter,simulator,source]"
	-p "require_source_switch:=${SOURCE_SWITCH_SCENARIO}"
)
printf 'Measurement command:'
printf ' %q' "${MEASUREMENT_COMMAND[@]}"
printf '\n'
printf -v MEASUREMENT_COMMAND_TEXT '%q ' "${MEASUREMENT_COMMAND[@]}"

set +e
setsid "${MEASUREMENT_COMMAND[@]}" > "${RUN_DIR}/logs/measurement.log" 2>&1 &
MEASUREMENT_PID=$!
MEASUREMENT_PGID="${MEASUREMENT_PID}"

if [[ "${SOURCE_SWITCH_SCENARIO}" == true ]]; then
	MEASUREMENT_READY=false
	for _ in $(seq 1 120); do
		if ! kill -0 "${MEASUREMENT_PID}" 2>/dev/null; then
			break
		fi
		if grep -q 'ADAS runtime measurement ready:' "${RUN_DIR}/logs/measurement.log"; then
			MEASUREMENT_READY=true
			break
		fi
		sleep 0.25
	done
	[[ "${MEASUREMENT_READY}" == true ]] ||
		fail "measurement process did not become ready for source-switch scenario" 6

	# warmup 结束后再出现 teleop，使 planning -> teleop 事件落在 measurement window 内
	sleep "$((WARMUP_SECONDS + 1))"
	SECONDARY_SOURCE_COMMAND=(
		ros2 run control_link_adapters mock_control_source_node --ros-args
		-r __ns:=/control_link
		-r __node:=mock_source_teleop
		-p "profile_path:=${PROFILE_PATH}"
		-p "config_root:=${CONFIG_ROOT}"
		-p "source_id:=teleop"
	)
	printf 'Secondary source command:'
	printf ' %q' "${SECONDARY_SOURCE_COMMAND[@]}"
	printf '\n'
	printf -v SECONDARY_SOURCE_COMMAND_TEXT '%q ' "${SECONDARY_SOURCE_COMMAND[@]}"
	setsid "${SECONDARY_SOURCE_COMMAND[@]}" > "${RUN_DIR}/logs/secondary_source.log" 2>&1 &
	SECONDARY_SOURCE_PID=$!
	SECONDARY_SOURCE_PGID="${SECONDARY_SOURCE_PID}"
fi

wait "${MEASUREMENT_PID}"
MEASUREMENT_EXIT_CODE=$?
set -e
MEASUREMENT_PID=""
tail -n 20 "${RUN_DIR}/logs/measurement.log"

cleanup_secondary_source
cleanup_launch
LAUNCH_PID=""
END_SYSTEM_TIME="$(date --iso-8601=seconds)"
END_MONOTONIC_SECONDS="$(cut -d' ' -f1 /proc/uptime)"

STATUS="ERROR"
if [[ "${MEASUREMENT_EXIT_CODE}" -eq 0 ]] &&
	jq -e '.completed == true' "${RUN_DIR}/performance/summary.json" >/dev/null 2>&1; then
	if jq -e '.protocol.protocol_conformant_parameters == true' \
		"${RUN_DIR}/performance/summary.json" >/dev/null; then
		STATUS="PASS"
	else
		STATUS="INVALID"
	fi
fi

(
	cd -- "${RUN_DIR}"
	sha256sum \
		contract_snapshot/* \
		environment.json \
		performance/*.csv \
		performance/summary.json > artifact_sha256.txt
)

cat > "${RUN_DIR}/run_manifest.yaml" <<EOF
schema_version: 1
run_id: ${RUN_ID}
profile: adas
status: ${STATUS}
scope: VM_ONLY_ADAS_EXTERNAL_OBSERVER
git_commit: ${GIT_COMMIT}
git_dirty: ${GIT_DIRTY}
build_type: RelWithDebInfo
ros_domain_id: ${ROS_DOMAIN_ID_VALUE}
rmw: ${RMW_IMPLEMENTATION}
fastdds_profile: ${FAST_DDS_PATH}
can_interface: ${CAN_INTERFACE}
started_at: ${START_SYSTEM_TIME}
finished_at: ${END_SYSTEM_TIME}
started_monotonic_seconds: ${START_MONOTONIC_SECONDS}
finished_monotonic_seconds: ${END_MONOTONIC_SECONDS}
warmup_seconds: ${WARMUP_SECONDS}
measurement_seconds: ${MEASUREMENT_SECONDS}
minimum_output_ticks: ${MIN_OUTPUT_TICKS}
source_switch_scenario: ${SOURCE_SWITCH_SCENARIO}
measurement_exit_code: ${MEASUREMENT_EXIT_CODE}
launch_command: >-
  ${LAUNCH_COMMAND_TEXT}
measurement_command: >-
  ${MEASUREMENT_COMMAND_TEXT}
secondary_source_command: >-
  ${SECONDARY_SOURCE_COMMAND_TEXT:-not_started}
artifacts:
  environment: environment.json
  hashes: artifact_sha256.txt
  output_ticks: performance/output_ticks.csv
  callback_to_output: performance/callback_to_output.csv
  can_round_trip: performance/can_round_trip.csv
  source_switch: performance/source_switch.csv
  resources: performance/resources.csv
  summary: performance/summary.json
  launch_log: logs/launch.log
  measurement_log: logs/measurement.log
  secondary_source_log: logs/secondary_source.log
limitations:
  - vm_only_soft_realtime
  - canonical_interval_is_external_dds_delivery_interval
  - callback_latency_is_single_observer_arrival_delta
  - can_round_trip_is_vcan_observer_delta
  - source_switch_secondary_source_is_not_in_resource_targets_when_enabled
EOF

printf 'Run status: %s\n' "${STATUS}"
printf 'Run directory: %s\n' "${RUN_DIR}"
if [[ "${STATUS}" != "PASS" ]]; then
	exit 7
fi
