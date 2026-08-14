#!/usr/bin/env bash

set -Eeuo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
WORKSPACE_DIR="${ROOT_DIR}/ros2_ws"
BUILD_BASE="${WORKSPACE_DIR}/build/e12"
INSTALL_BASE="${WORKSPACE_DIR}/install/e12"
LOG_BASE="${WORKSPACE_DIR}/log/e12"
E12_LOCK_PATH="${TMPDIR:-/tmp}/control_link_guardian_e12.lock"
ROS_DISTRO_VALUE="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID_VALUE=210
SKIP_ROSDEP=false
CURRENT_STEP="argument validation"

usage()
{
	printf '%s\n' \
		"Usage: $0 [--skip-rosdep] [--domain-id <0..232>]" \
		"" \
		"Runs structured-file checks, a clean RelWithDebInfo build," \
		"the existing unit/headless launch tests, and dynamic-link checks"
}

fail()
{
	printf 'ERROR: %s\n' "$1" >&2
	exit "${2:-4}"
}

while (($# > 0)); do
	case "$1" in
	--skip-rosdep)
		SKIP_ROSDEP=true
		shift
		;;
	--domain-id)
		(($# >= 2)) || fail "--domain-id requires a value" 2
		ROS_DOMAIN_ID_VALUE="$2"
		shift 2
		;;
	-h|--help)
		usage
		exit 0
		;;
	*)
		fail "unknown argument: $1" 2
		;;
	esac
done

[[ "${ROS_DOMAIN_ID_VALUE}" =~ ^[0-9]+$ ]] || fail "domain-id must be unsigned" 2
if ((${#ROS_DOMAIN_ID_VALUE} > 3)) ||
	((${#ROS_DOMAIN_ID_VALUE} == 3 && 10#${ROS_DOMAIN_ID_VALUE} > 10#232)); then
	fail "domain-id must be at most 232" 2
fi
[[ "${ROS_DISTRO_VALUE}" == "humble" ]] ||
	fail "E12 is pinned to ROS2 Humble; actual=${ROS_DISTRO_VALUE}" 3
[[ -d "${WORKSPACE_DIR}/src" ]] || fail "ROS2 workspace source directory is missing" 3
[[ -f "/opt/ros/${ROS_DISTRO_VALUE}/setup.bash" ]] ||
	fail "ROS2 ${ROS_DISTRO_VALUE} setup is missing" 3
command -v flock >/dev/null || fail "required command is missing: flock" 3

# e12 使用固定的 build/install/log 目录，必须禁止两个流水线互相清理证据
exec 9>"${E12_LOCK_PATH}"
flock -n 9 || fail "another E12 pipeline is already using the workspace evidence directories" 3

# 只清理脚本独占的 e12 子目录，不触碰开发者日常 build/install/log
for owned_path in "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"; do
	case "${owned_path}" in
	"${WORKSPACE_DIR}/build/e12"|"${WORKSPACE_DIR}/install/e12"|"${WORKSPACE_DIR}/log/e12")
		;;
	*)
		fail "refusing to clean an unexpected path: ${owned_path}" 3
		;;
	esac
done
rm -rf -- "${BUILD_BASE}" "${INSTALL_BASE}" "${LOG_BASE}"
mkdir -p -- "${LOG_BASE}/control_link_ci"
STATUS_PATH="${LOG_BASE}/control_link_ci/status.env"

on_exit()
{
	local exit_code=$?
	local status_temp_path="${STATUS_PATH}.tmp"
	local status_write_failed=false
	# EXIT trap 可能在失败路径执行，先写完整临时文件再安装状态快照
	if ! printf 'schema_version=1\nexit_code=%s\nlast_step=%q\n' \
		"${exit_code}" "${CURRENT_STEP}" > "${status_temp_path}"; then
		status_write_failed=true
	else
		# status.env 属于本次运行，必须原子替换旧快照，否则失败运行会残留旧的成功状态
		if ! mv --force -- "${status_temp_path}" "${STATUS_PATH}"; then
			status_write_failed=true
		fi
	fi
	rm -f -- "${status_temp_path}"
	if [[ "${status_write_failed}" == true ]]; then
		printf 'ERROR: could not install E12 status file: %s\n' "${STATUS_PATH}" >&2
		# 原本成功但状态证据无法落盘时，不能继续报告成功；原始失败码保持不变
		if ((exit_code == 0)); then
			exit_code=4
		fi
	fi
	if ((exit_code == 0)); then
		printf 'E12 local pipeline completed successfully\n'
	else
		printf 'E12 local pipeline failed: step=%s exit_code=%s\n' \
			"${CURRENT_STEP}" "${exit_code}" >&2
	fi
	trap - EXIT
	exit "${exit_code}"
}
trap on_exit EXIT

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_VALUE}/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_VALUE}"
export RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
export RMW_FASTRTPS_USE_QOS_FROM_XML="0"
export LIBGL_ALWAYS_SOFTWARE="1"
# 这台 VM 的内存不足以稳定并行编译 bringup 中多个大型验证工具，
# 固定 CMake 单作业保证本地与 CI 的构建结果可复现
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="-j1"

for command_name in colcon python3 readelf ldd realpath; do
	command -v "${command_name}" >/dev/null || fail "required command is missing: ${command_name}" 3
done
python3 -c 'import yaml' >/dev/null 2>&1 || fail "Python PyYAML is required" 3

CURRENT_STEP="shell syntax check"
mapfile -t SHELL_FILES < <(find "${ROOT_DIR}/scripts" -maxdepth 1 -type f -name '*.sh' -print | sort)
((${#SHELL_FILES[@]} > 0)) || fail "no shell scripts were found" 3
bash -n "${SHELL_FILES[@]}"
if command -v shellcheck >/dev/null; then
	shellcheck --severity=warning "${SHELL_FILES[@]}"
else
	printf 'WARN: shellcheck is unavailable; bash -n completed\n'
fi

CURRENT_STEP="YAML XML and Python syntax check"
python3 - "${ROOT_DIR}" <<'PY'
import ast
import pathlib
import sys
import xml.etree.ElementTree as element_tree

import yaml
from yaml.constructor import ConstructorError


class UniqueKeyLoader(yaml.SafeLoader):
	pass


def construct_unique_mapping(loader, node, deep=False):
	mapping = {}
	for key_node, value_node in node.value:
		key = loader.construct_object(key_node, deep=deep)
		if key in mapping:
			raise ConstructorError(
				"while constructing a mapping",
				node.start_mark,
				"duplicate key: " + repr(key),
				key_node.start_mark,
			)
		mapping[key] = loader.construct_object(value_node, deep=deep)
	return mapping


UniqueKeyLoader.add_constructor(
	yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG,
	construct_unique_mapping,
)

root = pathlib.Path(sys.argv[1]).resolve(strict=True)
yaml_files = sorted((root / "config").rglob("*.yaml"))
yaml_files += sorted((root / "config").rglob("*.yml"))
workflow_root = root / ".github" / "workflows"
if workflow_root.is_dir():
	yaml_files += sorted(workflow_root.glob("*.yaml"))
	yaml_files += sorted(workflow_root.glob("*.yml"))

xml_files = sorted((root / "config").rglob("*.xml"))
for pattern in ("package.xml", "*.xml", "*.xacro", "*.sdf"):
	xml_files += sorted((root / "ros2_ws" / "src").rglob(pattern))
xml_files = sorted(set(xml_files))

python_files = sorted((root / "ros2_ws" / "src").rglob("*.py"))

if not yaml_files or not xml_files or not python_files:
	raise SystemExit("structured-file discovery returned an empty category")

for path in yaml_files:
	with path.open("r", encoding="utf-8") as source:
		yaml.load(source, Loader=UniqueKeyLoader)
for path in xml_files:
	element_tree.parse(path)
for path in python_files:
	source = path.read_text(encoding="utf-8")
	ast.parse(source, filename=str(path))

print(
	"Structured files parsed successfully: "
	+ f"yaml={len(yaml_files)} xml={len(xml_files)} python={len(python_files)}"
)
PY

if [[ "${SKIP_ROSDEP}" == false ]]; then
	CURRENT_STEP="rosdep install"
	command -v rosdep >/dev/null || fail "rosdep is required" 3
	rosdep install \
		--from-paths "${WORKSPACE_DIR}/src" \
		--ignore-src \
		--rosdistro "${ROS_DISTRO_VALUE}" \
		-y
else
	printf 'Skipping rosdep because --skip-rosdep was requested\n'
fi

CURRENT_STEP="colcon build"
cd -- "${WORKSPACE_DIR}"
colcon --log-base "${LOG_BASE}" build \
	--base-paths src \
	--build-base "${BUILD_BASE}" \
	--install-base "${INSTALL_BASE}" \
	--symlink-install \
	--executor sequential \
	--event-handlers console_direct+ \
	--cmake-args \
		-DCMAKE_BUILD_TYPE=RelWithDebInfo \
		-DBUILD_TESTING=ON

set +u
# shellcheck disable=SC1090
source "${INSTALL_BASE}/setup.bash"
set -u

CURRENT_STEP="all registered package tests"
colcon --log-base "${LOG_BASE}" test \
	--base-paths src \
	--build-base "${BUILD_BASE}" \
	--install-base "${INSTALL_BASE}" \
	--test-result-base "${BUILD_BASE}" \
	--packages-select \
		control_link_interfaces \
		control_link_contract \
		control_link_gateway \
		control_link_adapters \
		control_link_bringup \
	--executor sequential \
	--event-handlers console_direct+ \
	--return-code-on-test-failure

CURRENT_STEP="colcon test result audit"
TEST_RESULT_PATH="${LOG_BASE}/control_link_ci/test-result.txt"
colcon test-result \
	--test-result-base "${BUILD_BASE}" \
	--verbose | tee "${TEST_RESULT_PATH}"
grep -Eq 'Summary: [1-9][0-9]* tests, 0 errors, 0 failures, 0 skipped' \
	"${TEST_RESULT_PATH}" ||
	fail "test-result audit did not report a non-empty, clean test suite" 4

CURRENT_STEP="dynamic link audit"
LDD_LOG_PATH="${LOG_BASE}/control_link_ci/ldd-r.log"
: > "${LDD_LOG_PATH}"
declare -A SEEN_ELF_FILES=()
ELF_COUNT=0
while IFS= read -r -d '' candidate; do
	resolved="$(realpath -e -- "${candidate}")" || fail "broken install artifact: ${candidate}"
	[[ -z "${SEEN_ELF_FILES[${resolved}]:-}" ]] || continue
	SEEN_ELF_FILES["${resolved}"]=1
	readelf -h "${resolved}" >/dev/null 2>&1 || continue
	((ELF_COUNT += 1))
	set +e
	ldd_output="$(ldd -r -- "${resolved}" 2>&1)"
	ldd_status=$?
	set -e
	printf '\n### %s\n%s\n' "${resolved}" "${ldd_output}" >> "${LDD_LOG_PATH}"
	((ldd_status == 0)) || fail "ldd -r failed for ${resolved}"
	if grep -Eq 'not found|undefined symbol' <<< "${ldd_output}"; then
		fail "unresolved dynamic symbol or dependency in ${resolved}"
	fi
done < <(
	find -L "${INSTALL_BASE}" -path '*/lib/*' -type f -print0
)
((ELF_COUNT > 0)) || fail "dynamic link audit did not find any ELF artifact"
printf 'Dynamic link audit passed: elf_artifacts=%s log=%s\n' \
	"${ELF_COUNT}" "${LDD_LOG_PATH}"

CURRENT_STEP="completed"
