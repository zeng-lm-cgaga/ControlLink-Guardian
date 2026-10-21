ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
RUNTIME=${1:?runtime build directory required}
_control_link_nounset=0
if [[ $- == *u* ]]; then _control_link_nounset=1; set +u; fi
source /opt/ros/humble/setup.bash
INTERFACES="$ROOT/reports/2026-10-12_p1_f1/environment/core-install/control_link_interfaces"
source "$INTERFACES/share/control_link_interfaces/local_setup.bash"
NATIVE="$ROOT/reports/valgrind-focused-20261013T142907Z-3199f9"
export LD_LIBRARY_PATH="$RUNTIME/build/control_link_gateway:$RUNTIME/build/control_link_adapters:$NATIVE/install/lib:$LD_LIBRARY_PATH"
export AMENT_PREFIX_PATH="$NATIVE/install:$INTERFACES:$AMENT_PREFIX_PATH"
export FASTRTPS_DEFAULT_PROFILES_FILE="$ROOT/config/common/fastdds_default_profiles.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp RMW_FASTRTPS_USE_QOS_FROM_XML=0
export PYTHONDONTWRITEBYTECODE=1
if [[ $_control_link_nounset == 1 ]]; then set -u; fi
unset _control_link_nounset
