#!/usr/bin/env bash
# Source this file to give the demo a self-contained, loopback-only DDS setup.
#
# Why: ROS 2 nodes in separate processes (Nav2, the Gazebo bridge, the benchmark
# runner) must discover each other over DDS. On some hosts that fails because:
#   * a stale ROS_DISCOVERY_SERVER points at an unreachable address, and/or
#   * the loopback device has no MULTICAST flag (common on WSL2), so the default
#     multicast discovery never reaches other local processes.
#
# This script clears any inherited discovery config, starts a *local* Fast DDS
# discovery server on 127.0.0.1:11811, and points every participant at it using
# a loopback-only transport profile (demo/config/fastdds_localhost.xml). All
# discovery and data then flow over loopback unicast / shared memory.
#
# Usage:
#   source demo/scripts/ros_env.sh        # set up + start server
#   source demo/scripts/ros_env.sh stop   # stop the local discovery server
#
# Modes:
#   export DEMO_DDS=server   # default: Fast DDS discovery server on loopback
#   export DEMO_DDS=cyclone  # use CycloneDDS, useful if Fast-DDS/Fast-CDR ABI is broken
#   export DEMO_DDS=off      # leave the host's normal DDS config untouched

_DEMO_SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
_DEMO_DIR="$(cd "${_DEMO_SCRIPTS_DIR}/.." && pwd)"
_DEMO_DDS_PROFILE="${_DEMO_DIR}/config/fastdds_localhost.xml"
_DEMO_DDS_PIDFILE="/tmp/demo_fastdds_discovery.pid"
_DEMO_DDS_PORT="${DEMO_DDS_PORT:-11811}"

demo_dds_stop() {
  if [[ -f "${_DEMO_DDS_PIDFILE}" ]]; then
    kill -9 "$(cat "${_DEMO_DDS_PIDFILE}")" 2>/dev/null
    rm -f "${_DEMO_DDS_PIDFILE}"
  fi
  pkill -9 -f "fastdds discovery -i 0 -l 127.0.0.1 -p ${_DEMO_DDS_PORT}" 2>/dev/null
}

if [[ "${1:-}" == "stop" ]]; then
  demo_dds_stop
  echo "[ros_env] local discovery server stopped"
  return 0 2>/dev/null || exit 0
fi

if [[ "${DEMO_DDS:-server}" == "off" ]]; then
  echo "[ros_env] DEMO_DDS=off -> leaving host DDS config untouched"
  return 0 2>/dev/null || exit 0
fi

if [[ "${DEMO_DDS:-server}" == "cyclone" ]]; then
  demo_dds_stop
  unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT ROS_DISCOVERY_INTERFACE ROS_DISCOVERY_PORT
  unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  echo "[ros_env] using CycloneDDS (${RMW_IMPLEMENTATION})"
  return 0 2>/dev/null || exit 0
fi

# Clear inherited (possibly broken) discovery configuration.
unset RMW_IMPLEMENTATION
unset ROS_DISCOVERY_INTERFACE
unset ROS_DISCOVERY_PORT

export ROS_DISCOVERY_SERVER="127.0.0.1:${_DEMO_DDS_PORT}"
export ROS_SUPER_CLIENT=TRUE
export FASTRTPS_DEFAULT_PROFILES_FILE="${_DEMO_DDS_PROFILE}"
export FASTDDS_DEFAULT_PROFILES_FILE="${_DEMO_DDS_PROFILE}"

# Start a local discovery server if nothing is already listening on the port.
if ! ss -uln 2>/dev/null | grep -q ":${_DEMO_DDS_PORT}\b"; then
  fastdds discovery -i 0 -l 127.0.0.1 -p "${_DEMO_DDS_PORT}" \
    >/tmp/demo_fastdds_discovery.log 2>&1 &
  echo $! > "${_DEMO_DDS_PIDFILE}"
  sleep 3
  echo "[ros_env] started local Fast DDS discovery server on 127.0.0.1:${_DEMO_DDS_PORT} (pid $(cat ${_DEMO_DDS_PIDFILE}))"
else
  echo "[ros_env] reusing discovery server already on 127.0.0.1:${_DEMO_DDS_PORT}"
fi

echo "[ros_env] ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER}  profile=${_DEMO_DDS_PROFILE}"
