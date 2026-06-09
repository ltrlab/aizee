#!/bin/bash
# Jetson side: log tegrastats + camera-publisher RSS once per second.
# Run on the Jetson while collect_demo.py is teleoperating from the host.
#
# Output:
#   /tmp/aizee_jetson_<unix-ts>.tegrastats    raw tegrastats stream
#   /tmp/aizee_jetson_<unix-ts>.rss           per-process RSS over time
#
# Stop with Ctrl-C — both child processes are cleaned up by the trap.

set -u
TS=$(date +%s)
BASE=/tmp/aizee_jetson_${TS}
TEGRA="${BASE}.tegrastats"
RSS="${BASE}.rss"

echo "tegrastats → ${TEGRA}"
echo "rss        → ${RSS}"

if ! command -v tegrastats >/dev/null; then
    echo "tegrastats not found on PATH; skipping thermal/clock log" >&2
    TEGRA_PID=""
else
    tegrastats --interval 1000 > "${TEGRA}" &
    TEGRA_PID=$!
fi

cleanup() {
    [ -n "${TEGRA_PID}" ] && kill "${TEGRA_PID}" 2>/dev/null || true
    wait 2>/dev/null || true
    echo
    echo "Logs:"
    [ -n "${TEGRA_PID}" ] && echo "  ${TEGRA}"
    echo "  ${RSS}"
}
trap cleanup EXIT INT TERM

# Process names we care about — extend if you add more node types.
PROCS=(gripper_camera_node camera_node)

echo "ts pid name rss_kb vmsize_kb threads" > "${RSS}"
while true; do
    now=$(date +%H:%M:%S)
    for proc in "${PROCS[@]}"; do
        # pgrep -a returns "pid cmdline"; -f matches the full command line.
        while read -r pid cmdline; do
            [ -z "${pid}" ] && continue
            if [ -r "/proc/${pid}/status" ]; then
                rss=$(awk '/^VmRSS:/   {print $2}' /proc/${pid}/status)
                vsz=$(awk '/^VmSize:/  {print $2}' /proc/${pid}/status)
                thr=$(awk '/^Threads:/ {print $2}' /proc/${pid}/status)
                echo "${now} ${pid} ${proc} ${rss:-?} ${vsz:-?} ${thr:-?}" >> "${RSS}"
            fi
        done < <(pgrep -af "${proc}" 2>/dev/null || true)
    done
    sleep 1
done
