#!/bin/bash
###############################################################################
# udp_port_free.sh
# Linux 版 UdpPortFree - 释放 RflySim 仿真占用的 UDP/TCP 端口
###############################################################################

# 需要 root 权限
if [ "$(id -u)" -ne 0 ]; then
    echo "需要 root 权限，请使用 sudo 运行"
    exec sudo "$0" "$@"
fi

num=2

if [ -z "$num" ] || [ "$num" -le 0 ] 2>/dev/null; then
    echo "Num must be greater than 0. Exiting."
    exit 1
fi

# 检查并释放 UDP 端口
check_and_kill_udp() {
    local port=$1
    local pids=$(ss -lunp | grep ":${port} " | grep -oP 'pid=\K[0-9]+' | sort -u)
    if [ -n "$pids" ]; then
        for pid in $pids; do
            echo "UDP port ${port} is occupied by PID ${pid}. Killing..."
            kill -9 "$pid" 2>/dev/null && echo "  Successfully killed PID ${pid}" || echo "  Failed to kill PID ${pid}"
        done
    else
        echo "UDP port ${port} is free."
    fi
}

# 检查并释放 TCP 端口
check_and_kill_tcp() {
    local port=$1
    local pids=$(ss -tlnp | grep ":${port} " | grep -oP 'pid=\K[0-9]+' | sort -u)
    if [ -n "$pids" ]; then
        for pid in $pids; do
            echo "TCP port ${port} is occupied by PID ${pid}. Killing..."
            kill -9 "$pid" 2>/dev/null && echo "  Successfully killed PID ${pid}" || echo "  Failed to kill PID ${pid}"
        done
    else
        echo "TCP port ${port} is free."
    fi
}

echo ""
echo "=== Kill ROS / RflySim related processes ==="
pkill -f "roscore" 2>/dev/null
pkill -f "rosmaster" 2>/dev/null
pkill -f "bt_ros" 2>/dev/null
pkill -f "ego_planner" 2>/dev/null
pkill -f "main.py" 2>/dev/null
pkill -f "det.py" 2>/dev/null
pkill -f "aruco_detect_node" 2>/dev/null

echo ""
echo "=== Checking fixed ports ==="
for port in 20005 20006 20007 20008 20009 20010 20011 14550; do
    check_and_kill_udp $port
done

echo ""
echo "=== Checking variable ports for $num vehicles ==="

for ((i=0; i<num; i++)); do
    # TCP
    check_and_kill_tcp $((4560 + i))
    # UDP
    check_and_kill_udp $((20100 + i * 2))
    check_and_kill_udp $((20100 + i * 2 + 1))
    check_and_kill_udp $((30100 + i * 2))
    check_and_kill_udp $((30100 + i * 2 + 1))
    check_and_kill_udp $((16540 + i))
    check_and_kill_udp $((17540 + i))
    check_and_kill_udp $((18570 + i))
    check_and_kill_udp $((6001 + i))
done

echo ""
echo "All specified ports have been checked and freed."
