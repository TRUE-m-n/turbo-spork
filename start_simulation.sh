#!/bin/bash
# set -e
# ==================== 全局环境与路径设置 ====================
export LANG=en_US.UTF-8
export LC_ALL=en_US.UTF-8
export LC_CTYPE=en_US.UTF-8
export TERM=xterm-256color
export PYTHONIOENCODING=utf-8
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
if [ -z "$PSP_PATH" ]; then
    export PSP_PATH="/home/ubuntu/PX4PSP"
fi
# ==================== 进程清理机制 ====================
# 捕捉 Ctrl+C (SIGINT) 和脚本退出 (EXIT)，确保彻底清理所有进程
cleanup() {
    echo -e "\nCaught interrupt signal or script exiting. Cleaning up processes..."
    pkill -x px4 2>/dev/null || true
    pkill -x CopterSim 2>/dev/null || true
    pkill -x QGroundControl 2>/dev/null || true
    pkill -x RflySim3D 2>/dev/null || true
    pkill -x roscore 2>/dev/null || true
    pkill -x rosmaster 2>/dev/null || true
    pkill -x rostrans 2>/dev/null || true
    pkill -f "CreateScene.py" 2>/dev/null || true
    # 杀死所有由本脚本启动的后台子进程
    jobs -p | xargs -I {} kill -- {} 2>/dev/null || true
    echo "Cleanup complete."
    exit 0
}
trap cleanup SIGINT SIGTERM EXIT
# ==================== 前置脚本执行 ====================
bash ./udp_port_free.sh
bash ./kill_ros_pid.sh
# ==================== RflySim 仿真参数配置 ====================
modelList="MulticopterNOpx4:0:1:1,CarR1Diff:r1_rover:2:1"
UDP_START_PORT=20100
SimMode=2
UE4_MAP=StreetBattleScene
ORIGIN_POS_X=0
ORIGIN_POS_Y=0
ORIGIN_YAW=0
VEHICLE_INTERVAL=2
IS_BROADCAST=0
UDPSIMMODE=2
CLASS_3D_ID_1=310
CLASS_3D_ID_2=101000825
TOTOAL_COPTER=0
# ==================== ROS & Xterm 通用参数 ====================
XTERM_OPTS="-iconic -en UTF-8 -u8 -lc -sl 10000 -bg black -fg white -fa 'Monospace' -fs 12"
# 提取 xterm 中重复的终端环境变量配置
XTERM_ENV="export LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 TERM=xterm-256color PYTHONIOENCODING=utf-8; source /opt/ros/noetic/setup.bash 2>/dev/null;"
# ==================== 核心功能函数 ====================
# 启动 QGroundControl 和 RflySim3D
start_rflysim_ui() {
    \cp -rf "${SCRIPT_DIR}/MulticopterNOpx4.so" "${PSP_PATH}/CopterSim/external/model/MulticopterNOpx4.so" 2>/dev/null || true
    echo "Copy MulticopterNOpx4.so to CopterSim (综合模型)"
    local ld_path=$LD_LIBRARY_PATH
    export LD_LIBRARY_PATH=/opt/gstreamer/lib/x86_64-linux-gnu:$PSP_PATH/QGroundControl/squashfs-root/Qt/libs:$ld_path
    # cd ${PSP_PATH}/QGroundControl/squashfs-root
    # ./AppRun >/dev/null 2>&1 &
    sleep 1
    pkill -x RflySim3D || true
    echo "Start RflySim3D"
    cd ${PSP_PATH}/RflySimUE5/RflySim3D/Binaries/Linux
    ./RflySim3D >/dev/null 2>&1 &
    sleep 5
}
# 启动 CopterSim 实例群
start_coptersim() {
    cd "$PSP_PATH/CopterSim"
    IFS=',' read -ra MODELS <<< "$modelList"
    # 统计模型总数并计算 sqrtNum
    for G in "${MODELS[@]}"; do
        IFS=':' read -ra FIELDS <<< "$G"
        TOTOAL_COPTER=$((TOTOAL_COPTER + FIELDS[3]))
    done
    local sqrtNum=1
    while [ $((sqrtNum * sqrtNum)) -lt $TOTOAL_COPTER ]; do
        sqrtNum=$((sqrtNum + 1))
    done
    local modelIdx=0
    for G in "${MODELS[@]}"; do
        IFS=':' read -ra FIELDS <<< "$G"
        local DLLModel=${FIELDS[0]}
        local PX4Frame=${FIELDS[1]}
        local startIdx=${FIELDS[2]}
        local numVeh=${FIELDS[3]}
        modelIdx=$((modelIdx + 1))
        local CUR_CLASS_3D_ID=$([ $modelIdx -eq 1 ] && echo $CLASS_3D_ID_1 || echo $CLASS_3D_ID_2)
        local CUR_SIM_MODE CUR_UDP_MODE
        if [ "$DLLModel" = "MulticopterNOpx4" ]; then
            CUR_SIM_MODE=3
            CUR_UDP_MODE="Mavlink_Vision"
        else
            CUR_SIM_MODE=$SimMode
            CUR_UDP_MODE=$UDPSIMMODE
        fi
        local cntr=$startIdx
        local endNum=$((startIdx + numVeh - 1))
        while [ $cntr -le $endNum ]; do
            local PosXX PosYY
            if [ "$DLLModel" = "CarR1Diff" ]; then
                PosXX=-4.5; PosYY=1.9
            elif [ "$DLLModel" = "MulticopterNOpx4" ]; then
                PosXX=-4.47; PosYY=-1.7
            else
                PosXX=$(( (cntr - 1) / sqrtNum * VEHICLE_INTERVAL + ORIGIN_POS_X ))
                PosYY=$(( (cntr - 1) % sqrtNum * VEHICLE_INTERVAL + ORIGIN_POS_Y ))
            fi
            echo "Start CopterSim #${cntr}  model=${DLLModel}  SimMode=${CUR_SIM_MODE}  UDP=${CUR_UDP_MODE}  pos=(${PosXX}, ${PosYY})"
            ./CopterSim 1 $cntr $CUR_CLASS_3D_ID $DLLModel $CUR_SIM_MODE $UE4_MAP $IS_BROADCAST $PosXX $PosYY $ORIGIN_YAW 1 $CUR_UDP_MODE &
            sleep 2
            cntr=$((cntr + 1))
        done
    done
    cd "$SCRIPT_DIR"
}
# 生成并运行 PX4 SITL 脚本
run_px4_sitl() {
    local TEMP_SCRIPT="$PSP_PATH/RflySimAPIs/BatScripts/run_sitl_script.sh"
    mkdir -p "$(dirname "$TEMP_SCRIPT")"
    # echo -e "#!/bin/bash\necho Starting PX4 Build\ncd $PSP_PATH/Firmware\n./BkFile/EnvOri.sh\nmake px4_sitl_default" > "$TEMP_SCRIPT"
    IFS=',' read -ra MODELS <<< "$modelList"
    for G in "${MODELS[@]}"; do
        IFS=':' read -ra FIELDS <<< "$G"
        echo "./Tools/sitl_multiple_run_rfly.sh ${FIELDS[3]} ${FIELDS[2]} ${FIELDS[1]}" >> "$TEMP_SCRIPT"
    done
    echo -e "echo 'Press any key to exit'\nread -n 1" >> "$TEMP_SCRIPT"
    chmod +x "$TEMP_SCRIPT"
    "$TEMP_SCRIPT"
}
# 在 Xterm 中启动 ROS 相关节点
start_ros_nodes() {
    echo "Starting ROS and terminals with xterm-256color"
    # 1. roscore
    xterm $XTERM_OPTS -T "roscore" -e "bash --login -c '${XTERM_ENV} roscore 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
    sleep 20
    # 2. CreateScene
    xterm $XTERM_OPTS -T "load_scene" -e "bash --login -c '${XTERM_ENV} cd \"$SCRIPT_DIR\" && python3 -u ./uav/CreateScene.py 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
    sleep 5
    # 3. UGV mavros
    xterm $XTERM_OPTS -T "mavros_ugv" -e "bash --login -c '${XTERM_ENV} roslaunch px4_ns.launch ns:=ugv fcu_url:=udp://:20103@127.0.0.1:20102 tgt_system:=2 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
    sleep 5
    # 4. RosTrans
    xterm $XTERM_OPTS -T "RosTrans" -e "bash --login -c '${XTERM_ENV} rostrans'" &
    sleep 5
}
# 启动 foxglove_bridge 节点
start_foxglove_bridge() {
    echo "Starting foxglove_bridge on port 9090..."
    xterm $XTERM_OPTS -T "foxglove_bridge" -e "bash --login -c '${XTERM_ENV} roslaunch foxglove_bridge foxglove_bridge.launch port:=9090 send_buffer_limit:=50000000 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
    sleep 3
}
# 启动 UAV-System AppRun
start_uav_system() {
    echo "Starting UAV-System AppRun..."
    xterm $XTERM_OPTS -T "uav_system" -e "bash --login -c 'cd /home/ubuntu/PX4PSP/demo/UAV-System.AppDir && ./AppRun 2>&1; echo \"Press Enter to close\"; read; exec bash'" &
    sleep 3
}
# ==================== 主执行流程 ====================
echo "=========================================="
echo "Initializing Simulation Environment..."
echo "=========================================="
start_rflysim_ui
start_coptersim
start_ros_nodes
start_foxglove_bridge
start_uav_system
run_px4_sitl
echo "Simulation run ended."
exit 0