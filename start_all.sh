#!/usr/bin/env bash
# start_all.sh — 3-DOF Robotic Arm: Gazebo + RViz + Slider GUI + Arduino Bridge
source /opt/ros/humble/setup.bash

echo "=========================================="
echo " 1. Stopping any old processes..."
echo "=========================================="
pkill -f 'ign gazebo'           2>/dev/null
pkill -f 'gz sim'               2>/dev/null
pkill -f 'ruby.*ign'            2>/dev/null
pkill -f 'gazebo.*ruby'         2>/dev/null
pkill -f robot_state_publisher  2>/dev/null
pkill -f bridge.py              2>/dev/null
pkill -f gui_controller.py       2>/dev/null
pkill -f 'rviz2'                2>/dev/null
sleep 2

export IGN_GAZEBO_SYSTEM_PLUGIN_PATH=/opt/ros/humble/lib
export IGN_GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib
export LD_LIBRARY_PATH=/opt/ros/humble/lib:/opt/ros/humble/opt:${LD_LIBRARY_PATH:-}

LOGDIR=/tmp/three_dof_arm_logs
mkdir -p "$LOGDIR"

echo "=========================================="
echo " 2. Launching Gazebo + ros2_control..."
echo "    (Log: $LOGDIR/gazebo.log)"
echo "=========================================="
ros2 launch ~/three_dof_arm/launch/launch_sim.py > "$LOGDIR/gazebo.log" 2>&1 &

echo "    Waiting for arm_controller to activate..."
ACTIVE=
for i in $(seq 1 25); do
  if ros2 topic list 2>/dev/null | grep -q '/arm_controller/joint_trajectory'; then
    ACTIVE=1
    break
  fi
  sleep 1
done

if [ -z "$ACTIVE" ]; then
  echo "    [NOTE] Gazebo controller taking time or still initializing. Check $LOGDIR/gazebo.log"
else
  echo "    ✓ arm_controller is ACTIVE in Gazebo!"
fi

echo "=========================================="
echo " 3. Launching RViz..."
echo "    (Log: $LOGDIR/rviz.log)"
echo "=========================================="
rviz2 -d ~/three_dof_arm/config/rviz_arm.rviz > "$LOGDIR/rviz.log" 2>&1 &
sleep 2

echo "=========================================="
echo " 4. Launching GUI Controller with Sliders..."
echo "=========================================="
python3 ~/three_dof_arm/gui_controller.py &

echo
echo "=========================================="
echo " All systems launched!"
echo " Slider will move RViz, Gazebo & Arduino in sync!"
echo "=========================================="
