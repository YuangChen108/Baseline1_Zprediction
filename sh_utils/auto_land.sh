#!/bin/bash

# ==========================================
# 步骤 0: 既然是静止靶，彻底封印小车的移动指令！
# ==========================================
echo "[Script] 0. Boat is stationary. Skipping CAR movement."
# 把原来发布 /move_base_simple/goal 的代码全部删掉或注释掉

# ==========================================
# 步骤 1: 发送 TRACK 信号 (让飞机飞到头顶)
# ==========================================
echo "[Script] 1. Sending TRACK Trigger..."

rostopic pub /triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] TRACK command sent!"

# ==========================================
# 步骤 2: 🚨 核心修改！给足对准时间 (改成 8 秒)
# ==========================================
echo "[Script] Waiting for 8 seconds to ensure perfect alignment..."
# 给飞机充足的时间去飞到船的正上方并彻底稳住！
sleep 8

# ==========================================
# 步骤 3: 发送 LAND 信号 (化身打桩机垂直下落)
# ==========================================
echo "[Script] 2. Drone is aligned! Sending LAND Trigger..."

rostopic pub /land_triger geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: now
  frame_id: 'world'
pose:
  position:
    x: 0.0
    y: 0.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

echo "[Script] LAND command sent! Watch the drone."