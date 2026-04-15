#!/usr/bin/env bash
set -euo pipefail

# You can override this at runtime:
#   SUDO_PASS='your_password' ./run_robodog_stack.sh
SUDO_PASS="${SUDO_PASS:-robodog1234}"
WORKSPACE="/home/robodog/RoboDog_ws"
LOG_DIR="/tmp/robodog_logs"
USER_NAME="${USER:-robodog}"

sudo_run() {
  printf '%s\n' "$SUDO_PASS" | sudo -S -p '' "$@"
}

is_port_open() {
  local port="$1"
  ss -ltn "( sport = :${port} )" | grep -q ":${port}"
}

group_has_user() {
  local group="$1"
  id -nG | tr ' ' '\n' | grep -qx "$group"
}

print_topic_status() {
  local topic="$1"
  local info
  info="$(ros2 topic info "$topic" 2>/dev/null || true)"
  local pubs
  pubs="$(echo "$info" | awk -F': ' '/Publisher count/{print $2}')"
  if [[ -n "$pubs" && "$pubs" != "0" ]]; then
    echo "  - $topic: publishers=$pubs (OK)"
  else
    echo "  - $topic: publishers=0 (NO DATA)"
  fi
}

echo "[0/8] Preparing permissions (I2C/GPIO) and preflight checks..."
sudo_run groupadd -f gpio
sudo_run usermod -aG i2c,dialout,gpio "$USER_NAME"
sudo_run bash -lc "echo 'SUBSYSTEM==\"gpio\", KERNEL==\"gpiochip*\", GROUP=\"gpio\", MODE=\"0660\"' > /etc/udev/rules.d/99-gpio.rules"
sudo_run udevadm control --reload-rules
sudo_run udevadm trigger

echo "  - Current groups: $(id -nG)"
if group_has_user i2c && group_has_user dialout && group_has_user gpio; then
  echo "  - Group membership in current session: OK"
else
  echo "  - Group membership in current session: INCOMPLETE"
  echo "    Re-login or reboot required after first-time permission setup."
fi

echo "  - Device permissions:"
ls -l /dev/i2c-1 2>/dev/null || echo "    /dev/i2c-1 not found"
ls -l /dev/gpiochip* 2>/dev/null || echo "    /dev/gpiochip* not found"

echo "[1/8] Killing old processes..."
sudo_run killall -9 \
  rosbridge_websocket \
  web_video_server \
  camera_detection_node \
  ncnn_detection_node \
  rpicam-vid \
  rpicam-still \
  libcamera-vid \
  libcamera-still \
  2>/dev/null || true

echo "[2/8] Cleaning shared memory and temp camera files..."
sudo_run rm -rf /dev/shm/fastdds* /tmp/robodog_camera 2>/dev/null || true
mkdir -p "$LOG_DIR"

# ROS environment
cd "$WORKSPACE"

# ROS setup scripts may read optional vars that are unset; source them with nounset disabled.
set +u
source /opt/ros/kilted/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "[3/8] Starting main launch stack..."
nohup ros2 launch robodog_bringup web_interface.launch.py > "$LOG_DIR/web_interface.log" 2>&1 &
sleep 4

echo "[4/8] Ensuring rosbridge is running on port 9090..."
if is_port_open 9090; then
  echo "  - Port 9090 already open"
else
  nohup ros2 run rosbridge_server rosbridge_websocket \
    --ros-args -p port:=9090 -p address:=0.0.0.0 \
    > "$LOG_DIR/rosbridge.log" 2>&1 &
  sleep 2
fi

echo "[5/8] Ensuring web video server is running on port 8080..."
if is_port_open 8080; then
  echo "  - Port 8080 already open"
else
  nohup ros2 run web_video_server web_video_server \
    --ros-args -p port:=8080 -p address:=0.0.0.0 \
    > "$LOG_DIR/web_video_server.log" 2>&1 &
  sleep 2
fi

echo "[6/8] Port status:"
if is_port_open 8080; then
  echo "  - Port 8080: OPEN"
else
  echo "  - Port 8080: CLOSED"
fi

if is_port_open 9090; then
  echo "  - Port 9090: OPEN"
else
  echo "  - Port 9090: CLOSED"
fi

echo "[7/8] Sensor topic publisher checks:"
print_topic_status /temperature
print_topic_status /humidity
print_topic_status /temperature2
print_topic_status /humidity2
print_topic_status /imu/data_raw
print_topic_status /imu/mag
print_topic_status /ultrasonic1
print_topic_status /ultrasonic2
print_topic_status /robodog/interface/status_json

echo "[8/8] WebSocket status_json sample check:"
python3 - <<'PY' || true
import json
from websocket import create_connection

try:
  ws = create_connection('ws://127.0.0.1:9090', timeout=5)
  ws.send(json.dumps({
    'op': 'subscribe',
    'topic': '/robodog/interface/status_json',
    'type': 'std_msgs/msg/String'
  }))
  received = False
  for _ in range(6):
    msg = json.loads(ws.recv())
    if 'msg' in msg and isinstance(msg['msg'], dict) and 'data' in msg['msg']:
      payload = msg['msg']['data']
      if isinstance(payload, str):
        try:
          payload = json.loads(payload)
        except Exception:
          pass
      print('  - WebSocket sample received: OK')
      if isinstance(payload, dict):
        sensors = payload.get('sensors', {})
        imu = payload.get('imu', {})
        print(f"    sensors.temp1_c={sensors.get('temp1_c')} hum1_pct={sensors.get('hum1_pct')}")
        print(f"    imu.accel.x={imu.get('accel', {}).get('x')}")
      else:
        print(f"    payload={payload}")
      received = True
      break
  if not received:
    print('  - WebSocket sample received: NO')
  ws.close()
except Exception as exc:
  print(f'  - WebSocket check failed: {exc}')
PY

echo
echo "Video URL:"
echo "  http://192.168.1.26:8080/stream?topic=/robodog/camera/detections/image&width=320&height=240&quality=35"
echo "WebSocket URL:"
echo "  ws://192.168.1.26:9090"
echo "Status JSON topic over WebSocket:"
echo "  /robodog/interface/status_json"
echo
echo "Logs:"
echo "  $LOG_DIR/web_interface.log"
echo "  $LOG_DIR/rosbridge.log"
echo "  $LOG_DIR/web_video_server.log"
