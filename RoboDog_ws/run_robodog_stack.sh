#!/usr/bin/env bash
set -euo pipefail

# You can override this at runtime:
#   SUDO_PASS='your_password' ./run_robodog_stack.sh
SUDO_PASS="${SUDO_PASS:-robodog1234}"
WORKSPACE="/home/robodog/RoboDog_ws"
LOG_DIR="/tmp/robodog_logs"
USER_NAME="${USER:-robodog}"
I2C_SCAN_TIMEOUT_SEC="${I2C_SCAN_TIMEOUT_SEC:-6}"

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

check_and_enable_i2c() {
  echo "[I2C-CHECK] Verifying I2C interface..."
  local boot_cfg=""

  if [[ -f /boot/firmware/config.txt ]]; then
    boot_cfg="/boot/firmware/config.txt"
  elif [[ -f /boot/config.txt ]]; then
    boot_cfg="/boot/config.txt"
  fi
  
  # Check if /dev/i2c-1 exists
  if [[ -e /dev/i2c-1 ]]; then
    echo "  ✓ /dev/i2c-1 is PRESENT and ACCESSIBLE"
    return 0
  fi
  
  echo "  ✗ /dev/i2c-1 NOT FOUND, attempting to enable I2C..."
  
  # Try raspi-config
  if command -v raspi-config &>/dev/null; then
    echo "    Enabling I2C via raspi-config..."
    sudo_run raspi-config nonint do_i2c 0 || true
    sleep 2
    
    if [[ -e /dev/i2c-1 ]]; then
      echo "  ✓ /dev/i2c-1 NOW AVAILABLE after raspi-config"
      return 0
    fi
  fi
  
  # Ensure I2C is enabled and force standard-mode clock (100 kHz) in boot config.
  if [[ -n "$boot_cfg" ]]; then
    if ! sudo_run grep -q "^dtparam=i2c_arm=on" "$boot_cfg" 2>/dev/null; then
      echo "    Adding dtparam=i2c_arm=on to $boot_cfg..."
      sudo_run bash -c "echo 'dtparam=i2c_arm=on' >> '$boot_cfg'"
    fi

    if sudo_run grep -q "^dtparam=i2c_arm_baudrate=" "$boot_cfg" 2>/dev/null; then
      echo "    Setting dtparam=i2c_arm_baudrate=100000 in $boot_cfg..."
      sudo_run sed -i 's/^dtparam=i2c_arm_baudrate=.*/dtparam=i2c_arm_baudrate=100000/' "$boot_cfg"
    else
      echo "    Adding dtparam=i2c_arm_baudrate=100000 to $boot_cfg..."
      sudo_run bash -c "echo 'dtparam=i2c_arm_baudrate=100000' >> '$boot_cfg'"
    fi

    sleep 1
  fi
  
  if [[ -e /dev/i2c-1 ]]; then
    echo "  ✓ /dev/i2c-1 IS NOW AVAILABLE"
    return 0
  else
    echo "  ⚠ /dev/i2c-1 still not found after configuration"
    echo "    If running in container/VM, I2C may not be available"
    echo "    If on physical RPi, you may need to REBOOT after first setup"
    return 2
  fi
}

detect_i2c_devices() {
  echo "[I2C-SCAN] Scanning I2C bus 1 for devices..."
  
  if ! [[ -e /dev/i2c-1 ]]; then
    echo "  ✗ /dev/i2c-1 not available, skipping scan"
    return 1
  fi
  
  # Try to run i2cdetect
  if ! command -v i2cdetect &>/dev/null; then
    echo "  ⚠ i2c-tools not installed; skipping hardware detection"
    echo "    Install with: sudo apt-get install i2c-tools"
    return 1
  fi
  
  local scan_output
  local scan_rc=0

  if command -v timeout &>/dev/null; then
    scan_output=$(timeout "${I2C_SCAN_TIMEOUT_SEC}"s bash -lc "printf '%s\\n' \"$SUDO_PASS\" | sudo -S -p '' i2cdetect -y 1" 2>&1) || scan_rc=$?
  else
    scan_output=$(sudo_run i2cdetect -y 1 2>&1) || scan_rc=$?
  fi

  if [[ "$scan_rc" -eq 124 ]]; then
    echo "  ⚠ I2C scan timed out after ${I2C_SCAN_TIMEOUT_SEC}s (bus may be stuck)"
    echo "    Continuing startup without blocking."
    return 2
  fi

  if [[ "$scan_rc" -ne 0 ]]; then
    echo "  ⚠ I2C scan failed (rc=$scan_rc):"
    echo "    $scan_output" | head -n 4
    echo "    Continuing startup without blocking."
    return 2
  fi
  
  echo "$scan_output" | head -n 2
  
  local aht10_status="NOT FOUND"
  local mpu_status="NOT FOUND"
  
  # Check for AHT10 at 0x38 or 0x39
  if echo "$scan_output" | grep -qE "^\s+30:\s+.*\s+3[89]" || \
     echo "$scan_output" | grep -q "0x3[89]"; then
    aht10_status="FOUND"
  fi
  
  # Check for MPU9250 at 0x68 or 0x69
  if echo "$scan_output" | grep -qE "^\s+60:\s+.*\s+6[89]" || \
     echo "$scan_output" | grep -q "0x6[89]"; then
    mpu_status="FOUND"
  fi
  
  echo "  - AHT10 (0x38/0x39): $aht10_status"
  echo "  - MPU9250 (0x68/0x69): $mpu_status"
  
  if [[ "$aht10_status" == "FOUND" ]] && [[ "$mpu_status" == "FOUND" ]]; then
    echo "  ✓ Both devices detected! Hardware is properly connected."
    return 0
  else
    echo "  ⚠ Some devices not detected. Check wiring:"
    echo "    - SDA: GPIO 2 (pin 3)"
    echo "    - SCL: GPIO 3 (pin 5)"
    echo "    - GND: Pin 6/9/14/20/25/30/34/39"
    echo "    - 3.3V: Pin 1/17"
    echo "    - Run: i2cdetect -y 1  (with i2c-tools installed)"
    return 2
  fi
}

echo "[0/10] Preparing permissions and I2C interface..."
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

# Check and enable I2C
check_and_enable_i2c || true

echo "[1/10] Detecting I2C hardware devices..."
detect_i2c_devices || true

echo "[2/10] Killing old processes..."
sudo_run killall -9 \
  rosbridge_websocket \
  web_video_server \
  mpu9250_node \
  aht10_node \
  ultrasonic_node \
  monitor_node \
  joystick_node \
  joy_node \
  manual_controller_node \
  stm32_bridge_node \
  camera_detection_node \
  ncnn_detection_node \
  rpicam-vid \
  rpicam-still \
  libcamera-vid \
  libcamera-still \
  2>/dev/null || true

echo "[3/10] Cleaning shared memory and temp camera files..."
sudo_run rm -rf /dev/shm/fastdds* /tmp/robodog_camera 2>/dev/null || true
mkdir -p "$LOG_DIR"

# ROS environment
cd "$WORKSPACE"

# ROS setup scripts may read optional vars that are unset; source them with nounset disabled.
set +u
source /opt/ros/kilted/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

echo "[4/10] Starting main launch stack..."
nohup ros2 launch robodog_bringup web_interface.launch.py > "$LOG_DIR/web_interface.log" 2>&1 &
sleep 4

echo "[5/10] Ensuring rosbridge is running on port 9090..."
if is_port_open 9090; then
  echo "  - Port 9090 already open"
else
  nohup ros2 run rosbridge_server rosbridge_websocket \
    --ros-args -p port:=9090 -p address:=0.0.0.0 \
    > "$LOG_DIR/rosbridge.log" 2>&1 &
  sleep 2
fi

echo "[6/10] Ensuring web video server is running on port 8080..."
if is_port_open 8080; then
  echo "  - Port 8080 already open"
else
  nohup ros2 run web_video_server web_video_server \
    --ros-args -p port:=8080 -p address:=0.0.0.0 \
    > "$LOG_DIR/web_video_server.log" 2>&1 &
  sleep 2
fi

echo "[7/10] Port status:"
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

echo "[8/10] Sensor topic publisher checks:"
print_topic_status /temperature
print_topic_status /humidity
print_topic_status /temperature2
print_topic_status /humidity2
print_topic_status /imu/data_raw
print_topic_status /imu/data
print_topic_status /imu/mag
print_topic_status /tf
print_topic_status /ultrasonic1
print_topic_status /ultrasonic2
print_topic_status /robodog/interface/status_json

echo "[9/10] I2C device verification:"
detect_i2c_devices || true

echo "[10/10] WebSocket status_json sample check:"
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
        tf_data = payload.get('tf', {}).get('map_to_imu', {})
        print(f"    sensors.temp1_c={sensors.get('temp1_c')} hum1_pct={sensors.get('hum1_pct')}")
        print(f"    imu.accel.x={imu.get('accel', {}).get('x')}")
        print(
          f"    tf.map_to_imu.available={tf_data.get('available')} "
          f"parent={tf_data.get('parent_frame')} child={tf_data.get('child_frame')}"
        )
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
