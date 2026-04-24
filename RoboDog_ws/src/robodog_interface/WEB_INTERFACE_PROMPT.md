# Web Interface WebSocket & Data Parsing Prompt

## Overview: ROS2 Robot Status Data via WebSocket

Your RPi is sending real-time robot telemetry data through ROS2 topics and your web interface reads it over the rosbridge WebSocket. This prompt explains how to connect, receive, parse, and display all the data.

---

## What Data Are You Receiving?

**One continuous string message every ~1.5 seconds:**

```
joy_cmd[lin=0.00 ang=0.00] | joy_raw[n/a] | sensors[temp1=23.60C hum1=52.29% temp2=23.96C hum2=52.19% u1=2.61cm u2=3.33cm] | imu[accel=x=-0.63 y=0.15 z=9.59m/s2 gyro=x=-0.00 y=-0.09 z=-0.00rad/s mag=x=0.00000 y=0.00011 z=0.00004T temp=27.66C] | robot_mode[stand] | robot_move[stand] | camera[fps=1.3 detections=none] | stream[http://192.168.1.28:8080/stream?topic=/robodog/camera/detections/image]
```

**Format breakdown:**
- Sections separated by ` | ` (pipe)
- Each section: `key[content]`
- Content format varies by section

---

## Data Dictionary

### 1. **joy_cmd** - Joystick Command
- **Source:** Joystick node
- **Format:** `lin=X.XX ang=X.XX` or `n/a`
- **Meaning:**
  - `lin` = Linear velocity (forward/backward, m/s)
  - `ang` = Angular velocity (rotation, rad/s)
- **Example:** `lin=0.50 ang=0.25` = moving forward while turning right

### 2. **joy_raw** - Raw Joystick Input
- **Source:** Joystick hardware
- **Format:** `axes[a,b,c,d] buttons[1,0,0,1,...]` or `n/a`
- **Meaning:** Raw analog stick positions and button states
- **Example:** `axes[0.99,-0.50,0.00,0.10] buttons[1,0,0,1,0,0,0,0]`

### 3. **sensors** - Environmental Sensors
- **Source:** AHT10 temperature/humidity sensor + Ultrasonic distance sensors
- **Format:** `temp1=X.XXC hum1=X.XX% temp2=X.XXC hum2=X.XX% u1=X.XXcm u2=X.XXcm`
- **Breakdown:**
  - `temp1` = Temperature from sensor 1 (°C)
  - `hum1` = Humidity from sensor 1 (%)
  - `temp2` = Temperature from sensor 2 (°C)
  - `hum2` = Humidity from sensor 2 (%)
  - `u1` = Ultrasonic distance sensor 1 (cm)
  - `u2` = Ultrasonic distance sensor 2 (cm)
  - **Note:** `nancm` means sensor is out of range or failed
- **Example:** `temp1=23.60C hum1=52.29% temp2=23.96C hum2=52.19% u1=2.61cm u2=3.33cm`

### 4. **imu** - Inertial Measurement Unit (MPU9250)
- **Source:** 9-DOF IMU sensor
- **Format:** `accel=x=-0.63 y=0.15 z=9.59m/s2 gyro=x=-0.00 y=-0.09 z=-0.00rad/s mag=x=0.00000 y=0.00011 z=0.00004T temp=27.66C`
- **Breakdown:**
  - `accel` = 3-axis acceleration (x, y, z in m/s²)
  - `gyro` = 3-axis rotation rates (x, y, z in rad/s)
  - `mag` = 3-axis magnetic field (x, y, z in Tesla)
  - `temp` = IMU internal temperature (°C)
- **Example:** Flat on table: `accel=x=-0.63 y=0.15 z=9.59m/s2`

### 5. **robot_mode** - Control Mode
- **Source:** Robot status monitor
- **Format:** Single word mode name
- **Possible values:** `stand`, `move_forward`, `move_backward`, `rotate_angle_left`, `rotate_angle_right`, `walk`, `turn`
- **Meaning:** Current control mode / what the robot is trying to do
- **Example:** `stand` = robot is idle and standing

### 6. **robot_move** - Movement State
- **Source:** Motor controller
- **Format:** Single word movement state
- **Possible values:** `stand`, `move_forward`, `move_backward`, `rotate_angle_left`, `rotate_angle_right`
- **Meaning:** Actual movement state (what the robot is currently doing)
- **Example:** `stand` = robot is not moving
- **Note:** Can differ from `robot_mode` during transitions

### 7. **camera** - Camera Feed Status
- **Source:** Vision node
- **Format:** `fps=X.X detections=WORD`
- **Breakdown:**
  - `fps` = Camera frame rate (frames/sec)
  - `detections` = Detection result
    - `none` = no objects detected
    - `person`, `dog`, `cat`, etc. = detected object type
    - `multiple` = multiple objects detected
- **Example:** `fps=1.3 detections=none` = running at 1.3 FPS with no detections

### 8. **stream** - Camera Stream URL
- **Source:** Vision node
- **Format:** Full HTTP URL
- **Meaning:** MJPEG stream URL for live camera feed
- **Use in HTML:** `<img src="URL">` or `<video src="URL">`
- **Example:** `http://192.168.1.28:8080/stream?topic=/robodog/camera/detections/image`

---

## Implementation Steps

### Step 1: Connect via WebSocket (rosbridge)

```javascript
let ros = null;

function connectWebSocket() {
    ros = new ROSLIB.Ros({
        url: 'ws://localhost:9090'
    });

    ros.on('connection', () => {
        console.log('Connected to rosbridge WebSocket');
        document.getElementById('status').textContent = '✓ Connected';
    });

    ros.on('error', (error) => {
        console.error('WebSocket connection failed:', error);
        document.getElementById('status').textContent = '✗ Connection Failed';
    });

    ros.on('close', () => {
        document.getElementById('status').textContent = 'Disconnected';
    });

    const statusTopic = new ROSLIB.Topic({
        ros,
        name: '/robodog/interface/status_json',
        messageType: 'std_msgs/msg/String'
    });

    statusTopic.subscribe(onRobotDataReceived);
}

function disconnectWebSocket() {
    if (ros) {
        ros.close();
        ros = null;
    }
}

document.getElementById('connectBtn').addEventListener('click', connectWebSocket);
document.getElementById('disconnectBtn').addEventListener('click', disconnectWebSocket);
```

---

### Step 2: Parse Incoming Data

```javascript
let robotData = {
    joy_cmd: { linear_x: 0.0, angular_z: 0.0 },
    joy_raw: { axes: [], buttons: [] },
    sensors: {
        temp1_c: null, hum1_pct: null,
        temp2_c: null, hum2_pct: null,
        ultrasonic1_cm: null, ultrasonic2_cm: null
    },
    imu: {
        accel: { x: 0.0, y: 0.0, z: 0.0 },
        gyro: { x: 0.0, y: 0.0, z: 0.0 },
        mag: { x: 0.0, y: 0.0, z: 0.0 },
        temp_c: null
    },
    robot: { mode: 'stand', move: 'stand' },
    camera: { fps: 0.0, detections: 'none', stream_url: '' },
    system: { bridge: 'rosbridge', battery_pct: null, uptime_s: 0 }
};

function onRobotDataReceived(event) {
    const statusString = event.data;
    console.log('Received:', statusString);
    robotData = JSON.parse(statusString);
    updateUI(robotData);
}
```

---

### Step 3: Update UI with Data

```javascript
function updateUI(data) {
    const n = (value, digits) => Number(value ?? 0).toFixed(digits);

    document.getElementById('joyLinear').textContent = n(data.joy_cmd.linear_x, 2);
    document.getElementById('joyAngular').textContent = n(data.joy_cmd.angular_z, 2);

    document.getElementById('robotMode').textContent = data.robot.mode ?? 'n/a';
    document.getElementById('robotMove').textContent = data.robot.move ?? 'n/a';

    document.getElementById('temp1').textContent = data.sensors.temp1_c ?? 'n/a';
    document.getElementById('hum1').textContent = data.sensors.hum1_pct ?? 'n/a';
    document.getElementById('temp2').textContent = data.sensors.temp2_c ?? 'n/a';
    document.getElementById('hum2').textContent = data.sensors.hum2_pct ?? 'n/a';
    document.getElementById('u1').textContent = data.sensors.ultrasonic1_cm ?? 'n/a';
    document.getElementById('u2').textContent = data.sensors.ultrasonic2_cm ?? 'n/a';

    document.getElementById('accelX').textContent = n(data.imu.accel.x, 2);
    document.getElementById('accelY').textContent = n(data.imu.accel.y, 2);
    document.getElementById('accelZ').textContent = n(data.imu.accel.z, 2);

    document.getElementById('gyroX').textContent = n(data.imu.gyro.x, 2);
    document.getElementById('gyroY').textContent = n(data.imu.gyro.y, 2);
    document.getElementById('gyroZ').textContent = n(data.imu.gyro.z, 2);

    document.getElementById('magX').textContent = n(data.imu.mag.x, 5);
    document.getElementById('magY').textContent = n(data.imu.mag.y, 5);
    document.getElementById('magZ').textContent = n(data.imu.mag.z, 5);

    document.getElementById('imuTemp').textContent = data.imu.temp_c ?? 'n/a';

    document.getElementById('fps').textContent = n(data.camera.fps, 1);
    document.getElementById('detections').textContent = data.camera.detections ?? 'none';

    if (data.camera.stream_url) {
        document.getElementById('cameraStream').src = data.camera.stream_url;
    }
}
```

---

### Step 4: HTML Structure

```html
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width">
    <title>RoboDog Interface</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .container { max-width: 1200px; margin: 0 auto; }
        .section { 
            border: 1px solid #ccc; 
            border-radius: 8px; 
            padding: 15px; 
            margin: 15px 0;
            background: #f9f9f9;
        }
        .grid { display: grid; grid-template-columns: 1fr 1fr; gap: 15px; }
        .widget { background: white; padding: 10px; border-radius: 4px; border-left: 4px solid #007bff; }
        .label { color: #666; font-size: 0.9em; }
        .value { font-size: 1.3em; font-weight: bold; color: #007bff; }
        button { padding: 10px 20px; margin: 5px; cursor: pointer; }
        img { max-width: 100%; height: auto; border-radius: 4px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>RoboDog Live Interface</h1>
        
        <!-- Connection Status -->
        <div class="section">
            <button id="connectBtn">Connect WebSocket</button>
            <button id="disconnectBtn">Disconnect</button>
            <span id="status" style="margin-left: 20px; font-weight: bold;">Not Connected</span>
        </div>
        
        <!-- Joystick Info -->
        <div class="section">
            <h2>Joystick Input</h2>
            <div class="grid">
                <div class="widget">
                    <div class="label">Linear Velocity (m/s)</div>
                    <div class="value" id="joyLinear">0.00</div>
                </div>
                <div class="widget">
                    <div class="label">Angular Velocity (rad/s)</div>
                    <div class="value" id="joyAngular">0.00</div>
                </div>
            </div>
        </div>
        
        <!-- Robot Status -->
        <div class="section">
            <h2>Robot Status</h2>
            <div class="grid">
                <div class="widget">
                    <div class="label">Control Mode</div>
                    <div class="value" id="robotMode">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Movement State</div>
                    <div class="value" id="robotMove">n/a</div>
                </div>
            </div>
        </div>
        
        <!-- Environmental Sensors -->
        <div class="section">
            <h2>Environmental Sensors</h2>
            <div class="grid">
                <div class="widget">
                    <div class="label">Temperature 1</div>
                    <div class="value" id="temp1">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Humidity 1</div>
                    <div class="value" id="hum1">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Temperature 2</div>
                    <div class="value" id="temp2">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Humidity 2</div>
                    <div class="value" id="hum2">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Distance Sensor 1</div>
                    <div class="value" id="u1">n/a</div>
                </div>
                <div class="widget">
                    <div class="label">Distance Sensor 2</div>
                    <div class="value" id="u2">n/a</div>
                </div>
            </div>
        </div>
        
        <!-- IMU Data -->
        <div class="section">
            <h2>Inertial Measurement Unit (IMU)</h2>
            
            <h3>Acceleration (m/s²)</h3>
            <div class="grid">
                <div class="widget">
                    <div class="label">X-axis</div>
                    <div class="value" id="accelX">0.00</div>
                </div>
                <div class="widget">
                    <div class="label">Y-axis</div>
                    <div class="value" id="accelY">0.00</div>
                </div>
                <div class="widget">
                    <div class="label">Z-axis</div>
                    <div class="value" id="accelZ">0.00</div>
                </div>
            </div>
            
            <h3>Gyroscope (rad/s)</h3>
            <div class="grid">
                <div class="widget">
                    <div class="label">X-axis</div>
                    <div class="value" id="gyroX">0.00</div>
                </div>
                <div class="widget">
                    <div class="label">Y-axis</div>
                    <div class="value" id="gyroY">0.00</div>
                </div>
                <div class="widget">
                    <div class="label">Z-axis</div>
                    <div class="value" id="gyroZ">0.00</div>
                </div>
            </div>
            
            <h3>Magnetometer (Tesla)</h3>
            <div class="grid">
                <div class="widget">
                    <div class="label">X-axis</div>
                    <div class="value" id="magX">0.00000</div>
                </div>
                <div class="widget">
                    <div class="label">Y-axis</div>
                    <div class="value" id="magY">0.00000</div>
                </div>
                <div class="widget">
                    <div class="label">Z-axis</div>
                    <div class="value" id="magZ">0.00000</div>
                </div>
            </div>
            
            <h3>IMU Temperature</h3>
            <div class="widget">
                <div class="label">Temperature (°C)</div>
                <div class="value" id="imuTemp">0.00</div>
            </div>
        </div>
        
        <!-- Camera Feed -->
        <div class="section">
            <h2>Camera Feed</h2>
            <div class="grid">
                <div class="widget">
                    <div class="label">Frame Rate</div>
                    <div class="value" id="fps">0.0</div>
                </div>
                <div class="widget">
                    <div class="label">Detections</div>
                    <div class="value" id="detections">none</div>
                </div>
            </div>
            <img id="cameraStream" src="" alt="Camera Stream" style="margin-top: 15px;">
        </div>
    </div>
    
    <script src="interface.js"></script>
</body>
</html>
```

---

## Summary: What Your Interface Should Do

1. ✅ **Connect** to rosbridge over WebSocket
2. ✅ **Receive** JSON status messages every ~1.5 seconds
3. ✅ **Parse** the JSON into structured data
4. ✅ **Read** the joy_cmd, joy_raw, sensors, imu, robot, camera, and system fields
5. ✅ **Display** all sensor values in real-time
6. ✅ **Stream** camera feed from the MJPEG URL
7. ✅ **Update** UI automatically on new data

---

## Debugging Tips

**Check WebSocket connection:**
```javascript
console.log('ROS bridge connected');
console.log('Raw status string:', statusString);
console.log('Parsed data:', robotData);
```

**If parsing fails:**
- Print the raw string to see format
- Check regex patterns match your data
- Open browser DevTools → Console to see errors

**If camera doesn't show:**
- Verify stream URL is reachable: `curl http://localhost:8080/stream?topic=/robodog/camera/detections/image`
- Check if MJPEG stream is running on RPi

---

## Data Flow Diagram

```
ROS2 (RPi)                       WebSocket / rosbridge           Web Browser
─────────────────────────────────────────────────────────────────────────

aht10_node                                                        
├─ Temperature         ┐                                          
├─ Humidity           │                                          
mpu9250_node          │                                          
├─ IMU Accel/Gyro/Mag │                                          
├─ Temperature        │                → monitor_node ──────────→ rosbridge_websocket ────→ WebSocket client
joystick_node         │                   (collects &  every         (publishes)            (subscribes)
├─ Joy cmd            │                    formats)  1.5 sec                    ↓
├─ Joy raw            │                                         JSON.parse()
ultrasonic_node       │                                         robotData object
├─ Distance           │                                              ↓
camera_node           │                                         updateUI()
└─ Stream URL    ─────┘                                              ↓
                                                              HTML Display Updated
```

---

Use this as your complete reference for building the web interface!
