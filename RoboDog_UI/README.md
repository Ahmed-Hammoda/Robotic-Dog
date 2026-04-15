# SPOT-MICRO Control App

Native desktop app (.exe) built with Electron.
Connects directly to the Raspberry Pi 5 over ROS 2 using rosbridge WebSocket and web_video_server.

## Folder structure

```
spot-micro-app/
├── main.js          ← Electron main process (rosbridge + commands)
├── preload.js       ← Secure IPC bridge
├── package.json     ← Project config + build settings
├── index.html       ← Full UI (all pages)
├── rosbridge_listener.py ← Terminal listener for ROS telemetry
└── assets/
    └── icon.png     ← App icon (add your own 256x256 PNG)
```

## Step 1 — Install Node.js

Download from https://nodejs.org (LTS version, 18+)

## Step 2 — Install dependencies

Open a terminal in the spot-micro-app folder:

```bash
npm install
```

## Step 3 — Run in development mode

```bash
npm start
```

This opens the app window immediately. Test everything here first.

## Step 4 — Build the .exe

```bash
npm run build
```

Output will be in the `dist/` folder:
- `dist/SPOT-MICRO Control Setup 1.0.0.exe`  ← installer
- `dist/win-unpacked/SPOT-MICRO Control.exe`  ← portable

## Step 5 — Run ROS bridge on your RPi 5

Start rosbridge_server and web_video_server on the Pi, then connect the desktop app to the Pi IP, for example `ws://192.168.1.28:9090`.

## Step 6 — Verify telemetry in a terminal

Run the listener script and enter the Pi IP when prompted:

```bash
python rosbridge_listener.py
```

## Step 7 — Connect

1. Make sure the Windows machine can reach the Pi over the local network
2. Open the app
3. Enter the Pi IP in the ROS bridge field
4. Click "CONNECT ROS"
5. The dashboard fills from ROS telemetry JSON

## Notes

- Use Chrome DevTools inside the app if you need renderer debugging
- The app auto-reconnects if rosbridge drops
- Motor sliders send commands on release (onchange) to avoid flooding rosbridge
- PS4 controller works via the Gamepad API — plug in or pair to your PC
