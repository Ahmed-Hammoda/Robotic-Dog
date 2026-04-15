/**
 * SPOT-MICRO — Electron Main Process
 * ROS2 dashboard transport via rosbridge WebSocket.
 */

const { app, BrowserWindow, ipcMain, Menu } = require('electron');
const path = require('path');
const ROSLIB = require('roslib');

const DEFAULT_ROSBRIDGE_URL = 'ws://127.0.0.1:9090';
const STATUS_TOPIC = '/robodog/interface/status_json';
const STATUS_TOPIC_TYPE = 'std_msgs/msg/String';
const CMD_VEL_TOPIC = '/robodog/interface/cmd_vel';
const CMD_VEL_TOPIC_TYPE = 'geometry_msgs/Twist';

let win = null;
let ros = null;
let statusTopic = null;
let cmdVelTopic = null;
let rosbridgeUrl = DEFAULT_ROSBRIDGE_URL;

function createWindow() {
  win = new BrowserWindow({
    width: 1280,
    height: 820,
    minWidth: 900,
    minHeight: 600,
    backgroundColor: '#070a0f',
    titleBarStyle: 'hidden',
    frame: false,
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      contextIsolation: true,
      nodeIntegration: false,
    },
    icon: path.join(__dirname, 'assets', 'icon.png'),
  });

  // index.html is at repository root, next to main.js.
  win.loadFile(path.join(__dirname, 'index.html'));
  Menu.setApplicationMenu(null);
  win.on('closed', () => { win = null; });
}

app.whenReady().then(() => {
  createWindow();
  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on('window-all-closed', () => {
  disconnectRosbridge();
  if (process.platform !== 'darwin') app.quit();
});

ipcMain.on('win-minimize', () => win?.minimize());
ipcMain.on('win-maximize', () => {
  if (win?.isMaximized()) win.unmaximize(); else win?.maximize();
});
ipcMain.on('win-close', () => win?.close());

ipcMain.on('ros-connect', (_event, payload) => {
  const nextUrl = String(payload?.url || '').trim() || DEFAULT_ROSBRIDGE_URL;
  connectRosbridge(nextUrl);
});

ipcMain.on('ros-disconnect', () => {
  disconnectRosbridge();
  sendConnectionState('disconnected', 'Disconnected by user');
});

ipcMain.on('ros-send-cmdvel', (_event, payload) => {
  publishCmdVel(payload || {});
});

function sendToRenderer(channel, data) {
  if (win && !win.isDestroyed()) win.webContents.send(channel, data);
}

function sendConnectionState(state, message = '') {
  sendToRenderer('ros-connection-state', {
    state,
    url: rosbridgeUrl,
    message,
  });
}

function connectRosbridge(url) {
  disconnectRosbridge();
  rosbridgeUrl = url;
  sendConnectionState('connecting', `Connecting to ${url}`);

  ros = new ROSLIB.Ros({ url });

  ros.on('connection', () => {
    sendConnectionState('connected', `Connected to ${url}`);

    statusTopic = new ROSLIB.Topic({
      ros,
      name: STATUS_TOPIC,
      messageType: STATUS_TOPIC_TYPE,
    });

    statusTopic.subscribe((message) => {
      try {
        const raw = message?.data;
        const parsed = typeof raw === 'string' ? JSON.parse(raw) : raw;
        sendToRenderer('ros-state', parsed || {});
      } catch (error) {
        sendConnectionState('error', `Telemetry parse error: ${error.message}`);
      }
    });

    cmdVelTopic = new ROSLIB.Topic({
      ros,
      name: CMD_VEL_TOPIC,
      messageType: CMD_VEL_TOPIC_TYPE,
    });
  });

  ros.on('error', (error) => {
    sendConnectionState('error', error?.message || 'rosbridge error');
  });

  ros.on('close', () => {
    cleanupRosTopics();
    sendConnectionState('disconnected', 'rosbridge connection closed');
  });
}

function cleanupRosTopics() {
  if (statusTopic) {
    try { statusTopic.unsubscribe(); } catch (_) {}
    statusTopic = null;
  }
  cmdVelTopic = null;
}

function disconnectRosbridge() {
  cleanupRosTopics();
  if (ros) {
    try { ros.close(); } catch (_) {}
    ros = null;
  }
}

function publishCmdVel(payload) {
  if (!cmdVelTopic) return;
  const linearX = Number(payload?.linear_x ?? 0);
  const angularZ = Number(payload?.angular_z ?? 0);
  const msg = new ROSLIB.Message({
    linear: { x: Number.isFinite(linearX) ? linearX : 0, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: Number.isFinite(angularZ) ? angularZ : 0 },
  });
  try {
    cmdVelTopic.publish(msg);
  } catch (error) {
    sendConnectionState('error', `cmd_vel publish failed: ${error.message}`);
  }
}
