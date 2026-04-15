/**
 * SPOT-MICRO — Preload Script
 * Secure bridge exposing only specific IPC channels to the renderer.
 */
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('spotAPI', {
  // Window controls
  minimize: () => ipcRenderer.send('win-minimize'),
  maximize: () => ipcRenderer.send('win-maximize'),
  close:    () => ipcRenderer.send('win-close'),

  // ROS bridge
  connectRos: (url) => ipcRenderer.send('ros-connect', { url }),
  disconnectRos: () => ipcRenderer.send('ros-disconnect'),
  sendCmdVel: (payload) => ipcRenderer.send('ros-send-cmdvel', payload),

  // Listen for events from main process
  on: (channel, fn) => {
    const allowed = [
      'ros-connection-state',
      'ros-state',
    ];
    if (allowed.includes(channel)) ipcRenderer.on(channel, (_, data) => fn(data));
  },

  off: (channel) => ipcRenderer.removeAllListeners(channel),
});
