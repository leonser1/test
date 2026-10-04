const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('giant', {
  setIgnore: (value) => ipcRenderer.send('ignore', value),
  sendIcon: (dataUrl) => ipcRenderer.send('icon', dataUrl),
  ready: () => ipcRenderer.send('ready'),
  onSettings: (cb) => ipcRenderer.on('settings', (_e, s) => cb(s)),
  onSummon: (cb) => ipcRenderer.on('summon', () => cb()),
  onBattle: (cb) => ipcRenderer.on('battle', () => cb()),
  onChase: (cb) => ipcRenderer.on('chase', () => cb()),
  onBoy: (cb) => ipcRenderer.on('boy', () => cb()),
  // данные о ПК: {windows, cursor, idle, fullscreenApp}, приходят частями
  onPC: (cb) => ipcRenderer.on('pc', (_e, p) => cb(p)),
});
