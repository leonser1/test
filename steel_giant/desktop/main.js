// Стальной гигант: прозрачное окно поверх рабочего стола, клики проходят насквозь
const { app, BrowserWindow, screen, Tray, Menu, ipcMain, nativeImage, powerMonitor } = require('electron');
const path = require('path');
const fs = require('fs');
const { createPcWatcher, convertWindows } = require('./pcwatch');

if (!app.requestSingleInstanceLock()) app.quit();

let win = null;
let tray = null;
const settings = { size: 'medium', paused: false, sound: true, pcWindows: true };
const settingsFile = () => path.join(app.getPath('userData'), 'settings.json');

function loadSettings() {
  try { Object.assign(settings, JSON.parse(fs.readFileSync(settingsFile(), 'utf8'))); } catch {}
}
function saveSettings() {
  try { fs.writeFileSync(settingsFile(), JSON.stringify(settings, null, 2)); } catch {}
}
function pushSettings() {
  if (win) win.webContents.send('settings', settings);
}
function fitToScreen() {
  if (win) win.setBounds(screen.getPrimaryDisplay().workArea);
}

/* ---------- связь с ПК: окна, курсор, простой ---------- */
// Читаются только прямоугольники окон, положение курсора и время простоя.
// Заголовки окон, их содержимое и нажатые клавиши не читаются и никуда не уходят.
const pc = { windows: [], cursor: { x: -1e4, y: -1e4, down: false }, idle: 0, fullscreenApp: false };
let pcLastWin = '', pcRaw = null, pcWatcher = null;
const overlayBounds = () => (win && !win.isDestroyed() ? win.getContentBounds() : screen.getPrimaryDisplay().workArea);
function pcSend(part) {
  if (win && !win.isDestroyed()) win.webContents.send('pc', part);
}
function pcApplyRaw(raw) {
  pcRaw = raw;
  const toDip = typeof screen.screenToDipRect === 'function' ? (r) => screen.screenToDipRect(null, r) : null;
  const windows = settings.pcWindows ? convertWindows(raw, { physical: !!raw.pa, toDip, bounds: overlayBounds() }) : [];
  const fullscreenApp = settings.pcWindows && !!raw.fs;
  const key = JSON.stringify(windows) + fullscreenApp;
  if (key === pcLastWin) return;
  pcLastWin = key;
  pc.windows = windows; pc.fullscreenApp = fullscreenApp;
  pcSend({ windows, fullscreenApp });
}
function pcExclusions() {
  const pids = new Set([process.pid]);
  try { for (const m of app.getAppMetrics()) pids.add(m.pid); } catch {}
  try { if (win) pids.add(win.webContents.getOSProcessId()); } catch {}
  const hwnds = [];
  try {
    const b = win.getNativeWindowHandle();
    hwnds.push((b.length >= 8 ? b.readBigUInt64LE(0) : BigInt(b.readUInt32LE(0))).toString());
  } catch {}
  return { pids: [...pids], hwnds };
}
function pcUpdateWatcher() {
  if (!pcWatcher) pcWatcher = createPcWatcher({ onData: pcApplyRaw, getExclusions: pcExclusions, log: (m) => console.log(m) });
  if (settings.pcWindows) pcWatcher.start();
  else { pcWatcher.stop(); pcApplyRaw({ pa: 0, fs: 0, w: [] }); }
}
function pcStartPolling() {
  setInterval(() => {
    if (!win || win.isDestroyed()) return;
    const p = screen.getCursorScreenPoint(), b = overlayBounds();
    const x = p.x - b.x, y = p.y - b.y;
    if (x === pc.cursor.x && y === pc.cursor.y) return;
    pc.cursor = { x, y, down: false };
    pcSend({ cursor: pc.cursor });
  }, 33);
  setInterval(() => {
    let idle = 0;
    try { idle = powerMonitor.getSystemIdleTime(); } catch {}
    if (idle === pc.idle) return;
    pc.idle = idle;
    pcSend({ idle });
  }, 1000);
}

function createWindow() {
  const wa = screen.getPrimaryDisplay().workArea;
  win = new BrowserWindow({
    x: wa.x, y: wa.y, width: wa.width, height: wa.height,
    transparent: true,
    frame: false,
    resizable: false,
    movable: false,
    minimizable: false,
    maximizable: false,
    fullscreenable: false,
    skipTaskbar: true,
    hasShadow: false,
    focusable: false,
    alwaysOnTop: true,
    backgroundColor: '#00000000',
    webPreferences: {
      preload: path.join(__dirname, 'preload.js'),
      backgroundThrottling: false,
      autoplayPolicy: 'no-user-gesture-required',
    },
  });
  win.setAlwaysOnTop(true, 'screen-saver');
  win.setVisibleOnAllWorkspaces(true);
  win.setIgnoreMouseEvents(true, { forward: true });
  win.loadFile(path.join(__dirname, 'giant.html'));
}

function buildMenu() {
  const set = (key, value) => { settings[key] = value; saveSettings(); pushSettings(); buildMenu(); if (key === 'pcWindows') pcUpdateWatcher(); };
  const items = [
    { label: 'Позвать гиганта', click: () => win && win.webContents.send('summon') },
    { label: 'Бой с дронами', click: () => win && win.webContents.send('battle') },
    { label: 'Погоня', click: () => win && win.webContents.send('chase') },
    { label: 'Мальчик', click: () => win && win.webContents.send('boy') },
    { type: 'separator' },
    { label: 'Пауза', type: 'checkbox', checked: settings.paused, click: (i) => set('paused', i.checked) },
    { label: 'Звук', type: 'checkbox', checked: settings.sound, click: (i) => set('sound', i.checked) },
    { label: 'Взаимодействие с окнами', type: 'checkbox', checked: settings.pcWindows, click: (i) => set('pcWindows', i.checked) },
    {
      label: 'Размер',
      submenu: [['small', 'Маленький'], ['medium', 'Средний'], ['large', 'Большой']].map(([key, label]) => ({
        label, type: 'radio', checked: settings.size === key, click: () => set('size', key),
      })),
    },
  ];
  if (app.isPackaged) {
    items.push({
      label: 'Запускать вместе с Windows', type: 'checkbox',
      checked: app.getLoginItemSettings().openAtLogin,
      click: (i) => { app.setLoginItemSettings({ openAtLogin: i.checked }); buildMenu(); },
    });
  }
  items.push({ type: 'separator' }, { label: 'Выход', click: () => app.quit() });
  tray.setContextMenu(Menu.buildFromTemplate(items));
}

app.whenReady().then(() => {
  loadSettings();
  tray = new Tray(nativeImage.createEmpty());
  tray.setToolTip('Стальной гигант');
  buildMenu();
  createWindow();

  ipcMain.on('ignore', (_e, value) => win && win.setIgnoreMouseEvents(!!value, { forward: true }));
  ipcMain.on('icon', (_e, dataUrl) => {
    const img = nativeImage.createFromDataURL(dataUrl);
    if (!img.isEmpty()) tray.setImage(img.resize({ width: 16, height: 16 }));
  });
  ipcMain.on('ready', () => { pushSettings(); pcSend({ ...pc }); });

  screen.on('display-metrics-changed', fitToScreen);
  screen.on('display-added', fitToScreen);
  screen.on('display-removed', fitToScreen);

  pcStartPolling();
  win.webContents.once('did-finish-load', pcUpdateWatcher);
  win.on('resize', () => { if (pcRaw) { pcLastWin = ''; pcApplyRaw(pcRaw); } });
  win.on('move', () => { if (pcRaw) { pcLastWin = ''; pcApplyRaw(pcRaw); } });
});

app.on('will-quit', () => { if (pcWatcher) pcWatcher.stop(); });

app.on('second-instance', () => win && win.webContents.send('summon'));
app.on('window-all-closed', () => app.quit());
