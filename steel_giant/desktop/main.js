// Стальной гигант: прозрачное окно поверх рабочего стола, клики проходят насквозь
const { app, BrowserWindow, screen, Tray, Menu, ipcMain, nativeImage } = require('electron');
const path = require('path');
const fs = require('fs');

if (!app.requestSingleInstanceLock()) app.quit();

let win = null;
let tray = null;
const settings = { size: 'medium', paused: false };
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
    },
  });
  win.setAlwaysOnTop(true, 'screen-saver');
  win.setVisibleOnAllWorkspaces(true);
  win.setIgnoreMouseEvents(true, { forward: true });
  win.loadFile(path.join(__dirname, 'giant.html'));
}

function buildMenu() {
  const set = (key, value) => { settings[key] = value; saveSettings(); pushSettings(); buildMenu(); };
  const items = [
    { label: 'Позвать гиганта', click: () => win && win.webContents.send('summon') },
    { label: 'Бой с дронами', click: () => win && win.webContents.send('battle') },
    { label: 'Погоня', click: () => win && win.webContents.send('chase') },
    { type: 'separator' },
    { label: 'Пауза', type: 'checkbox', checked: settings.paused, click: (i) => set('paused', i.checked) },
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
  ipcMain.on('ready', pushSettings);

  screen.on('display-metrics-changed', fitToScreen);
  screen.on('display-added', fitToScreen);
  screen.on('display-removed', fitToScreen);
});

app.on('second-instance', () => win && win.webContents.send('summon'));
app.on('window-all-closed', () => app.quit());
