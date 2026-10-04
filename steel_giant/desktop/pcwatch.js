// Слежение за окнами Windows без нативных модулей: один скрытый процесс PowerShell
// с C#-кодом (user32/dwmapi) раз в ~700 мс печатает в stdout одну строку JSON:
//   {"pa":1,"fs":0,"w":[[hwnd,x,y,w,h,zoomed],...]}
// pa — процесс DPI-aware (координаты в физических пикселях), fs — активное окно
// во весь экран, w — окна сверху вниз по Z-порядку. Заголовки окон НЕ читаются.
'use strict';
const { spawn } = require('child_process');

const CS = String.raw`
using System;
using System.Text;
using System.Diagnostics;
using System.Collections.Generic;
using System.Runtime.InteropServices;
public static class GiantPC {
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L, T, R, B; }
  [StructLayout(LayoutKind.Sequential)] public struct MONITORINFO { public int cbSize; public RECT rcMonitor; public RECT rcWork; public uint dwFlags; }
  public delegate bool EnumProc(IntPtr h, IntPtr l);
  [DllImport("user32.dll")] static extern bool EnumWindows(EnumProc cb, IntPtr l);
  [DllImport("user32.dll")] static extern bool IsWindowVisible(IntPtr h);
  [DllImport("user32.dll")] static extern bool IsIconic(IntPtr h);
  [DllImport("user32.dll")] static extern bool IsZoomed(IntPtr h);
  [DllImport("user32.dll", EntryPoint = "GetWindowLongW")] static extern int GetWindowLong(IntPtr h, int i);
  [DllImport("user32.dll")] static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll", CharSet = CharSet.Unicode)] static extern int GetClassNameW(IntPtr h, StringBuilder sb, int n);
  [DllImport("user32.dll")] static extern uint GetWindowThreadProcessId(IntPtr h, out uint pid);
  [DllImport("user32.dll")] static extern IntPtr GetForegroundWindow();
  [DllImport("user32.dll")] static extern IntPtr MonitorFromWindow(IntPtr h, uint flags);
  [DllImport("user32.dll")] static extern bool GetMonitorInfoW(IntPtr m, ref MONITORINFO mi);
  [DllImport("user32.dll")] static extern bool SetProcessDpiAwarenessContext(IntPtr v);
  [DllImport("user32.dll")] static extern bool SetProcessDPIAware();
  [DllImport("dwmapi.dll")] static extern int DwmGetWindowAttribute(IntPtr h, int a, out RECT r, int n);
  [DllImport("dwmapi.dll")] static extern int DwmGetWindowAttribute(IntPtr h, int a, out int v, int n);

  const int GWL_EXSTYLE = -20;
  const int WS_EX_TOOLWINDOW = 0x80, WS_EX_TRANSPARENT = 0x20, WS_EX_APPWINDOW = 0x40000, WS_EX_NOACTIVATE = 0x08000000;
  const int DWMWA_EXTENDED_FRAME_BOUNDS = 9, DWMWA_CLOAKED = 14;
  static readonly string[] SkipClass = { "Progman", "WorkerW", "Shell_TrayWnd", "Shell_SecondaryTrayWnd",
    "Windows.UI.Core.CoreWindow", "#32768", "#32769", "NotifyIconOverflowWindow", "TopLevelWindowForOverflowXamlIsland",
    "Xaml_WindowedPopupClass", "tooltips_class32", "SysShadow" };

  static HashSet<uint> skipPid = new HashSet<uint>();
  static HashSet<long> skipHwnd = new HashSet<long>();
  static StringBuilder outSb = new StringBuilder(4096);
  static StringBuilder cls = new StringBuilder(256);
  static int count;
  static EnumProc proc = new EnumProc(OnWindow); // держим ссылку, чтобы GC не собрал делегат

  public static int Aware() {
    try { if (SetProcessDpiAwarenessContext(new IntPtr(-4))) return 1; } catch { }
    try { if (SetProcessDpiAwarenessContext(new IntPtr(-3))) return 1; } catch { }
    try { if (SetProcessDPIAware()) return 1; } catch { }
    return 0;
  }

  static string ClassOf(IntPtr h) {
    cls.Length = 0;
    return GetClassNameW(h, cls, cls.Capacity) > 0 ? cls.ToString() : "";
  }

  static bool Bounds(IntPtr h, out RECT r) {
    try { if (DwmGetWindowAttribute(h, DWMWA_EXTENDED_FRAME_BOUNDS, out r, Marshal.SizeOf(typeof(RECT))) == 0 && r.R > r.L) return true; } catch { }
    return GetWindowRect(h, out r);
  }

  static bool OnWindow(IntPtr h, IntPtr l) {
    try {
      if (count >= 64) return false;
      if (!IsWindowVisible(h) || IsIconic(h)) return true;
      if (skipHwnd.Contains(h.ToInt64())) return true;
      int ex = GetWindowLong(h, GWL_EXSTYLE);
      if ((ex & WS_EX_TOOLWINDOW) != 0 || (ex & WS_EX_TRANSPARENT) != 0) return true;
      if ((ex & WS_EX_NOACTIVATE) != 0 && (ex & WS_EX_APPWINDOW) == 0) return true;
      int cloaked = 0;
      try { if (DwmGetWindowAttribute(h, DWMWA_CLOAKED, out cloaked, 4) != 0) cloaked = 0; } catch { cloaked = 0; }
      if (cloaked != 0) return true;
      string c = ClassOf(h);
      if (Array.IndexOf(SkipClass, c) >= 0) return true;
      uint pid; GetWindowThreadProcessId(h, out pid);
      if (skipPid.Contains(pid)) return true;
      RECT r; if (!Bounds(h, out r)) return true;
      int w = r.R - r.L, hh = r.B - r.T;
      if (w < 40 || hh < 20) return true;
      if (count > 0) outSb.Append(',');
      outSb.Append('[').Append(h.ToInt64()).Append(',').Append(r.L).Append(',').Append(r.T).Append(',')
           .Append(w).Append(',').Append(hh).Append(',').Append(IsZoomed(h) ? 1 : 0).Append(']');
      count++;
    } catch { }
    return true;
  }

  static int Fullscreen() {
    try {
      IntPtr f = GetForegroundWindow();
      if (f == IntPtr.Zero || skipHwnd.Contains(f.ToInt64())) return 0;
      string c = ClassOf(f);
      if (c == "Progman" || c == "WorkerW" || c == "Shell_TrayWnd") return 0;
      uint pid; GetWindowThreadProcessId(f, out pid);
      if (skipPid.Contains(pid)) return 0;
      RECT r; if (!GetWindowRect(f, out r)) return 0;
      MONITORINFO mi = new MONITORINFO(); mi.cbSize = Marshal.SizeOf(typeof(MONITORINFO));
      if (!GetMonitorInfoW(MonitorFromWindow(f, 2), ref mi)) return 0;
      return (r.L <= mi.rcMonitor.L && r.T <= mi.rcMonitor.T && r.R >= mi.rcMonitor.R && r.B >= mi.rcMonitor.B) ? 1 : 0;
    } catch { return 0; }
  }

  public static string Tick(int aware) {
    outSb.Length = 0; count = 0;
    string w;
    try { EnumWindows(proc, IntPtr.Zero); w = outSb.ToString(); } catch { w = ""; }
    return "{\"pa\":" + aware + ",\"fs\":" + Fullscreen() + ",\"w\":[" + w + "]}";
  }

  public static void Run(string pids, string hwnds, int parent, int ms) {
    foreach (string s in pids.Split(',')) { uint v; if (uint.TryParse(s, out v)) skipPid.Add(v); }
    foreach (string s in hwnds.Split(',')) { long v; if (long.TryParse(s, out v)) skipHwnd.Add(v); }
    skipPid.Add((uint)Process.GetCurrentProcess().Id);
    int aware = Aware();
    Process par = null;
    try { par = Process.GetProcessById(parent); } catch { return; }
    string last = null; int same = 0;
    while (true) {
      try { if (par.HasExited) return; } catch { return; }
      string line;
      try { line = Tick(aware); } catch { line = "{\"pa\":" + aware + ",\"fs\":0,\"w\":[]}"; }
      // печатаем, только если что-то изменилось (и раз в ~10 с как «пульс»)
      if (line != last || ++same >= 14) {
        try { Console.Out.WriteLine(line); Console.Out.Flush(); } catch { return; }
        last = line; same = 0;
      }
      System.Threading.Thread.Sleep(ms);
    }
  }
}
`;

function buildPsScript({ pids = [], hwnds = [], parent = process.pid, interval = 700 } = {}) {
  const num = (a) => a.map((v) => String(v).replace(/[^0-9]/g, '')).filter(Boolean).join(',');
  return [
    "$ErrorActionPreference = 'Stop'",
    'try {',
    "  Add-Type -TypeDefinition @'",
    CS.trim(),
    "'@",
    `  [GiantPC]::Run('${num(pids)}', '${num(hwnds)}', ${parent | 0}, ${interval | 0})`,
    '} catch {',
    "  [Console]::Out.WriteLine('{\"pa\":0,\"fs\":0,\"w\":[],\"err\":1}')",
    '  [Console]::Error.WriteLine($_.Exception.Message)',
    '  exit 3',
    '}',
  ].join('\r\n');
}

// Разбор одной строки из stdout. Возвращает null для мусора.
function parseLine(line) {
  line = String(line).trim();
  if (!line || line[0] !== '{') return null;
  try {
    const o = JSON.parse(line);
    if (!o || !Array.isArray(o.w)) return null;
    return o;
  } catch { return null; }
}

// Сырые прямоугольники (экранные пиксели) -> CSS px относительно оверлея.
// bounds — границы оверлея (DIP), toDip — screen.screenToDipRect или null.
function convertWindows(raw, { physical, toDip, bounds }) {
  const out = [];
  if (!raw || !Array.isArray(raw.w) || !bounds) return out;
  const BW = bounds.width, BH = bounds.height;
  for (const e of raw.w) {
    if (!Array.isArray(e) || e.length < 5) continue;
    let r = { x: +e[1], y: +e[2], width: +e[3], height: +e[4] };
    if (![r.x, r.y, r.width, r.height].every(Number.isFinite)) continue;
    if (physical && toDip) {
      try { r = toDip(r); } catch { continue; }
    }
    if (r.width < 80 || r.height < 40) continue;
    const x0 = Math.max(0, r.x - bounds.x), y0 = Math.max(0, r.y - bounds.y);
    const x1 = Math.min(BW, r.x - bounds.x + r.width), y1 = Math.min(BH, r.y - bounds.y + r.height);
    const w = Math.round(x1 - x0), h = Math.round(y1 - y0);
    if (w < 20 || h < 10) continue; // вне оверлея или торчит тонкой полоской
    const max = !!e[5] || (w >= BW * 0.9 && h >= BH * 0.85);
    out.push({ id: String(e[0]), x: Math.round(x0), y: Math.round(y0), w, h, max });
  }
  return out;
}

// Команда через shell порождает внука: убиваем всё дерево.
function killTree(p) {
  try {
    if (p.viaShell && p.pid) {
      if (process.platform === 'win32') spawn('taskkill', ['/pid', String(p.pid), '/T', '/F'], { windowsHide: true, stdio: 'ignore' });
      else process.kill(-p.pid, 'SIGTERM');
    } else p.kill();
  } catch { try { p.kill(); } catch {} }
  try { p.stdout && p.stdout.destroy(); p.stderr && p.stderr.destroy(); } catch {}
}

// Менеджер процесса: запуск, построчный разбор, перезапуск с паузой.
function createPcWatcher(opts) {
  const {
    onData,                         // (raw) => void — каждая разобранная строка
    getExclusions = () => ({}),     // () => ({pids, hwnds})
    log = () => {},
    platform = process.platform,
    cmd = process.env.GIANT_PC_CMD, // для тестов: любая команда, печатающая строки JSON
    interval = 700,
    maxRestarts = 5,
    spawnFn = spawn,
  } = opts;
  let child = null, wanted = false, restarts = 0, timer = null, startedAt = 0;

  function launch() {
    timer = null;
    if (!wanted) return;
    let proc;
    try {
      if (cmd) {
        proc = spawnFn(cmd, { shell: true, windowsHide: true, detached: platform !== 'win32', stdio: ['ignore', 'pipe', 'pipe'] });
        proc.viaShell = true;
      } else {
        if (platform !== 'win32') { log('pc: не Windows, окна не отслеживаются'); return; }
        const ex = getExclusions() || {};
        const script = buildPsScript({ pids: ex.pids || [], hwnds: ex.hwnds || [], parent: process.pid, interval });
        const enc = Buffer.from(script, 'utf16le').toString('base64');
        proc = spawnFn('powershell.exe',
          ['-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass', '-WindowStyle', 'Hidden', '-EncodedCommand', enc],
          { windowsHide: true, stdio: ['ignore', 'pipe', 'pipe'] });
      }
    } catch (e) { log('pc: не удалось запустить: ' + e.message); scheduleRestart(); return; }
    child = proc; startedAt = Date.now();
    let buf = '', errTail = '';
    proc.stdout.setEncoding('utf8');
    proc.stdout.on('data', (d) => {
      buf += d;
      if (buf.length > 1 << 20) buf = buf.slice(-65536);
      let i;
      while ((i = buf.indexOf('\n')) >= 0) {
        const raw = parseLine(buf.slice(0, i)); buf = buf.slice(i + 1);
        if (raw && child === proc) onData(raw);
      }
    });
    proc.stderr.setEncoding('utf8');
    proc.stderr.on('data', (d) => { errTail = (errTail + d).slice(-2000); });
    proc.on('error', (e) => { log('pc: ' + e.message); });
    proc.on('close', (code) => {
      if (child !== proc) return;
      child = null;
      onData({ pa: 0, fs: 0, w: [] });
      if (!wanted) return;
      log('pc: процесс завершился (' + code + ') ' + errTail.trim().slice(-300));
      scheduleRestart();
    });
  }

  function scheduleRestart() {
    if (!wanted || timer) return;
    if (Date.now() - startedAt > 60000) restarts = 0; // долго проработал — счётчик заново
    if (restarts >= maxRestarts) { log('pc: слишком много сбоев, слежение за окнами выключено'); return; }
    const delay = Math.min(30000, 1500 * 2 ** restarts++);
    timer = setTimeout(launch, delay);
  }

  return {
    start() { if (wanted) return; wanted = true; restarts = 0; launch(); },
    stop() {
      wanted = false;
      if (timer) { clearTimeout(timer); timer = null; }
      const p = child; child = null;
      if (p) killTree(p);
    },
    get running() { return !!child; },
  };
}

module.exports = { createPcWatcher, buildPsScript, parseLine, convertWindows, CS };
