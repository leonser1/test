// Огненный дождь — заставка Windows (.scr): падающие горящие точки, как угольки.
// Собирается Mono/.NET Framework 4: mcs -target:winexe -unsafe -r:System.Windows.Forms.dll -r:System.Drawing.dll -out:Embers.scr Embers.cs
// Аргументы заставки: /s — запуск, /c — настройки, /p <hwnd> — предпросмотр в окне «Параметры заставки».
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Runtime.InteropServices;
using System.Windows.Forms;
using Microsoft.Win32;

static class Program
{
    [DllImport("user32.dll")] static extern bool SetProcessDPIAware();

    [STAThread]
    static void Main(string[] args)
    {
        try { SetProcessDPIAware(); } catch { }
        Application.EnableVisualStyles();
        string mode = args.Length > 0 ? args[0].Trim().ToLowerInvariant() : "/c";
        string arg2 = args.Length > 1 ? args[1] : null;
        if (mode.Length > 2 && mode[2] == ':') { arg2 = mode.Substring(3); mode = mode.Substring(0, 2); }

        if (mode == "/p")
        {
            IntPtr parent;
            long h;
            if (arg2 == null || !long.TryParse(arg2, out h)) return;
            parent = new IntPtr(h);
            Application.Run(new EmberForm(parent));
        }
        else if (mode == "/s")
        {
            var forms = new List<Form>();
            foreach (Screen s in Screen.AllScreens) { var f = new EmberForm(s.Bounds); forms.Add(f); f.Show(); }
            Application.Run(forms[0]);
        }
        else
        {
            Application.Run(new SettingsForm());
        }
    }
}

static class Settings
{
    const string Key = @"Software\SteelGiant\Embers";
    public static int Density = 35, Speed = 55, Wind = 12;
    public static void Load()
    {
        try
        {
            using (var k = Registry.CurrentUser.OpenSubKey(Key))
            {
                if (k == null) return;
                Density = Clamp((int)k.GetValue("Density", Density), 0, 100);
                Speed = Clamp((int)k.GetValue("Speed", Speed), 20, 100);
                Wind = Clamp((int)k.GetValue("Wind", Wind), -100, 100);
            }
        }
        catch { }
    }
    public static void Save()
    {
        try
        {
            using (var k = Registry.CurrentUser.CreateSubKey(Key))
            {
                k.SetValue("Density", Density, RegistryValueKind.DWord);
                k.SetValue("Speed", Speed, RegistryValueKind.DWord);
                k.SetValue("Wind", Wind, RegistryValueKind.DWord);
            }
        }
        catch { }
    }
    static int Clamp(int v, int a, int b) { return v < a ? a : v > b ? b : v; }
}

/* ---------- ember simulation and software renderer ---------- */
class Ember
{
    public float X, Y, Vx, Vy, T, Cool, R, Ph, Fl, Sw;
}

class EmberField
{
    readonly List<Ember> embers = new List<Ember>();
    readonly Random rnd = new Random();
    public int W, H;
    float k, acc;
    // colour ramp: temperature 1 → 0
    static readonly float[] RT = { 1f, 0.82f, 0.6f, 0.38f, 0.18f, 0f };
    static readonly int[,] RC = { { 255, 250, 225 }, { 255, 214, 120 }, { 255, 150, 50 }, { 235, 80, 25 }, { 160, 30, 12 }, { 60, 10, 6 } };
    // pre-computed round glow kernels: radius 1..12 px
    readonly float[][] kern = new float[13][];

    public EmberField(int w, int h)
    {
        W = w; H = h; k = Math.Max(0.8f, h / 1080f);
        for (int r = 1; r <= 12; r++)
        {
            int n = r * 2 + 1; var a = new float[n * n];
            for (int y = 0; y < n; y++) for (int x = 0; x < n; x++)
            {
                float dx = (x - r) / (float)r, dy = (y - r) / (float)r, d = (float)Math.Sqrt(dx * dx + dy * dy);
                float v = d >= 1 ? 0 : (1 - d); v = (float)Math.Pow(v, 1.6);
                a[y * n + x] = v;
            }
            kern[r] = a;
        }
        for (int i = 0; i < 220; i++) { var e = Spawn(); e.Y = (float)rnd.NextDouble() * H; e.T *= 0.6f + 0.4f * (float)rnd.NextDouble(); }
    }

    float R01() { return (float)rnd.NextDouble(); }

    Ember Spawn()
    {
        if (embers.Count > 2500) embers.RemoveAt(0);
        float r = R01();
        var e = new Ember
        {
            X = R01() * (W + 200 * k) - 100 * k, Y = -10 * k - R01() * 40 * k,
            Vx = (R01() - 0.5f) * 20 * k, Vy = (20 + R01() * 40) * k,
            T = 0.55f + R01() * 0.35f, Cool = 0.045f + R01() * 0.065f, R = 0.7f + r * r * 1.6f,
            Ph = R01() * 6.28f, Fl = 5 + R01() * 9, Sw = 0.6f + R01() * 1.6f
        };
        embers.Add(e);
        return e;
    }

    static void Tint(float t, out float r, out float g, out float b)
    {
        for (int i = 1; i < RT.Length; i++)
        {
            if (t >= RT[i])
            {
                float q = (t - RT[i]) / (RT[i - 1] - RT[i]);
                r = RC[i - 1, 0] * q + RC[i, 0] * (1 - q); g = RC[i - 1, 1] * q + RC[i, 1] * (1 - q); b = RC[i - 1, 2] * q + RC[i, 2] * (1 - q);
                return;
            }
        }
        r = RC[5, 0]; g = RC[5, 1]; b = RC[5, 2];
    }

    public void Step(float dt, double now)
    {
        float tScale = 0.35f + Settings.Speed / 100f * 0.65f; dt *= tScale;
        acc += dt * Settings.Density * 1.6f * Math.Max(1f, W / 1920f);
        while (acc > 1) { acc--; Spawn(); }
        float wx = Settings.Wind * 0.9f * k;
        for (int i = embers.Count - 1; i >= 0; i--)
        {
            var p = embers[i];
            p.Ph += dt * p.Sw * 2;
            float drag = Math.Min(1f, dt * (0.9f + 1.4f / p.R));
            p.Vx += (wx + (float)Math.Sin(p.Ph) * 18 * p.Sw * k - p.Vx) * drag;
            p.Vy += ((55 + 28 * p.R) * k - p.Vy) * drag * 0.6f + 40 * k * dt;
            p.X += p.Vx * dt; p.Y += p.Vy * dt;
            p.T -= p.Cool * dt * (1.2f - p.R * 0.12f);
            if (p.T <= 0 || p.Y > H + 20 * k || p.X < -150 * k || p.X > W + 150 * k) embers.RemoveAt(i);
        }
    }

    public unsafe void Render(uint* pix, int stride, double now)
    {
        // clear to night colour
        uint bg = 0xFF07060Au;
        for (int y = 0; y < H; y++) { uint* row = pix + y * stride; for (int x = 0; x < W; x++) row[x] = bg; }
        foreach (var p in embers)
        {
            float flick = 0.75f + 0.25f * (float)Math.Sin(now * p.Fl + p.Ph);
            float a = Math.Min(1f, p.T * 1.6f) * flick;
            float cr, cg, cb; Tint(p.T, out cr, out cg, out cb);
            // short tail
            int steps = 6; float tx = p.Vx * 0.06f, ty = p.Vy * 0.06f;
            for (int s = 1; s <= steps; s++)
            {
                float q = s / (float)steps;
                Add(pix, stride, (int)(p.X - tx * q), (int)(p.Y - ty * q), cr, cg, cb, a * 0.7f * (1 - q));
            }
            // soft glow + hot core
            int gr = Math.Max(2, Math.Min(12, (int)Math.Round(p.R * (3f + 2.4f * p.T) * k)));
            Splat(pix, stride, p.X, p.Y, gr, cr, cg, cb, a * 0.8f);
            float hr, hg, hb; Tint(Math.Min(1f, p.T + 0.12f), out hr, out hg, out hb);
            int cr2 = Math.Max(1, Math.Min(12, (int)Math.Round(p.R * 1.2f * k + 0.4f)));
            Splat(pix, stride, p.X, p.Y, cr2, hr, hg, hb, Math.Min(1.6f, a * 1.5f));
        }
    }

    unsafe void Splat(uint* pix, int stride, float fx, float fy, int r, float cr, float cg, float cb, float a)
    {
        var kk = kern[r]; int n = r * 2 + 1, cx = (int)fx - r, cy = (int)fy - r;
        for (int y = 0; y < n; y++)
        {
            int py = cy + y; if (py < 0 || py >= H) continue;
            uint* row = pix + py * stride;
            for (int x = 0; x < n; x++)
            {
                int px = cx + x; if (px < 0 || px >= W) continue;
                float v = kk[y * n + x] * a; if (v <= 0.004f) continue;
                Blend(row + px, cr * v, cg * v, cb * v);
            }
        }
    }

    unsafe void Add(uint* pix, int stride, int x, int y, float cr, float cg, float cb, float a)
    {
        if (x < 0 || y < 0 || x >= W || y >= H || a <= 0.004f) return;
        Blend(pix + y * stride + x, cr * a, cg * a, cb * a);
    }

    static unsafe void Blend(uint* d, float r, float g, float b)
    {
        uint c = *d;
        int R = (int)((c >> 16) & 255) + (int)r, G = (int)((c >> 8) & 255) + (int)g, B = (int)(c & 255) + (int)b;
        if (R > 255) R = 255; if (G > 255) G = 255; if (B > 255) B = 255;
        *d = 0xFF000000u | ((uint)R << 16) | ((uint)G << 8) | (uint)B;
    }
}

/* ---------- full-screen / preview window ---------- */
class EmberForm : Form
{
    [DllImport("user32.dll")] static extern IntPtr SetParent(IntPtr child, IntPtr parent);
    [DllImport("user32.dll")] static extern int SetWindowLong(IntPtr h, int idx, int val);
    [DllImport("user32.dll")] static extern int GetWindowLong(IntPtr h, int idx);
    [DllImport("user32.dll")] static extern bool GetClientRect(IntPtr h, out RECT r);
    [DllImport("user32.dll")] static extern bool IsWindow(IntPtr h);
    [StructLayout(LayoutKind.Sequential)] struct RECT { public int Left, Top, Right, Bottom; }

    readonly bool preview;
    readonly IntPtr previewParent;
    EmberField field;
    Bitmap buf;
    readonly Timer timer = new Timer();
    readonly System.Diagnostics.Stopwatch clock = System.Diagnostics.Stopwatch.StartNew();
    double last;
    Point? mouseStart;

    public EmberForm(Rectangle bounds)
    {
        Init();
        StartPosition = FormStartPosition.Manual;
        Bounds = bounds;
        TopMost = true;
        Cursor.Hide();
    }

    public EmberForm(IntPtr parent)
    {
        preview = true; previewParent = parent;
        Init();
        RECT r; GetClientRect(parent, out r);
        SetParent(Handle, parent);
        SetWindowLong(Handle, -16, GetWindowLong(Handle, -16) | 0x40000000); // WS_CHILD
        Location = new Point(0, 0);
        Size = new Size(Math.Max(1, r.Right - r.Left), Math.Max(1, r.Bottom - r.Top));
    }

    void Init()
    {
        Settings.Load();
        FormBorderStyle = FormBorderStyle.None;
        ShowInTaskbar = false;
        BackColor = Color.FromArgb(7, 6, 10);
        SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.Opaque, true);
        timer.Interval = 15;
        timer.Tick += Tick;
        timer.Start();
    }

    protected override void OnResize(EventArgs e)
    {
        base.OnResize(e);
        if (ClientSize.Width < 1 || ClientSize.Height < 1) return;
        if (buf != null) buf.Dispose();
        buf = new Bitmap(ClientSize.Width, ClientSize.Height, PixelFormat.Format32bppRgb);
        field = new EmberField(ClientSize.Width, ClientSize.Height);
    }

    unsafe void Tick(object s, EventArgs e)
    {
        if (preview && !IsWindow(previewParent)) { Application.Exit(); return; }
        if (field == null || buf == null) return;
        double now = clock.Elapsed.TotalSeconds;
        float dt = (float)Math.Min(0.05, now - last); last = now;
        field.Step(dt, now);
        var data = buf.LockBits(new Rectangle(0, 0, buf.Width, buf.Height), ImageLockMode.WriteOnly, PixelFormat.Format32bppRgb);
        try { field.Render((uint*)data.Scan0, data.Stride / 4, now); }
        finally { buf.UnlockBits(data); }
        Invalidate();
    }

    protected override void OnPaintBackground(PaintEventArgs e) { }

    protected override void OnPaint(PaintEventArgs e)
    {
        if (buf == null) return;
        e.Graphics.CompositingMode = System.Drawing.Drawing2D.CompositingMode.SourceCopy;
        e.Graphics.InterpolationMode = System.Drawing.Drawing2D.InterpolationMode.NearestNeighbor;
        e.Graphics.DrawImageUnscaled(buf, 0, 0);
    }

    protected override void OnMouseMove(MouseEventArgs e)
    {
        if (preview) return;
        if (mouseStart == null) { mouseStart = e.Location; return; }
        if (Math.Abs(e.X - mouseStart.Value.X) > 10 || Math.Abs(e.Y - mouseStart.Value.Y) > 10) Application.Exit();
    }
    protected override void OnMouseDown(MouseEventArgs e) { if (!preview) Application.Exit(); }
    protected override void OnKeyDown(KeyEventArgs e) { if (!preview) Application.Exit(); }
}

/* ---------- settings dialog (/c) ---------- */
class SettingsForm : Form
{
    TrackBar dens, speed, wind;
    public SettingsForm()
    {
        Settings.Load();
        Text = "Огненный дождь — параметры";
        FormBorderStyle = FormBorderStyle.FixedDialog; MaximizeBox = false; MinimizeBox = false;
        StartPosition = FormStartPosition.CenterScreen;
        Font = new Font("Segoe UI", 9f);
        ClientSize = new Size(380, 230);
        dens = Row("Густота", 0, 100, Settings.Density, 16);
        speed = Row("Скорость", 20, 100, Settings.Speed, 66);
        wind = Row("Ветер", -100, 100, Settings.Wind, 116);
        var ok = new Button { Text = "OK", DialogResult = DialogResult.OK, Location = new Point(196, 184), Size = new Size(80, 28) };
        var cancel = new Button { Text = "Отмена", DialogResult = DialogResult.Cancel, Location = new Point(284, 184), Size = new Size(80, 28) };
        ok.Click += (s, e) => { Settings.Density = dens.Value; Settings.Speed = speed.Value; Settings.Wind = wind.Value; Settings.Save(); Close(); };
        cancel.Click += (s, e) => Close();
        Controls.Add(ok); Controls.Add(cancel);
        AcceptButton = ok; CancelButton = cancel;
    }
    TrackBar Row(string label, int min, int max, int val, int y)
    {
        Controls.Add(new Label { Text = label, Location = new Point(16, y + 8), AutoSize = true });
        var t = new TrackBar { Minimum = min, Maximum = max, Value = Math.Max(min, Math.Min(max, val)), TickFrequency = (max - min) / 10, Location = new Point(100, y), Width = 264 };
        Controls.Add(t);
        return t;
    }
}
