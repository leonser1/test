// Модель посадки: 1 степень свободы, n ножек-пружин, демпфирование, трение пяток.
const G = {
  H: 72.7,      // мм, ось шарнира -> земля по вертикали
  e: 6.4,       // мм, пятка смещена внутрь от оси (95°)
  rSec: 10,     // мм, опасное сечение ножки от оси
  Wx: 8*7*7/6,  // мм³, изгиб ножки в плоскости луча
  Wy: 7*8*8/6,  // мм³, изгиб поперёк луча
  rTooth: 6.3, toothArm: 2.3, Wtooth: 8*4.5*4.5/6,
  earA: 2*3*3.1, hubA: 8*3.25,
  armL: 75, Warm: 11.22*8*8/6,
  threadArm: 3.39, spoolR: 8,
};
const SURF = { concrete: {k: 1e9, mu: 0.6}, dirt: {k: 60, mu: 0.5}, grass: {k: 15, mu: 0.4} };
const MAT = { PETG: 50, PACF: 95 };
function simulate(p) {
  const kFoot = 250, kArm = 170, kLeg = 1400, kHinge = 600;       // Н/мм
  const kG = SURF[p.surface].k;
  const kLegTot = 1 / (1/kFoot + 1/kArm + 1/kLeg + 1/kHinge + 1/kG); // Н/мм
  const n = p.legs, m = p.mass;
  const K = n * kLegTot * 1000;       // Н/м
  const zeta = 0.15, C = 2 * zeta * Math.sqrt(K * m);
  let x = 0, v = p.vz, t = 0, dt = 2e-6, Fmax = 0; const trace = [];
  while (t < 0.05) {
    const F = Math.max(0, K * x + C * v);
    const a = 9.81 - F / m;
    v += a * dt; x += v * dt; t += dt;
    if (x < 0 && t > 1e-4) break;
    Fmax = Math.max(Fmax, F);
    if (trace.length < 400 && Math.round(t / dt) % 25 === 0) trace.push([t * 1000, F / n]);
  }
  const F = Fmax / n;                          // пиковая сила на ножку, Н
  const w = Math.sqrt(K / m);
  const FhNeed = m * p.vh * w / (2 * n);       // сила, чтобы погасить снос
  const Fh = Math.min(SURF[p.surface].mu * F, FhNeed);
  const sinE = G.e / Math.hypot(G.e, G.H);
  const L = Math.hypot(G.e, G.H) - G.rSec;
  const sAllow = MAT[p.mat];
  const r = {};
  r.g = Fmax / m / 9.81; r.F = F; r.Fh = Fh; r.trace = trace; r.defl = x;
  // ножка: передняя (трение внутрь) и боковая
  const sLegFront = (F * sinE + Fh) * L / G.Wx;
  const sLegSide = Fh * L / G.Wy;
  r.leg = { s: Math.max(sLegFront, sLegSide), allow: sAllow };
  // зуб-упор (передняя ножка: снос прижимает к упору)
  const Ftooth = (F * G.e + Fh * G.H) / G.rTooth;
  r.tooth = { s: Ftooth * G.toothArm / G.Wtooth, allow: sAllow, F: Ftooth };
  // ось и уши (смятие)
  const R = F + Ftooth;
  r.ears = { s: R / G.earA, allow: sAllow };
  // задняя ножка: снос складывает, держит нить и серво
  const Mfold = Fh * G.H - F * G.e;
  const T = Math.max(0, Mfold) / G.threadArm;
  r.fold = { T, servoMax: p.servoTorque / (G.spoolR / 1000), threadMax: 200,
             ratio: Fh / F, limit: G.e / G.H };
  r.arm = { s: F * G.armL / G.Warm, allow: 500 };
  return r;
}
if (typeof module !== 'undefined') module.exports = { simulate };
