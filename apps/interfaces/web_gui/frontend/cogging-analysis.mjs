// Server-side analysis of existing C0 measurements. No commands or shared-memory writes.
export const createCapture = () => ({ axes: [[], [], []], resets: [0, 0, 0] });

export function appendSamples(capture, rows) {
  let added = 0;
  for (const row of rows ?? []) {
    if (!Array.isArray(row)) continue;
    const [axis, milliseconds, sequence, q, dq, zero] = row;
    if (!Number.isInteger(axis) || axis < 0 || axis > 2 ||
        !Number.isFinite(milliseconds) || !Number.isFinite(q) ||
        !Number.isSafeInteger(sequence) || sequence < 1) continue;
    const samples = capture.axes[axis], last = samples.at(-1), t = milliseconds / 1000;
    if (last && sequence === last.sequence) continue;
    if (last && (sequence < last.sequence || t <= last.t || zero !== last.zero || t - last.t > .25)) {
      samples.length = 0;
      capture.resets[axis]++;
    }
    samples.push({ t, q, dq: Number.isFinite(dq) ? dq : null, sequence, zero });
    if (samples.length > 20000) samples.splice(0, samples.length - 10000);
    added++;
  }
  return added;
}

const mean = (a) => a.reduce((sum, x) => sum + x, 0) / a.length;
const median = (a) => {
  const sorted = [...a].sort((x, y) => x - y), middle = Math.floor(sorted.length / 2);
  return sorted.length % 2 ? sorted[middle] : (sorted[middle - 1] + sorted[middle]) / 2;
};

// Angle-domain radix-2 FFT, periodic Hann, output-shaft velocity coefficients [rad/s].
// theta_i = 2*pi*(originRev + direction*i/samplesPerRevolution).
// Report each bin as sine*sin(order*theta) + cosine*cos(order*theta) about q=0.
export function amplitudeSpectrum(values, samplesPerRevolution, { originRev = 0, direction = 1 } = {}) {
  const n = values.length;
  if (n < 4 || n > 8192 || (n & (n - 1)) || !(samplesPerRevolution > 0) ||
      !Number.isFinite(originRev) || ![1, -1].includes(direction) ||
      values.some((v) => !Number.isFinite(v))) throw new Error("Invalid FFT input");
  const offset = mean(values), real = new Float64Array(n), imag = new Float64Array(n);
  let gain = 0;
  for (let i = 0; i < n; i++) {
    const window = .5 - .5 * Math.cos(2 * Math.PI * i / n);
    real[i] = (values[i] - offset) * window;
    gain += window;
  }
  for (let i = 1, j = 0; i < n; i++) {
    let bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) [real[i], real[j]] = [real[j], real[i]];
  }
  for (let size = 2; size <= n; size *= 2) {
    const angle = -2 * Math.PI / size, wr = Math.cos(angle), wi = Math.sin(angle);
    for (let start = 0; start < n; start += size) {
      let tr = 1, ti = 0;
      for (let j = 0; j < size / 2; j++) {
        const a = start + j, b = a + size / 2;
        const br = tr * real[b] - ti * imag[b], bi = tr * imag[b] + ti * real[b];
        real[b] = real[a] - br; imag[b] = imag[a] - bi;
        real[a] += br; imag[a] += bi;
        const next = tr * wr - ti * wi;
        ti = tr * wi + ti * wr; tr = next;
      }
    }
  }
  return Array.from({ length: n / 2 + 1 }, (_, k) => {
    const order = k * samplesPerRevolution / n;
    const scale = (k === 0 || k === n / 2 ? 1 : 2) / gain;
    const localCosine = real[k] * scale, localSine = -direction * imag[k] * scale;
    // Undo the sliding window's starting phase; reverse sampling changes the
    // imaginary component's sign, not the definition of the absolute angle.
    const phase = 2 * Math.PI * ((order * originRev) % 1);
    const sine = localSine * Math.cos(phase) + localCosine * Math.sin(phase);
    const cosine = localCosine * Math.cos(phase) - localSine * Math.sin(phase);
    return { order, amplitude: Math.hypot(localCosine, localSine), sine, cosine };
  });
}

export function spectralPeaks(spectrum, maximumOrder, count = 5) {
  if (!spectrum) return [];
  const peaks = spectrum.filter((p, i) => i > 0 && p.order <= maximumOrder &&
    p.amplitude > 1e-8 && p.amplitude >= spectrum[i - 1].amplitude &&
    (!spectrum[i + 1] || p.amplitude > spectrum[i + 1].amplitude));
  return peaks.sort((a, b) => b.amplitude - a.amplitude).slice(0, count);
}

// First form velocities in measurement time, then interpolate them in MOTOR ANGLE.
// No conversion from temporal FFT frequency to order is involved.
export function analyseAxis(samples, {
  size = 1024, lpf = true, cutoff = 10, gearRatio = 9, revolutions = 1,
} = {}) {
  if (!Number.isInteger(size) || size < 128 || size > 4096 || (size & (size - 1)))
    throw new Error("FFT size must be a power of two from 128 to 4096");
  if (lpf && (!Number.isFinite(cutoff) || cutoff <= 0)) throw new Error("LPF cutoff must be positive");
  if (!Number.isFinite(gearRatio) || gearRatio <= 0 ||
      !Number.isFinite(revolutions) || revolutions < .125 || revolutions > 16)
    throw new Error("Invalid motor gear ratio or revolution window");
  const empty = { domain: "motor-angle", ready: false, progress: 0, count: samples.length,
    gearRatio, revolutions, resolution: 1 / revolutions, angleStepRev: revolutions / size };
  if (samples.length < 3) return empty;
  const recent = samples.slice(-Math.min(size + 1, samples.length));
  const dt = median(recent.slice(1).map((s, i) => s.t - recent[i].t)), rate = 1 / dt;
  const base = { ...empty, rate, temporalNyquist: rate / 2 };
  if (!(dt > 0) || !Number.isFinite(rate)) return base;
  if (lpf && cutoff >= rate / 2)
    return { ...base, problem: "LPF 주파수를 시간 샘플의 Nyquist보다 낮춰주세요." };

  const turn = 2 * Math.PI, settle = lpf ? 5 / (turn * cutoff) : 0;
  let points = [], completed = null;
  let direction = 0, filtered, segmentStart = samples[0].t, lastMotion = samples[0].t;
  let anchor = samples[0];
  const finishSegment = () => {
    if (points.length > 1 && direction * (points.at(-1).angle - points[0].angle) >= revolutions)
      completed = { points, direction };
    points = [];
  };
  for (let i = 1; i < samples.length; i++) {
    const a = samples[i - 1], b = samples[i], elapsed = b.t - a.t;
    if (elapsed <= 0 || elapsed > Math.max(.025, dt * 8)) {
      points = []; completed = null; direction = 0; filtered = undefined;
      segmentStart = lastMotion = b.t; anchor = b;
      continue;
    }
    const delta = b.q - a.q, sign = Math.abs(delta) > 1e-12 ? Math.sign(delta) : 0;
    if ((sign && direction && sign !== direction) || b.t - lastMotion > .1) {
      // Keep the last completed spectrum, but never splice separate passes together.
      finishSegment();
      direction = 0; filtered = undefined; segmentStart = a.t; anchor = a;
    }
    if (!sign) continue;
    // Integrate the elapsed time across repeated encoder codes. Taking only the
    // nonzero 2-ms differences would overestimate speed at low shaft velocities.
    const interval = b.t - anchor.t, velocity = (b.q - anchor.q) / interval;
    const alpha = lpf ? -Math.expm1(-turn * cutoff * interval) : 1;
    filtered = filtered == null ? velocity : filtered + alpha * (velocity - filtered);
    direction = sign; lastMotion = b.t;
    if (b.t - segmentStart >= settle) points.push({
      angle: (anchor.q + b.q) * .5 * gearRatio / turn,
      t: (anchor.t + b.t) * .5, derived: filtered,
      direct: anchor.dq == null || b.dq == null ? null : (anchor.dq + b.dq) * .5,
      index: i,
    });
    anchor = b;
  }
  const available = points.length > 1 ? direction * (points.at(-1).angle - points[0].angle) : 0;
  const holding = available < revolutions && completed != null;
  const pending = { ...base, direction, availableRevolutions: available,
    progress: Math.min(1, available / revolutions), holding };
  if (holding) {
    points = completed.points;
    direction = completed.direction;
  } else {
    if (samples.at(-1).t - lastMotion > .1 || !direction)
      return { ...pending, progress: 0, problem: "정지 중입니다. 작은 분석 각도를 선택하거나 새 회전 구간을 기다리세요." };
    if (available < revolutions || points.length < 3) return {
      ...pending, problem: `출력축 ${(360 * revolutions / gearRatio).toFixed(1)}° 구간 수집 중 ` +
        `(${(360 * available / gearRatio).toFixed(1)}° 확보)`,
    };
  }

  const end = points.at(-1).angle, start = end - direction * revolutions;
  let first = 0;
  while (first + 1 < points.length && direction * (points[first + 1].angle - start) <= 0) first++;
  const source = points.slice(first);
  const maxAngularGap = source.slice(1).reduce((max, p, i) =>
    Math.max(max, Math.abs(p.angle - source[i].angle)), 0);
  // Interpolation cannot recover orders above the original angular sampling limit.
  const nyquistOrder = Math.min(size / (2 * revolutions), .5 / maxAngularGap);
  if (nyquistOrder < 1 / revolutions) return {
    ...pending, nyquistOrder, problem: "각도 샘플 간격이 너무 큽니다. 회전 속도를 낮춰주세요.",
  };
  const derived = [], direct = [], times = [];
  let index = first;
  for (let i = 0; i < size; i++) {
    const angle = start + direction * i * revolutions / size;
    while (index + 1 < points.length - 1 && direction * (points[index + 1].angle - angle) < 0) index++;
    const a = points[index], b = points[index + 1];
    const mix = Math.max(0, Math.min(1, (angle - a.angle) / (b.angle - a.angle)));
    derived.push(a.derived + (b.derived - a.derived) * mix);
    direct.push(a.direct == null || b.direct == null ? null : a.direct + (b.direct - a.direct) * mix);
    times.push(a.t + (b.t - a.t) * mix);
  }
  const directValid = direct.every(Number.isFinite), derivedMean = mean(derived);
  const directMean = directValid ? mean(direct) : null;
  const windowSamples = samples.slice(source[0].index - 1, points.at(-1).index + 1);
  const gaps = windowSamples.slice(1).map((s, i) => s.t - windowSamples[i].t);
  const missing = windowSamples.slice(1).reduce((sum, s, i) =>
    sum + Math.max(0, s.sequence - windowSamples[i].sequence - 1), 0);
  const spectrum = (values) => amplitudeSpectrum(values, size / revolutions, { originRev: start, direction })
    .filter((p) => p.order <= nyquistOrder);
  return {
    ...pending, ready: true, progress: 1, direction, nyquistOrder, sourceSamples: source.length,
    startMotorRev: start, endMotorRev: end, derived, direct: directValid ? direct : null,
    derivedMean, directMean, duration: points.at(-1).t - times[0], missing,
    jitter: median(gaps.map((v) => Math.abs(v - dt))) / dt * 100,
    derivedRms: Math.sqrt(mean(derived.map((v) => (v - derivedMean) ** 2))),
    directRms: directValid ? Math.sqrt(mean(direct.map((v) => (v - directMean) ** 2))) : null,
    derivedSpectrum: spectrum(derived), directSpectrum: directValid ? spectrum(direct) : null,
  };
}
