import test from "node:test";
import assert from "node:assert/strict";
import { createCapture, appendSamples, amplitudeSpectrum, analyseAxis, spectralPeaks }
  from "../apps/interfaces/web_gui/frontend/cogging-analysis.mjs";

const close = (a, b, tolerance = 1e-8) => assert.ok(Math.abs(a - b) <= tolerance, `${a} != ${b}`);
// Integrate a motor-angle-periodic ripple in time. A temporal sinusoid would not
// test angle resampling, especially when the shaft's speed changes.
const series = ({ count = 4000, rate = 500, jitter = 0, missingVelocity = false,
  speed = 1, variableSpeed = false, direction = 1, order = 6, amplitude = .3,
  sine = 0, cosine = amplitude, initialPosition = 10 } = {}) => {
  const capture = createCapture(), rows = [], gearRatio = 9;
  let q = initialPosition, previousTime = 0;
  const velocity = (t, angle) => direction * (speed * (variableSpeed ? 1 + .4 * Math.sin(1.2 * t) : 1) +
    sine * Math.sin(order * gearRatio * angle) + cosine * Math.cos(order * gearRatio * angle));
  for (let i = 0; i < count; i++) {
    const t = i / rate + jitter * Math.sin(i * 1.7), dt = t - previousTime;
    const a = velocity(previousTime, q), b = velocity(previousTime + dt / 2, q + a * dt / 2);
    const c = velocity(previousTime + dt / 2, q + b * dt / 2), d = velocity(t, q + c * dt);
    q += dt / 6 * (a + 2 * b + 2 * c + d);
    rows.push([0, (100 + t) * 1000, i + 1, q, missingVelocity ? null : velocity(t, q), 1]);
    previousTime = t;
  }
  appendSamples(capture, rows);
  return { capture, rows, order };
};
const primary = (result, signal = "derivedSpectrum") => spectralPeaks(result[signal], 100)[0];

test("single-sided angular Hann amplitudes, DC rejection, order bins and Nyquist", () => {
  const n = 1024, samplesPerRev = 256;
  const samples = Array.from({ length: n }, (_, i) => 4 + .3 * Math.sin(2 * Math.PI * 24 * i / n) +
    .1 * Math.cos(2 * Math.PI * 48 * i / n));
  const spectrum = amplitudeSpectrum(samples, samplesPerRev);
  close(spectrum[24].order, 6);
  close(spectrum[24].amplitude, .3);
  close(spectrum[48].amplitude, .1);
  close(spectrum[0].amplitude, 0);
  const peaks = spectralPeaks(spectrum, 100);
  close(peaks[0].order, 6);
  close(peaks[1].order, 12);
  const nyquist = amplitudeSpectrum(Array.from({ length: n }, (_, i) => .2 * (-1) ** i), samplesPerRev);
  close(nyquist[n / 2].amplitude, .2);
  assert.throws(() => amplitudeSpectrum([1, 2, 3, 4, 5], samplesPerRev));
});

test("sine and cosine coefficients use the absolute angle origin in either direction", () => {
  const n = 1024, expectedSine = .18, expectedCosine = -.24;
  for (const originRev of [0, .173, -3.421, 21.319]) for (const direction of [1, -1])
    for (const [revolutions, order] of [[1, 6], [.5, 6], [.25, 12], [.125, 24]]) {
      const samplesPerRev = n / revolutions;
      const values = Array.from({ length: n }, (_, i) => {
        const theta = 2 * Math.PI * (originRev + direction * i / samplesPerRev);
        return 2 + expectedSine * Math.sin(order * theta) + expectedCosine * Math.cos(order * theta);
      });
      const point = amplitudeSpectrum(values, samplesPerRev, { originRev, direction })
        .find((p) => p.order === order);
      close(point.sine, expectedSine, 1e-10);
      close(point.cosine, expectedCosine, 1e-10);
      close(point.amplitude, Math.hypot(point.sine, point.cosine));
    }
  assert.throws(() => amplitudeSpectrum([0, 1, 0, -1], 4, { direction: 0 }));
});

test("q-origin coefficients remain stable while the window moves over real-time samples", () => {
  for (const direction of [1, -1]) for (const initialPosition of [-2.153, 3.728]) {
    const { capture } = series({ direction, initialPosition, sine: .18, cosine: -.24 });
    for (const samples of [capture.axes[0].slice(0, 2500), capture.axes[0]]) {
      const result = analyseAxis(samples, { lpf: false, revolutions: .5 });
      assert.equal(result.ready, true);
      for (const signal of ["derivedSpectrum", "directSpectrum"]) {
        const peak = primary(result, signal);
        close(peak.order, 6);
        close(peak.sine, direction * .18, .004);
        close(peak.cosine, direction * -.24, .004);
      }
    }
  }
});

test("q and dq are resampled in mechanical motor angle, retaining cumulative q", () => {
  const { capture, order } = series();
  const result = analyseAxis(capture.axes[0], { size: 1024, lpf: false, revolutions: 4 });
  assert.equal(result.ready, true);
  assert.equal(result.domain, "motor-angle");
  close(result.rate, 500, 1e-6);
  close(result.resolution, .25);
  close(Math.abs(result.endMotorRev - result.startMotorRev), 4);
  close(result.derivedMean, 1, .002);
  close(result.directMean, 1, .002);
  close(primary(result).order, order);
  close(primary(result, "directSpectrum").order, order);
  close(primary(result).amplitude, .3, .004);
  close(primary(result, "directSpectrum").amplitude, .3, .004);
  assert.ok(capture.axes[0].at(-1).q > 2 * Math.PI, "q was wrapped to a single turn");
});

test("fixed angular order survives changing speed, irregular timing and reverse rotation", () => {
  for (const speed of [.8, 1.8]) for (const direction of [1, -1]) {
    const { capture, order } = series({ speed, direction, variableSpeed: true, jitter: .00003 });
    const result = analyseAxis(capture.axes[0], { lpf: false, revolutions: 4 });
    assert.equal(result.ready, true);
    assert.equal(result.direction, direction);
    close(primary(result).order, order);
    close(primary(result, "directSpectrum").order, order);
    close(primary(result).amplitude, .3, .02);
  }
});

test("gear ratio defines the FFT coordinate and revolution count defines resolution", () => {
  const { capture } = series();
  const result = analyseAxis(capture.axes[0], { lpf: false, gearRatio: 18, revolutions: 2 });
  assert.equal(result.ready, true);
  close(primary(result).order, 3); // Same q, double the assumed motor rotations.
  close(result.resolution, .5);
  close(result.angleStepRev, 2 / 1024);
});

test("optional first-order time LPF only filters the q-derived velocity", () => {
  const { capture } = series();
  const raw = analyseAxis(capture.axes[0], { lpf: false });
  const filtered = analyseAxis(capture.axes[0], { lpf: true, cutoff: 10 });
  assert.equal(filtered.ready, true);
  assert.ok(primary(filtered).amplitude / primary(raw).amplitude > .7);
  assert.ok(primary(filtered).amplitude / primary(raw).amplitude < .85);
  close(primary(raw, "directSpectrum").amplitude, primary(filtered, "directSpectrum").amplitude);
  assert.match(analyseAxis(capture.axes[0], { cutoff: 300 }).problem, /Nyquist/);
});

test("sparse angular samples limit reported orders despite FFT upsampling", () => {
  const { capture } = series({ rate: 50, speed: 4, order: 1 });
  const result = analyseAxis(capture.axes[0], { size: 4096, lpf: false });
  assert.equal(result.ready, true);
  assert.ok(result.nyquistOrder < 5);
  assert.ok(result.derivedSpectrum.every((p) => p.order <= result.nyquistOrder));
  assert.ok(result.derivedSpectrum.length < 30, "interpolated bins invented angular bandwidth");
});

test("repeated MIT encoder codes retain elapsed time instead of inflating speed", () => {
  const { rows } = series({ count: 18000, speed: .1, amplitude: .02 });
  const capture = createCapture(), quantum = 12.56 / 32768;
  appendSamples(capture, rows.map((row) => {
    const copy = [...row]; copy[3] = Math.round(row[3] / quantum) * quantum; return copy;
  }));
  const result = analyseAxis(capture.axes[0], { lpf: true, cutoff: 10, revolutions: 1 });
  assert.equal(result.ready, true);
  close(result.derivedMean, .1, .003);
  close(primary(result).order, 6);
  close(primary(result).amplitude, .02, .004);
});

test("default needs one motor revolution; fractional windows fit small joint travel", () => {
  const one = analyseAxis(series({ count: 650 }).capture.axes[0], { lpf: false });
  assert.equal(one.ready, true);
  close(one.revolutions, 1);
  close(one.resolution, 1);
  close(Math.abs(one.endMotorRev - one.startMotorRev) * 360 / 9, 40);
  for (const revolutions of [.125, .25, .5]) {
    // 24th-order synthetic ripple fits exactly 3, 6 or 12 periods in these windows.
    const { capture } = series({ count: 250, order: 24, amplitude: .1 });
    const result = analyseAxis(capture.axes[0], { lpf: false, revolutions });
    assert.equal(result.ready, true);
    close(result.resolution, 1 / revolutions);
    close(primary(result).order, 24);
    close(Math.abs(result.endMotorRev - result.startMotorRev) * 360 / 9, revolutions * 40);
  }
});

test("stop and reversal retain a completed spectrum without joining opposite passes", () => {
  const { capture, rows } = series();
  const last = rows.at(-1);
  const before = analyseAxis(capture.axes[0], { lpf: false });
  const reverse = Array.from({ length: 50 }, (_, i) =>
    [0, last[1] + (i + 1) * 2, last[2] + i + 1, last[3] - (i + 1) * .002, -1, 1]);
  appendSamples(capture, reverse);
  let result = analyseAxis(capture.axes[0], { lpf: false });
  assert.equal(result.ready, true);
  assert.equal(result.holding, true);
  assert.equal(result.direction, 1);
  assert.ok(result.availableRevolutions < .2);
  assert.deepEqual(result.derivedSpectrum, before.derivedSpectrum);
  const end = reverse.at(-1);
  appendSamples(capture, Array.from({ length: 100 }, (_, i) =>
    [0, end[1] + (i + 1) * 2, end[2] + i + 1, end[3], 0, 1]));
  result = analyseAxis(capture.axes[0], { lpf: false });
  assert.equal(result.ready, true);
  assert.equal(result.holding, true);
  assert.deepEqual(result.derivedSpectrum, before.derivedSpectrum);
  const gap = series();
  appendSamples(gap.capture, [[0, last[1] + 100, last[2] + 1, last[3] + .1, 1, 1]]);
  assert.equal(analyseAxis(gap.capture.axes[0], { lpf: false }).ready, false);

  const small = createCapture();
  const shortPasses = Array.from({ length: 800 }, (_, i) => {
    const phase = i % 200, q = (phase < 100 ? phase : 200 - phase) * .002;
    return [0, i * 2, i + 1, q, phase < 100 ? 1 : -1, 1];
  });
  appendSamples(small, shortPasses);
  assert.equal(analyseAxis(small.axes[0], { lpf: false }).ready, false,
    "repeated partial passes must not invent a full revolution of angle coverage");
});

test("duplicate, skipped, restarted and zero-set samples remain distinguishable", () => {
  const { capture, rows } = series();
  const count = capture.axes[0].length;
  assert.equal(appendSamples(capture, [rows.at(-1)]), 0);
  assert.equal(capture.axes[0].length, count);
  const last = rows.at(-1);
  appendSamples(capture, [[0, last[1] + 4, last[2] + 2, last[3] + .004, 1, 1]]);
  assert.equal(analyseAxis(capture.axes[0], { lpf: false }).missing, 1);
  appendSamples(capture, [[0, last[1] + 6, last[2] + 3, 0, 1, 2]]);
  assert.equal(capture.axes[0].length, 1, "C3 frame change mixed two coordinate origins");
  appendSamples(capture, [[1, last[1] + 6, 1, .5, 0, 1]]);
  assert.equal(capture.axes[1].length, 1);
  assert.equal(capture.axes[2].length, 0);
  appendSamples(capture, [[0, last[1] + 500, 1, 0, 0, 2]]);
  assert.equal(capture.axes[0].length, 1, "restart/gap kept old spectrum input");
});

test("warm-up and absent dq never invent a complete second signal", () => {
  const { capture } = series({ count: 300 });
  const warm = analyseAxis(capture.axes[0]);
  assert.equal(warm.ready, false);
  assert.ok(warm.progress > 0 && warm.progress < 1);
  const missing = analyseAxis(series({ missingVelocity: true }).capture.axes[0]);
  assert.equal(missing.ready, true);
  assert.equal(missing.direct, null);
  assert.equal(missing.directSpectrum, null);
});
