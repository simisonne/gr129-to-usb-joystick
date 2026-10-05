#!/usr/bin/env python3
"""Offline harness: replay synthetic frame streams through the feeder's
resampler and score smoothness.  Pure stdlib, no pad or serial needed.

Scores per axis, over the run minus a 1s warm-up:
  rms@0      RMS(output - truth) in raw units, no latency compensation
  lag        best-fit latency of the output, ms
  rms@lag    RMS error at that latency = distortion rather than lag
  p99step    p99 of |d(out)/dt| between output samples, units/s.  The truth
             peaks near 1600 units/s, so a value much above that is a step.
  stuck%     % of ticks where the output froze (moved <0.05 units) while the
             truth was moving faster than 50 units/s.  This is the freeze.
"""
import math
import random
import sys

import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                       # sibling sim modules
sys.path.insert(0, os.path.dirname(HERE))      # feeder.py at the repo root
from feeder import Smoother, AXES  # noqa: E402

TAU = 2.0 * math.pi


def truth(t):
    return {
        "thr": 128 + 70 * math.sin(TAU * 0.7 * t) + 26 * math.sin(TAU * 3.3 * t + 1.0),
        "yaw": 85 * math.sin(TAU * 1.1 * t + 0.4) + 20 * math.sin(TAU * 4.7 * t),
        "pit": 85 * math.sin(TAU * 0.9 * t + 2.1) + 20 * math.sin(TAU * 5.3 * t + 0.7),
        "ail": 85 * math.sin(TAU * 1.4 * t + 3.3) + 20 * math.sin(TAU * 4.1 * t + 1.9),
    }


def quant(t):
    v = truth(t)
    return {
        "thr": max(0, min(255, int(round(v["thr"])))),
        "yaw": max(-114, min(114, int(round(v["yaw"])))),
        "pit": max(-114, min(114, int(round(v["pit"])))),
        "ail": max(-114, min(114, int(round(v["ail"])))),
    }


# ------------------------------------------------------------------ streams
def stream_clean(rate):
    return [(i / rate, quant(i / rate)) for i in range(int(rate * 30))]


def stream_slots(nslots=4, cycles=235, jitter=0.0015, dropout_cycles=(),
                 rnd=None):
    """Reproduce the observed pattern: one 128ms cycle, 16 slots of 8ms, each
    slot carrying a pair of frames 4ms apart, only `nslots` of them caught per
    cycle, plus a long dropout for the cycles listed."""
    rnd = rnd or random
    out = []
    for c in range(cycles):
        if c in dropout_cycles:
            continue
        base = c * 0.128
        slots = sorted(rnd.sample(range(16), nslots))
        for s in slots:
            for off in (0.0, 0.004):
                t = base + s * 0.008 + off + rnd.uniform(-jitter, jitter)
                out.append((t, quant(t)))
    out.sort()
    return out


# ------------------------------------------------------------- v1 emulation
class V1:
    """The original resampler, exactly as feeder v1 ran it."""

    def __init__(self, rate=125.0):
        self.interval = 1.0 / rate
        self.hist = []

    def add(self, t, f):
        self.hist.append((t, dict(f)))
        del self.hist[:-2]

    def step(self, now):
        h = self.hist
        if not h:
            return None
        if len(h) == 2:
            (t0, f0), (t1, f1) = h
            span = max(t1 - t0, 1e-3)
            dt = now - t1
            if dt > max(0.15, 3 * span):
                return dict(f1)
            a = min(dt / span, 1.5)
            return {k: f0[k] + (f1[k] - f0[k]) * a for k in AXES}
        return dict(h[0][1])


def run(stream, T, sm, rate):
    interval = 1.0 / rate
    dt = 0.0002
    i, n = 0, len(stream)
    out = []
    next_tick = 0.0
    t = 0.0
    while t < T:
        while i < n and stream[i][0] <= t:
            sm.add(stream[i][0], stream[i][1])
            i += 1
        if t >= next_tick:
            v = sm.step(t)
            if v is not None:
                out.append((t, v))
            next_tick += interval
        t += dt
    return out


def score(out, T, warm=1.0):
    rows = {}
    for k in AXES:
        pts = [(t, v[k]) for (t, v) in out if t >= warm]
        if len(pts) < 3:
            continue
        err0 = [p[1] - truth(p[0])[k] for p in pts]
        rms0 = math.sqrt(sum(e * e for e in err0) / len(err0))
        best = (1e9, 0.0)
        for L in range(0, 46):
            Ls = L / 1000.0
            e = [p[1] - truth(p[0] - Ls)[k] for p in pts]
            r = math.sqrt(sum(x * x for x in e) / len(e))
            if r < best[0]:
                best = (r, Ls)
        steps = []
        stuck = tot = 0
        for i in range(1, len(pts)):
            dtv = pts[i][0] - pts[i - 1][0]
            if dtv <= 0:
                continue
            rate = abs(pts[i][1] - pts[i - 1][1]) / dtv
            steps.append(rate)
            tmid = (pts[i][0] + pts[i - 1][0]) / 2
            tp = truth(tmid)[k]
            tq = truth(tmid + 0.001)[k]
            if abs(tq - tp) / 0.001 > 50 and abs(pts[i][1] - pts[i - 1][1]) < 0.05:
                stuck += 1
            tot += 1
        steps.sort()
        p99 = steps[int(0.99 * (len(steps) - 1))] if steps else 0.0
        rows[k] = (rms0, best[1] * 1000, best[0], p99, 100.0 * stuck / max(tot, 1))
    return rows


def main():
    rnd = random.Random(42)
    T = 30.0
    dropout = set(range(6, 235, 25))
    streams = [
        ("clean 62/s", stream_slots(4, 235, 0.0, (), rnd)),
        ("clean 125/s", stream_clean(125)),
        ("log bursty 62/s", stream_slots(4, 235, 0.0015, (), rnd)),
        ("log + 1.2s holes", stream_slots(4, 235, 0.0015, dropout, rnd)),
        ("sparse 31/s", stream_slots(2, 235, 0.0015, dropout, rnd)),
    ]
    algos = [
        ("v1 125Hz (old)", lambda: V1(125.0), 125.0),
        ("v2 default", lambda: Smoother(mode="extrap", rate=250, ema_ms=10.0,
                                        vel_tau_ms=100.0, deadband=1.5), 250.0),
        ("v2 buffer 15ms", lambda: Smoother(mode="buffer", rate=250, delay=0.015,
                                            ema_ms=10.0, vel_tau_ms=100.0,
                                            deadband=1.5), 250.0),
    ]

    print(f"{'stream':18} {'algo':15} {'rate':>6} {'rms@0':>6} {'lag':>5} "
          f"{'rms@lag':>7} {'p99step':>8} {'stuck%':>6}")
    print("-" * 78)
    for name, st in streams:
        rate = len(st) / T
        for aname, make, pr in algos:
            sm = make()
            out = run(st, T, sm, pr)
            rows = score(out, T)
            agg = [sum(r[i] for r in rows.values()) / len(rows) for i in range(5)]
            print(f"{name:18} {aname:15} {rate:6.1f} {agg[0]:6.1f} {agg[1]:5.0f} "
                  f"{agg[2]:7.1f} {agg[3]:8.0f} {agg[4]:6.1f}")
        print()


if __name__ == "__main__":
    main()
