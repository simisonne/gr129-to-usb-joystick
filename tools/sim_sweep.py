#!/usr/bin/env python3
"""Parameter sweep for the feeder v2 resampler on the two log-like streams.
Picks the defaults by minimising step size and distortion at the lowest lag.
"""
import random
import sys

import os
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)                       # sibling sim modules
sys.path.insert(0, os.path.dirname(HERE))      # feeder.py at the repo root
from feeder import Smoother  # noqa: E402
from sim_smooth import score, run, stream_slots  # noqa: E402

T = 30.0
rnd = random.Random(7)
dropout = set(range(6, 235, 25))
STREAMS = [
    stream_slots(4, 235, 0.0015, (), rnd),
    stream_slots(4, 235, 0.0015, dropout, rnd),
]

CONFIGS = [
    ("extrap base60 tau60 ema8 cap20", dict(mode="extrap", ema_ms=8)),
    ("extrap base60 tau60 ema14 cap20", dict(mode="extrap", ema_ms=14)),
    ("extrap base60 tau60 ema20 cap20", dict(mode="extrap", ema_ms=20)),
    ("extrap base100 tau100 ema8 cap20",
     dict(mode="extrap", ema_ms=8, vel_base=0.100, vel_tau_ms=100)),
    ("extrap base100 tau60 ema14 cap12",
     dict(mode="extrap", ema_ms=14, vel_base=0.100, dt_cap=0.12)),
    ("extrap base60 tau60 ema8 cap10",
     dict(mode="extrap", ema_ms=8, dt_cap=0.10)),
    ("buffer d10 ema8", dict(mode="buffer", delay=0.010, ema_ms=8)),
    ("buffer d15 ema8", dict(mode="buffer", delay=0.015, ema_ms=8)),
    ("buffer d20 ema8", dict(mode="buffer", delay=0.020, ema_ms=8)),
    ("buffer d30 ema8", dict(mode="buffer", delay=0.030, ema_ms=8)),
    ("buffer d15 ema14", dict(mode="buffer", delay=0.015, ema_ms=14)),
    ("buffer d25 ema14", dict(mode="buffer", delay=0.025, ema_ms=14)),
]

print(f"{'config':34} {'rms@0':>6} {'lag':>5} {'rms@lag':>7} {'p99step':>8} {'stuck%':>6}")
print("-" * 74)
for name, kw in CONFIGS:
    acc = [0.0] * 5
    for st in STREAMS:
        sm = Smoother(rate=250, deadband=1.5, **kw)
        out = run(st, T, sm, 250.0)
        rows = score(out, T)
        for i in range(5):
            acc[i] += sum(r[i] for r in rows.values()) / len(rows)
    agg = [a / len(STREAMS) for a in acc]
    print(f"{name:34} {agg[0]:6.1f} {agg[1]:5.0f} {agg[2]:7.1f} {agg[3]:8.0f} {agg[4]:6.1f}")
