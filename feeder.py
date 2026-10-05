#!/usr/bin/env python3
"""GR129 -> virtual Xbox gamepad feeder (Windows).  v2

Two sources:
  * default: UDP from the Pi forwarder (nRF on raspfour) - sends HELLO to
    the Pi so it learns this PC, receives frames on udp/5005
  * --serial COM5 : read the nRF52840 CDC directly when plugged into this PC

Axis mapping (Mode 2):
  throttle -> left stick Y     yaw -> left stick X
  pitch    -> right stick Y    roll -> right stick X

v2 resampling.  v1 linearly interpolated between the last two frames, then
hard froze once the stream went stale for 12ms.  With ~60 frames/s arriving
burstily (the log shows 40-120ms gaps and occasional 1.2s holes) that means a
freeze on every gap and a step on every resume.  v2 instead:

  * damped velocity extrapolation: the output keeps moving on the last known
    velocity with a 100ms decay, so a gap glides instead of freezing, and a
    resumed stream finds the output already in the right place
  * velocity is measured over a 50-80ms baseline rather than adjacent frames,
    so a burst of duplicates cannot produce a wild slope (and is clamped)
  * light EMA (~10ms) on the output hides axis quantisation
  * optional small deadband on yaw/pitch/roll (soft, so no step at the edge)
  * pad pushed at 250Hz instead of 125Hz
  * inter-arrival gap p50/p95/max reported next to pkt/s, so the shape of the
    loss is visible, not just its average
  * the reader never batches. pyserial's read(n) waits to accumulate n bytes,
    so read(512) at ~2 kB/s of V lines returns ~20 frames every ~300ms and
    stamps them all with the same instant. That is 300ms of input lag and it
    is invisible in pkt/s. Now the reader takes whatever has arrived, and
    frames are timed off the board's own millisecond stamp, so the status line
    can separate the real frame gap from the USB delivery delay.

usage: python feeder.py [--dry-run] [--pi 192.168.1.81] [--serial COMx]
       [--inv-thr] [--inv-yaw] [--inv-pit] [--inv-ail]
       [--mode extrap|buffer] [--delay 0.020] [--ema-ms 10] [--deadband 1.5]
       [--rate 250] [--vel-tau-ms 100] [--vel-base 0.060] [--no-smooth]
"""
import argparse
import json
import math
import os
import socket
import threading
import time
from collections import deque

AXES = ("thr", "yaw", "pit", "ail")
BIPOLAR = ("yaw", "pit", "ail")


# ----------------------------------------------------------------- smoothing
class Smoother:
    """Resample irregular controller frames onto a steady output stream.

    mode "extrap" : damped velocity extrapolation from the newest frames
                    (no added latency, continuous through gaps)
    mode "buffer" : render `delay` seconds in the past and interpolate a
                    buffer (best jitter rejection, costs `delay` latency)
    """

    def __init__(self, mode="extrap", rate=250.0, delay=0.020, ema_ms=10.0,
                 vel_tau_ms=100.0, vel_base=0.060, vel_max=2500.0,
                 dt_cap=0.20, deadband=0.0, stale=0.6):
        self.mode = mode
        self.interval = 1.0 / rate
        self.delay = delay
        self.ema_alpha = 1.0 - math.exp(-self.interval / max(ema_ms / 1000.0, 1e-4))
        self.vel_tau = max(vel_tau_ms / 1000.0, 1e-3)
        self.vel_base = vel_base
        self.vel_max = vel_max
        self.dt_cap = dt_cap
        self.deadband = deadband
        self.stale = stale
        self.hist = deque(maxlen=64)
        self.ema = None
        self.lock = threading.Lock()

    # -- inputs ------------------------------------------------------------
    def add(self, t, f):
        with self.lock:
            self.hist.append((t, f))

    def _velocity(self):
        """units/s over a ~vel_base baseline, clamped."""
        h = self.hist
        if len(h) < 2:
            return {k: 0.0 for k in AXES}
        tl, fl = h[-1]
        i = len(h) - 1
        while i > 1 and (tl - h[i - 1][0]) < self.vel_base:
            i -= 1
        tr, fr = h[i - 1] if i > 0 else h[0]
        span = tl - tr
        if span < 5e-4:                      # identical stamps: no velocity
            return {k: 0.0 for k in AXES}
        v = {}
        for k in AXES:
            d = (fl[k] - fr[k]) / span
            v[k] = max(-self.vel_max, min(self.vel_max, d))
        return v

    def _extrap(self, tr):
        h = self.hist
        tl, fl = h[-1]
        dt = tr - tl
        if dt <= 0.0:
            return dict(fl)
        v = self._velocity()
        dte = min(dt, self.dt_cap)
        damp = math.exp(-dt / self.vel_tau)
        return {k: fl[k] + v[k] * dte * damp for k in AXES}

    def _interp(self, tr):
        """interpolate the buffer at time tr; extrapolate past the newest."""
        h = self.hist
        if len(h) == 1:
            return dict(h[0][1])
        i = len(h) - 1
        while i > 0 and h[i - 1][0] > tr:
            i -= 1
        if i == 0:
            return dict(h[0][1])             # tr older than the buffer
        t0, f0 = h[i - 1]
        t1, f1 = h[i]
        if t1 <= tr:
            return self._extrap(tr)
        span = max(t1 - t0, 1e-4)
        a = (tr - t0) / span
        return {k: f0[k] + (f1[k] - f0[k]) * a for k in AXES}

    # -- output ------------------------------------------------------------
    def step(self, now):
        with self.lock:
            h = self.hist
            if not h:
                return None
            if (now - h[-1][0]) > self.stale:
                raw = dict(h[-1][1])         # long silence: hold, no drift
            elif self.mode == "buffer":
                raw = self._interp(now - self.delay)
            else:
                raw = self._extrap(now)
        out = {}
        for k in AXES:
            v = raw[k]
            if self.deadband and k in BIPOLAR:
                if -self.deadband < v < self.deadband:
                    v = 0.0
                else:
                    v -= self.deadband if v > 0 else -self.deadband
            out[k] = v
        if self.ema is None:
            self.ema = dict(out)
        else:
            a = self.ema_alpha
            self.ema = {k: self.ema[k] + (out[k] - self.ema[k]) * a for k in AXES}
        return dict(self.ema)


# -------------------------------------------------------------------- clock
class ClockMap:
    """Map the board's own millisecond stamp onto the PC clock.

    The firmware stamps every frame with its uptime, which is the true radio
    arrival time.  The USB reader hands data over in whatever chunks the OS
    gives it, so the PC-side arrival time carries that delivery jitter.  The
    smallest (pc_arrival - mcu) seen over a window is the clock offset, and
    everything above it is transport delay.  Watch that delay: a p95 in the
    hundreds of milliseconds means the reader is batching, not that the radio
    is slow.
    """

    def __init__(self, win=2.0):
        self.win = win
        self.samples = deque()
        self.last_mcu = None
        self.off = None

    def calibrate(self, pc_now, mcus):
        """One calibration per handed-over chunk.

        Inside a chunk the freshest frame carries the smallest transport delay,
        so it alone is a good clock sample, and it must be taken BEFORE the
        older frames of the chunk are converted (otherwise each older frame
        drags the minimum down and pins itself to the delivery instant, which
        is exactly how a batching reader becomes hundreds of ms of input lag).
        """
        if not mcus:
            return
        mcu = mcus[-1] / 1000.0
        if self.last_mcu is not None and mcu < self.last_mcu - 1.0:
            self.samples.clear()                  # board rebooted mid-run
        self.last_mcu = mcu
        self.samples.append((pc_now, pc_now - mcu))
        while self.samples and pc_now - self.samples[0][0] > self.win:
            self.samples.popleft()
        self.off = min(s[1] for s in self.samples)

    def time_of(self, mcu_ms, pc_now=None):
        """True arrival time of a stamped frame, plus its transport delay."""
        if self.off is None:
            return (pc_now if pc_now is not None else mcu_ms / 1000.0), 0.0
        t = mcu_ms / 1000.0 + self.off
        return t, ((pc_now - t) if pc_now is not None else 0.0)


# ------------------------------------------------------------------- logging
LOG = None


def log(msg):
    """Print to stdout and, when logging is on, to this run's log file."""
    print(msg, flush=True)
    if LOG is not None:
        try:
            LOG.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}\n")
        except Exception:
            pass


# ------------------------------------------------------------------- runtime
ARGS = None
INVERT = {}
PAD = None
SM = None

last = {"thr": 128, "yaw": 0, "pit": 0, "ail": 0}
last_pkt = 0.0
pkts = 0
gaps = []
lags = []
prev_t = None
CLOCK = None
anchors = []          # last few anchor stamps (board ms)
anchorGaps = []       # anchor to anchor deltas, board ms
anchorCount = [0]     # anchors seen this window
warned = False
lock = threading.Lock()
pushCount = 0
lastPush = 0.0
rate_window = 0.0


def clamp(v, lo=-32768, hi=32767):
    return lo if v < lo else (hi if v > hi else v)


def push(vals):
    """Map a smoothed frame dict onto the virtual pad."""
    global warned, pushCount, lastPush
    pushCount += 1
    lastPush = time.perf_counter()
    try:
        thr = 255.0 - vals["thr"] if INVERT["thr"] else vals["thr"]
        yaw = -vals["yaw"] if INVERT["yaw"] else vals["yaw"]
        pit = -vals["pit"] if INVERT["pit"] else vals["pit"]
        ail = -vals["ail"] if INVERT["ail"] else vals["ail"]
        if PAD:
            PAD.left_joystick(x_value=clamp(round(yaw * 256)),
                              y_value=clamp(round((thr - 128) * 257)))
            PAD.right_joystick(x_value=clamp(round(ail * 256)),
                               y_value=clamp(round(-pit * 256)))
            PAD.update()
    except Exception as e:
        if not warned:
            warned = True
            log(f"[feeder] apply error: {e}")


def handle_frame(f, mcu_ms=None):
    """f = {"thr":0..255,"yaw":..,"pit":..,"ail":..} (ints).

    mcu_ms is the board's own timestamp for the frame.  When present the frame
    is placed on the PC clock at its true arrival, so a batching reader cannot
    smear the timing; when absent (older firmware, UDP without a stamp) the
    delivery time is used instead."""
    global last, last_pkt, pkts, prev_t
    pc_now = time.perf_counter()
    if mcu_ms is None:
        t, lag = pc_now, 0.0
    else:
        t, lag = CLOCK.time_of(mcu_ms, pc_now)
    with lock:
        last = f
        last_pkt = pc_now
        pkts += 1
        if prev_t is not None:
            gaps.append(t - prev_t)
        prev_t = t
        lags.append(lag)
    SM.add(t, {k: float(f[k]) for k in AXES})
    if ARGS.no_smooth:
        push({k: float(f[k]) for k in AXES})


def smooth_loop():
    """Push resampled frames at the configured rate."""
    interval = SM.interval
    next_t = time.perf_counter()
    while True:
        vals = SM.step(time.perf_counter())
        if vals is not None:
            push(vals)
        next_t += interval
        d = next_t - time.perf_counter()
        if d > 0:
            time.sleep(d)
        else:
            next_t = time.perf_counter()


def handle_line(txt):
    """Parse a raw sniffer V line into a frame; show probe/stat lines."""
    if not txt.startswith("V,"):
        if txt[:2] in ("Q,", "W,", "S,"):     # probe/stat lines
            log(f"[sniff] {txt}")
        elif txt.startswith("A,"):
            # anchor line: the board's own stamp here gives the true spacing
            # between anchors, i.e. the controller's real cycle length
            p = txt.split(",")
            try:
                m = int(p[1])
            except (ValueError, IndexError):
                return False
            with lock:
                if anchors:
                    d = m - anchors[-1]
                    if 0 < d < 5000:
                        anchorGaps.append(d)
                anchors.append(m)
                del anchors[:-4]
                anchorCount[0] += 1
        return False
    p = txt.split(",")
    if len(p) < 9:
        return False
    try:
        mcu = int(p[1])
    except (ValueError, IndexError):
        mcu = None
    try:
        handle_frame({"thr": int(p[5]), "yaw": int(p[6]),
                      "pit": int(p[7]), "ail": int(p[8])}, mcu)
    except ValueError:
        return False
    return True


def mcus_of(lines):
    """Board timestamps of the V lines in one delivered chunk."""
    out = []
    for txt in lines:
        if not txt.startswith("V,"):
            continue
        p = txt.split(",")
        if len(p) < 9:
            continue
        try:
            out.append(int(p[1]))
        except (ValueError, IndexError):
            continue
    return out


def pct(xs, q):
    if not xs:
        return 0.0
    xs = sorted(xs)
    i = min(len(xs) - 1, max(0, int(round((len(xs) - 1) * q))))
    return xs[i]


def report(now):
    global pkts, pushCount, rate_window, gaps, lags, anchorGaps
    if now - rate_window >= 2.0:
        iv = now - rate_window
        rate_window = now
        with lock:
            g, lg = gaps, lags
            gaps, lags = [], []
            ag, ac = anchorGaps, anchorCount[0]
            anchorGaps, anchorCount[0] = [], 0
            age = f"{now - last_pkt:.1f}s ago" if last_pkt else "never"
            f_ = dict(last)
        gp50, gp95 = pct(g, 0.5) * 1e3, pct(g, 0.95) * 1e3
        gmax = (max(g) * 1e3) if g else 0.0
        lp50, lp95 = pct(lg, 0.5) * 1e3, pct(lg, 0.95) * 1e3
        page = f"{now - lastPush:.2f}s ago" if lastPush else "never"
        log(f"[feeder] {pkts / iv:5.1f} pkt/s  pad {pushCount / iv:5.1f}/s "
              f"last {page}  frame gap p50={gp50:5.1f} p95={gp95:5.1f} "
              f"max={gmax:6.1f}ms  "
              f"usb delay p50={lp50:4.1f} p95={lp95:5.1f}ms  "
              f"anchors {ac}/{ac / iv:4.1f}Hz cyc={pct(ag, 0.5) if ag else 0:.0f}ms  "
              f"last packet {age}  "
              f"thr={f_['thr']} yaw={f_['yaw']} "
              f"pit={f_['pit']} ail={f_['ail']}")
        pkts = 0
        pushCount = 0


def main():
    global ARGS, INVERT, PAD, SM, LOG, rate_window
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", default="192.168.1.81")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--hello-port", type=int, default=5006)
    ap.add_argument("--serial", default=None, help="read nRF CDC directly, e.g. COM5")
    ap.add_argument("--dry-run", action="store_true", help="no virtual pad, just print")
    ap.add_argument("--inv-thr", action="store_true")
    ap.add_argument("--inv-yaw", action="store_true")
    ap.add_argument("--inv-pit", action="store_true")
    ap.add_argument("--inv-ail", action="store_true")
    ap.add_argument("--no-smooth", action="store_true",
                    help="disable resampling: push each raw frame as it arrives")
    ap.add_argument("--mode", default="extrap", choices=("extrap", "buffer"),
                    help="extrap: no added latency; buffer: delay+interpolate")
    ap.add_argument("--delay", type=float, default=0.020,
                    help="jitter buffer depth in seconds (--mode buffer)")
    ap.add_argument("--ema-ms", type=float, default=10.0,
                    help="output filter time constant")
    ap.add_argument("--vel-tau-ms", type=float, default=100.0,
                    help="how long extrapolated velocity is held across a gap")
    ap.add_argument("--vel-base", type=float, default=0.060,
                    help="baseline seconds for the velocity estimate")
    ap.add_argument("--deadband", type=float, default=1.5,
                    help="soft deadband on yaw/pitch/roll, raw units (0 to disable)")
    ap.add_argument("--rate", type=float, default=250.0, help="pad push rate Hz")
    ap.add_argument("--log", default="auto", metavar="PATH",
                    help="run log, truncated each start (default: feeder.log "
                         "next to this script)")
    ap.add_argument("--no-log", action="store_true", help="disable the run log")
    ARGS = ap.parse_args()

    # NOTE: argparse turns --inv-thr into args.inv_thr, not args.thr
    INVERT = {"thr": ARGS.inv_thr, "yaw": ARGS.inv_yaw,
              "pit": ARGS.inv_pit, "ail": ARGS.inv_ail}

    if ARGS.no_log:
        ARGS.log = None
    path = None
    if ARGS.log:
        path = ARGS.log
        if path == "auto":
            path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "feeder.log")
        try:
            LOG = open(path, "w", encoding="utf-8", buffering=1)
        except Exception as e:
            print(f"[feeder] cannot write log {path}: {e}", flush=True)
    if LOG is not None:
        log(f"[feeder] === {time.strftime('%Y-%m-%d %H:%M:%S')} "
            f"port={ARGS.serial or 'udp'} mode={ARGS.mode} "
            f"rate={ARGS.rate:.0f}Hz ema={ARGS.ema_ms:.0f}ms "
            f"vel_tau={ARGS.vel_tau_ms:.0f}ms deadband={ARGS.deadband:g} "
            f"log={path} ===")

    if not ARGS.dry_run:
        import vgamepad as vg
        PAD = vg.VX360Gamepad()

    global CLOCK
    CLOCK = ClockMap()
    SM = Smoother(mode=ARGS.mode, rate=ARGS.rate, delay=ARGS.delay,
                  ema_ms=ARGS.ema_ms, vel_tau_ms=ARGS.vel_tau_ms,
                  vel_base=ARGS.vel_base,
                  deadband=0.0 if ARGS.no_smooth else ARGS.deadband)

    if not ARGS.no_smooth:
        threading.Thread(target=smooth_loop, daemon=True).start()
        log(f"[feeder] resample {ARGS.rate:.0f}Hz mode={ARGS.mode} "
              f"delay={ARGS.delay * 1000:.0f}ms ema={ARGS.ema_ms:.0f}ms "
              f"vel_tau={ARGS.vel_tau_ms:.0f}ms deadband={ARGS.deadband:g}",)

    rate_window = time.perf_counter()

    if ARGS.serial:
        import serial
        log(f"[feeder] serial mode: {ARGS.serial}"
              + (" (dry run)" if ARGS.dry_run else " (virtual pad active)"),)
        while True:
            try:
                s = serial.Serial(ARGS.serial, 115200, timeout=0.02)
            except Exception as e:
                log(f"[feeder] serial open failed: {e}")
                time.sleep(1.5)
                continue
            buf = bytearray()
            while True:
                try:
                    # read(n) waits for n bytes or the timeout: at ~2 kB/s of
                    # V lines read(512) held ~20 frames for ~300ms and made the
                    # feeder 300ms late no matter how good the smoothing was.
                    data = s.read(s.in_waiting or 1)
                except Exception:
                    break
                if data:
                    buf.extend(data)
                    lines = []
                    while b"\n" in buf:
                        line, _, rest = buf.partition(b"\n")
                        buf = bytearray(rest)
                        lines.append(line.decode("ascii", "replace").strip())
                    if lines:
                        CLOCK.calibrate(time.perf_counter(), mcus_of(lines))
                        for txt in lines:
                            handle_line(txt)
                report(time.perf_counter())
            try:
                s.close()
            except Exception:
                pass
            time.sleep(0.5)
    else:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", ARGS.port))
        s.settimeout(0.25)
        last_hello = 0.0
        log(f"[feeder] udp mode: listening on {ARGS.port}, hellos -> "
              f"{ARGS.pi}:{ARGS.hello_port}"
              + (" (dry run)" if ARGS.dry_run else " (virtual pad active)"),)
        while True:
            now = time.perf_counter()
            if now - last_hello > 2.0:
                try:
                    s.sendto(b"HELLO", (ARGS.pi, ARGS.hello_port))
                except OSError as e:
                    log(f"[feeder] hello failed: {e}")
                last_hello = now
            try:
                data, _addr = s.recvfrom(2048)
            except (socket.timeout, OSError):
                data = None
            if data:
                try:
                    f = json.loads(data)
                    if all(k in f for k in AXES):
                        ms = f.get("ms")
                        if ms is not None:
                            CLOCK.calibrate(time.perf_counter(), [int(ms)])
                        handle_frame({k: int(f[k]) for k in AXES}, ms)
                except (ValueError, KeyError, TypeError):
                    pass
            report(time.perf_counter())


if __name__ == "__main__":
    main()
