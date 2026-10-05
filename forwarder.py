#!/usr/bin/env python3
"""GR129 serial forwarder for raspfour.

Reads decoded V lines from the nRF52840 sniffer, appends them to a CSV, and
streams every stick frame over UDP as JSON:
    {"thr": 0..255, "yaw": -128..127, "pit": -128..127, "ail": -128..127}

The PC feeder does not need to be preconfigured: it sends a datagram
"HELLO" to this host on port 5006 (from wherever it runs); the forwarder
then streams frames back to the sender's IP on port 5005. Until a HELLO
arrives, frames only go to the CSV.

usage: forwarder.py [csv_out] [serial_port]
"""
import csv
import json
import select
import socket
import sys
import threading
import time

import serial

out_csv = sys.argv[1] if len(sys.argv) > 1 else "gr129_live.csv"
ser_port = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyACM1"

dest = None  # (ip, 5005) learned from HELLO

rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
rx.bind(("0.0.0.0", 5006))
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

f = open(out_csv, "a", buffering=1)
w = csv.writer(f)
w.writerow(["t", "raw_line"])
t0 = time.time()
sent = 0
last_report = time.time()


def learn_dest():
    global dest
    while True:
        try:
            data, addr = rx.recvfrom(64)
        except Exception:
            time.sleep(0.5)
            continue
        if data.strip().upper().startswith(b"HELLO"):
            new = (addr[0], 5005)
            if new != dest:
                dest = new
                w.writerow([f"{time.time()-t0:.3f}", f"!DEST {dest}"])
                print(f"[fwd] feeder at {dest}", flush=True)


threading.Thread(target=learn_dest, daemon=True).start()

while True:
    try:
        s = serial.Serial(ser_port, 115200, timeout=0.02)
    except Exception:
        time.sleep(1.0)
        continue
    w.writerow([f"{time.time()-t0:.3f}", f"!OPEN {ser_port}"])
    buf = bytearray()
    while True:
        try:
            # read(n) waits to accumulate n bytes: read(256) held ~9 frames
            # for ~110ms, which became ~110ms of input lag and a burst of UDP
            # datagrams instead of a steady stream.
            data = s.read(s.in_waiting or 1)
        except Exception:
            break
        if not data:
            continue
        buf.extend(data)
        while b"\n" in buf:
            line, _, rest = buf.partition(b"\n")
            buf = bytearray(rest)
            txt = line.decode("ascii", "replace").strip()
            if not txt:
                continue
            w.writerow([f"{time.time()-t0:.3f}", txt])
            if txt.startswith("V,") and dest:
                p = txt.split(",")
                if len(p) >= 9:
                    try:
                        frame = {
                            "thr": int(p[5]),
                            "yaw": int(p[6]),
                            "pit": int(p[7]),
                            "ail": int(p[8]),
                        }
                        try:
                            frame["ms"] = int(p[1])   # board stamp for the PC
                        except (ValueError, IndexError):
                            pass
                        rx.sendto(json.dumps(frame).encode(), dest)
                        sent += 1
                    except ValueError:
                        pass
            now = time.time()
            if now - last_report >= 10:
                last_report = now
                tgt = f"{dest[0]}:{dest[1]}" if dest else "no-feeder-yet"
                print(f"[fwd] {sent} frames in last 10s -> {tgt}", flush=True)
                sent = 0
    try:
        s.close()
    except Exception:
        pass
    time.sleep(0.5)
