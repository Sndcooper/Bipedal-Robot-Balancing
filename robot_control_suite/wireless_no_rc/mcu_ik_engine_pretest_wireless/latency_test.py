"""
latency_test.py  —  3DR radio latency & loop-rate diagnostic
=============================================================
Connects to the 3DR telemetry radio ground unit and measures:

  • Packet receive rate  (should be ~20 Hz from firmware)
  • Packet interval jitter
  • Packet drop count     (via sequence number gaps)
  • MCU loop period DT    (embedded in packet, should be ~10 000 µs @ 100 Hz)
  • MCU body time  BD     (how long the loop body actually takes in µs)
  • Round-trip latency    (PING/PONG, fired every 2 s)

Usage:
    python latency_test.py [PORT] [BAUD]
    python latency_test.py COM4
    python latency_test.py COM4 115200   (default baud = 115200)

Output: live terminal table + optional CSV log (latency_log.csv).

Press Ctrl+C to stop.
"""

import sys
import time
import threading
import csv
import os
import statistics

try:
    import serial
except ImportError:
    print("[ERROR] pyserial not installed.  Run: pip install pyserial")
    sys.exit(1)

# ─── Configuration ─────────────────────────────────────────────────────────────
PORT        = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD        = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
CSV_LOG     = "latency_log.csv"
PING_EVERY  = 600.0        # disabled for baseline test — uplink PING was causing 97% drops
                           # (every PING transmission occupied the 3DR half-duplex air link,
                           #  dropping all downlink packets during the 2s window)
STATS_EVERY = 1.0          # seconds between terminal stats refresh
WINDOW      = 100          # rolling window for stats (packets)

# ─── Shared state (writer thread → main thread) ─────────────────────────────────
lock            = threading.Lock()
raw_log         = []        # (pc_recv_time, raw_line)
telemetry_log   = []        # parsed dicts
rtt_log         = []        # round-trip times in ms
pending_pings   = {}        # ping_key → pc_send_time
last_seq        = -1
drop_count      = 0
total_packets   = 0
running         = True

# ─── Byte-accumulator reader thread ─────────────────────────────────────────────
def read_loop(ser):
    global last_seq, drop_count, total_packets, running
    buf = b""
    while running:
        try:
            waiting = ser.in_waiting
            chunk   = ser.read(waiting if waiting > 0 else 1)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                raw_line, buf = buf.split(b"\n", 1)
                pc_t  = time.perf_counter()
                line  = raw_line.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                with lock:
                    raw_log.append((pc_t, line))

                # ── PONG handler ────────────────────────────────────────────
                if line.startswith("PONG:"):
                    key = line[5:].strip()
                    with lock:
                        if key in pending_pings:
                            rtt = (pc_t - pending_pings.pop(key)) * 1000.0  # ms
                            rtt_log.append(rtt)

                # ── Telemetry parser ────────────────────────────────────────
                elif line.startswith("S:"):
                    parsed = {}
                    for part in line.split(","):
                        if ":" in part:
                            k, v = part.split(":", 1)
                            try:
                                parsed[k.strip()] = float(v.strip())
                            except ValueError:
                                pass
                    if "S" in parsed:
                        seq = int(parsed["S"])
                        with lock:
                            if last_seq >= 0:
                                gap = seq - last_seq - 1
                                if gap > 0:
                                    drop_count += gap
                            last_seq = seq
                            total_packets += 1
                            parsed["_pc_t"] = pc_t
                            telemetry_log.append(parsed)

        except Exception as e:
            if running:
                print(f"[reader] {e}")
            time.sleep(0.05)

# ─── PING sender thread ──────────────────────────────────────────────────────────
def ping_loop(ser):
    while running:
        try:
            key = f"{time.perf_counter():.6f}"
            cmd = f"PING:{key}\n".encode()
            with lock:
                pending_pings[key] = time.perf_counter()
            ser.write(cmd)
            time.sleep(PING_EVERY)
        except Exception as e:
            if running:
                print(f"[ping] {e}")
            time.sleep(1.0)

# ─── Stats helpers ────────────────────────────────────────────────────────────────
def safe_stats(lst):
    if not lst:
        return 0.0, 0.0, 0.0, 0.0
    mn  = min(lst)
    mx  = max(lst)
    avg = statistics.mean(lst)
    std = statistics.stdev(lst) if len(lst) > 1 else 0.0
    return mn, mx, avg, std

def print_stats(elapsed_s):
    with lock:
        tlog  = list(telemetry_log)
        rlog  = list(rtt_log)
        drops = drop_count
        total = total_packets

    recent = tlog[-WINDOW:]  if len(tlog) >= WINDOW else tlog

    # Packet interval (pc side) — time between consecutive received packets
    intervals_ms = []
    for i in range(1, len(recent)):
        dt = (recent[i]["_pc_t"] - recent[i-1]["_pc_t"]) * 1000.0
        intervals_ms.append(dt)

    # MCU loop period (DT field, in µs)
    mcu_dt  = [r["DT"] for r in recent if "DT"  in r]
    mcu_bd  = [r["BD"] for r in recent if "BD"  in r]

    iv_mn, iv_mx, iv_avg, iv_std = safe_stats(intervals_ms)
    dt_mn, dt_mx, dt_avg, dt_std = safe_stats(mcu_dt)
    bd_mn, bd_mx, bd_avg, bd_std = safe_stats(mcu_bd)
    rt_mn, rt_mx, rt_avg, rt_std = safe_stats(rlog[-20:] if rlog else [])

    actual_hz = (1000.0 / iv_avg) if iv_avg > 0 else 0.0
    mcu_hz    = (1e6 / dt_avg)    if dt_avg > 0 else 0.0

    pitch_vals = [r["P"] for r in recent if "P" in r]
    cur_pitch  = pitch_vals[-1] if pitch_vals else 0.0

    os.system("cls" if os.name == "nt" else "clear")
    print("="*60)
    print("  3DR RADIO LATENCY DIAGNOSTIC")
    print(f"  Port: {PORT}  Baud: {BAUD}  Elapsed: {elapsed_s:.0f}s")
    print("="*60)
    print(f"\n{'PACKET RATE (PC side, window=%d)':}\n" % WINDOW)
    print(f"  Received total : {total:>6d}  Drops: {drops}")
    print(f"  Drop rate      : {100*drops/max(total+drops,1):.1f}%")
    print(f"  Interval avg   : {iv_avg:>7.1f} ms   ({actual_hz:.1f} Hz)")
    print(f"  Interval min   : {iv_mn:>7.1f} ms")
    print(f"  Interval max   : {iv_mx:>7.1f} ms")
    print(f"  Interval jitter: {iv_std:>7.2f} ms  (1σ)")

    print(f"\n{'MCU LOOP PERIOD (DT field, µs)':}")
    print(f"  avg  {dt_avg:>7.0f} µs  →  {mcu_hz:.1f} Hz  (target: 10000 µs / 100 Hz)")
    print(f"  min  {dt_mn:>7.0f} µs     max  {dt_mx:>7.0f} µs     σ {dt_std:.0f} µs")

    print(f"\n{'MCU BODY TIME (BD field, µs — work done each loop)':}")
    print(f"  avg  {bd_avg:>7.0f} µs")
    print(f"  min  {bd_mn:>7.0f} µs     max  {bd_mx:>7.0f} µs     σ {bd_std:.0f} µs")
    print(f"  Headroom: {10000-bd_avg:.0f} µs  ({(10000-bd_avg)/100:.1f}% of 10ms budget)")

    print(f"\n{'ROUND-TRIP LATENCY (PING/PONG, last 20)':}")
    if rlog:
        print(f"  avg  {rt_avg:>6.1f} ms    min {rt_mn:.1f} ms    max {rt_mx:.1f} ms    σ {rt_std:.1f} ms")
    else:
        print("  (waiting for first PONG...)")

    print(f"\n{'LIVE VALUES':}")
    if recent:
        r = recent[-1]
        print(f"  Pitch : {cur_pitch:+.2f}°")
        print(f"  Seq   : {int(r.get('S',0))}")
        print(f"  MCU T : {r.get('T',0):.0f} µs  (micros on MCU)")
    print()
    print("  Press Ctrl+C to stop and save CSV.")

# ─── CSV writer ──────────────────────────────────────────────────────────────────
def save_csv():
    with lock:
        tlog = list(telemetry_log)
    if not tlog:
        return
    fieldnames = ["pc_recv_t", "seq", "mcu_t_us", "pitch", "loop_dt_us", "body_us"]
    with open(CSV_LOG, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in tlog:
            w.writerow({
                "pc_recv_t"  : f"{r.get('_pc_t', 0):.6f}",
                "seq"        : int(r.get("S",  0)),
                "mcu_t_us"   : int(r.get("T",  0)),
                "pitch"      : f"{r.get('P', 0.0):.4f}",
                "loop_dt_us" : int(r.get("DT", 0)),
                "body_us"    : int(r.get("BD", 0)),
            })
    print(f"\n[Saved {len(tlog)} rows to {CSV_LOG}]")

# ─── Main ─────────────────────────────────────────────────────────────────────────
def main():
    global running
    print(f"Opening {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=None)
    except serial.SerialException as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    t_reader = threading.Thread(target=read_loop, args=(ser,), daemon=True)
    t_ping   = threading.Thread(target=ping_loop, args=(ser,), daemon=True)
    t_reader.start()
    t_ping.start()

    t_start = time.time()
    try:
        while True:
            time.sleep(STATS_EVERY)
            print_stats(time.time() - t_start)
    except KeyboardInterrupt:
        pass

    running = False
    print("\nStopping...")
    time.sleep(0.2)
    ser.close()
    save_csv()

if __name__ == "__main__":
    main()
