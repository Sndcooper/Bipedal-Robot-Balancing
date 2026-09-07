"""
prof_capture.py — read the pretest_wireless tick profiler off the wired port.

The firmware already computes its own 1 Hz report (worst 5 / best / mean / mode
/ worst-case free), so you can just open a plain terminal on COM3 and read it.
This script is for when you want the RAW per-tick rows aggregated your own way,
or logged to CSV for plotting.

PROTOCOL NOTE
-------------
These column letters belong to mcu_ik_engine_pretest_wireless ONLY. ax12_control
emits a different set (P B F R S U T L X) for a different loop. Do not point one
variant's capture script at the other's port — the columns silently mean
different things.

Wiring:
    Bluepill PA9 (TX) --> USB-TTL RX     <- the only signal wire needed
    Bluepill GND      --> USB-TTL GND
    (PA10 unused; the port is output-only by design)

Usage:
    python prof_capture.py [PORT] [--csv out.csv] [--raw]
    python prof_capture.py COM3
    python prof_capture.py COM3 --csv balance_run.csv
"""

import sys
import time

try:
    import serial
except ImportError:
    serial = None


# Column -> what it measures, in loop order.
FIELDS = [
    ("P", "tick period",       "target 10000"),
    ("B", "BODY consumed",     "of the 10000 budget"),
    ("F", "free left",         "10000 - B"),
    ("I", "readIMU (I2C)",     "usually the biggest single stage"),
    ("K", "calibrationTask",   "only non-zero during a cal"),
    ("Y", "safety cutoff",     "spikes on SAFETY:CUTOFF print"),
    ("E", "encoder->velocity", ""),
    ("C", "balance PID+motors", "the actual control law"),
    ("R", "telemetryRX+parse", "spikes on a blocking ack"),
    ("V", "pollLegServosTask", "AX-12 bus, every other tick"),
    ("T", "telemetry block",   "10 Hz"),
    ("X", "profiler cost",     "instrumentation tax"),
]
KEYS = [k for k, _, _ in FIELDS]
STAGE_KEYS = ["I", "K", "Y", "E", "C", "R", "V", "T"]


def parse(line):
    """'P10001 B1603 F8397 I1340 ...' -> dict, or None if not a data row."""
    if not line or line[0] in "#=" or line.startswith(" "):
        return None
    out = {}
    for tok in line.split():
        if tok.startswith("!"):
            out[tok] = out.get(tok, 0) + 1
            continue
        k, v = tok[:1], tok[1:]
        if k not in KEYS:
            return None
        try:
            out[k] = int(v)
        except ValueError:
            return None
    return out if "B" in out else None


def pct(vals, q):
    if not vals:
        return 0
    return vals[min(int(len(vals) * q), len(vals) - 1)]


def main():
    if serial is None:
        sys.exit("pyserial not installed. Run: pip install pyserial")

    args = list(sys.argv[1:])
    port, csv_path, raw = "COM3", None, False
    i = 0
    while i < len(args):
        if args[i] == "--csv":
            i += 1
            csv_path = args[i]
        elif args[i] == "--raw":
            raw = True
        else:
            port = args[i]
        i += 1

    print(f"Opening {port} @ 115200 (pretest_wireless tick profiler)")
    ser = serial.Serial(port, 115200, timeout=1.0)

    csv_f = None
    if csv_path:
        csv_f = open(csv_path, "w", newline="")
        csv_f.write(",".join(KEYS) + ",ovr\n")
        print(f"Logging raw rows to {csv_path}")

    window, buf = [], b""
    total, total_ovr, worst_body = 0, 0, 0
    t_win = time.time()

    print("\nFirmware report lines pass through verbatim. Aggregates below are "
          "computed from the raw rows.\n")
    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    rl, buf = buf.split(b"\n", 1)
                    line = rl.decode("utf-8", "ignore").rstrip()
                    if not line:
                        continue
                    row = parse(line)
                    if row is None:
                        # firmware's own report / legend — show it as-is
                        print(line)
                        continue
                    if raw:
                        print(line)
                    window.append(row)
                    if csv_f:
                        csv_f.write(",".join(str(row.get(k, 0)) for k in KEYS)
                                    + f",{row.get('!OVR', 0)}\n")

            now = time.time()
            if now - t_win >= 1.0 and window:
                n = len(window)
                total += n
                ovr = sum(r.get("!OVR", 0) for r in window)
                total_ovr += ovr
                bodies = sorted(r.get("B", 0) for r in window)
                worst_body = max(worst_body, bodies[-1])
                mean = sum(bodies) / len(bodies)

                print(f"[capture] {n} rows  mean B{mean:.0f}us ({mean/100:.1f}%)  "
                      f"p99 {pct(bodies,0.99)}  max {bodies[-1]}  overruns {ovr}")
                # rank the stages by mean cost -- the "what is eating my budget"
                ranked = []
                for k in STAGE_KEYS:
                    vals = [r.get(k, 0) for r in window]
                    ranked.append((sum(vals) / len(vals), k))
                ranked.sort(reverse=True)
                top = "  ".join(f"{k}{v:.0f}us" for v, k in ranked[:4] if v >= 1)
                print(f"           top stages: {top}")
                print(f"           worst-case free this second: "
                      f"{10000 - bodies[-1]}us\n")
                window = []
                t_win = now
    except KeyboardInterrupt:
        print(f"\nSession: {total} ticks, {total_ovr} overruns, worst body "
              f"{worst_body}us ({worst_body/100:.1f}% of the 10 ms budget), "
              f"worst-case free {10000 - worst_body}us")
    finally:
        if csv_f:
            csv_f.close()
        ser.close()


if __name__ == "__main__":
    main()
