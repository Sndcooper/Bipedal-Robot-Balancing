"""
prof_capture.py — read the ax12_control tick profiler off the wired debug port.

The firmware emits ONE raw row per 100 Hz tick on Serial1 (PA9 = TX), which is
~3.9 kB/s and completely unreadable as it scrolls. This reads those rows and
answers the actual question: how much of the 10 ms budget is being consumed,
and by what.

Wiring:
    Bluepill PA9  (TX) --> USB-TTL RX      <- this is the only wire needed
    Bluepill GND       --> USB-TTL GND
    (PA10 is unused; the port is output-only by design)

Usage:
    python prof_capture.py [PORT] [--csv out.csv] [--raw]
    python prof_capture.py COM3
    python prof_capture.py COM3 --csv run1.csv

Ctrl+C to stop and print the session total.
"""

import sys
import time

try:
    import serial
except ImportError:
    serial = None


# Column -> what it measures. Order here is the order printed.
FIELDS = [
    ("P", "tick period",      "target 10000"),
    ("B", "BODY (consumed)",  "of the 10000 budget"),
    ("F", "free left",        "10000 - B"),
    ("R", "rx + parse",       "handleTelemetryRX"),
    ("S", "settings push",    "applySettingsTask"),
    ("U", "AX-12 bus",        "sync-write / poll"),
    ("T", "telemetry",        "Serial3 build+write"),
    ("L", "servo rd latency", "request->reply, spans ticks"),
    ("X", "profiler cost",    "instrumentation tax"),
]
KEYS = [k for k, _, _ in FIELDS]


def parse(line):
    """'P10001 B41 F9959 R8 ...' -> dict, or None if it is not a data row."""
    if not line or line.startswith("#"):
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


def pct(sorted_vals, q):
    if not sorted_vals:
        return 0
    i = min(int(len(sorted_vals) * q), len(sorted_vals) - 1)
    return sorted_vals[i]


def main():
    if serial is None:
        sys.exit("pyserial not installed. Run: pip install pyserial")

    args = [a for a in sys.argv[1:]]
    port = "COM3"
    csv_path = None
    raw = False
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--csv":
            i += 1
            csv_path = args[i]
        elif a == "--raw":
            raw = True
        else:
            port = a
        i += 1

    print(f"Opening {port} @ 115200 (ax12_control tick profiler)")
    ser = serial.Serial(port, 115200, timeout=1.0)

    csv_f = None
    if csv_path:
        csv_f = open(csv_path, "w", newline="")
        csv_f.write(",".join(KEYS) + ",ovr,err\n")
        print(f"Logging raw rows to {csv_path}")

    window = []          # rows in the current 1 s window
    total_rows = 0
    total_ovr = 0
    total_err = 0
    worst_body = 0
    t_win = time.time()
    buf = b""

    print("\nOne summary per second. B is what matters: microseconds of the "
          "10000 us tick consumed.\n")

    try:
        while True:
            chunk = ser.read(ser.in_waiting or 1)
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    rawline, buf = buf.split(b"\n", 1)
                    line = rawline.decode("utf-8", "ignore").strip()
                    if raw and line:
                        print(line)
                    if line.startswith("#"):
                        if not raw:
                            print(line)
                        continue
                    row = parse(line)
                    if row is None:
                        continue
                    window.append(row)
                    if csv_f:
                        csv_f.write(",".join(str(row.get(k, 0)) for k in KEYS)
                                    + f",{row.get('!OVR', 0)},{row.get('!ERR', 0)}\n")

            now = time.time()
            if now - t_win >= 1.0 and window:
                n = len(window)
                total_rows += n
                ovr = sum(r.get("!OVR", 0) for r in window)
                err = sum(r.get("!ERR", 0) for r in window)
                total_ovr += ovr
                total_err += err

                print(f"--- {n} ticks/s "
                      f"{'(EXPECTED 100)' if abs(n - 100) > 3 else ''} "
                      f"overruns {ovr}  errors {err}")
                for k, label, note in FIELDS:
                    vals = sorted(r.get(k, 0) for r in window)
                    if not any(vals):
                        continue
                    avg = sum(vals) / len(vals)
                    line = (f"  {k}  {label:<17} "
                            f"min {vals[0]:>7}  avg {avg:>8.0f}  "
                            f"p99 {pct(vals, 0.99):>7}  max {vals[-1]:>7}")
                    if k == "B":
                        worst_body = max(worst_body, vals[-1])
                        line += f"   = {100.0 * avg / 10000:.1f}% of budget"
                    print(line)
                print()
                window = []
                t_win = now

    except KeyboardInterrupt:
        print(f"\nSession: {total_rows} ticks, {total_ovr} overruns, "
              f"{total_err} error rows, worst body {worst_body} us "
              f"({100.0 * worst_body / 10000:.1f}% of the 10 ms budget)")
    finally:
        if csv_f:
            csv_f.close()
        ser.close()


if __name__ == "__main__":
    main()
