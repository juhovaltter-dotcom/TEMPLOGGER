"""Convert the logger's log.bin (+ sync.bin) into a CSV with real timestamps.

    py dump_log.py log.bin sync.bin > readings.csv
    py dump_log.py log.bin sync.bin --tz Europe/Helsinki -o readings.csv

Get the files from the Pico (this stops the running program, so reset it afterwards):
    py -m mpremote connect COM4 cp :log.bin log.bin
    py -m mpremote connect COM4 cp :sync.bin sync.bin
    py -m mpremote connect COM4 reset

Time resolution: each record has (boot_id, seconds since boot). Anchors
(boot_id, uptime, unix) taken while the phone was connected map uptime to real
time for the whole boot, including records logged before the sync. With several
anchors in one boot the time is interpolated between them (corrects crystal drift).
time_quality: exact = between two anchors, anchor = extrapolated from one anchor,
approx = boot never synced; placed right after the previous boot (a power cut
of unknown length may be missing), blank = cannot be placed.
"""
import argparse
import csv
import struct
import sys
from datetime import datetime, timezone

HDR_SIZE, REC_SIZE, ANCHOR_SIZE = 16, 16, 10
T_MISSING, U_MISSING = 0x7FFF, 0xFFFF


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def read_log(data):
    if data[:4] != b"TLG1" or data[4] != 1 or data[5] != REC_SIZE:
        raise SystemExit("not a TLG1 log file (wrong header)")
    recs, bad = [], 0
    body = data[HDR_SIZE:]
    for i in range(len(body) // REC_SIZE):
        raw = body[i * REC_SIZE:(i + 1) * REC_SIZE]
        if crc8(raw[:15]) != raw[15]:
            bad += 1
            continue
        up, boot, t_out, t_in, rh, pres, flags, _ = struct.unpack("<IHhhHHBB", raw)
        recs.append(dict(index=i, uptime=up, boot=boot, t_out=t_out, t_in=t_in,
                         rh=rh, pres=pres, flags=flags))
    return recs, bad


def read_anchors(data):
    out = {}
    for i in range(len(data) // ANCHOR_SIZE):
        boot, up, unix = struct.unpack_from("<HII", data, i * ANCHOR_SIZE)
        out.setdefault(boot, []).append((up, unix))
    for lst in out.values():
        lst.sort()
    return out


def resolve(up, anchors):
    """Return (unix, quality) for an uptime inside one boot, or (None, '')."""
    if not anchors:
        return None, ""
    if len(anchors) >= 2:
        for (u1, t1), (u2, t2) in zip(anchors, anchors[1:]):
            if u1 <= up <= u2 and u2 > u1:
                return t1 + (up - u1) * (t2 - t1) / (u2 - u1), "exact"
    # outside the anchors (or only one): extend with 1 s per second from the nearest one
    u, t = min(anchors, key=lambda a: abs(a[0] - up))
    return t + (up - u), "anchor"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("sync", nargs="?")
    ap.add_argument("--tz", default="Europe/Helsinki", help="local time zone for the local column")
    ap.add_argument("-o", "--out")
    a = ap.parse_args()

    recs, bad = read_log(open(a.log, "rb").read())
    anchors = {}
    if a.sync:
        try:
            anchors = read_anchors(open(a.sync, "rb").read())
        except FileNotFoundError:
            print("no sync file: times will be blank", file=sys.stderr)

    tz = None
    try:
        from zoneinfo import ZoneInfo
        tz = ZoneInfo(a.tz)
    except Exception:
        print("time zone %s unavailable (pip install tzdata on Windows); local column left blank"
              % a.tz, file=sys.stderr)

    # resolve times, boot by boot in order of first appearance
    boots = []
    for r in recs:
        if r["boot"] not in boots:
            boots.append(r["boot"])
    prev_end = None
    for b in boots:
        rs = [r for r in recs if r["boot"] == b]
        for r in rs:
            r["unix"], r["quality"] = resolve(r["uptime"], anchors.get(b, []))
        if not any(r["unix"] is not None for r in rs) and prev_end is not None:
            first = rs[0]["uptime"]
            for r in rs:
                r["unix"], r["quality"] = prev_end + (r["uptime"] - first), "approx"
        known = [r["unix"] for r in rs if r["unix"] is not None]
        if known:
            prev_end = max(known)

    out = open(a.out, "w", newline="") if a.out else sys.stdout
    w = csv.writer(out)
    w.writerow(["time_utc", "time_local", "unix", "boot_id", "uptime_s", "outdoor_C",
                "indoor_C", "humidity_pct", "pressure_hPa", "flags", "time_quality"])
    for r in recs:
        u = r["unix"]
        utc = loc = ""
        if u is not None:
            dt = datetime.fromtimestamp(u, timezone.utc)
            utc = dt.strftime("%Y-%m-%d %H:%M:%S")
            if tz:
                loc = dt.astimezone(tz).strftime("%Y-%m-%d %H:%M:%S")
        w.writerow([
            utc, loc, "" if u is None else round(u),
            r["boot"], r["uptime"],
            "" if r["t_out"] == T_MISSING else r["t_out"] / 100,
            "" if r["t_in"] == T_MISSING else r["t_in"] / 100,
            "" if r["rh"] == U_MISSING else r["rh"] / 100,
            "" if r["pres"] == U_MISSING else r["pres"] / 10,
            r["flags"], r["quality"]])
    if bad:
        print("warning: %d records skipped (bad checksum)" % bad, file=sys.stderr)
    print("%d records, %d boot sessions" % (len(recs), len(boots)), file=sys.stderr)


if __name__ == "__main__":
    main()
