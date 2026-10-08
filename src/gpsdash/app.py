"""gpsdash - realtime GNSS dashboard for the Sierra EM7455 in the Panasonic CF-20.

Reads NMEA straight from the modem's NMEA port (no gpsd / ModemManager needed,
works without a SIM), starts the GPS engine with $GPS_START and shows everything
the receiver reports: fix, DOPs, per-satellite signal, sky plot, scatter/track,
timing, and sentence statistics.

It also polls the modem's AT port for engine status (satellites being tracked
before their position is known, session and XTRA state), and can inject UTC time
plus gpsOneXTRA assistance data over QMI (needs root for mbim-proxy; uses sudo).

Keys: q quit | s sky plot <-> position scatter | p pause raw log | r reset stats
      x download + inject XTRA assistance data and UTC time (skipped if loaded data is valid > 1 day)
      X same, but always refresh

--serve [HOST:]PORT re-serves the raw NMEA over TCP so other programs can share the
receiver while gpsdash owns the port, e.g.  gpsd -N -n tcp://127.0.0.1:10110
"""
import argparse
import calendar
import hashlib
import shutil
import collections
import curses
import datetime as dt
import json
import locale
import math
import os
import queue
import select
import socket
import statistics
import subprocess
import sys
import termios
import threading
import time
import re
import struct
import urllib.request

__version__ = "1.0.0"

# How to re-run ourselves (as root, via sudo) for the QMI/AT helper modes: as a module, so it
# works the same from a pipx venv, a checkout (python -m gpsdash) or an installed wheel.
SELF = [sys.executable, "-m", "gpsdash"]

DEFAULT_DEV = "/dev/ttyUSB2"
DEFAULT_AT = "/dev/ttyUSB3"
DEFAULT_QMI = "/dev/cdc-wdm3"
# The izatcloud servers each serve independently generated files (they ignore the file name), and
# this modem rejects some of them on the final part with invalid-parameter -- which can also wipe the
# XTRA data already loaded. So: try candidates in order (xtrapath3 has been the reliable one), verify
# with AT!GPSXTRASTATUS?, remember rejected files, and fall back to the last file that was accepted.
XTRA_CANDIDATES = [f"http://xtrapath{n}.izatcloud.net/{f}" for f in ("xtra3grc.bin", "xtra2.bin") for n in (3, 2, 1)]
XTRA_DIR = os.path.expanduser("~/.cache/gpsdash")
XTRA_GOOD = os.path.join(XTRA_DIR, "xtra-good.bin")
XTRA_REJECTED = os.path.join(XTRA_DIR, "xtra-rejected.json")
XTRA_MAX_AGE = 6 * 3600
XTRA_MIN_REMAINING = 24 * 3600
UERE_M = 5.0  # rough user-equivalent range error for civilian single-frequency

SYS_NAME = {"GPS": "GPS", "GLO": "GLONASS", "GAL": "Galileo", "BDS": "BeiDou", "QZS": "QZSS", "SBS": "SBAS"}
SYS_CHAR = {"GPS": "G", "GLO": "R", "GAL": "E", "BDS": "C", "QZS": "J", "SBS": "S"}
TALKER_SYS = {"GP": "GPS", "GL": "GLO", "GA": "GAL", "GB": "BDS", "BD": "BDS", "GQ": "QZS", "PQ": "QZS"}
GSA_SYSID = {"1": "GPS", "2": "GLO", "3": "GAL", "4": "BDS", "5": "QZS"}
GGA_QUALITY = {0: "NO FIX", 1: "GPS SPS", 2: "DGPS", 3: "PPS", 4: "RTK FIXED", 5: "RTK FLOAT",
               6: "DEAD RECKONING", 7: "MANUAL", 8: "SIMULATION"}
MODE_CHAR = {"N": "none", "A": "auto", "D": "diff", "E": "estimated", "F": "RTK float", "M": "manual",
             "P": "precise", "R": "RTK", "S": "sim"}
WINDS = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
ARROWS = "↑↗→↘↓↙←↖"
SPARK = "▁▂▃▄▅▆▇█"
BLOCKS = " ▏▎▍▌▋▊▉█"
SPINNER = "⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏"


def sys_for(talker, prn):
    t = TALKER_SYS.get(talker)
    if t and t != "GPS":
        return t
    if 1 <= prn <= 32:
        return "GPS"
    if 33 <= prn <= 64:
        return "SBS"
    if 65 <= prn <= 96:
        return "GLO"
    if 193 <= prn <= 200:
        return "QZS"
    if 201 <= prn <= 264 or 401 <= prn <= 437:
        return "BDS"
    if 301 <= prn <= 336:
        return "GAL"
    return t or "GPS"


def fnum(s, conv=float):
    try:
        return conv(s) if s not in ("", None) else None
    except ValueError:
        return None


def parse_latlon(v, hemi, deg_digits):
    if not v or not hemi:
        return None
    try:
        d = int(v[:deg_digits]) + float(v[deg_digits:]) / 60.0
    except ValueError:
        return None
    return -d if hemi in "SW" else d


def dms(deg, pos, neg):
    h = pos if deg >= 0 else neg
    deg = abs(deg)
    d = int(deg)
    m = int((deg - d) * 60)
    s = (deg - d - m / 60) * 3600
    return f"{d}°{m:02d}'{s:05.2f}\"{h}"


def maidenhead(lat, lon):
    lon += 180
    lat += 90
    return (chr(65 + int(lon / 20)) + chr(65 + int(lat / 10)) + str(int(lon % 20 / 2)) + str(int(lat % 10))
            + chr(97 + int((lon % 2) * 12)) + chr(97 + int((lat % 1) * 24)))


def haversine(lat1, lon1, lat2, lon2):
    r = 6371008.8
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = p2 - p1, math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def dop_word(d):
    if d is None:
        return ""
    for lim, w in ((1, "ideal"), (2, "excellent"), (5, "good"), (10, "moderate"), (20, "fair")):
        if d < lim:
            return w
    return "poor"


def sparkline(vals, width):
    vals = [v for v in list(vals)[-width:]]
    nums = [v for v in vals if v is not None]
    if not nums:
        return ""
    lo, hi = min(nums), max(nums)
    if hi == lo:
        return "".join(" " if v is None else SPARK[3] for v in vals)
    return "".join(" " if v is None else SPARK[min(7, int((v - lo) / (hi - lo) * 7.999))] for v in vals)


def hms(sec):
    sec = int(sec)
    return f"{sec // 3600:02d}:{sec % 3600 // 60:02d}:{sec % 60:02d}"


class State:
    def __init__(self):
        self.start = time.monotonic()
        self.reset_stats()
        self.raw = collections.deque(maxlen=400)
        self.counts = collections.Counter()
        self.rx_times = collections.defaultdict(lambda: collections.deque(maxlen=200))
        self.byte_log = collections.deque(maxlen=400)
        self.bytes = 0
        self.bad_cksum = 0
        self.last_rx = None
        self.fix_quality = 0
        self.gsa_mode = None
        self.rmc_status = None
        self.rmc_mode = None
        self.gns_modes = None
        self.lat = self.lon = self.alt = self.geoid = None
        self.gga_used = None
        self.gns_used = None
        self.hdop = self.vdop = self.pdop = None
        self.speed_kn = None
        self.course = None
        self.magvar = None
        self.utc_time = None
        self.utc_date = None
        self.clock_offset = None
        self.dgps_age = None
        self.gst = None
        self.sats = {}
        self.gsv_pending = {}
        self.gsv_last = {}
        self.gsa_used = {}
        self.ttff = None
        self.had_fix = False
        self.last_fix_t = None
        self.fix_losses = 0
        self.epoch_times = collections.deque(maxlen=60)
        self.modem = {}
        self.at = {}            # AT-port engine status, filled by at_poller
        self.at_xtra_req = False  # set to make at_poller re-read AT!GPSXTRASTATUS? right away
        self.at_path = DEFAULT_AT
        self.at_sats = {}       # (sys, prn) -> snr, from AT!GPSSATINFO?
        self.at_sats_t = None
        self.xtra = {"state": "idle", "msg": "press x to inject", "busy": False}

    def reset_stats(self):
        self.track = collections.deque(maxlen=3600)
        self.dist_m = 0.0
        self.max_speed_kmh = 0.0
        self.hist_used = collections.deque(maxlen=120)
        self.hist_hdop = collections.deque(maxlen=120)
        self.hist_snr = collections.deque(maxlen=120)
        self.stats_since = time.monotonic()

    # ---- derived ----
    @property
    def has_fix(self):
        return self.fix_quality > 0 and self.lat is not None

    def used_keys(self):
        now = time.monotonic()
        out = set()
        for keys, t in self.gsa_used.values():
            if now - t < 5:
                out |= keys
        return out

    def sat_list(self):
        now = time.monotonic()
        used = self.used_keys()
        sats = []
        for talker, last in list(self.gsv_last.items()):
            if now - last > 10:
                for k in [k for k, s in self.sats.items() if s["talker"] == talker]:
                    del self.sats[k]
                del self.gsv_last[talker]
        at_fresh = self.at_sats_t is not None and now - self.at_sats_t < 10
        for k, s in self.sats.items():
            d = dict(s, used=k in used, at=False)
            if at_fresh and not d["snr"] and self.at_sats.get(k):
                d["snr"], d["at"] = self.at_sats[k], True
            sats.append(d)
        if at_fresh:
            # tracked by the engine but not yet reported in GSV (no ephemeris/position yet)
            for k, snr in self.at_sats.items():
                if k not in self.sats and snr:
                    sats.append(dict(talker="AT", sys=k[0], prn=k[1], el=None, az=None, snr=snr, used=False, at=True))
        sats.sort(key=lambda s: (not s["used"], -(s["snr"] or 0), s["sys"], s["prn"]))
        return sats

    def rate(self, tag, window=5.0):
        now = time.monotonic()
        return sum(1 for t in self.rx_times[tag] if now - t <= window) / window

    def byte_rate(self, window=5.0):
        now = time.monotonic()
        return sum(n for t, n in self.byte_log if now - t <= window) / window

    def enu(self, n=None):
        pts = list(self.track)[-n:] if n else list(self.track)
        if len(pts) < 2:
            return None
        lat0 = statistics.fmean(p[1] for p in pts)
        lon0 = statistics.fmean(p[2] for p in pts)
        alts = [p[3] for p in pts if p[3] is not None]
        alt0 = statistics.fmean(alts) if alts else 0
        k = math.cos(math.radians(lat0))
        return [((p[2] - lon0) * 111320 * k, (p[1] - lat0) * 110574, (p[3] - alt0) if p[3] is not None else 0)
                for p in pts]

    # ---- parsing ----
    def feed(self, line, now):
        self.raw.append((now, line))
        self.last_rx = now
        self.bytes += len(line) + 2
        self.byte_log.append((now, len(line) + 2))
        if not line.startswith(("$", "!")):
            return
        body, star, ck = line[1:].partition("*")
        if star:
            calc = 0
            for c in body:
                calc ^= ord(c)
            try:
                ok = int(ck[:2], 16) == calc
            except ValueError:
                ok = False
            if not ok:
                self.bad_cksum += 1
                return
        f = body.split(",")
        tag = f[0]
        self.counts[tag] += 1
        self.rx_times[tag].append(now)
        if tag.startswith("P"):
            return
        talker, typ = tag[:2], tag[2:]
        h = getattr(self, "on_" + typ, None)
        if h:
            try:
                h(talker, f, now)
            except (IndexError, ValueError):
                pass

    def _time(self, v):
        if v:
            self.utc_time = v

    def on_GGA(self, talker, f, now):
        self._time(f[1])
        q = fnum(f[6], int) or 0
        prev_fix = self.has_fix
        self.fix_quality = q
        self.gga_used = fnum(f[7], int)
        self.hdop = fnum(f[8]) if f[8] else self.hdop
        self.dgps_age = fnum(f[13]) if len(f) > 13 else None
        self.epoch_times.append(now)
        self.hist_used.append(self.gga_used or 0)
        if q > 0:
            lat = parse_latlon(f[2], f[3], 2)
            lon = parse_latlon(f[4], f[5], 3)
            if lat is None or lon is None:
                return
            self.lat, self.lon = lat, lon
            self.alt = fnum(f[9])
            self.geoid = fnum(f[11])
            self.last_fix_t = now
            if self.ttff is None:
                self.ttff = now - self.start
            if self.track and self.speed_kn is not None and self.speed_kn * 1.852 > 1.5:
                p = self.track[-1]
                self.dist_m += haversine(p[1], p[2], lat, lon)
            self.track.append((now, lat, lon, self.alt))
            self.hist_hdop.append(self.hdop)
            self.had_fix = True
        else:
            self.hist_hdop.append(None)
            if prev_fix:
                self.fix_losses += 1

    def on_RMC(self, talker, f, now):
        self._time(f[1])
        self.rmc_status = f[2] or None
        if f[2] == "A":
            self.speed_kn = fnum(f[7])
            self.course = fnum(f[8])
            if self.speed_kn is not None:
                self.max_speed_kmh = max(self.max_speed_kmh, self.speed_kn * 1.852)
        mv = fnum(f[10])
        self.magvar = (-mv if f[11] == "W" else mv) if mv is not None else None
        self.rmc_mode = f[12] if len(f) > 12 and f[12] else None
        if f[9] and f[1]:
            self.utc_date = f[9]
            try:
                d = dt.datetime.strptime(f[9] + f[1].split(".")[0], "%d%m%y%H%M%S").replace(tzinfo=dt.timezone.utc)
                frac = float("0." + f[1].split(".")[1]) if "." in f[1] else 0.0
                self.clock_offset = time.time() - (d.timestamp() + frac)
            except ValueError:
                pass

    def on_VTG(self, talker, f, now):
        if f[1] and self.rmc_status == "A":
            self.course = fnum(f[1])
        if f[5] and self.rmc_status == "A":
            self.speed_kn = fnum(f[5])

    def on_GSA(self, talker, f, now):
        self.gsa_mode = fnum(f[2], int)
        sysid = GSA_SYSID.get(f[18]) if len(f) > 18 else None
        keys = set()
        for v in f[3:15]:
            prn = fnum(v, int)
            if prn:
                keys.add((sysid or sys_for(talker, prn), prn))
        # one GSA per constellation per epoch; key by talker+constellations it describes
        self.gsa_used[(talker, sysid or tuple(sorted({s for s, _ in keys})))] = (keys, now)
        self.pdop = fnum(f[15]) if f[15] else self.pdop
        self.hdop = fnum(f[16]) if f[16] else self.hdop
        v = f[17].split("*")[0] if len(f) > 17 else ""
        self.vdop = fnum(v) if v else self.vdop

    def on_GSV(self, talker, f, now):
        total, num = fnum(f[1], int), fnum(f[2], int)
        if num == 1:
            self.gsv_pending[talker] = {}
        pend = self.gsv_pending.setdefault(talker, {})
        groups = (len(f) - 4) // 4
        for i in range(groups):
            prn = fnum(f[4 + i * 4], int)
            if not prn:
                continue
            sysn = sys_for(talker, prn)
            # Qualcomm engines report el/az/SNR with decimals ("25.3,173.0,23.0")
            el, az, snr = (fnum(v) for v in f[5 + i * 4:8 + i * 4])
            pend[(sysn, prn)] = dict(talker=talker, sys=sysn, prn=prn, el=None if el is None else round(el),
                                     az=None if az is None else round(az), snr=None if snr is None else round(snr))
        if num == total:
            for k in [k for k, s in self.sats.items() if s["talker"] == talker]:
                del self.sats[k]
            self.sats.update(pend)
            self.gsv_last[talker] = now
            snrs = [s["snr"] for k, s in self.sats.items() if s["snr"] and k in self.used_keys()]
            self.hist_snr.append(statistics.fmean(snrs) if snrs else None)

    def on_GNS(self, talker, f, now):
        self._time(f[1])
        self.gns_modes = f[6] or None
        self.gns_used = fnum(f[7], int)

    def on_GST(self, talker, f, now):
        self.gst = dict(rms=fnum(f[2]), major=fnum(f[3]), minor=fnum(f[4]), orient=fnum(f[5]),
                        lat=fnum(f[6]), lon=fnum(f[7]), alt=fnum(f[8].split("*")[0]) if len(f) > 8 else None)

    def on_ZDA(self, talker, f, now):
        self._time(f[1])


# ---------------------------------------------------------------- sources

def open_serial(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0                                    # iflag
    attrs[1] = 0                                    # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0                                    # lflag: raw
    attrs[4] = attrs[5] = termios.B115200
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd


def serial_reader(fd, q, log, fan=None):
    buf = b""
    while True:
        try:
            chunk = os.read(fd, 4096)
        except OSError as e:
            q.put(("error", str(e)))
            return
        if not chunk:
            time.sleep(0.05)
            continue
        buf += chunk
        *lines, buf = buf.split(b"\n")
        for ln in lines:
            s = ln.decode("ascii", "replace").strip()
            if s:
                if log:
                    log.write(s + "\n")
                    log.flush()
                if fan:
                    fan.send(s)
                q.put(("line", s))


def replay_reader(path, q, speed, fan=None):
    last_t = None
    while True:
        with open(path) as fh:
            for ln in fh:
                s = ln.strip()
                if not s:
                    continue
                f = s.split(",")
                if len(f) > 1 and f[0][3:] in ("GGA", "RMC", "GNS") and f[1] and f[1] != last_t:
                    if last_t is not None:
                        time.sleep(1.0 / speed)
                    last_t = f[1]
                if fan:
                    fan.send(s)
                q.put(("line", s))


class NMEAFanout:
    """TCP server that copies every NMEA sentence to all connected clients (gpsd, map apps, nc)."""

    def __init__(self, spec):
        host, _, port = spec.rpartition(":")
        self.addr = (host or "127.0.0.1", int(port))
        self.srv = socket.create_server(self.addr, reuse_port=False)
        self.clients = []
        self.lock = threading.Lock()
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while True:
            c, _ = self.srv.accept()
            c.setblocking(False)
            with self.lock:
                self.clients.append(c)

    def send(self, line):
        data = (line + "\r\n").encode("ascii", "replace")
        with self.lock:
            for c in self.clients[:]:
                try:
                    c.sendall(data)
                except (BlockingIOError, OSError):   # gone or not keeping up: drop it
                    c.close()
                    self.clients.remove(c)

    @property
    def label(self):
        return f"tcp:{self.addr[1]}\u00d7{len(self.clients)}"


def modem_info(state):
    try:
        out = subprocess.run(["mmcli", "-m", "any", "-J"], capture_output=True, text=True, timeout=10).stdout
        m = json.loads(out)["modem"]
        state.modem = {"model": m["generic"].get("model", "?").split(" Qualcomm")[0],
                       "fw": m["generic"].get("revision", "?").split()[0],
                       "state": m["generic"].get("state", "?"),
                       "reason": m["generic"].get("state-failed-reason", "--")}
    except Exception:
        state.modem = {}


class ATPort:
    def __init__(self, path):
        self.fd = open_serial(path)
        # ModemManager shares this port: after select() says readable it may read the bytes first,
        # so a blocking read() could hang forever. Read non-blocking instead.
        os.set_blocking(self.fd, False)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self.cmd("ATE0")

    def cmd(self, c, timeout=3.0):
        os.write(self.fd, (c + "\r").encode())
        out, end = b"", time.monotonic() + timeout
        while time.monotonic() < end:
            r, _, _ = select.select([self.fd], [], [], 0.1)
            if r:
                try:
                    out += os.read(self.fd, 4096)
                except BlockingIOError:
                    continue
                if b"\r\nOK\r\n" in out or b"ERROR" in out:
                    break
        return out.decode("ascii", "replace")


def at_poller(state, path, interval):
    try:
        port = ATPort(path)
    except OSError as e:
        state.at = {"error": f"{path}: {e.strerror}"}
        return
    n = 0
    while True:
        try:
            at = dict(state.at)
            at.pop("error", None)
            txt = port.cmd("AT!GPSSTATUS?")
            m = re.search(r"Fix Session Status = (\w+)(?:,\s*FAILCODE = (\d+))?", txt)
            if m:
                at["session"], at["session_fail"] = m.group(1), m.group(2)
            m = re.search(r"Last Fix Status\s*= (\w+)(?:,\s*FAILCODE = (\d+))?", txt)
            if m:
                at["last_fix"], at["last_fail"] = m.group(1), m.group(2)
            m = re.search(r"TTFF \(sec\) = (\d+)", txt)
            at["ttff"] = int(m.group(1)) if m else None
            txt = port.cmd("AT!GPSSATINFO?")
            sats = {}
            for prn, el, az, snr in re.findall(r"SV:\s*(\d+)\s+ELEV:\s*(\d+)\s+AZI:\s*(\d+)\s+SNR:\s*(\d+)", txt):
                prn, snr = int(prn), int(snr)
                if prn != 255:
                    sats[(sys_for(None, prn), prn)] = snr
            at["unknown_sv"] = len(re.findall(r"SV:\s*255\b", txt))
            if "Satellites in view" in txt:
                state.at_sats, state.at_sats_t = sats, time.monotonic()
            if n % 5 == 0 or state.at_xtra_req:
                state.at_xtra_req = False
                txt = port.cmd("AT!GPSXTRASTATUS?")
                m = re.search(r"Xtra Time status\s*= (.+)", txt)
                at["xtra_time"] = m.group(1).strip() if m else None
                at["xtra_valid"], at["xtra_end"] = parse_xtra_status(txt)
                at["xtra_t"] = time.monotonic()
            at["t"] = time.monotonic()
            state.at = at
        except OSError as e:
            state.at = {"error": str(e)}
            return
        n += 1
        time.sleep(interval)


# ---- QMI LOC over mbim-proxy (root only). libqmi's GI bindings can't pass raw
# messages (GByteArray is marshalled as bytes), so the send goes through ctypes.

LOC_STATUS = {0: "success", 1: "general-failure", 2: "unsupported", 3: "invalid-parameter", 4: "engine-busy",
              5: "phone-offline", 6: "timeout", 7: "config-not-supported", 8: "insufficient-memory"}
QMI_LOC = 0x10
LOC_INJECT_PREDICTED_ORBITS = 0x0035
LOC_INJECT_UTC_TIME = 0x0038


def qmi_tlv(t, v):
    return struct.pack("<BH", t, len(v)) + v


def _qmi_tlvs(b):
    out, i = {}, 13  # QMUX header (6) + QMI service header (7)
    while i + 3 <= len(b):
        t, n = struct.unpack_from("<BH", b, i)
        out[t] = b[i + 3:i + 3 + n]
        i += 3 + n
    return struct.unpack_from("<H", b, 9)[0], out


class LocSession:
    def __init__(self, path):
        import ctypes as C
        import gi
        gi.require_version("Qmi", "1.0")
        from gi.repository import Gio, GLib, Qmi
        self.C, self.GLib, self.Qmi = C, GLib, Qmi

        class GByteArray(C.Structure):
            _fields_ = [("data", C.POINTER(C.c_uint8)), ("len", C.c_uint)]

        class GError(C.Structure):
            _fields_ = [("domain", C.c_uint32), ("code", C.c_int), ("message", C.c_char_p)]

        self.GError = GError
        self.glib = C.CDLL("libglib-2.0.so.0")
        self.lib = C.CDLL("libqmi-glib.so.5")
        self.glib.g_byte_array_new.restype = C.POINTER(GByteArray)
        self.glib.g_byte_array_append.argtypes = [C.POINTER(GByteArray), C.c_char_p, C.c_uint]
        self.CB = C.CFUNCTYPE(None, C.c_void_p, C.c_void_p, C.c_void_p)
        self.lib.qmi_device_command_full.argtypes = [C.c_void_p, C.POINTER(GByteArray), C.c_void_p, C.c_uint,
                                                     C.c_void_p, self.CB, C.c_void_p]
        self.lib.qmi_device_command_full_finish.argtypes = [C.c_void_p, C.c_void_p, C.POINTER(C.POINTER(GError))]
        self.lib.qmi_device_command_full_finish.restype = C.POINTER(GByteArray)
        C.pythonapi.PyCapsule_GetPointer.restype = C.c_void_p
        C.pythonapi.PyCapsule_GetPointer.argtypes = [C.py_object, C.c_char_p]
        self.loop = GLib.MainLoop()
        self.inds = []
        self.err = None

        def on_client(dev, res, _):
            try:
                self.client = dev.allocate_client_finish(res)
                self.cid = self.client.get_cid()
            except Exception as e:
                self.err = str(e)
            self.loop.quit()

        def on_open(dev, res, _):
            try:
                dev.open_finish(res)
            except Exception as e:
                self.err = str(e)
                return self.loop.quit()
            dev.allocate_client(Qmi.Service.LOC, Qmi.CID_NONE, 10, None, on_client, None)

        def on_new(_, res, __):
            try:
                self.dev = Qmi.Device.new_finish(res)
            except Exception as e:
                self.err = str(e)
                return self.loop.quit()
            self.dev.connect("indication", self._on_ind)
            self.dev.open(Qmi.DeviceOpenFlags.PROXY | Qmi.DeviceOpenFlags.MBIM, 15, None, on_open, None)

        Qmi.Device.new(Gio.File.new_for_path(path), None, on_new, None)
        self.loop.run()
        if self.err:
            raise RuntimeError(self.err)

    def _on_ind(self, dev, msg):
        b = bytes(msg)
        if len(b) > 6 and b[4] == QMI_LOC and b[5] in (self.cid, 0xFF):
            self.inds.append(_qmi_tlvs(b))

    def request(self, msgid, tlvs, timeout=5.0):
        """Send a LOC request; returns the LOC indication status (int), or a string on QMI-level errors."""
        C = self.C
        body = b"".join(tlvs)
        mux = struct.pack("<BBB", 0, QMI_LOC, self.cid) + struct.pack("<BHHH", 0, 0, msgid, len(body)) + body
        raw = b"\x01" + struct.pack("<H", len(mux) + 2) + mux
        ba = self.glib.g_byte_array_new()
        self.glib.g_byte_array_append(ba, raw, len(raw))
        st = {}

        def cb(src, res, ud):
            err = C.POINTER(self.GError)()
            out = self.lib.qmi_device_command_full_finish(src, res, C.byref(err))
            if out:
                st["resp"] = bytes(out.contents.data[:out.contents.len])
            else:
                st["err"] = err.contents.message.decode() if err else "unknown error"
            self.loop.quit()

        self._cb = self.CB(cb)
        self.inds = [i for i in self.inds if i[0] != msgid]
        self.lib.qmi_device_command_full(C.pythonapi.PyCapsule_GetPointer(self.dev.__gpointer__, None), ba, None, 10,
                                         None, self._cb, None)
        self.loop.run()
        if "err" in st:
            return st["err"]
        result, error = struct.unpack("<HH", _qmi_tlvs(st["resp"])[1][2])
        if result != 0:
            return f"qmi-error-{error}"
        end, ctx = time.monotonic() + timeout, self.loop.get_context()
        while time.monotonic() < end:
            for mid, tl in self.inds:
                if mid == msgid:
                    return struct.unpack("<I", tl[1])[0] if 1 in tl else "no-status"
            ctx.iteration(False)
            time.sleep(0.01)
        return "indication-timeout"

    def close(self):
        Qmi = self.Qmi
        self.dev.release_client(self.client, Qmi.DeviceReleaseClientFlags.RELEASE_CID, 5, None,
                                lambda d, r, u: self.loop.quit(), None)
        self.loop.run()
        self.dev.close_async(5, None, lambda d, r, u: self.loop.quit(), None)
        self.loop.run()


def xtra_helper(qmi_path, xtra_path, inject_time=True):
    """Root-side of 'x': inject UTC time, then XTRA data. Line protocol on stdout for the TUI."""
    def say(*a):
        print(*a, flush=True)
    try:
        s = LocSession(qmi_path)
    except Exception as e:
        say("error", f"QMI open failed: {e}")
        return 1
    try:
        if inject_time:
            r = s.request(LOC_INJECT_UTC_TIME, [qmi_tlv(0x01, struct.pack("<Q", int(time.time() * 1000))),
                                                qmi_tlv(0x02, struct.pack("<I", 500))])
            say("time", LOC_STATUS.get(r, r))
        if xtra_path:
            data = open(xtra_path, "rb").read()
            parts = [data[i:i + 1024] for i in range(0, len(data), 1024)]
            for n, p in enumerate(parts, 1):
                r = s.request(LOC_INJECT_PREDICTED_ORBITS, [
                    qmi_tlv(0x01, struct.pack("<I", len(data))), qmi_tlv(0x02, struct.pack("<H", len(parts))),
                    qmi_tlv(0x03, struct.pack("<H", n)), qmi_tlv(0x04, struct.pack("<H", len(p)) + p),
                    qmi_tlv(0x10, struct.pack("<I", 0))])  # format: XTRA
                say("part", n, len(parts), LOC_STATUS.get(r, r))
                if r != 0:
                    break
            say("xtra", LOC_STATUS.get(r, r))
    finally:
        s.close()
    return 0


def _load_rejected():
    try:
        with open(XTRA_REJECTED) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def _save_rejected(rej):
    try:
        with open(XTRA_REJECTED, "w") as fh:
            json.dump(rej, fh, indent=1)
    except OSError:
        pass


def parse_xtra_status(txt):
    """AT!GPSXTRASTATUS? text -> (start, end) strings and end as a UTC epoch (or None)"""
    m1 = re.search(r"Validity Start = (\d{4} \d\d \d\d) \d (\d\d:\d\d)", txt)
    m2 = re.search(r"Validity End\s*= (\d{4} \d\d \d\d) \d (\d\d:\d\d)", txt)
    if not (m1 and m2):
        return None, None
    v = (m1.group(1).replace(" ", "-") + " " + m1.group(2), m2.group(1).replace(" ", "-") + " " + m2.group(2))
    return v, calendar.timegm(time.strptime(v[1], "%Y-%m-%d %H:%M"))


def xtra_status_helper(at_path):
    """Root-side: read AT!GPSXTRASTATUS? (root can open the port even though ModemManager holds it
    with TIOCEXCL). Prints 'valid <end-epoch>' or 'error ...'."""
    try:
        port = ATPort(at_path)
        for _ in range(6):   # ModemManager shares the port and can swallow a reply
            v, end = parse_xtra_status(port.cmd("AT!GPSXTRASTATUS?"))
            if end is not None:
                print("valid", end, flush=True)
                return 0
            time.sleep(0.5)
        print("error no XTRA status", flush=True)
    except OSError as e:
        print("error", e, flush=True)
    return 1


def xtra_fetch(url):
    """Download (or reuse a cached copy of) one candidate. Returns path or None."""
    path = os.path.join(XTRA_DIR, url.split("//")[1].split(".")[0] + "-" + url.rsplit("/", 1)[1])
    if os.path.exists(path) and time.time() - os.path.getmtime(path) < XTRA_MAX_AGE:
        return path
    try:
        with urllib.request.urlopen(url, timeout=15) as r:
            data = r.read()
    except OSError:
        return None
    if len(data) < 1000:
        return None
    with open(path + ".tmp", "wb") as fh:
        fh.write(data)
    os.replace(path + ".tmp", path)
    return path


def _md5(path):
    with open(path, "rb") as fh:
        return hashlib.md5(fh.read()).hexdigest()


def xtra_remaining(state):
    """Seconds of XTRA validity left, or None if unknown. Uses the AT poller when it can open the
    port, otherwise asks the root helper."""
    if state.at.get("xtra_t") is not None:
        end = state.at.get("xtra_end")
        return None if end is None else end - time.time()
    try:
        out = subprocess.run(["sudo", "-n", *SELF, "--xtra-status",
                              "--at-device", state.at_path], capture_output=True, text=True, timeout=30).stdout
    except (OSError, subprocess.TimeoutExpired):
        return None
    f = out.split()
    if len(f) >= 2 and f[0] == "valid":
        end = int(f[1])
        state.xtra["valid_end"] = end
        return end - time.time()
    return None


def xtra_verify(state, timeout=10.0):
    """Re-read the XTRA status after an injection: True/False, or None if it can't be read."""
    if state.at.get("xtra_t") is not None:
        t0 = time.monotonic()
        state.at_xtra_req = True
        while time.monotonic() - t0 < timeout:
            if state.at.get("xtra_t", 0) > t0:
                break
            time.sleep(0.2)
    rem = xtra_remaining(state)
    return None if rem is None else rem > 0


def xtra_worker(state, qmi_path, force=False):
    x = state.xtra
    x.update(busy=True, state="running", msg="starting…")
    helper = ["sudo", "-n", *SELF, "--qmi", qmi_path, "--xtra-helper"]

    def inject(path, with_time):
        """run the root helper; returns the LOC status string of the XTRA injection (or an error)"""
        p = subprocess.Popen(helper + [path] + ([] if with_time else ["--helper-no-time"]),
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        res = None
        for line in p.stdout:
            f = line.split()
            if not f:
                continue
            if f[0] == "time":
                x["time"] = f[1]
            elif f[0] == "part":
                x["msg"] = f"{os.path.basename(path)}: part {f[1]}/{f[2]}"
            elif f[0] == "xtra":
                res = f[1]
            elif f[0] == "error":
                res = " ".join(f[1:])
        rc = p.wait()
        if res is None:
            res = (p.stderr.read().strip().splitlines() or [f"helper exit {rc}"])[-1][:80]
        return res

    try:
        os.makedirs(XTRA_DIR, exist_ok=True)
        rem = xtra_remaining(state)
        if not force and rem is not None and rem > XTRA_MIN_REMAINING:
            x.update(state="ok", msg=f"loaded XTRA still valid {rem / 86400:.1f} d (X to force)")
            return
        rejected = _load_rejected()
        with_time = True
        tried = 0
        for url in XTRA_CANDIDATES:
            host = url.split("//")[1].split(".")[0]
            x.update(msg=f"downloading {host}/{url.rsplit('/', 1)[1]}…")
            path = xtra_fetch(url)
            if not path:
                continue
            h = _md5(path)
            if h in rejected:
                continue
            tried += 1
            x.update(file=f"{host} {os.path.getsize(path) // 1024} KiB", msg=f"injecting {host}…")
            res = inject(path, with_time)
            with_time = False
            ok = res == "success" and xtra_verify(state) is not False
            if ok:
                shutil.copyfile(path, XTRA_GOOD)
                x.update(state="ok", msg=f"XTRA accepted from {host}", result=res)
                return
            rejected[h] = {"url": url, "result": res, "t": time.strftime("%Y-%m-%d %H:%M:%S")}
            _save_rejected(rejected)
        # nothing new was accepted: make sure a failed attempt didn't leave the modem without data
        # (a rejected file can wipe what was loaded, so restore after any attempt)
        if os.path.exists(XTRA_GOOD) and (tried or (xtra_remaining(state) or 0) <= 0):
            x.update(msg=("all candidates rejected" if tried else "no new candidates (all known-bad)")
                     + ", re-injecting last good file…")
            res = inject(XTRA_GOOD, with_time)
            if res == "success" and xtra_verify(state) is not False:
                x.update(state="fail", msg="no new XTRA accepted; restored last good file")
                return
        x.update(state="fail", msg="no XTRA accepted" + ("" if tried else " (download failed / all known-bad)"))
    except Exception as e:
        x.update(state="fail", msg=str(e)[:80])
    finally:
        x["busy"] = False
        x["t"] = time.monotonic()


def sudo_ready():
    return subprocess.run(["sudo", "-n", "true"], capture_output=True).returncode == 0


# ---------------------------------------------------------------- drawing

class UI:
    def __init__(self, scr, state, src_label):
        self.scr = scr
        self.st = state
        self.src = src_label
        self.view = "sky"
        self.paused = False
        self.frozen_raw = None
        self.error = None
        self.tick = 0
        self.qmi = None
        self.no_at = False
        self.fan = None
        curses.curs_set(0)
        curses.start_color()
        curses.use_default_colors()
        for i, c in enumerate([curses.COLOR_GREEN, curses.COLOR_YELLOW, curses.COLOR_RED, curses.COLOR_CYAN,
                               curses.COLOR_MAGENTA, curses.COLOR_BLUE, curses.COLOR_WHITE], 1):
            curses.init_pair(i, c, -1)
        curses.init_pair(8, curses.COLOR_BLACK, curses.COLOR_CYAN)
        curses.init_pair(9, curses.COLOR_BLACK, curses.COLOR_GREEN)
        curses.init_pair(10, curses.COLOR_BLACK, curses.COLOR_RED)
        curses.init_pair(11, curses.COLOR_BLACK, curses.COLOR_YELLOW)
        self.G, self.Y, self.R, self.C, self.M, self.B, self.W = (curses.color_pair(i) for i in range(1, 8))
        self.DIM = curses.A_DIM
        self.BOLD = curses.A_BOLD
        self.SYS_COLOR = {"GPS": self.G, "GLO": self.C, "GAL": self.M, "BDS": self.Y, "QZS": self.B, "SBS": self.W}

    def put(self, y, x, s, attr=0, maxw=None):
        h, w = self.scr.getmaxyx()
        if y < 0 or y >= h or x >= w:
            return
        lim = w - x
        if maxw is not None:
            lim = min(lim, maxw)
        if lim <= 0:
            return
        s = s[:lim]
        try:
            if y == h - 1 and x + len(s) >= w:
                # addstr into the last cell scrolls/errors; insert the final char instead
                self.scr.addstr(y, x, s[:-1], attr)
                self.scr.insstr(y, w - 1, s[-1], attr)
            else:
                self.scr.addstr(y, x, s, attr)
        except curses.error:
            pass

    def box(self, y, x, h, w, title):
        if h < 2 or w < 4:
            return
        a = self.B
        self.put(y, x, "╭" + "─" * (w - 2) + "╮", a)
        for i in range(1, h - 1):
            self.put(y + i, x, "│", a)
            self.put(y + i, x + w - 1, "│", a)
        self.put(y + h - 1, x, "╰" + "─" * (w - 2) + "╯", a)
        self.put(y, x + 2, f" {title} ", self.C | self.BOLD)

    def snr_color(self, snr):
        if not snr:
            return self.DIM
        return self.R if snr < 20 else self.Y if snr < 30 else self.G if snr < 40 else self.C | self.BOLD

    def kv(self, y, x, k, v, attr=0, kw=11, maxw=None):
        self.put(y, x, f"{k:<{kw}}", self.DIM)
        self.put(y, x + kw, v, attr, maxw=(maxw - kw) if maxw else None)

    # ---- panels ----
    def header(self, w):
        st = self.st
        now = time.monotonic()
        if st.has_fix:
            mode = {3: "3D", 2: "2D"}.get(st.gsa_mode, "")
            badge, battr = f" {mode} FIX ", curses.color_pair(9) | self.BOLD
        elif st.last_rx and now - st.last_rx < 3:
            badge, battr = f" {SPINNER[self.tick % len(SPINNER)]} SEARCHING ", curses.color_pair(11) | self.BOLD
        else:
            badge, battr = " NO DATA ", curses.color_pair(10) | self.BOLD
        self.put(0, 0, " " * w, curses.color_pair(8))
        left = f" GPSDASH  {st.modem.get('model', 'GNSS')}  {self.src} "
        if self.fan:
            left += f"\u2192 {self.fan.label} "
        self.put(0, 0, left, curses.color_pair(8) | self.BOLD)
        self.put(0, len(left) + 1, badge, battr)
        utc = st.utc_time or "--"
        if len(utc) >= 6:
            utc = f"{utc[0:2]}:{utc[2:4]}:{utc[4:]}"
        right = f" UTC {utc}  up {hms(now - st.start)}  [q]uit [s]ky [p]ause [r]eset [x]tra "
        self.put(0, max(len(left) + len(badge) + 2, w - len(right)), right, curses.color_pair(8))

    def fix_panel(self, y, x, h, w):
        st = self.st
        self.box(y, x, h, w, "FIX")
        iw = w - 4
        r = y + 1
        q = GGA_QUALITY.get(st.fix_quality, str(st.fix_quality))
        attr = self.G | self.BOLD if st.has_fix else self.Y | self.BOLD
        mode = {1: "no fix", 2: "2D", 3: "3D"}.get(st.gsa_mode, "?")
        self.kv(r, x + 2, "Status", f"{q}  ({mode})", attr, maxw=iw); r += 1
        gns = st.gns_modes or "--"
        gdesc = " ".join(f"{s}:{MODE_CHAR.get(c, c)}" for s, c in zip(("GPS", "GLO", "GAL", "BDS"), gns)) \
            if st.gns_modes else "--"
        self.kv(r, x + 2, "RMC/Mode", f"{st.rmc_status or '-'} / {MODE_CHAR.get(st.rmc_mode, st.rmc_mode or '-')}",
                maxw=iw); r += 1
        self.kv(r, x + 2, "GNS modes", gdesc, maxw=iw); r += 1
        stale = st.last_fix_t is None or time.monotonic() - st.last_fix_t > 3
        pattr = self.DIM if stale else self.W | self.BOLD
        if st.lat is not None:
            self.kv(r, x + 2, "Latitude", f"{st.lat:+.7f}°  {dms(st.lat, 'N', 'S')}", pattr, maxw=iw); r += 1
            self.kv(r, x + 2, "Longitude", f"{st.lon:+.7f}°  {dms(st.lon, 'E', 'W')}", pattr, maxw=iw); r += 1
            alt = f"{st.alt:.1f} m MSL" if st.alt is not None else "--"
            if st.alt is not None and st.geoid is not None:
                alt += f"  {st.alt + st.geoid:.1f} m ellip"
            self.kv(r, x + 2, "Altitude", alt, pattr, maxw=iw); r += 1
            self.kv(r, x + 2, "Geoid sep", f"{st.geoid:+.1f} m" if st.geoid is not None else "--", maxw=iw); r += 1
            self.kv(r, x + 2, "Grid", f"{maidenhead(st.lat, st.lon)}  (Maidenhead)", self.M, maxw=iw); r += 1
            self.kv(r, x + 2, "OSM", f"osm.org/?mlat={st.lat:.6f}&mlon={st.lon:.6f}", self.B, maxw=iw); r += 1
        else:
            for k in ("Latitude", "Longitude", "Altitude", "Geoid sep", "Grid", "OSM"):
                self.kv(r, x + 2, k, "--", self.DIM, maxw=iw); r += 1
        sats = st.sat_list()
        tracking = sum(1 for s in sats if s["snr"])
        used = st.gga_used if st.gga_used is not None else len(st.used_keys())
        self.kv(r, x + 2, "Satellites", f"{used} used / {tracking} tracked / {len(sats)} in view",
                self.G if used and used >= 4 else self.Y, maxw=iw); r += 1
        per = collections.Counter(s["sys"] for s in sats)
        peru = collections.Counter(s["sys"] for s in sats if s["used"])
        line = "  ".join(f"{SYS_CHAR[k]}{peru[k]}/{per[k]}" for k in SYS_NAME if per[k])
        self.kv(r, x + 2, "  by const", line or "--", maxw=iw)

    def accuracy_panel(self, y, x, h, w):
        st = self.st
        self.box(y, x, h, w, "ACCURACY")
        iw = w - 4
        r = y + 1
        def d(v):
            return f"{v:.2f}" if v is not None else "--"
        self.kv(r, x + 2, "PDOP", f"{d(st.pdop):>6}  {dop_word(st.pdop)}", maxw=iw); r += 1
        self.kv(r, x + 2, "HDOP", f"{d(st.hdop):>6}  {dop_word(st.hdop)}", maxw=iw); r += 1
        self.kv(r, x + 2, "VDOP", f"{d(st.vdop):>6}  {dop_word(st.vdop)}", maxw=iw); r += 1
        if st.hdop and st.has_fix:
            self.kv(r, x + 2, "Est error", f"±{st.hdop * UERE_M:.1f} m H  ±{(st.vdop or 0) * UERE_M:.1f} m V",
                    self.Y, maxw=iw)
        else:
            self.kv(r, x + 2, "Est error", "--", self.DIM, maxw=iw)
        r += 1
        if st.gst:
            g = st.gst
            self.kv(r, x + 2, "GST σ", f"lat {d(g['lat'])} lon {d(g['lon'])} alt {d(g['alt'])} m", maxw=iw); r += 1
        pts = st.enu(300)
        if pts and len(pts) >= 5:
            rad = sorted(math.hypot(e, n) for e, n, _ in pts)
            sN = statistics.pstdev(p[1] for p in pts)
            sE = statistics.pstdev(p[0] for p in pts)
            sU = statistics.pstdev(p[2] for p in pts)
            cep50 = rad[len(rad) // 2]
            cep95 = rad[min(len(rad) - 1, int(len(rad) * 0.95))]
            self.kv(r, x + 2, "Scatter σ", f"N {sN:.1f}  E {sE:.1f}  U {sU:.1f} m", maxw=iw); r += 1
            self.kv(r, x + 2, "CEP50/95", f"{cep50:.1f} / {cep95:.1f} m   2DRMS {2 * math.hypot(sN, sE):.1f} m",
                    maxw=iw); r += 1
            self.kv(r, x + 2, "  samples", f"{len(pts)} fixes", self.DIM, maxw=iw); r += 1
        else:
            self.kv(r, x + 2, "Scatter σ", "need ≥5 fixes", self.DIM, maxw=iw); r += 1
        sw = max(0, iw - 18)
        for label, hist, attr, fmt in (("Used hist", st.hist_used, self.G, "{:.0f}"),
                                       ("HDOP hist", st.hist_hdop, self.Y, "{:.2f}"),
                                       ("SNR hist", st.hist_snr, self.C, "{:.0f}dB")):
            if r >= y + h - 1:
                break
            cur = next((v for v in reversed(hist) if v is not None), None)
            self.kv(r, x + 2, label, sparkline(hist, sw), attr)
            self.put(r, x + 2 + 11 + sw + 1, fmt.format(cur) if cur is not None else "--", self.W)
            r += 1

    def motion_time_panel(self, y, x, h, w):
        st = self.st
        now = time.monotonic()
        self.box(y, x, h, w, "MOTION & TIME")
        iw = w - 4
        r = y + 1
        if st.speed_kn is not None:
            kmh = st.speed_kn * 1.852
            self.kv(r, x + 2, "Speed", f"{kmh:.1f} km/h  {kmh / 1.609344:.1f} mph  {st.speed_kn:.1f} kn",
                    self.W | self.BOLD, maxw=iw)
        else:
            self.kv(r, x + 2, "Speed", "--", self.DIM)
        r += 1
        if st.course is not None:
            c = st.course
            self.kv(r, x + 2, "Course", f"{c:5.1f}° {WINDS[int((c + 11.25) / 22.5) % 16]:<3} "
                    f"{ARROWS[int((c + 22.5) / 45) % 8]}", self.W | self.BOLD, maxw=iw)
        else:
            self.kv(r, x + 2, "Course", "--", self.DIM)
        r += 1
        self.kv(r, x + 2, "Distance", f"{st.dist_m / 1000:.3f} km   max {st.max_speed_kmh:.1f} km/h", maxw=iw); r += 1
        if st.magvar is not None:
            self.kv(r, x + 2, "Mag var", f"{st.magvar:+.1f}°", maxw=iw); r += 1
        r += 1
        if st.utc_time:
            t = st.utc_time
            date = st.utc_date
            ds = f"20{date[4:6]}-{date[2:4]}-{date[0:2]} " if date else ""
            self.kv(r, x + 2, "GNSS UTC", f"{ds}{t[0:2]}:{t[2:4]}:{t[4:]}", self.W | self.BOLD, maxw=iw)
        else:
            self.kv(r, x + 2, "GNSS UTC", "--", self.DIM)
        r += 1
        self.kv(r, x + 2, "System", dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M:%S.%f")[:-4],
                maxw=iw); r += 1
        if st.clock_offset is not None:
            off = st.clock_offset
            self.kv(r, x + 2, "Clk offset", f"{off:+.3f} s (sys − GNSS)",
                    self.G if abs(off) < 1 else self.R, maxw=iw)
        else:
            self.kv(r, x + 2, "Clk offset", "needs RMC date (valid fix)", self.DIM, maxw=iw)
        r += 1
        if st.ttff is not None:
            self.kv(r, x + 2, "TTFF", f"{st.ttff:.1f} s", self.G, maxw=iw)
        else:
            self.kv(r, x + 2, "TTFF", f"searching… {now - st.start:.0f} s", self.Y, maxw=iw)
        r += 1
        age = f"{now - st.last_fix_t:.1f} s ago" if st.last_fix_t else "never"
        self.kv(r, x + 2, "Last fix", f"{age}   losses {st.fix_losses}", maxw=iw); r += 1
        et = list(st.epoch_times)
        if len(et) >= 3:
            iv = [b - a for a, b in zip(et, et[1:])]
            self.kv(r, x + 2, "Epoch", f"{statistics.fmean(iv):.3f} s ± {statistics.pstdev(iv) * 1000:.0f} ms",
                    maxw=iw)
        else:
            self.kv(r, x + 2, "Epoch", "--", self.DIM)

    def sky_panel(self, y, x, h, w):
        self.box(y, x, h, w, "SKY PLOT  (elev rings 0/30/60°)")
        ih, iw = h - 2, w - 2
        cy, cx = y + 1 + ih // 2, x + 1 + iw // 2
        ry, rx = (ih - 1) / 2, (iw - 1) / 2
        rad = min(ry, rx / 2)
        ry, rx = rad, rad * 2
        for el in (0, 30, 60):
            f = (90 - el) / 90
            for a in range(0, 360, 4 if el == 0 else 8):
                yy = round(cy - f * ry * math.cos(math.radians(a)))
                xx = round(cx + f * rx * math.sin(math.radians(a)))
                self.put(yy, xx, "·", self.B | self.DIM)
        self.put(cy, cx, "+", self.B)
        self.put(round(cy - ry), cx, "N", self.C | self.BOLD)
        self.put(round(cy + ry), cx, "S", self.C | self.BOLD)
        self.put(cy, round(cx + rx) + 1, "E", self.C | self.BOLD)
        self.put(cy, round(cx - rx) - 1, "W", self.C | self.BOLD)
        nopos = 0
        sats = sorted(self.st.sat_list(), key=lambda s: s["used"])
        for s in sats:
            if s["el"] is None or s["az"] is None:
                nopos += 1
                continue
            f = (90 - max(0, s["el"])) / 90
            yy = round(cy - f * ry * math.cos(math.radians(s["az"])))
            xx = round(cx + f * rx * math.sin(math.radians(s["az"])))
            lbl = f"{SYS_CHAR[s['sys']]}{s['prn']}"
            attr = self.snr_color(s["snr"])
            if s["used"]:
                attr |= self.BOLD | curses.A_REVERSE
            self.put(yy, xx - len(lbl) // 2, lbl, attr)
        legend = "rev=used  "
        self.put(y + h - 2, x + 2, legend, self.DIM)
        if nopos:
            self.put(y + h - 2, x + 2 + len(legend), f"{nopos} w/o position", self.DIM)

    def scatter_panel(self, y, x, h, w):
        st = self.st
        pts = st.enu()
        ih, iw = h - 3, w - 2
        cy, cx = y + 1 + ih // 2, x + 1 + iw // 2
        if not pts:
            self.box(y, x, h, w, "POSITION SCATTER / TRACK")
            self.put(cy, cx - 8, "waiting for fix…", self.DIM)
            return
        ext = max(max(abs(e), abs(n)) for e, n, _ in pts) or 1.0
        nice = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000]
        scale = next((v for v in nice if v >= ext * 1.1), ext * 1.1)
        self.box(y, x, h, w, f"POSITION SCATTER / TRACK  (±{scale:g} m)")
        ry, rx = ih / 2 - 0.5, iw / 2 - 1
        for i in range(-int(rx), int(rx) + 1, 2):
            self.put(cy, cx + i, "·", self.B | self.DIM)
        for i in range(-int(ry), int(ry) + 1):
            self.put(cy + i, cx, "·", self.B | self.DIM)
        grid = collections.Counter()
        for e, n, _ in pts:
            grid[(round(cy - n / scale * ry), round(cx + e / scale * rx))] += 1
        mx = max(grid.values())
        for (yy, xx), c in grid.items():
            ch = "░▒▓█"[min(3, int(c / mx * 3.999))]
            self.put(yy, xx, ch, self.G)
        e, n, _ = pts[-1]
        self.put(round(cy - n / scale * ry), round(cx + e / scale * rx), "◉", self.Y | self.BOLD)
        self.put(y + 1, cx, "N", self.C | self.BOLD)
        self.put(y + h - 2, x + 2, f"{len(pts)} fixes, centred on mean  ◉=latest", self.DIM)

    def sats_panel(self, y, x, h, w):
        sats = self.st.sat_list()
        self.box(y, x, h, w, f"SATELLITES ({len(sats)})")
        iw = w - 4
        barw = max(5, iw - 33)
        self.put(y + 1, x + 2, f"{'SYS':<8}{'PRN':>4}{'EL':>4}{'AZ':>5}{'SNR':>4}  {'dB-Hz':<{barw}} USE",
                 self.DIM | curses.A_UNDERLINE, maxw=iw)
        rows = h - 3
        for i, s in enumerate(sats[:rows]):
            r = y + 2 + i
            snr = s["snr"]
            attr = self.SYS_COLOR[s["sys"]]
            self.put(r, x + 2, f"{SYS_NAME[s['sys']]:<8}", attr)
            el = "" if s["el"] is None else str(s["el"])
            az = "" if s["az"] is None else str(s["az"])
            self.put(r, x + 10, f"{s['prn']:>4}{el:>4}{az:>5}{snr if snr else '':>4}", self.W)
            if snr:
                frac = min(1.0, snr / 50) * barw
                full = int(frac)
                bar = "█" * full + (BLOCKS[int((frac - full) * 8)] if full < barw else "")
                self.put(r, x + 29, bar, self.snr_color(snr))
            else:
                self.put(r, x + 29, "searching" if self.st.last_rx else "", self.DIM)
            use = " ● " if s["used"] else " at" if s.get("at") else " · "
            self.put(r, x + 30 + barw, use, self.G | self.BOLD if s["used"] else self.Y if s.get("at") else self.DIM)
        if len(sats) > rows:
            self.put(y + h - 1, x + w - 12, f" +{len(sats) - rows} more ", self.DIM)

    def nmea_panel(self, y, x, h, w):
        st = self.st
        now = time.monotonic()
        self.box(y, x, h, w, "NMEA STREAM")
        iw = w - 4
        r = y + 1
        age = f"{now - st.last_rx:.1f}s" if st.last_rx else "never"
        self.put(r, x + 2, f"{st.byte_rate():.0f} B/s  total {st.bytes / 1024:.1f} KiB  last rx {age}",
                 self.W, maxw=iw); r += 1
        self.put(r, x + 2, f"checksum errors: {st.bad_cksum}", self.R if st.bad_cksum else self.DIM, maxw=iw); r += 1
        if st.modem:
            m = st.modem
            self.put(r, x + 2, f"fw {m['fw']}  modem {m['state']} ({m['reason']})", self.DIM, maxw=iw); r += 1
        tags = sorted(st.counts, key=lambda t: t[2:] + t[:2])
        colw = 21
        ncol = max(1, iw // colw)
        for i, t in enumerate(tags):
            rr, cc = r + i // ncol, x + 2 + (i % ncol) * colw
            if rr >= y + h - 1:
                break
            self.put(rr, cc, f"{t:<6}", self.C)
            self.put(rr, cc + 6, f"{st.counts[t]:>6} {st.rate(t):4.1f}/s", self.W)

    def engine_panel(self, y, x, h, w):
        st = self.st
        at = st.at
        self.box(y, x, h, w, "ENGINE (AT) & XTRA")
        iw = w - 4
        r = y + 1
        if "error" in at:
            self.put(r, x + 2, at["error"], self.R, maxw=iw)
            r += 1
        elif not at:
            self.put(r, x + 2, "polling AT port…" if not self.no_at else "AT polling disabled", self.DIM, maxw=iw)
            r += 1
        else:
            sess = at.get("session", "?")
            sattr = self.G if sess == "ACTIVE" else self.R if sess == "FAIL" else self.Y
            self.kv(r, x + 2, "Session", sess + (f" (code {at['session_fail']})" if sess == "FAIL" else ""),
                    sattr, kw=10, maxw=iw)
            lf = at.get("last_fix", "?")
            self.put(r, x + 2 + 10 + len(sess) + 3, f"last fix {lf}" + (f" ttff {at['ttff']}s" if at.get("ttff") else ""),
                     self.G if lf == "SUCCESS" else self.DIM, maxw=iw - 13 - len(sess))
            r += 1
            v = at.get("xtra_valid")
            if v and not v[0].startswith("1980"):
                self.kv(r, x + 2, "XTRA data", f"{v[0]} → {v[1]}", self.G, kw=10, maxw=iw)
            else:
                self.kv(r, x + 2, "XTRA data", "none loaded", self.Y, kw=10, maxw=iw)
            r += 1
            self.kv(r, x + 2, "XTRA time", at.get("xtra_time") or "?", self.DIM, kw=10, maxw=iw)
            r += 1
        xs = st.xtra
        xattr = {"ok": self.G, "fail": self.R, "running": self.Y}.get(xs["state"], self.DIM)
        spin = SPINNER[self.tick % len(SPINNER)] + " " if xs["busy"] else ""
        self.kv(r, x + 2, "Inject", spin + xs["msg"], xattr, kw=10, maxw=iw)
        r += 1
        if "time" in xs:
            self.kv(r, x + 2, "Time inj", xs["time"] + (f"   file {xs['file']}" if xs.get("file") else ""),
                    self.G if xs["time"] == "success" else self.R, kw=10, maxw=iw)
            r += 1
        if r >= y + h - 1:
            return
        tr = sorted(st.at_sats.items(), key=lambda kv: -kv[1]) if st.at_sats_t else []
        unk = at.get("unknown_sv", 0)
        self.put(r, x + 2, f"Tracked by engine: {len(tr)}" + (f" (+{unk} unidentified)" if unk else ""),
                 self.DIM | curses.A_UNDERLINE, maxw=iw)
        r += 1
        colw = 18
        ncol = max(1, iw // colw)
        for i, ((sysn, prn), snr) in enumerate(tr):
            rr, cc = r + i // ncol, x + 2 + (i % ncol) * colw
            if rr >= y + h - 1:
                break
            self.put(rr, cc, f"{SYS_CHAR[sysn]}{prn:<3}", self.SYS_COLOR[sysn])
            bw = 10
            frac = min(1.0, snr / 50) * bw
            self.put(rr, cc + 4, ("█" * int(frac)).ljust(bw, "·"), self.snr_color(snr))
            self.put(rr, cc + 5 + bw, f"{snr}", self.W)

    def raw_panel(self, y, x, h, w):
        self.box(y, x, h, w, "RAW NMEA" + ("  [PAUSED]" if self.paused else ""))
        lines = self.frozen_raw if self.paused else list(self.st.raw)
        n = h - 2
        now = time.monotonic()
        for i, (t, ln) in enumerate(lines[-n:]):
            tag = ln[1:6] if len(ln) > 6 else ""
            typ = tag[2:]
            attr = {"GGA": self.G, "RMC": self.Y, "GSV": self.C, "GSA": self.M, "VTG": self.B,
                    "GNS": self.G}.get(typ, self.W)
            if not self.paused and now - t < 1.1:
                attr |= self.BOLD
            else:
                attr |= self.DIM
            self.put(y + 1 + i, x + 2, ln, attr, maxw=w - 4)

    # ---- layout ----
    def draw(self):
        self.tick += 1
        scr = self.scr
        scr.erase()
        H, W = scr.getmaxyx()
        if H < 20 or W < 60:
            self.put(0, 0, f"terminal too small ({W}x{H}); need ≥ 60x20, ideally 150x45")
            scr.refresh()
            return
        self.header(W)
        if self.error:
            self.put(1, 0, f" ERROR: {self.error} ", curses.color_pair(10) | self.BOLD)
        top = 1
        bot_h = max(8, min(14, H // 4))
        main_h = H - top - bot_h
        lw = 44
        fix_h = 15
        acc_h = 13
        # left column: FIX over MOTION & TIME (falls back to FIX only on short terminals)
        if main_h >= fix_h + 12:
            self.fix_panel(top, 0, fix_h, lw)
            self.motion_time_panel(top + fix_h, 0, main_h - fix_h, lw)
        else:
            self.fix_panel(top, 0, main_h, lw)
        rest_x = lw
        rw = W - rest_x
        # middle: sky plot / scatter, sized to leave the satellite table ≥ 58 columns
        sky_w = min(2 * main_h + 2, rw - 58)
        if sky_w >= 30:
            (self.sky_panel if self.view == "sky" else self.scatter_panel)(top, rest_x, main_h, sky_w)
        else:
            sky_w = 0
        # right: SATELLITES over ACCURACY
        sx, sw = rest_x + sky_w, rw - sky_w
        if main_h >= acc_h + 8:
            self.sats_panel(top, sx, main_h - acc_h, sw)
            self.accuracy_panel(top + main_h - acc_h, sx, acc_h, sw)
        else:
            self.sats_panel(top, sx, main_h, sw)
        nw = 44
        ew = min(62, max(0, W - nw - 40))
        self.nmea_panel(top + main_h, 0, bot_h, nw)
        if ew >= 40:
            self.engine_panel(top + main_h, nw, bot_h, ew)
        else:
            ew = 0
        self.raw_panel(top + main_h, nw + ew, bot_h, W - nw - ew)
        scr.refresh()

    def key(self, k):
        if k in (ord("q"), ord("Q"), 27):
            return False
        if k == ord("s"):
            self.view = "scatter" if self.view == "sky" else "sky"
        elif k == ord("p"):
            self.paused = not self.paused
            self.frozen_raw = list(self.st.raw)
        elif k == ord("r"):
            self.st.reset_stats()
        elif k == ord("x"):
            self.start_xtra()
        elif k == ord("X"):
            self.start_xtra(force=True)
        return True

    def start_xtra(self, force=False):
        if self.st.xtra["busy"] or self.qmi is None:
            if self.qmi is None:
                self.st.xtra.update(state="fail", msg="not available in replay mode")
            return
        if not sudo_ready():
            # drop out of curses so sudo can prompt for a password on the terminal
            curses.def_prog_mode()
            curses.endwin()
            print("\ngpsdash: XTRA/time injection talks QMI through mbim-proxy, which needs root.")
            ok = subprocess.run(["sudo", "-v"]).returncode == 0
            curses.reset_prog_mode()
            self.scr.clear()
            if not ok:
                self.st.xtra.update(state="fail", msg="sudo authentication failed")
                return
        threading.Thread(target=xtra_worker, args=(self.st, self.qmi, force), daemon=True).start()


def run(scr, args):
    st = State()
    q = queue.Queue()
    fd = None
    log = open(args.log, "a") if args.log else None
    fan = NMEAFanout(args.serve) if args.serve else None
    if args.replay:
        src = f"replay:{os.path.basename(args.replay)}"
        threading.Thread(target=replay_reader, args=(args.replay, q, args.speed, fan), daemon=True).start()
    else:
        src = args.device
        fd = open_serial(args.device)
        if not args.no_start:
            os.write(fd, b"$GPS_START\r\n")
        threading.Thread(target=serial_reader, args=(fd, q, log, fan), daemon=True).start()
        threading.Thread(target=modem_info, args=(st,), daemon=True).start()
        if not args.no_at:
            threading.Thread(target=at_poller, args=(st, args.at_device, 2.0), daemon=True).start()
    st.at_path = args.at_device
    ui = UI(scr, st, src)
    ui.no_at = args.no_at
    ui.fan = fan
    if not args.replay:
        ui.qmi = args.qmi
        if args.xtra:
            ui.start_xtra()
    scr.timeout(100)
    next_draw = 0
    try:
        while True:
            try:
                while True:
                    kind, val = q.get_nowait()
                    if kind == "line":
                        st.feed(val, time.monotonic())
                    else:
                        ui.error = val
            except queue.Empty:
                pass
            now = time.monotonic()
            if now >= next_draw:
                ui.draw()
                next_draw = now + 0.1
            k = scr.getch()
            if k != -1 and not ui.key(k):
                break
    finally:
        if fd is not None and args.stop:
            try:
                os.write(fd, b"$GPS_STOP\r\n")
            except OSError:
                pass


def main():
    p = argparse.ArgumentParser(prog="gpsdash", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("-V", "--version", action="version", version=f"%(prog)s {__version__}")
    p.add_argument("-d", "--device", default=DEFAULT_DEV, help=f"NMEA tty (default {DEFAULT_DEV})")
    p.add_argument("--replay", metavar="FILE", help="replay an NMEA log instead of the live device")
    p.add_argument("--speed", type=float, default=1.0, help="replay speed multiplier")
    p.add_argument("--log", metavar="FILE", help="append raw NMEA to FILE")
    p.add_argument("--no-start", action="store_true", help="don't send $GPS_START")
    p.add_argument("--stop", action="store_true", help="send $GPS_STOP on exit (saves battery)")
    p.add_argument("--at-device", default=DEFAULT_AT, help=f"modem AT port for engine status (default {DEFAULT_AT})")
    p.add_argument("--no-at", action="store_true", help="don't poll the AT port")
    p.add_argument("--qmi", default=DEFAULT_QMI, help=f"QMI/MBIM control device (default {DEFAULT_QMI})")
    p.add_argument("--xtra", action="store_true", help="inject XTRA + UTC time at startup (same as pressing x)")
    p.add_argument("--serve", metavar="[HOST:]PORT", help="re-serve raw NMEA over TCP (e.g. 10110) for gpsd / map apps")
    p.add_argument("--xtra-helper", metavar="FILE", help=argparse.SUPPRESS)
    p.add_argument("--helper-no-time", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--xtra-status", action="store_true", help=argparse.SUPPRESS)
    args = p.parse_args()
    if args.xtra_status:
        sys.exit(xtra_status_helper(args.at_device))
    if args.xtra_helper:
        sys.exit(xtra_helper(args.qmi, args.xtra_helper, not args.helper_no_time))
    locale.setlocale(locale.LC_ALL, "")
    if not args.replay and not os.access(args.device, os.R_OK | os.W_OK):
        sys.exit(f"cannot open {args.device}: need rw access (are you in the 'dialout' group?)")
    try:
        curses.wrapper(run, args)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
