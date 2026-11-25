# pyright: reportOptionalMemberAccess=false, reportOptionalContextManager=false, reportAttributeAccessIssue=false
# main.py - Cottage temperature logger, Raspberry Pi Pico 2 W (MicroPython)
#
# Sensors : DS18B20 outdoor probe  -> GP13 (1-Wire)
#           AHT20 + BMP280         -> I2C0, SDA GP8, SCL GP9
# Storage : one 16-byte record per interval in /log.bin (flash)
# Phone   : Bluetooth LE (aioble). Needs:  py -m mpremote connect COM4 mip install aioble
#
# ---------------------------------------------------------------------------
# TIME MODEL
# The Pico has no clock. Every record carries (boot_id, seconds since boot).
# Whenever the phone connects it sends the real time (SET_TIME) and the Pico
# stores a "sync anchor" (boot_id, uptime, unix time) in /sync.bin. The app
# (or dump_log.py) turns uptime into real time using the anchors of the same
# boot, which also fixes records logged BEFORE the sync. A new boot_id marks
# a power loss.
#
# FILES
# /log.bin   16-byte header + 16-byte records
#   header : b"TLG1", version u8, record size u8, 10 zero bytes
#   record : <IHhhHHBB  uptime_s u32, boot_id u16, outdoor_C*100 i16,
#            indoor_C*100 i16, humidity_%*100 u16, pressure_hPa*10 u16,
#            flags u8, crc8 u8      (crc8 poly 0x07, init 0, over first 15 bytes)
#   missing value: 0x7FFF for temperatures, 0xFFFF for humidity / pressure
#   flags  : bit0 outdoor sensor failed, bit1 AHT20 failed, bit2 BMP280 failed
# /sync.bin  10-byte anchors: <HII  boot_id, uptime_s, unix_utc
# /boot.cnt  boot counter,   /interval.bin  sample interval (seconds)
#
# BLE (device name "TempLogger")
# Custom service 7e5a0001-6c0a-4b2e-9a3f-5d1c8b7a2f10
#   ...0002 CONTROL  write      : commands (below)
#   ...0003 DATA     notify     : replies. SUBSCRIBE BEFORE SENDING COMMANDS.
#   ...0004 LIVE     read+notify: latest 16-byte record
# Standard Environmental Sensing service 0x181A (indoor temp, humidity, pressure)
#
# Commands (CONTROL):
#   01 <u32 unix>              SET_TIME     (also replies with INFO)
#   02 <u32 start> [<u32 n>]   READ_LOG     records from index start (n=0: all)
#   03 <u32 start> [<u32 n>]   READ_SYNC    anchors from index start
#   04 <u16 seconds>           SET_INTERVAL (10..3600, stored in flash)
#   05 "ERASE-LOG"             ERASE        deletes log and anchors
#   06                         INFO
#   07                         SAMPLE_NOW   measure now, update LIVE (not logged)
# Replies (DATA notifications, first byte = type):
#   01 <whole records>         log chunk
#   02 <whole anchors>         anchor chunk
#   03 <BBHHIIIB>              INFO: version, record size, interval, boot_id,
#                              uptime_s, record count, anchor count,
#                              state (bit0 log full, bits1-3 = last sensor flags)
#   04 <cmd> <status>          ACK (status 0 ok, 1 ignored, 2 invalid)
#   FF <type> <u32 sent> <u32 total>   end of a stream
# ---------------------------------------------------------------------------

import os
import struct
import sys
import asyncio
import bluetooth
import aioble
from machine import Pin, I2C, WDT, reset
import onewire
import ds18x20
from time import ticks_ms, ticks_diff, ticks_add

# ---------------- configuration ----------------
DEPLOY = False            # True when installed: enables watchdog + auto-restart on crash
BLE_NAME = "TempLogger"
PIN_SDA, PIN_SCL, PIN_OW = 8, 9, 13
DEFAULT_INTERVAL = 300    # seconds between logged samples
MIN_INTERVAL, MAX_INTERVAL = 10, 3600
FLUSH_SECONDS = 900       # write the RAM buffer to flash about this often
MIN_FREE_BYTES = 64 * 1024
ADV_INTERVAL_US = 400_000
MTU_WANTED = 247

# ---------------- constants ----------------
LOG_PATH = "/log.bin"
SYNC_PATH = "/sync.bin"
BOOT_PATH = "/boot.cnt"
CFG_PATH = "/interval.bin"
MAGIC = b"TLG1"
FMT_VER = 1
HDR_SIZE = 16
REC_SIZE = 16
ANCHOR_SIZE = 10
T_MISSING = 0x7FFF
U_MISSING = 0xFFFF
F_OUTDOOR, F_AHT, F_BMP = 1, 2, 4
AHT_ADDR = 0x38
MIN_UNIX = 1_700_000_000

CMD_SET_TIME, CMD_READ_LOG, CMD_READ_SYNC, CMD_SET_INTERVAL = 1, 2, 3, 4
CMD_ERASE, CMD_INFO, CMD_SAMPLE = 5, 6, 7
T_LOG, T_SYNC, T_INFO, T_ACK, T_END = 1, 2, 3, 4, 0xFF

SVC_UUID = bluetooth.UUID("7e5a0001-6c0a-4b2e-9a3f-5d1c8b7a2f10")
CTRL_UUID = bluetooth.UUID("7e5a0002-6c0a-4b2e-9a3f-5d1c8b7a2f10")
DATA_UUID = bluetooth.UUID("7e5a0003-6c0a-4b2e-9a3f-5d1c8b7a2f10")
LIVE_UUID = bluetooth.UUID("7e5a0004-6c0a-4b2e-9a3f-5d1c8b7a2f10")

# ---------------- state ----------------
boot_id = 0
interval = DEFAULT_INTERVAL
interval_changed = False
pending = bytearray()     # records not yet written to flash
persisted = 0             # records in /log.bin
sync_count = 0
last_anchor_up = None
log_full = False
reading = False           # a stream is reading the files: do not append meanwhile
last_flags = 0
last_rec = bytes(REC_SIZE)

i2c = None
ds = None
roms = []
bmp = None
aht_ready = False
meas_lock = None
led = None
ctrl_char = data_char = live_char = None
ess_temp = ess_hum = ess_pres = None

_last_tick = ticks_ms()
_up_ms = 0


def uptime_s():
    """Seconds since boot. Accumulated so it never wraps; called at least once a second."""
    global _last_tick, _up_ms
    now = ticks_ms()
    _up_ms += ticks_diff(now, _last_tick)
    _last_tick = now
    return _up_ms // 1000


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


# ====================== storage ======================
def file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return -1


def free_bytes():
    s = os.statvfs("/")
    return s[0] * s[3]


def new_log():
    with open(LOG_PATH, "wb") as f:
        f.write(MAGIC + bytes((FMT_VER, REC_SIZE)) + bytes(HDR_SIZE - 6))


def align_file(path, hdr, rec):
    """Drop a partial trailing record (power cut mid-write). Returns whole-record count."""
    size = file_size(path)
    if size < hdr:
        return 0
    extra = (size - hdr) % rec
    if extra:
        keep = size - extra
        tmp = path + ".tmp"
        with open(path, "rb") as src, open(tmp, "wb") as dst:
            left = keep
            while left > 0:
                b = src.read(min(left, 256))
                if not b:
                    break
                dst.write(b)
                left -= len(b)
        os.remove(path)
        os.rename(tmp, path)
        size = keep
    return (size - hdr) // rec


def next_boot_id():
    try:
        with open(BOOT_PATH, "rb") as f:
            n = struct.unpack("<H", f.read(2))[0]
    except Exception:
        n = 0
    n = (n % 65535) + 1
    with open(BOOT_PATH, "wb") as f:
        f.write(struct.pack("<H", n))
    return n


def load_interval():
    try:
        with open(CFG_PATH, "rb") as f:
            v = struct.unpack("<H", f.read(2))[0]
        if MIN_INTERVAL <= v <= MAX_INTERVAL:
            return v
    except Exception:
        pass
    return DEFAULT_INTERVAL


def init_state():
    global boot_id, interval, persisted, sync_count
    boot_id = next_boot_id()
    interval = load_interval()
    ok = False
    size = file_size(LOG_PATH)
    if size >= HDR_SIZE:
        with open(LOG_PATH, "rb") as f:
            h = f.read(HDR_SIZE)
        ok = h[:4] == MAGIC and h[4] == FMT_VER and h[5] == REC_SIZE
    if not ok:
        if size >= 0:  # unknown format: keep it aside instead of destroying it
            try:
                os.remove(LOG_PATH + ".old")
            except OSError:
                pass
            os.rename(LOG_PATH, LOG_PATH + ".old")
        new_log()
    persisted = align_file(LOG_PATH, HDR_SIZE, REC_SIZE)
    sync_count = align_file(SYNC_PATH, 0, ANCHOR_SIZE)


def record_count():
    return persisted + len(pending) // REC_SIZE


def flush():
    global persisted, pending, log_full
    if not pending or reading:
        return
    if free_bytes() < MIN_FREE_BYTES:
        log_full = True
        pending = bytearray()
        return
    with open(LOG_PATH, "ab") as f:
        f.write(pending)
    persisted += len(pending) // REC_SIZE
    pending = bytearray()


def add_anchor(unix):
    """0 = stored, 1 = ignored (a recent anchor of this boot exists), 2 = invalid time."""
    global sync_count, last_anchor_up
    if not MIN_UNIX <= unix <= 4_000_000_000:
        return 2
    up = uptime_s()
    if last_anchor_up is not None and up - last_anchor_up < 60:
        return 1
    with open(SYNC_PATH, "ab") as f:
        f.write(struct.pack("<HII", boot_id, up, unix))
    sync_count += 1
    last_anchor_up = up
    return 0


def erase_all():
    global persisted, pending, sync_count, last_anchor_up, log_full
    pending = bytearray()
    new_log()
    persisted = 0
    try:
        os.remove(SYNC_PATH)
    except OSError:
        pass
    sync_count = 0
    last_anchor_up = None
    log_full = False


def build_record(t_out, t_in, rh, pres, flags):
    body = struct.pack("<IHhhHHB", uptime_s(), boot_id, t_out, t_in, rh, pres, flags)
    return body + bytes((crc8(body),))


# ====================== sensors ======================
def setup_hardware():
    global i2c, ds
    i2c = I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=100_000)
    ds = ds18x20.DS18X20(onewire.OneWire(Pin(PIN_OW)))


async def read_ds():
    """Outdoor temperature in 0.01 C."""
    global roms
    try:
        if not roms:
            roms = ds.scan()
            if not roms:
                raise OSError("no DS18B20")
        ds.convert_temp()
        await asyncio.sleep_ms(750)
        t = ds.read_temp(roms[0])
        if t is None or t == 85.0 or not -55 <= t <= 125:
            raise ValueError("DS18B20 bad value")
        return int(round(t * 100))
    except Exception:
        roms = []
        raise


async def aht_ensure():
    global aht_ready
    if aht_ready:
        return
    if AHT_ADDR not in i2c.scan():
        raise OSError("AHT20 not found")
    await asyncio.sleep_ms(40)
    if not (i2c.readfrom(AHT_ADDR, 1)[0] & 0x08):
        i2c.writeto(AHT_ADDR, b"\xbe\x08\x00")
        await asyncio.sleep_ms(20)
    aht_ready = True


async def read_aht():
    """Returns (temperature 0.01 C, humidity 0.01 %RH)."""
    global aht_ready
    await aht_ensure()
    try:
        i2c.writeto(AHT_ADDR, b"\xac\x33\x00")
        await asyncio.sleep_ms(80)
        d = None
        for _ in range(5):
            d = i2c.readfrom(AHT_ADDR, 7)
            if not d[0] & 0x80:
                break
            await asyncio.sleep_ms(20)
        else:
            raise OSError("AHT20 busy")
    except OSError:
        aht_ready = False
        raise
    raw_h = (d[1] << 12) | (d[2] << 4) | (d[3] >> 4)
    raw_t = ((d[3] & 0x0F) << 16) | (d[4] << 8) | d[5]
    rh = raw_h * 10000 // 1048576
    t = raw_t * 20000 // 1048576 - 5000
    if not (0 <= rh <= 10000 and -4000 <= t <= 8500):
        raise ValueError("AHT20 out of range")
    return t, rh


class BMP280:
    def __init__(self, addr):
        self.a = addr
        cid = i2c.readfrom_mem(addr, 0xD0, 1)[0]
        if cid not in (0x58, 0x60):      # 0x60 = BME280, same pressure maths
            raise OSError("unexpected BMP280 chip id")
        (self.T1, self.T2, self.T3, self.P1, self.P2, self.P3, self.P4,
         self.P5, self.P6, self.P7, self.P8, self.P9) = struct.unpack(
            "<HhhHhhhhhhhh", i2c.readfrom_mem(addr, 0x88, 24))
        i2c.writeto_mem(addr, 0xF5, b"\x00")

    async def pressure(self):
        """Pressure in 0.1 hPa."""
        i2c.writeto_mem(self.a, 0xF4, b"\x55")      # temp x2, pressure x16, forced mode
        await asyncio.sleep_ms(60)
        d = i2c.readfrom_mem(self.a, 0xF7, 6)
        adc_p = (d[0] << 12) | (d[1] << 4) | (d[2] >> 4)
        adc_t = (d[3] << 12) | (d[4] << 4) | (d[5] >> 4)
        v1 = (((adc_t >> 3) - (self.T1 << 1)) * self.T2) >> 11
        v2 = (((((adc_t >> 4) - self.T1) * ((adc_t >> 4) - self.T1)) >> 12) * self.T3) >> 14
        t_fine = v1 + v2
        v1 = t_fine - 128000
        v2 = v1 * v1 * self.P6
        v2 += (v1 * self.P5) << 17
        v2 += self.P4 << 35
        v1 = ((v1 * v1 * self.P3) >> 8) + ((v1 * self.P2) << 12)
        v1 = (((1 << 47) + v1) * self.P1) >> 33
        if v1 == 0:
            raise OSError("BMP280 bad calibration")
        pr = 1048576 - adc_p
        pr = (((pr << 31) - v2) * 3125) // v1
        v1 = (self.P9 * (pr >> 13) * (pr >> 13)) >> 25
        v2 = (self.P8 * pr) >> 19
        pr = ((pr + v1 + v2) >> 8) + (self.P7 << 4)    # Pa in Q24.8
        return int(round(pr / 2560))                   # 0.1 hPa


async def read_bmp():
    global bmp
    if bmp is None:
        found = i2c.scan()
        for a in (0x76, 0x77):
            if a in found:
                bmp = BMP280(a)
                break
        else:
            raise OSError("BMP280 not found")
    try:
        p = await bmp.pressure()
    except OSError:
        bmp = None
        raise
    if not 3000 <= p <= 11000:
        raise ValueError("pressure out of range")
    return p


async def measure():
    """Returns (outdoor, indoor, humidity, pressure, failure flags)."""
    async with meas_lock:
        flags = 0
        t_out, t_in, rh, pres = T_MISSING, T_MISSING, U_MISSING, U_MISSING
        try:
            t_out = await read_ds()
        except Exception as e:
            flags |= F_OUTDOOR
            print("DS18B20:", repr(e))
        try:
            t_in, rh = await read_aht()
        except Exception as e:
            flags |= F_AHT
            print("AHT20:", repr(e))
        try:
            pres = await read_bmp()
        except Exception as e:
            flags |= F_BMP
            print("BMP280:", repr(e))
        return t_out, t_in, rh, pres, flags


def publish(rec, t_in, rh, pres):
    """Update the LIVE characteristic and the standard Environmental Sensing ones."""
    global last_rec
    last_rec = rec
    try:
        live_char.write(rec, send_update=True)
        ess_temp.write(struct.pack("<h", t_in if t_in != T_MISSING else -32768), send_update=True)
        ess_hum.write(struct.pack("<H", rh), send_update=True)
        ess_pres.write(struct.pack("<I", pres * 100 if pres != U_MISSING else 0xFFFFFFFF),
                       send_update=True)
    except OSError as e:
        print("publish:", repr(e))


# ====================== sampling ======================
async def sampler():
    global last_flags, pending, interval_changed
    while True:
        t0 = ticks_ms()
        t_out, t_in, rh, pres, flags = await measure()
        last_flags = flags
        rec = build_record(t_out, t_in, rh, pres, flags)
        if not log_full:
            pending += rec
        publish(rec, t_in, rh, pres)
        if len(pending) // REC_SIZE >= max(1, FLUSH_SECONDS // interval):
            try:
                flush()
            except OSError as e:
                print("flush:", repr(e))
        interval_changed = False
        deadline = ticks_add(t0, interval * 1000)
        while ticks_diff(deadline, ticks_ms()) > 0 and not interval_changed:
            await asyncio.sleep_ms(500)


async def housekeeping():
    wdt = WDT(timeout=8000) if DEPLOY else None
    while True:
        if wdt:
            wdt.feed()
        uptime_s()
        await asyncio.sleep_ms(1000)


# ====================== BLE ======================
async def notify(conn, payload):
    for _ in range(100):
        if not conn.is_connected():
            raise OSError("disconnected")
        try:
            data_char.notify(conn, payload)
            return
        except OSError:
            await asyncio.sleep_ms(10)
    raise OSError("notify failed")


def info_bytes():
    state = (1 if log_full else 0) | (last_flags << 1)
    return bytes((T_INFO,)) + struct.pack(
        "<BBHHIIIB", FMT_VER, REC_SIZE, interval, boot_id, uptime_s(),
        record_count(), sync_count, state)


async def stream(conn, path, hdr, rec, kind, start, count):
    """Send records [start, start+count) of a file as notifications."""
    global reading
    size = file_size(path)
    total = (size - hdr) // rec if size >= hdr else 0
    sent = 0
    if start < total:
        n = total - start
        if count and count < n:
            n = count
        per = max(1, ((getattr(conn, "mtu", None) or 23) - 4) // rec)
        reading = True
        try:
            with open(path, "rb") as f:
                f.seek(hdr + start * rec)
                while sent < n:
                    buf = f.read(min(per, n - sent) * rec)
                    take = len(buf) // rec
                    if take == 0:
                        break
                    await notify(conn, bytes((kind,)) + buf[:take * rec])
                    sent += take
                    await asyncio.sleep_ms(5)
        finally:
            reading = False
    await notify(conn, bytes((T_END, kind)) + struct.pack("<II", sent, total))


async def handle_command(conn, d):
    global interval, interval_changed
    if not d:
        return
    cmd = d[0]
    if cmd == CMD_SET_TIME and len(d) >= 5:
        status = add_anchor(struct.unpack_from("<I", d, 1)[0])
        await notify(conn, bytes((T_ACK, cmd, status)))
        await notify(conn, info_bytes())
    elif cmd in (CMD_READ_LOG, CMD_READ_SYNC) and len(d) >= 5:
        start = struct.unpack_from("<I", d, 1)[0]
        count = struct.unpack_from("<I", d, 5)[0] if len(d) >= 9 else 0
        try:
            flush()
            if cmd == CMD_READ_LOG:
                await stream(conn, LOG_PATH, HDR_SIZE, REC_SIZE, T_LOG, start, count)
            else:
                await stream(conn, SYNC_PATH, 0, ANCHOR_SIZE, T_SYNC, start, count)
        finally:
            try:
                flush()     # records that arrived while streaming
            except OSError as e:
                print("flush:", repr(e))
    elif cmd == CMD_SET_INTERVAL and len(d) >= 3:
        v = struct.unpack_from("<H", d, 1)[0]
        if MIN_INTERVAL <= v <= MAX_INTERVAL:
            with open(CFG_PATH, "wb") as f:
                f.write(struct.pack("<H", v))
            interval = v
            interval_changed = True
            await notify(conn, bytes((T_ACK, cmd, 0)))
        else:
            await notify(conn, bytes((T_ACK, cmd, 2)))
    elif cmd == CMD_ERASE:
        if bytes(d[1:]) == b"ERASE-LOG":
            erase_all()
            await notify(conn, bytes((T_ACK, cmd, 0)))
        else:
            await notify(conn, bytes((T_ACK, cmd, 2)))
    elif cmd == CMD_INFO:
        await notify(conn, info_bytes())
    elif cmd == CMD_SAMPLE:
        t_out, t_in, rh, pres, flags = await measure()
        publish(build_record(t_out, t_in, rh, pres, flags), t_in, rh, pres)
        await notify(conn, bytes((T_ACK, cmd, 0)))
    else:
        await notify(conn, bytes((T_ACK, cmd, 2)))


async def command_task():
    while True:
        try:
            conn, data = await ctrl_char.written()
            await handle_command(conn, bytes(data))
        except Exception as e:
            print("command:", repr(e))
            await asyncio.sleep_ms(100)


async def ble_task():
    while True:
        try:
            async with await aioble.advertise(ADV_INTERVAL_US, name=BLE_NAME) as conn:
                led.on()
                await conn.disconnected(timeout_ms=None)
        except Exception as e:
            print("ble:", repr(e))
            await asyncio.sleep_ms(1000)
        led.off()


def setup_ble():
    global ctrl_char, data_char, live_char, ess_temp, ess_hum, ess_pres
    try:
        bluetooth.BLE().config(mtu=MTU_WANTED)
    except Exception as e:
        print("mtu config:", repr(e))
    svc = aioble.Service(SVC_UUID)
    ctrl_char = aioble.Characteristic(svc, CTRL_UUID, write=True, capture=True)
    data_char = aioble.Characteristic(svc, DATA_UUID, notify=True)
    live_char = aioble.Characteristic(svc, LIVE_UUID, read=True, notify=True)
    ess = aioble.Service(bluetooth.UUID(0x181A))
    ess_temp = aioble.Characteristic(ess, bluetooth.UUID(0x2A6E), read=True, notify=True)
    ess_hum = aioble.Characteristic(ess, bluetooth.UUID(0x2A6F), read=True, notify=True)
    ess_pres = aioble.Characteristic(ess, bluetooth.UUID(0x2A6D), read=True, notify=True)
    aioble.register_services(svc, ess)
    live_char.write(last_rec)


# ====================== main ======================
async def main():
    global meas_lock, led
    led = Pin("LED", Pin.OUT)
    meas_lock = asyncio.Lock()
    setup_hardware()
    init_state()
    setup_ble()
    print("boot", boot_id, "| interval", interval, "s | records", persisted,
          "| anchors", sync_count)
    await asyncio.gather(housekeeping(), sampler(), ble_task(), command_task())


def run():
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        raise
    except Exception as e:
        sys.print_exception(e)
        if DEPLOY:
            from time import sleep
            sleep(5)
            reset()
        raise
    finally:
        asyncio.new_event_loop()


if __name__ == "__main__":
    run()
