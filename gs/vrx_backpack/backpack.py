#!/usr/bin/env python3
# vrx_mock.py  /dev/ttyUSB0  [initial_channel_index]
#
# • Dummy HDZero / analog VRX for ExpressLRS backpacks
# • Full 0x0300–0x030D & 0x0383 MSP support
# • Single‑slot shell‑command scheduler (channel‑set & DVR‑toggle)
# • GET_CHANNEL_INDEX now refreshes via gsmenu.sh (deduped)
# • Deduplicate repeated fid/payload within 2 s window
# • Graceful shutdown (SIGINT/SIGTERM, force‑kill after 3 s)

import sys, struct, signal, time, serial, subprocess, threading

# ────────── CLI & globals ───────────────────────────────────────────────────
PORT      = sys.argv[1]
CUR_INDEX = int(sys.argv[2]) if len(sys.argv) > 2 else 27
BAUD      = 115_200
CRC_POLY  = 0xD5

CMD_REC   = ["bash", "-c", "kill -SIGUSR1 $(pidof pixelpilot)"]
def cmd_set(wifi_ch: int):
    return ["gsmenu.sh", "set", "air", "wfbng", "air_channel", str(wifi_ch)]
CMD_GET   = ["gsmenu.sh", "get", "air", "wfbng", "air_channel"]

# ────────── Tables & helpers ────────────────────────────────────────────────
BANDS = ['A','B','E','F','R','L']
FREQ_TABLE_KHZ = [
 5865,5845,5825,5805,5785,5765,5745,5725,
 5733,5752,5771,5790,5809,5828,5847,5866,
 5705,5685,5665,5645,5885,5905,5925,5945,
 5740,5760,5780,5800,5820,5840,5860,5880,
 5658,5695,5732,5769,5806,5843,5880,5917,
 5333,5372,5411,5450,5489,5528,5567,5606
]
WIFI_CH_TABLE = [
 173,169,165,161,157,153,149,144,
 149,151,155,157,161,165,169,173,
 140,136,132,128,177,181,185,189,
 149,151,155,159,163,167,171,175,
 132,140,144,153,161,169,175,183,
  64, 64, 96, 96, 96,106,114,122
]

def idx_to_text(idx):
    return f"{BANDS[idx//8]}{idx%8+1} ({FREQ_TABLE_KHZ[idx]} MHz | Wi‑Fi ch{WIFI_CH_TABLE[idx]})"

def wifi_ch_to_idx(ch):
    return WIFI_CH_TABLE.index(ch) if ch in WIFI_CH_TABLE else None

# ────────── CRC‑8 DVB‑S2 ----------------------------------------------------
def crc8(buf):
    c = 0
    for b in buf:
        c ^= b
        for _ in range(8):
            c = ((c<<1)^CRC_POLY)&0xFF if c&0x80 else (c<<1)&0xFF
    return c

def make_msp(fid, payload=b'', flags=0, direction=b'$X>'):
    body = struct.pack('<BHH', flags, fid, len(payload)) + payload
    return direction + body + bytes([crc8(body)])

# ────────── Runtime state ---------------------------------------------------
state = {
    "index":         CUR_INDEX,
    "frequency_khz": FREQ_TABLE_KHZ[CUR_INDEX],
    "recording":     0,
    "vrx_mode":      0,
    "rssi":          200,
    "voltage_mv10":  1220,
    "fw_string":     b"MockVRX 1.5-dedup",
}

# ────────── Scheduler & shutdown --------------------------------------------
command_pending = False
timer_handle    = None
proc_handle     = None
stop_evt        = threading.Event()

def schedule_cmd(argv, delay=0):
    global command_pending, timer_handle, proc_handle
    if command_pending:
        print("   ↳ Command already pending – request ignored")
        return
    command_pending = True

    def _run():
        global command_pending, proc_handle
        print("   ↳ Executing:", " ".join(argv))
        proc_handle = subprocess.Popen(argv,
                                       stdout=subprocess.DEVNULL,
                                       stderr=subprocess.DEVNULL)
        proc_handle.wait()
        proc_handle = None
        command_pending = False
        print("   ↳ Command finished")

    if delay == 0:
        _run()
    else:
        print(f"   ↳ Scheduling in {delay}s")
        timer_handle = threading.Timer(delay, _run)
        timer_handle.start()

def sig_handler(sig, _f):
    print(f"\nSignal {sig} – shutting down …")
    stop_evt.set()

signal.signal(signal.SIGINT,  sig_handler)
signal.signal(signal.SIGTERM, sig_handler)

def cleanup():
    global timer_handle, proc_handle
    if timer_handle and timer_handle.is_alive():
        print("Cancelling pending timer …")
        timer_handle.cancel(); timer_handle.join()
    if proc_handle and proc_handle.poll() is None:
        print("Waiting up to 3 s for command …")
        try:
            proc_handle.wait(3)
        except subprocess.TimeoutExpired:
            print("Force‑terminating …"); proc_handle.terminate()
            try: proc_handle.wait(1)
            except subprocess.TimeoutExpired:
                print("Killing …"); proc_handle.kill()

# ────────── Deduplication state & window ────────────────────────────────────
DEDUP_WINDOW = 2.0
_last_exec = {}   # fid -> (timestamp, payload_bytes)

# ────────── Verbose decoder ------------------------------------------------
def verbose(fid, pl, inc=True):
    a = "←" if inc else "→"
    if fid == 0x0300: return f"{a} GET_CHANNEL_INDEX"
    if fid == 0x0301: return f"{a} SET_CHANNEL_INDEX: {idx_to_text(pl[0])}"
    if fid in (0x0302,0x0303):
        hz = struct.unpack('<I',pl)[0]/1000 if pl else None
        return f"{a} {'SET' if fid==0x0303 else 'GET'}_FREQUENCY" + (f":{hz:.3f}MHz" if hz else "")
    if fid == 0x0304: return f"{a} GET_RECORDING_STATE"
    if fid == 0x0305:
        st,d = pl[0], struct.unpack('<H',pl[1:3])[0]
        return f"{a} SET_RECORDING_STATE: state={st} delay={d}s"
    if fid in (0x0306,0x0307):
        return f"{a} {'SET' if fid==0x0307 else 'GET'}_VRX_MODE" + (f":{pl[0]}" if pl else "")
    if fid == 0x0308: return f"{a} GET_RSSI"
    if fid == 0x0309: return f"{a} GET_BATTERY_VOLTAGE"
    if fid == 0x030A: return f"{a} GET_FIRMWARE"
    if fid == 0x030B: return f"{a} SET_BUZZER:{'ON' if pl and pl[0] else 'OFF'}"
    if fid == 0x030C: return f"{a} SET_OSD_ELEMENT(len={len(pl)})"
    if fid == 0x030D: return f"{a} SET_HEAD_TRACKING:{'enable' if pl and pl[0] else 'disable'}"
    if fid == 0x0383: return f"{a} SET_PTR(len={len(pl)})"
    return f"{a} UNKNOWN fid 0x{fid:04X}"

# ────────── Serial I/O loop -------------------------------------------------
print(f"Listening on {PORT} @ {BAUD} baud – Ctrl‑C to quit")
with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
    buf = b''
    while not stop_evt.is_set():
        buf += ser.read(128)

        while b'$X<' in buf:
            s = buf.index(b'$X<')
            if len(buf) < s+8: break
            flags,fid,ln = struct.unpack('<BHH', buf[s+3:s+8])
            flen = 3+5+ln+1
            if len(buf) < s+flen: break

            frame   = buf[s:s+flen]
            payload = frame[8:-1]
            crc_ok  = crc8(frame[3:-1]) == frame[-1]
            buf     = buf[s+flen:]

            print(verbose(fid,payload,True),
                  f"[fid=0x{fid:04X} len={ln} flags=0x{flags:02X} crc={'OK' if crc_ok else 'BAD'}]")

            if not crc_ok:
                continue

            rp = b''

            # GET_CHANNEL_INDEX ▶ refresh only once per window
            if fid == 0x0300:
                now = time.time()
                last_t, _ = _last_exec.get(fid, (0, None))
                if now - last_t > DEDUP_WINDOW:
                    try:
                        out = subprocess.check_output(CMD_GET, text=True).strip()
                        ch  = int(out)
                        idx = wifi_ch_to_idx(ch)
                        if idx is not None:
                            state['index'] = idx
                            state['frequency_khz'] = FREQ_TABLE_KHZ[idx]
                    except Exception:
                        pass
                    _last_exec[fid] = (now, b'')
                else:
                    print("   ↳ duplicate GET_CHANNEL_INDEX, skipping fetch")
                print("   ↳ returning", idx_to_text(state['index']))
                rp = bytes([state['index']])

            # SET_CHANNEL_INDEX ▶ dedupe repeats
            elif fid == 0x0301:
                idx = payload[0]
                state['index'] = idx
                state['frequency_khz'] = FREQ_TABLE_KHZ[idx]
                print("   ↳ channel ->", idx_to_text(idx))
                now = time.time()
                prev_t, prev_pl = _last_exec.get(fid, (0, None))
                if payload != prev_pl or now - prev_t > DEDUP_WINDOW:
                    _last_exec[fid] = (now, payload)
                    schedule_cmd(cmd_set(WIFI_CH_TABLE[idx]))
                else:
                    print("   ↳ duplicate SET_CHANNEL_INDEX, skipping")
                rp = payload

            # GET/SET_FREQUENCY
            elif fid == 0x0302:
                rp = struct.pack('<I', state['frequency_khz'])
            elif fid == 0x0303:
                hz = struct.unpack('<I', payload)[0]
                state['frequency_khz'] = hz
                print("   ↳ frequency ->", f"{hz/1000:.3f}MHz")
                rp = payload

            # GET_RECORDING_STATE
            elif fid == 0x0304:
                rp = bytes([state['recording']])

            # SET_RECORDING_STATE ▶ dedupe
            elif fid == 0x0305:
                st, d = payload[0], struct.unpack('<H', payload[1:3])[0]
                state['recording'] = st
                print("   ↳ recording ->", "ON" if st else "OFF", f"(delay={d}s)")
                now = time.time()
                prev_t, prev_pl = _last_exec.get(fid, (0, None))
                if payload != prev_pl or now - prev_t > DEDUP_WINDOW:
                    _last_exec[fid] = (now, payload)
                    schedule_cmd(CMD_REC, d)
                else:
                    print("   ↳ duplicate SET_RECORDING_STATE, skipping")
                rp = payload

            # GET/SET_VRX_MODE, GET_RSSI, GET_BATTERY_VOLTAGE, GET_FIRMWARE
            elif fid == 0x0306: rp = bytes([state['vrx_mode']])
            elif fid == 0x0307: state['vrx_mode']=payload[0]; rp=payload
            elif fid == 0x0308: rp = bytes([state['rssi']])
            elif fid == 0x0309: rp = struct.pack('<H', state['voltage_mv10'])
            elif fid == 0x030A: rp = state['fw_string'] + b'\x00'

            # Simple echo for these
            elif fid in (0x030B,0x030C,0x030D,0x0383):
                rp = payload

            # Reply unless NO_REPLY flag set
            if not (flags & 0x01):
                ser.write(make_msp(fid, rp))
                print(verbose(fid, rp, False))

print("Serial loop exited")
cleanup()
print("Shutdown complete")
