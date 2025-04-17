#!/usr/bin/env python3
# vrx_mock.py  /dev/ttyUSB0  [initial_channel_index]
#
# – Acts as a dummy HDZero/analog VRX for ExpressLRS backpacks
# – Prints human‑readable info *plus* raw frame details in [ … ]
# – Echoes a valid $X> reply for every request (unless NO_REPLY flag set)
# – NEW: executes `kill -SIGUSR1 $(pidof pixelpilot)` on every
#        SET_RECORDING_STATE, using the VRX‑supplied delay (0 = immediate).
#        While the command (or its timer) is active, additional requests
#        are ignored until it finishes.

import sys, struct, serial, time, subprocess, threading

# ────────── Setup ───────────────────────────────────────────────────────────
PORT      = sys.argv[1]
CUR_INDEX = int(sys.argv[2]) if len(sys.argv) > 2 else 27     # default F4
BAUD      = 115_200
CRC_POLY  = 0xD5
CMD       = "kill -SIGUSR1 $(pidof pixelpilot)"

# ────────── CRC‑8 DVB‑S2 helper ────────────────────────────────────────────
def crc8(buf: bytes) -> int:
    crc = 0
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc & 0xFF

def make_msp(fid: int, payload=b'', flags=0, direction=b'$X>'):
    body = struct.pack('<BHH', flags, fid, len(payload)) + payload
    return direction + body + bytes([crc8(body)])

# ────────── Lookup tables ──────────────────────────────────────────────────
BANDS = ['A', 'B', 'E', 'F', 'R', 'L']
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

def idx_to_text(idx: int) -> str:
    band = BANDS[idx // 8]
    chan = idx % 8 + 1
    freq = FREQ_TABLE_KHZ[idx] if idx < 48 else '???'
    wifi = WIFI_CH_TABLE[idx]  if idx < 48 else '?'
    return f"{band}{chan} ({freq} MHz | Wi‑Fi ch{wifi})"

# ────────── Emulated VRX state ─────────────────────────────────────────────
state = {
    "index"        : CUR_INDEX,
    "frequency_khz": FREQ_TABLE_KHZ[CUR_INDEX],
    "recording"    : 0,
    "vrx_mode"     : 0,           # 0 = RF
    "rssi"         : 200,
    "voltage_mv10" : 1220,        # 12.20 V
    "fw_string"    : b"MockVRX 1.2-cmd",
}

# ────────── System‑command scheduler ───────────────────────────────────────
command_pending = False           # guarded only by GIL – fine for this script
pending_timer   = None

def _run_cmd():
    global command_pending
    print(f"   ↳ Executing system command: {CMD}")
    # Use bash so $() works; ignore return code
    subprocess.call(["bash", "-c", CMD])
    command_pending = False
    print("   ↳ Command finished")

def schedule_cmd(delay_sec: int):
    """Run CMD after delay_sec seconds; ignore if one is already pending."""
    global command_pending, pending_timer
    if command_pending:
        print("   ↳ Command already pending – new request ignored")
        return
    command_pending = True
    if delay_sec == 0:
        _run_cmd()
    else:
        print(f"   ↳ Scheduling command in {delay_sec}s")
        pending_timer = threading.Timer(delay_sec, _run_cmd)
        pending_timer.start()

# ────────── Pretty printer ────────────────────────────────────────────────
def verbose(fid: int, payload: bytes, incoming=True):
    a = "←" if incoming else "→"
    if fid == 0x0300:
        return f"{a} GET_CHANNEL_INDEX"
    if fid == 0x0301:
        idx = payload[0] if payload else state["index"]
        return f"{a} SET_CHANNEL_INDEX: {idx_to_text(idx)}"
    if fid in (0x0302, 0x0303):
        if payload:
            freq = struct.unpack('<I', payload)[0]
            return f"{a} SET_FREQUENCY: {freq/1000:.3f} MHz"
        return f"{a} GET_FREQUENCY"
    if fid == 0x0304:
        return f"{a} GET_RECORDING_STATE"
    if fid == 0x0305:
        if len(payload) >= 3:
            st, dly = payload[0], struct.unpack('<H', payload[1:3])[0]
            return f"{a} SET_RECORDING_STATE: state={st} delay={dly} s"
        return f"{a} SET_RECORDING_STATE (len={len(payload)})"
    if fid in (0x0306, 0x0307):
        return (f"{a} {'SET' if fid==0x0307 else 'GET'}_VRX_MODE"
                + (f": mode={payload[0]}" if payload else ""))
    if fid == 0x0308:  return f"{a} GET_RSSI"
    if fid == 0x0309:  return f"{a} GET_BATTERY_VOLTAGE"
    if fid == 0x030A:  return f"{a} GET_FIRMWARE"
    if fid == 0x030B:  return f"{a} SET_BUZZER: {'ON' if payload and payload[0] else 'OFF'}"
    if fid == 0x030C:  return f"{a} SET_OSD_ELEMENT (len={len(payload)})"
    if fid == 0x030D:  return f"{a} SET_HEAD_TRACKING: {'enable' if payload and payload[0] else 'disable'}"
    if fid == 0x0383:
        if len(payload) >= 5:
            x, y, btn = struct.unpack('<hhB', payload[:5])
            return f"{a} SET_PTR: dx={x} dy={y} btn={btn}"
        return f"{a} SET_PTR (len={len(payload)})"
    return f"{a} UNKNOWN fid 0x{fid:04X}"

# ────────── Main loop ─────────────────────────────────────────────────────
print(f"Mock VRX listening on {PORT} @ {BAUD} baud  –  Ctrl‑C to quit")

with serial.Serial(PORT, BAUD, timeout=0.05) as ser:
    buf = b''

    while True:
        buf += ser.read(128)

        while b'$X<' in buf:
            start = buf.index(b'$X<')
            if len(buf) < start + 8:
                break

            flags, fid, length = struct.unpack('<BHH', buf[start+3:start+8])
            frame_len = 3 + 5 + length + 1
            if len(buf) < start + frame_len:
                break

            body_crc = buf[start+3:start+3+5+length+1]
            payload  = body_crc[5:-1]
            crc_ok   = crc8(body_crc[:-1]) == body_crc[-1]

            raw = (f"[fid=0x{fid:04X} len={length:2} flags=0x{flags:02X} "
                   f"crc={'OK' if crc_ok else 'BAD'} "
                   f"payload={payload.hex() or '-'}]")

            print(verbose(fid, payload, True), raw)

            if not crc_ok:
                buf = buf[start+frame_len:]
                continue

            # ------------ emulate behaviour and craft reply ----------------
            rp = b''

            if fid == 0x0300:                       # GET_CHANNEL_INDEX
                rp = bytes([state["index"]])

            elif fid == 0x0301:                     # SET_CHANNEL_INDEX
                idx = payload[0]
                state["index"] = idx
                state["frequency_khz"] = FREQ_TABLE_KHZ[idx]
                print(f"   ↳ channel updated to {idx_to_text(idx)}")
                rp = payload                        # echo index

            elif fid == 0x0302:                     # GET_FREQUENCY
                rp = struct.pack('<I', state["frequency_khz"])

            elif fid == 0x0303:                     # SET_FREQUENCY
                freq = struct.unpack('<I', payload)[0]
                state["frequency_khz"] = freq
                print(f"   ↳ frequency updated to {freq/1000:.3f} MHz")
                rp = payload

            elif fid == 0x0304:                     # GET_RECORDING_STATE
                rp = bytes([state["recording"]])

            elif fid == 0x0305:                     # SET_RECORDING_STATE
                st, dly = payload[0], struct.unpack('<H', payload[1:3])[0]
                state["recording"] = st
                rp = payload                        # echo state + delay
                schedule_cmd(dly)                   # ← new feature

            elif fid == 0x0306:                     # GET_VRX_MODE
                rp = bytes([state["vrx_mode"]])

            elif fid == 0x0307:                     # SET_VRX_MODE
                state["vrx_mode"] = payload[0]
                rp = payload

            elif fid == 0x0308:                     # GET_RSSI
                rp = bytes([state["rssi"]])

            elif fid == 0x0309:                     # GET_BATTERY_VOLTAGE
                rp = struct.pack('<H', state["voltage_mv10"])

            elif fid == 0x030A:                     # GET_FIRMWARE
                rp = state["fw_string"] + b'\x00'

            elif fid in (0x030B, 0x030C, 0x030D, 0x0383):
                rp = payload                        # simple echo

            # reply unless NO_REPLY flag bit 0 set
            if not (flags & 0x01):
                ser.write(make_msp(fid, rp, 0, b'$X>'))
                print(verbose(fid, rp, False), raw.replace('←', '→'))

            buf = buf[start + frame_len:]
