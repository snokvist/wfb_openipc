#!/usr/bin/env python3
# vrx_mock.py  /dev/ttyUSB0  [initial_channel_index]

import sys, struct, serial, time

PORT        = sys.argv[1]
CUR_INDEX   = int(sys.argv[2]) if len(sys.argv) > 2 else 27     # F‑band Ch 4
BAUD        = 115_200
CRC_POLY    = 0xD5

# ─────────────────────────────────────────────────────────────────────────────
# CRC‑8  (DVB‑S2)
def crc8(buf: bytes) -> int:
    crc = 0
    for b in buf:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ CRC_POLY) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc & 0xFF

# Build MSP‑v2 frame
def make_msp(fid: int, payload=b'', flags=0, direction=b'$X>'):
    body = struct.pack('<BHH', flags, fid, len(payload)) + payload
    return direction + body + bytes([crc8(body)])

# ─────────────────────────────────────────────────────────────────────────────
# Lookup tables
BANDS = ['A', 'B', 'E', 'F', 'R', 'L']
FREQ_TABLE_KHZ = [
    5865,5845,5825,5805,5785,5765,5745,5725,      # A
    5733,5752,5771,5790,5809,5828,5847,5866,      # B
    5705,5685,5665,5645,5885,5905,5925,5945,      # E
    5740,5760,5780,5800,5820,5840,5860,5880,      # F
    5658,5695,5732,5769,5806,5843,5880,5917,      # R
    5333,5372,5411,5450,5489,5528,5567,5606       # L
]

WIFI_CH_TABLE = [                               # user‑supplied
    173,169,165,161,157,153,149,144,
    149,151,155,157,161,165,169,173,
    140,136,132,128,177,181,185,189,
    149,151,155,159,163,167,171,175,
    132,140,144,153,161,169,175,183,
     64, 64, 96, 96, 96,106,114,122
]

def idx_to_text(idx: int) -> str:
    """Band/chan + analogue freq + Wi‑Fi channel."""
    band = BANDS[idx // 8]
    chan = idx % 8 + 1
    freq = FREQ_TABLE_KHZ[idx] if idx < 48 else '???'
    wifi = WIFI_CH_TABLE[idx]  if idx < 48 else '?'
    return f"{band}{chan} ({freq} MHz | Wi‑Fi ch{wifi})"

# ─────────────────────────────────────────────────────────────────────────────
# State we emulate
state = {
    "index"        : CUR_INDEX,
    "frequency_khz": FREQ_TABLE_KHZ[CUR_INDEX],
    "recording"    : 0,
    "vrx_mode"     : 0,           # 0 = RF
    "rssi"         : 200,
    "voltage_mv10" : 1220,        # 12.20 V
    "fw_string" : "MockVRX 1.1‑wifi‑print".encode("utf‑8"),
}

# Pretty print helper
def verbose(fid: int, payload: bytes, incoming=True):
    arrow = "←" if incoming else "→"
    if fid == 0x0300:                                 # GET_CHANNEL_INDEX
        return f"{arrow} GET_CHANNEL_INDEX"
    if fid == 0x0301:                                 # SET_CHANNEL_INDEX
        idx = payload[0] if payload else state['index']
        return f"{arrow} SET_CHANNEL_INDEX: {idx_to_text(idx)}"
    if fid in (0x0302, 0x0303):                       # GET/SET_FREQ
        if payload:
            freq = struct.unpack('<I', payload)[0]
            return f"{arrow} SET_FREQUENCY: {freq/1000:.3f} MHz"
        return f"{arrow} GET_FREQUENCY"
    if fid == 0x0304:
        return f"{arrow} GET_RECORDING_STATE"
    if fid == 0x0305:
        if len(payload) >= 3:
            st, dly = payload[0], struct.unpack('<H', payload[1:3])[0]
            return f"{arrow} SET_RECORDING_STATE: state={st} delay={dly} ms"
        return f"{arrow} SET_RECORDING_STATE (len={len(payload)})"
    if fid in (0x0306, 0x0307):
        return (f"{arrow} {'SET' if fid==0x0307 else 'GET'}_VRX_MODE"
                + (f": mode={payload[0]}" if payload else ""))
    if fid == 0x0308:
        return f"{arrow} GET_RSSI"
    if fid == 0x0309:
        return f"{arrow} GET_BATTERY_VOLTAGE"
    if fid == 0x030A:
        return f"{arrow} GET_FIRMWARE"
    if fid == 0x030B:
        return f"{arrow} SET_BUZZER: {'ON' if payload[0] else 'OFF'}"
    if fid == 0x030C:
        return f"{arrow} SET_OSD_ELEMENT (len={len(payload)})"
    if fid == 0x030D:
        return f"{arrow} SET_HEAD_TRACKING: {'enable' if payload[0] else 'disable'}"
    if fid == 0x0383:
        if len(payload) >= 5:
            x, y, btn = struct.unpack('<hhB', payload[:5])
            return f"{arrow} SET_PTR: dx={x} dy={y} btn={btn}"
        return f"{arrow} SET_PTR (len={len(payload)})"
    return f"{arrow} UNKNOWN fid 0x{fid:04X}"

# ─────────────────────────────────────────────────────────────────────────────
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

            if crc8(body_crc[:-1]) != body_crc[-1]:
                print("CRC error, dropping frame")
                buf = buf[start+frame_len:]
                continue

            print(verbose(fid, payload, True))

            # --------------- emulate behaviour & craft reply ---------------
            reply_payload = b''
            if fid == 0x0300:                                # GET_CHANNEL_INDEX
                reply_payload = bytes([state["index"]])

            elif fid == 0x0301:                              # SET_CHANNEL_INDEX
                state["index"] = payload[0]
                state["frequency_khz"] = FREQ_TABLE_KHZ[state["index"]]
                print(f"   ↳ channel updated to {idx_to_text(state['index'])}")
                reply_payload = payload                      # echo

            elif fid == 0x0302:                              # GET_FREQUENCY
                reply_payload = struct.pack('<I', state["frequency_khz"])

            elif fid == 0x0303:                              # SET_FREQUENCY
                freq = struct.unpack('<I', payload)[0]
                state["frequency_khz"] = freq
                print(f"   ↳ frequency updated to {freq/1000:.3f} MHz")
                reply_payload = struct.pack('<I', freq)

            elif fid == 0x0304:                              # GET_RECORDING_STATE
                reply_payload = bytes([state["recording"]])

            elif fid == 0x0305:                              # SET_RECORDING_STATE
                state["recording"] = payload[0]
                reply_payload = payload                      # echo full 3 B

            elif fid == 0x0306:                              # GET_VRX_MODE
                reply_payload = bytes([state["vrx_mode"]])

            elif fid == 0x0307:                              # SET_VRX_MODE
                state["vrx_mode"] = payload[0]
                reply_payload = payload

            elif fid == 0x0308:                              # GET_RSSI
                reply_payload = bytes([state["rssi"]])

            elif fid == 0x0309:                              # GET_BATTERY_VOLTAGE
                reply_payload = struct.pack('<H', state["voltage_mv10"])

            elif fid == 0x030A:                              # GET_FIRMWARE
                reply_payload = state["fw_string"] + b'\x00'

            elif fid in (0x030B, 0x030C, 0x030D, 0x0383):
                reply_payload = payload

            # send reply unless NO_REPLY flag is set
            if not (flags & 0x01):
                ser.write(make_msp(fid, reply_payload, 0, b'$X>'))
                print(verbose(fid, reply_payload, False))

            buf = buf[start + frame_len:]
