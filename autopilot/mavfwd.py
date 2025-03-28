#!/usr/bin/env python3
import argparse
import time
import subprocess
import socket
import struct
import threading
import msgpack
from pymavlink import mavutil
import alink
import flask_api  # renamed file to avoid conflict with the installed flask package

# Shared state class to hold all global data shared between modules.
class SharedState:
    def __init__(self):
        self.available_params = {}
        self.params_lock = threading.Lock()
        self.streaming_history = []
        self.history_by_type = {}
        self.alink_sent_history = {}
        self.history_lock = threading.Lock()
        self.dest_mav_conn = None  # To be set when the MAVLink connection is established

# Helper function to add a message (with timestamp) to the history.
def add_to_history(msg_dict, shared_state):
    msg_dict['timestamp'] = time.time()
    with shared_state.history_lock:
        shared_state.streaming_history.append(msg_dict)
        if len(shared_state.streaming_history) > 300:
            shared_state.streaming_history.pop(0)
        mtype = msg_dict.get("mavpackettype", "unknown")
        if mtype not in shared_state.history_by_type:
            shared_state.history_by_type[mtype] = []
        shared_state.history_by_type[mtype].append(msg_dict)
        if len(shared_state.history_by_type[mtype]) > 300:
            shared_state.history_by_type[mtype].pop(0)

# Helper to log ALINK-sent messages.
def add_to_alink_sent_history(msg_dict, shared_state):
    msg_dict['timestamp'] = time.time()
    with shared_state.history_lock:
        if 'RADIO_STATUS' not in shared_state.alink_sent_history:
            shared_state.alink_sent_history['RADIO_STATUS'] = []
        shared_state.alink_sent_history['RADIO_STATUS'].append(msg_dict)
        if len(shared_state.alink_sent_history['RADIO_STATUS']) > 300:
            shared_state.alink_sent_history['RADIO_STATUS'].pop(0)

# Global shutdown event for graceful termination.
shutdown_event = threading.Event()

# ----------------------
# SYNC MODE FUNCTION
# ----------------------
def run_sync_mode(args, shared_state):
    print("Entering sync mode: Listening for MAVLink packets on UDP port 14550 (ELRS backpack broadcast)...")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 14550))
    data, addr = sock.recvfrom(2048)
    remote_ip, remote_port = addr
    mav_temp = mavutil.mavlink.MAVLink(None)
    try:
        msg = mav_temp.decode(data)
    except Exception:
        msg = None
    if args.verbose:
        print(f"Sync mode packet received from {remote_ip}:{remote_port}:")
        if msg:
            print(msg.to_dict())
        else:
            print(data)
    else:
        print(f"Sync mode packet received from {remote_ip}:{remote_port}")
    print("Exiting sync mode. Transitioning to ELRS normal mode (heartbeat + parameter request).")
    sock.close()
    in_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    hb_conn = mavutil.mavlink_connection(f'udpout:{remote_ip}:{remote_port}')
    shared_state.dest_mav_conn = hb_conn
    forward_conn = None
    if args.forward is not None:
        forward_conn = mavutil.mavlink_connection(f'udpout:127.0.0.1:{args.forward}')
    print(f"ELRS normal mode: Listening on port 14550 and sending heartbeats to {remote_ip}:{remote_port}" +
          (f", and forwarding to port {args.forward}" if forward_conn else ""))
    print("Requesting parameter list...")
    hb_conn.mav.param_request_list_send(1, 1)
    param_requested = {}
    if args.alink:
        print("Starting ALINK thread to fetch Wi-Fi data and send RADIO_STATUS messages...")
        alink_thread = threading.Thread(target=alink.run_alink_thread, args=(hb_conn, shutdown_event, shared_state), daemon=True)
        alink_thread.start()
    last_hb_time = time.time()
    hb_interval = 1.0  # seconds
    while not shutdown_event.is_set():
        now = time.time()
        if now - last_hb_time >= hb_interval:
            hb_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0
            )
            if args.verbose:
                print("Heartbeat sent.")
            last_hb_time = now
        msg = in_conn.recv_match(blocking=False)
        if msg:
            msg_dict = msg.to_dict()
            add_to_history(msg_dict, shared_state)
            if args.verbose:
                print(f"Received message: {msg_dict}")
            if msg.get_type() == "PARAM_VALUE":
                if isinstance(msg.param_id, bytes):
                    param_id = msg.param_id.decode('utf-8').strip('\x00')
                else:
                    param_id = msg.param_id.strip('\x00')
                print(f"Parameter received: {param_id} = {msg.param_value}")
                with shared_state.params_lock:
                    shared_state.available_params[param_id] = {
                        "value": msg.param_value,
                        "type": msg.param_type,
                        "index": msg.param_index,
                        "count": msg.param_count
                    }
                if param_id not in param_requested:
                    print(f"Requesting read for parameter {param_id}...")
                    param_id_bytes = param_id.encode('ascii')
                    hb_conn.mav.param_request_read_send(1, 1, param_id_bytes, -1)
                    param_requested[param_id] = True
            if forward_conn:
                raw_msg = msg.get_msgbuf()
                forward_conn.write(raw_msg)
        else:
            time.sleep(0.01)
    print("Shutdown signal received in sync mode. Exiting run_sync_mode.")

# ----------------------
# MANUAL MODE FUNCTION
# ----------------------
def run_normal_mode(args, shared_state):
    if args.out is None:
        print("Error: In manual mode, --out must be provided for the bidirectional connection.")
        return
    in_conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.in_port}')
    hb_conn = mavutil.mavlink_connection(f'udpout:{args.target}:{args.out}')
    shared_state.dest_mav_conn = hb_conn
    forward_conn = None
    if args.forward is not None:
        forward_conn = mavutil.mavlink_connection(f'udpout:127.0.0.1:{args.forward}')
    print(f"Manual mode: Listening on UDP port {args.in_port}, sending heartbeats to {args.target}:{args.out}" +
          (f", and forwarding to port {args.forward}" if forward_conn else ""))
    channel_state = {}
    wait_seconds = args.wait / 1000.0
    persist_seconds = args.persist / 1000.0
    tolerance_ratio = args.tolerance / 100.0
    last_hb_time = time.time()
    hb_interval = 1.0
    while not shutdown_event.is_set():
        now = time.time()
        if now - last_hb_time >= hb_interval:
            hb_conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GENERIC,
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, 0
            )
            if args.verbose:
                print("Heartbeat sent.")
            last_hb_time = now
        msg = in_conn.recv_match(blocking=False)
        if msg:
            msg_dict = msg.to_dict()
            add_to_history(msg_dict, shared_state)
            if args.verbose:
                print(f"Received message: {msg_dict}")
                print("Full message details:")
                print(msg.to_dict())
            if forward_conn:
                raw_msg = msg.get_msgbuf()
                forward_conn.write(raw_msg)
            if args.channels and msg.get_type() == 'RC_CHANNELS_OVERRIDE':
                for ch in args.channels_list:
                    attr_name = f'chan{ch}_raw'
                    if not hasattr(msg, attr_name):
                        continue
                    value = getattr(msg, attr_name)
                    now = time.time()
                    if ch not in channel_state:
                        channel_state[ch] = {
                            "candidate": value,
                            "stable_since": now,
                            "last_triggered_value": value,
                            "last_executed": 0
                        }
                        continue
                    state = channel_state[ch]
                    if abs(value - state["candidate"]) <= tolerance_ratio * state["candidate"]:
                        if (now - state["stable_since"] >= persist_seconds) and (now - state["last_executed"] >= wait_seconds):
                            if abs(value - state["last_triggered_value"]) > tolerance_ratio * state["last_triggered_value"]:
                                cmd = ["/usr/bin/channels.sh", str(ch), str(value)]
                                subprocess.Popen(cmd)
                                state["last_triggered_value"] = value
                                state["last_executed"] = now
                    else:
                        state["candidate"] = value
                        state["stable_since"] = now
        else:
            time.sleep(0.01)
    print("Shutdown signal received in manual mode. Exiting run_normal_mode.")

# ----------------------
# Main Entry Point
# ----------------------
def main():
    parser = argparse.ArgumentParser(
        description="Replicate and process MAVLink UDP streams with multiple modes and a Flask API."
    )
    parser.add_argument('-i', '--in', dest='in_port', type=int, default=14600,
                        help="Local UDP port to listen for MAVLink messages (manual mode).")
    parser.add_argument('-o', '--out', type=int, default=None,
                        help=("Remote output port for bidirectional connection with the ELRS backpack (manual mode). "
                              "In manual mode, --target must be specified."))
    parser.add_argument('--target', type=str, default="127.0.0.1",
                        help="Remote target IP for the ELRS backpack in manual mode (default: 127.0.0.1).")
    parser.add_argument('-f', '--forward', type=int, default=None,
                        help="Local UDP port to forward received MAVLink packets (optional, works in both modes).")
    parser.add_argument('-c', '--channels', type=str, default=None,
                        help=("RC channels to monitor (comma-separated list, e.g., '5,6,7'). "
                              "If not provided, RC channel logic is skipped."))
    parser.add_argument('-w', '--wait', type=int, default=2000,
                        help="Delay in milliseconds after each command execution (default: 2000).")
    parser.add_argument('-p', '--persist', type=int, default=0,
                        help="Time in milliseconds that a channel value must persist to generate a command (default: 0).")
    parser.add_argument('--tolerance', type=float, default=3.0,
                        help="Tolerance percentage for value stability (default: 3.0%%).")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Display detailed MAVLink packet information (default: off).")
    parser.add_argument('--sync', action='store_true',
                        help="Activate sync mode: listen on UDP port 14550 for an ELRS broadcast. "
                             "In sync mode, --out is ignored and destination is derived from the sync packet.")
    parser.add_argument('--alink', action='store_true',
                        help="In sync mode, also connect to the JSON server (ALINK) to fetch Wi-Fi data and send RADIO_STATUS messages.")
    args = parser.parse_args()
    if args.channels is not None:
        args.channels_list = [int(ch.strip()) for ch in args.channels.split(',') if ch.strip()]
    else:
        args.channels_list = []
    shared_state = SharedState()
    flask_thread = threading.Thread(target=flask_api.run_flask, args=(shared_state,), daemon=True)
    flask_thread.start()
    try:
        if args.sync:
            run_sync_mode(args, shared_state)
        else:
            run_normal_mode(args, shared_state)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received in main thread.")
    finally:
        shutdown_event.set()
        print("Shutdown event set. Waiting for threads to exit...")
        time.sleep(1)
        print("Exiting main.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        shutdown_event.set()
        print("Exiting due to KeyboardInterrupt.")
