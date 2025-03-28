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
import flask_api  # renamed file to avoid conflict with the installed Flask package

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
# SYNC MODE FUNCTION (only mode available)
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
    print("Exiting sync mode. Transitioning to ELRS mode (heartbeat + parameter request).")
    sock.close()
    # Create incoming connection on port 14550.
    in_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Create outgoing connection using the extracted remote IP and port.
    hb_conn = mavutil.mavlink_connection(f'udpout:{remote_ip}:{remote_port}')
    shared_state.dest_mav_conn = hb_conn
    forward_conn = None
    if args.forward is not None:
        forward_conn = mavutil.mavlink_connection(f'udpout:127.0.0.1:{args.forward}')
    print(f"ELRS mode: Listening on port 14550 and sending heartbeats to {remote_ip}:{remote_port}" +
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
# Main Entry Point (only sync mode is supported)
# ----------------------
def main():
    parser = argparse.ArgumentParser(
        description="Replicate and process MAVLink UDP streams in sync mode with a Flask API."
    )
    # In sync mode, many normal mode parameters are not needed.
    parser.add_argument('-f', '--forward', type=int, default=None,
                        help="Local UDP port to forward received MAVLink packets (optional).")
    parser.add_argument('-c', '--channels', type=str, default=None,
                        help="RC channels to monitor (comma-separated list, e.g., '5,6,7'). If not provided, RC channel logic is skipped.")
    parser.add_argument('-w', '--wait', type=int, default=2000,
                        help="Delay in milliseconds after each command execution (default: 2000).")
    parser.add_argument('-p', '--persist', type=int, default=0,
                        help="Time in milliseconds that a channel value must persist to generate a command (default: 0).")
    parser.add_argument('--tolerance', type=float, default=3.0,
                        help="Tolerance percentage for value stability (default: 3.0%%).")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Display detailed MAVLink packet information (default: off).")
    parser.add_argument('--alink', action='store_true',
                        help="Also connect to the JSON server (ALINK) to fetch Wi-Fi data and send RADIO_STATUS messages.")
    # Although we no longer use --sync as a flag, we still include it for clarity.
    parser.add_argument('--sync', action='store_true',
                        help="Activate sync mode (the only mode available).")
    args = parser.parse_args()
    if args.channels is not None:
        args.channels_list = [int(ch.strip()) for ch in args.channels.split(',') if ch.strip()]
    else:
        args.channels_list = []
    shared_state = SharedState()
    # Start the Flask API in its own daemon thread.
    flask_thread = threading.Thread(target=flask_api.run_flask, args=(shared_state,), daemon=True)
    flask_thread.start()
    try:
        run_sync_mode(args, shared_state)
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
