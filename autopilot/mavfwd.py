#!/usr/bin/env python

import argparse
import time
import subprocess
import socket
from pymavlink import mavutil

def parse_channels(channels_str):
    """Parse a comma-separated list of channel numbers into a list of ints."""
    return [int(ch.strip()) for ch in channels_str.split(',') if ch.strip()]

def run_sync_mode(args):
    """
    Sync mode:
      - Listen on UDP port 14550 for an ELRS backpack broadcast.
      - Extract the source IP and source port from the first received packet.
      - Print the packet in verbose mode.
      - Exit sync mode and transition to a heartbeat-only mode with parameter requests:
            * Incoming port is forced to 14550.
            * Outgoing heartbeats (and parameter requests) are sent to the extracted remote IP and port.
            * Immediately request the parameter list.
            * For each received PARAM_VALUE, print the parameter and, once, request a read.
            * Optionally forward received packets if --forward is provided.
    """
    print("Entering sync mode: Listening for MAVLink packets on UDP port 14550 (ELRS backpack broadcast)...")
    # Bind a raw UDP socket to capture the sender's address.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 14550))
    data, addr = sock.recvfrom(2048)
    remote_ip, remote_port = addr

    # Attempt to decode the MAVLink message for logging purposes.
    mav = mavutil.mavlink.MAVLink(None)
    try:
        msg = mav.decode(data)
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

    # Set up incoming connection on forced port 14550.
    in_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Outgoing connection: use the extracted remote IP and port.
    hb_conn = mavutil.mavlink_connection(f'udpout:{remote_ip}:{remote_port}')

    # Optionally, set up a forwarding connection.
    forward_conn = None
    if args.forward is not None:
        forward_conn = mavutil.mavlink_connection(f'udpout:127.0.0.1:{args.forward}')

    print(f"ELRS normal mode: Listening on port 14550 and sending heartbeats to {remote_ip}:{remote_port}" +
          (f", and forwarding to port {args.forward}" if forward_conn else ""))

    # Immediately request the parameter list.
    print("Requesting parameter list...")
    hb_conn.mav.param_request_list_send(1, 1)
    # Dictionary to record which parameters we've requested a read for.
    param_requested = {}

    last_hb_time = time.time()
    hb_interval = 1.0  # heartbeat interval in seconds

    while True:
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

        # Check for incoming messages.
        msg = in_conn.recv_match(blocking=False)
        if msg:
            if args.verbose:
                print(f"Received message: {msg.to_dict()}")
            # Process PARAM_VALUE messages.
            if msg.get_type() == "PARAM_VALUE":
                # Decode param_id from bytes if needed.
                if isinstance(msg.param_id, bytes):
                    param_id = msg.param_id.decode('utf-8').strip('\x00')
                else:
                    param_id = msg.param_id.strip('\x00')
                print(f"Parameter received: {param_id} = {msg.param_value}")
                if param_id not in param_requested:
                    print(f"Requesting read for parameter {param_id}...")
                    # Encode the parameter ID to ASCII bytes before sending.
                    param_id_bytes = param_id.encode('ascii')
                    hb_conn.mav.param_request_read_send(1, 1, param_id_bytes, -1)
                    param_requested[param_id] = True

            # Optionally forward the message.
            if forward_conn:
                raw_msg = msg.get_msgbuf()
                forward_conn.write(raw_msg)
        else:
            time.sleep(0.01)

def run_normal_mode(args):
    """
    Manual mode:
      - Listen on the user-provided --in port.
      - Set up a heartbeat connection to the remote target specified by --target and --out.
      - Optionally forward every received MAVLink packet to --forward.
      - Process RC_CHANNELS_OVERRIDE messages if --channels is provided.
    """
    if args.out is None:
        print("Error: In manual mode, --out must be provided for the bidirectional connection.")
        return

    in_conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.in_port}')
    hb_conn = mavutil.mavlink_connection(f'udpout:{args.target}:{args.out}')

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

    while True:
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
            if args.verbose:
                print(f"Received message: Type = {msg.get_type()}, ID = {msg.get_msgId() if hasattr(msg, 'get_msgId') else 'N/A'}")
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

def main():
    parser = argparse.ArgumentParser(
        description="Replicate and process MAVLink UDP streams with multiple modes."
    )
    # Manual mode arguments.
    parser.add_argument('-i', '--in', dest='in_port', type=int, default=14600,
                        help="Local UDP port to listen for MAVLink messages (manual mode).")
    parser.add_argument('-o', '--out', type=int, default=None,
                        help=("Remote output port for bidirectional connection with the ELRS backpack (manual mode). "
                              "In manual mode, --target must be specified."))
    parser.add_argument('--target', type=str, default="127.0.0.1",
                        help=("Remote target IP for the ELRS backpack in manual mode (default: 127.0.0.1)."))
    parser.add_argument('-f', '--forward', type=int, default=None,
                        help="Local UDP port to forward received MAVLink packets (optional, works in both modes).")
    parser.add_argument('-c', '--channels', type=str, default=None,
                        help=("RC channels to monitor (comma-separated list, e.g., '5,6,7'). "
                              "If not provided, RC channel logic is skipped."))
    parser.add_argument('-w', '--wait', type=int, default=2000,
                        help="Delay in milliseconds after each command execution (default: 2000).")
    parser.add_argument('-p', '--persist', type=int, default=0,
                        help=("Time in milliseconds that a channel value must persist to generate a command "
                              "(default: 0)."))
    parser.add_argument('--tolerance', type=float, default=3.0,
                        help="Tolerance percentage for value stability (default: 3.0%%).")
    parser.add_argument('-v', '--verbose', action='store_true',
                        help="Display detailed MAVLink packet information (default: off).")
    # Sync mode argument.
    parser.add_argument('--sync', action='store_true',
                        help="Activate sync mode: listen on UDP port 14550 for an ELRS broadcast. "
                             "When a packet is received, use its source IP and source port for heartbeat transmission "
                             "and request parameter list and values. In sync mode, --out is ignored.")

    args = parser.parse_args()

    if args.channels is not None:
        args.channels_list = parse_channels(args.channels)
    else:
        args.channels_list = []

    if args.sync:
        run_sync_mode(args)
    else:
        run_normal_mode(args)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
