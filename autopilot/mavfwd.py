#!/usr/bin/env python

import argparse
import time
import subprocess
import socket
import struct
import threading
from pymavlink import mavutil
import msgpack

# --- Global variables used by the WiFi thread ---
# These globals are updated by calculate_link_health (you can tune or change the logic as needed)
link_health_score_rssi = 1000
link_health_score_snr = 1000
best_antennas_rssi = [-105, -105, -105, -105]
best_antennas_snr = [-105, -105, -105, -105]

# --- Functions from your WiFi code (extracted parts) ---

def calculate_link_health(video_rx):
    """
    Calculate link health from the JSON (msgpack) data.
    This function extracts RSSI/SNR info from the received dictionary.
    """
    global link_health_score_rssi, link_health_score_snr, best_antennas_rssi, best_antennas_snr
    try:
        # Example: assume video_rx contains a dict "rx_ant_stats" with antenna measurements.
        rx_ant_stats = video_rx.get('rx_ant_stats', {})
        rssi_values = []
        snr_values = []
        if rx_ant_stats:
            for antenna in rx_ant_stats.values():
                if len(antenna) >= 6:
                    rssi_values.append(antenna[2])
                    snr_values.append(antenna[5])
            rssi_values.sort(reverse=True)
            snr_values.sort(reverse=True)
            best_antennas_rssi = rssi_values[:4] if rssi_values else [-105]*4
            best_antennas_snr = snr_values[:4] if snr_values else [-105]*4
            # Simple linear mapping example:
            rssi_to_use = best_antennas_rssi[0]
            # Map from a range (e.g., -80 to -30) to a score between 1000 and 2000.
            if rssi_to_use < -80:
                link_health_score_rssi = 1000
            elif rssi_to_use > -30:
                link_health_score_rssi = 2000
            else:
                link_health_score_rssi = 1000 + ((rssi_to_use + 80) / 50) * 1000
            # Similar for SNR (assume range 12 to 38)
            snr_to_use = best_antennas_snr[0]
            if snr_to_use < 12:
                link_health_score_snr = 1000
            elif snr_to_use > 38:
                link_health_score_snr = 2000
            else:
                link_health_score_snr = 1000 + ((snr_to_use - 12) / 26) * 1000
        else:
            link_health_score_rssi = 1000
            link_health_score_snr = 1000
    except Exception as e:
        print(f"[WiFi Data] Error in calculate_link_health: {e}")
        link_health_score_rssi = 1000
        link_health_score_snr = 1000
    return link_health_score_rssi, link_health_score_snr

def send_radio_status_via_conn(hb_conn, rssi, snr, recovered_packets):
    """
    Build and send a RADIO_STATUS message over the given MAVLink connection.
    The mapping here is similar to your original function.
    """
    def convert_health(x):
        if x == 999:
            return 0
        return int(round((x - 999) * 254 / 1001))  # (2000 - 999) = 1001

    rssi_value = convert_health(rssi)
    noise_value = convert_health(snr)
    # Map best antenna RSSI (assumed in range -128 to 128) to 0-254
    rem_rssi = int(round((best_antennas_rssi[0] + 128) * 254 / 256))
    # Map best antenna SNR (assumed in range 0 to 50) to 0-254
    rem_snr_raw = best_antennas_snr[0]
    if rem_snr_raw < 0:
        rem_snr_raw = 0
    elif rem_snr_raw > 50:
        rem_snr_raw = 50
    rem_noise = int(round(rem_snr_raw * 254 / 50))
    fixed = min(recovered_packets, 254)
    rxerrors = fixed  # For simplicity
    hb_conn.mav.radio_status_send(rssi_value, rem_rssi, 0, noise_value, rem_noise, rxerrors, fixed)
    print(f"[WiFi Data] Sent RADIO_STATUS: rssi={rssi_value}, rem_rssi={rem_rssi}, noise={noise_value}, rem_noise={rem_noise}, rxerrors={rxerrors}, fixed={fixed}")

def send_statustext(hb_conn, message):
    """
    Send a STATUSTEXT message with the given message string.
    """
    # MAVLink STATUSTEXT messages typically have a maximum of 50 characters.
    message = message[:50]
    hb_conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO, message.encode('ascii', errors='replace'))
    print(f"[WiFi Data] Sent STATUSTEXT: {message}")

def wifi_data_thread(hb_conn):
    """
    Connect to the JSON (msgpack) server to get WiFi connection data,
    calculate link health, and send RADIO_STATUS and STATUSTEXT messages
    to the autopilot.
    
    This function runs in its own thread.
    """
    json_host = "127.0.0.1"   # Set these as needed
    json_port = 8003
    retry_interval = 1  # seconds
    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                print(f"[WiFi Data] Connecting to {json_host}:{json_port}...")
                client_socket.connect((json_host, json_port))
                print(f"[WiFi Data] Connected to {json_host}:{json_port}")
                while True:
                    length_prefix = client_socket.recv(4)
                    if not length_prefix:
                        print("[WiFi Data] Connection closed by server.")
                        break
                    msg_length = struct.unpack('!I', length_prefix)[0]
                    data = b""
                    while len(data) < msg_length:
                        chunk = client_socket.recv(min(4096, msg_length - len(data)))
                        if not chunk:
                            break
                        data += chunk
                    if len(data) == msg_length:
                        try:
                            unpacked_data = msgpack.unpackb(data, use_list=False, strict_map_key=False)
                            # For example, if the data type is "rx" with id "video rx":
                            if unpacked_data.get("type") == "rx" and unpacked_data.get("id") == "video rx":
                                rssi, snr = calculate_link_health(unpacked_data)
                                fec_rec = unpacked_data.get("packets", {}).get("fec_rec", [0, 0])
                                recovered_pkt = fec_rec[0] if fec_rec else 0
                                send_radio_status_via_conn(hb_conn, rssi, snr, recovered_pkt)
                            # Send any special messages as STATUSTEXT:
                            if "special_request" in unpacked_data:
                                special_message = f"special:{unpacked_data['special_request']}"
                                send_statustext(hb_conn, special_message)
                        except Exception as e:
                            print(f"[WiFi Data] Error unpacking data: {e}")
                    else:
                        print("[WiFi Data] Incomplete data received.")
                        break
        except Exception as e:
            print(f"[WiFi Data] Connection error: {e}. Retrying in {retry_interval} seconds...")
            time.sleep(retry_interval)

# --- End of WiFi data functions ---


# --- Existing sync-mode and normal-mode functions ---

def run_sync_mode(args):
    """
    Sync mode:
      - Listen on UDP port 14550 for an ELRS backpack broadcast.
      - Extract the source IP and source port from the first received packet.
      - Print the packet in verbose mode.
      - Exit sync mode and transition to a heartbeat/parameter request mode:
          * Incoming port is forced to 14550.
          * Outgoing heartbeats (and parameter requests) are sent to the extracted remote IP and port.
          * Immediately request the parameter list.
          * For each received PARAM_VALUE, print the parameter and, once, request a read.
          * Optionally forward received packets if --forward is provided.
          * If --alink is specified, start a separate thread to send WiFi data messages.
    """
    print("Entering sync mode: Listening for MAVLink packets on UDP port 14550 (ELRS backpack broadcast)...")
    # Bind a raw UDP socket to capture the sender's address.
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 14550))
    data, addr = sock.recvfrom(2048)
    remote_ip, remote_port = addr

    # Attempt to decode the MAVLink message for logging purposes.
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
    param_requested = {}

    # If --alink is provided, start the WiFi data thread.
    if args.alink:
        wifi_thread = threading.Thread(target=wifi_data_thread, args=(hb_conn,), daemon=True)
        wifi_thread.start()
        print("Started WiFi data thread (--alink enabled).")

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

        msg = in_conn.recv_match(blocking=False)
        if msg:
            if args.verbose:
                print(f"Received message: {msg.to_dict()}")
            if msg.get_type() == "PARAM_VALUE":
                if isinstance(msg.param_id, bytes):
                    param_id = msg.param_id.decode('utf-8').strip('\x00')
                else:
                    param_id = msg.param_id.strip('\x00')
                print(f"Parameter received: {param_id} = {msg.param_value}")
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

    # (RC channel processing code would go here if needed.)
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
                print(f"Received message: {msg.to_dict()}")
            if forward_conn:
                raw_msg = msg.get_msgbuf()
                forward_conn.write(raw_msg)
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
                        help="Remote target IP for the ELRS backpack in manual mode (default: 127.0.0.1).")
    parser.add_argument('-f', '--forward', type=int, default=None,
                        help="Local UDP port to forward received MAVLink packets (optional, works in both modes).")
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
    # Sync mode argument.
    parser.add_argument('--sync', action='store_true',
                        help="Activate sync mode: listen on UDP port 14550 for an ELRS broadcast. "
                             "When a packet is received, use its source IP and source port for heartbeat transmission, "
                             "parameter requests, and (if --alink is provided) WiFi data messages.")
    # Optional alink argument for WiFi data integration in sync mode.
    parser.add_argument('--alink', action='store_true',
                        help="Enable WiFi data integration: connect to the JSON server and send MAVLink RADIO_STATUS and STATUSTEXT messages.")

    args = parser.parse_args()

    if args.sync:
        run_sync_mode(args)
    else:
        run_normal_mode(args)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Exiting...")
