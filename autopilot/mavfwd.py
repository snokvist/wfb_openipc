#!/usr/bin/env python3
import argparse
import time
import subprocess
import socket
import struct
import threading
import msgpack
from flask import Flask, jsonify, request, send_from_directory
from pymavlink import mavutil

#########################################
# Global Variables, Locks, and Shutdown Event
#########################################

# Global MAVLink connection used for heartbeat/parameter commands.
dest_mav_conn = None

# Dictionary holding available parameters (populated from PARAM_VALUE messages).
available_params = {}
params_lock = threading.Lock()

# List holding a history (max 300) of received MAVLink messages.
streaming_history = []
# Dictionary keyed by MAVLink type, each holding a list of messages (max 300 each).
history_by_type = {}

# Global dictionary holding ALINK sent messages (RADIO_STATUS only).
alink_sent_history = {}

# Lock for histories.
history_lock = threading.Lock()

# Global shutdown event to signal threads to exit gracefully.
shutdown_event = threading.Event()

#########################################
# Helper Functions for History Logging
#########################################

def add_to_history(msg_dict):
    """Append a message dictionary to the streaming history (max 300 items)
       and also add it to a type-indexed dictionary, with an added timestamp."""
    msg_dict['timestamp'] = time.time()
    with history_lock:
        streaming_history.append(msg_dict)
        if len(streaming_history) > 300:
            streaming_history.pop(0)
        mtype = msg_dict.get("mavpackettype", "unknown")
        if mtype not in history_by_type:
            history_by_type[mtype] = []
        history_by_type[mtype].append(msg_dict)
        if len(history_by_type[mtype]) > 300:
            history_by_type[mtype].pop(0)

def add_to_alink_sent_history(msg_dict):
    """Append a sent ALINK message (RADIO_STATUS) to the alink_sent_history (max 300 items),
       with an added timestamp."""
    msg_dict['timestamp'] = time.time()
    with history_lock:
        if 'RADIO_STATUS' not in alink_sent_history:
            alink_sent_history['RADIO_STATUS'] = []
        alink_sent_history['RADIO_STATUS'].append(msg_dict)
        if len(alink_sent_history['RADIO_STATUS']) > 300:
            alink_sent_history['RADIO_STATUS'].pop(0)

#########################################
# Core Functionality (Heartbeat, RC, etc.)
#########################################

def parse_channels(channels_str):
    """Parse a comma-separated list of channel numbers into a list of ints."""
    return [int(ch.strip()) for ch in channels_str.split(',') if ch.strip()]

#########################################
# SYNC MODE and MANUAL MODE Functions
#########################################

def run_sync_mode(args):
    """
    Sync mode:
      - Listen on UDP port 14550 for an ELRS backpack broadcast.
      - Extract the source IP and source port from the first received packet.
      - Create a heartbeat/parameter connection to that destination.
      - Immediately request the parameter list.
      - If --alink is specified, start the ALINK thread.
      - Incoming messages update available_params and are added to streaming_history.
    """
    global dest_mav_conn

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

    # Create incoming connection on port 14550.
    in_conn = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    # Create outgoing connection using the extracted remote IP and port.
    hb_conn = mavutil.mavlink_connection(f'udpout:{remote_ip}:{remote_port}')
    dest_mav_conn = hb_conn  # For Flask endpoint usage

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
        alink_thread = threading.Thread(target=run_alink_thread, args=(hb_conn,), daemon=True)
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
            add_to_history(msg_dict)
            if args.verbose:
                print(f"Received message: {msg_dict}")
            if msg.get_type() == "PARAM_VALUE":
                if isinstance(msg.param_id, bytes):
                    param_id = msg.param_id.decode('utf-8').strip('\x00')
                else:
                    param_id = msg.param_id.strip('\x00')
                print(f"Parameter received: {param_id} = {msg.param_value}")
                with params_lock:
                    available_params[param_id] = {
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

def run_normal_mode(args):
    """
    Manual mode:
      - Listen on the user-specified --in port.
      - Set up a heartbeat connection to the remote target (using --target and --out).
      - Process incoming MAVLink messages (updating history and parameters) and forward if requested.
      - Also process RC_CHANNELS_OVERRIDE messages.
    """
    global dest_mav_conn
    if args.out is None:
        print("Error: In manual mode, --out must be provided for the bidirectional connection.")
        return

    in_conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{args.in_port}')
    hb_conn = mavutil.mavlink_connection(f'udpout:{args.target}:{args.out}')
    dest_mav_conn = hb_conn
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
            add_to_history(msg_dict)
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

#########################################
# ALINK (Wi-Fi/JSON) Functions (used only when --alink is set in sync mode)
#########################################

# Hard-coded ALINK connection settings.
ALINK_HOST = "127.0.0.1"
ALINK_PORT = 8003
ALINK_RETRY_INTERVAL = 1      # seconds
ALINK_MESSAGE_INTERVAL = 0.1  # seconds
ACCUMULATOR_PERIOD = 1.0      # seconds

# Global variables for ALINK processing.
alink_recovered_packets_accumulator = 0
alink_last_accumulator_reset = time.time()
alink_best_antennas_rssi = [-105, -105, -105, -105]
alink_best_antennas_snr = [-105, -105, -105, -105]
alink_link_health_score_rssi = 1000
alink_link_health_score_snr = 1000

def update_alink_recovered(new_val):
    global alink_recovered_packets_accumulator
    alink_recovered_packets_accumulator += new_val

def calculate_alink_link_health(video_rx):
    """
    Recalculate link health from JSON/msgpack data using hard-coded parameters.
    """
    global alink_link_health_score_rssi, alink_link_health_score_snr, alink_best_antennas_rssi, alink_best_antennas_snr
    try:
        use_best_rssi = True
        min_rssi = -80.0
        max_rssi = -30.0
        min_snr = 12.0
        max_snr = 38.0

        packets = video_rx.get('packets', {})
        fec_rec = packets.get('fec_rec', [0, 0])
        recovered = fec_rec[0]
        update_alink_recovered(recovered)
        rx_ant_stats = video_rx.get('rx_ant_stats', {})
        rssi_values = []
        snr_values = []
        for antenna in rx_ant_stats.values():
            if len(antenna) >= 6:
                rssi_values.append(antenna[2])
                snr_values.append(antenna[5])
        if not rssi_values or not snr_values:
            alink_link_health_score_rssi = 1000
            alink_link_health_score_snr = 1000
            alink_best_antennas_rssi = [-105, -105, -105, -105]
            alink_best_antennas_snr = [-105, -105, -105, -105]
            return alink_link_health_score_rssi, alink_link_health_score_snr

        rssi_values.sort(reverse=True)
        snr_values.sort(reverse=True)
        alink_best_antennas_rssi = rssi_values[:4] + [-105]*(4 - len(rssi_values))
        alink_best_antennas_snr = snr_values[:4] + [-105]*(4 - len(snr_values))
        rssi_to_use = alink_best_antennas_rssi[0] if use_best_rssi else sum(alink_best_antennas_rssi)/4.0
        if rssi_to_use > max_rssi:
            alink_link_health_score_rssi = 2000
        elif rssi_to_use < min_rssi:
            alink_link_health_score_rssi = 1000
        else:
            alink_link_health_score_rssi = 1000 + ((rssi_to_use - min_rssi)/(max_rssi - min_rssi))*1000

        avg_snr = alink_best_antennas_snr[0] if use_best_rssi else sum(alink_best_antennas_snr)/4.0
        if avg_snr > max_snr:
            alink_link_health_score_snr = 2000
        elif avg_snr < min_snr:
            alink_link_health_score_snr = 1000
        else:
            alink_link_health_score_snr = 1000 + ((avg_snr - min_snr)/(max_snr - min_snr))*1000

        alink_link_health_score_rssi = round(alink_link_health_score_rssi)
        alink_link_health_score_snr = round(alink_link_health_score_snr)
        return alink_link_health_score_rssi, alink_link_health_score_snr

    except Exception as e:
        print(f"ALINK: Error calculating link health: {e}, data: {video_rx}")
        return 1000, 1000

def send_radio_status_alink(conn, recovered_packet_count):
    """
    Constructs and sends a MAVLink RADIO_STATUS message based on ALINK link health data.
    Uses the local recovered_packet_count for both fixed and rxerrors.
    Also logs the sent message in alink_sent_history.
    """
    def convert_health(x):
        return 0 if x == 999 else int(round((x - 999) * 254 / 1001))
    rssi_value = convert_health(alink_link_health_score_rssi)
    noise_value = convert_health(alink_link_health_score_snr)
    rem_rssi = int(round((alink_best_antennas_rssi[0] + 128) * 254 / 256))
    rem_snr = alink_best_antennas_snr[0]
    if rem_snr < 0:
        rem_snr = 0
    elif rem_snr > 50:
        rem_snr = 50
    rem_noise = int(round(rem_snr * 254 / 50))
    fixed = min(recovered_packet_count, 254)
    rxerrors = min(recovered_packet_count, 254)
    conn.mav.radio_status_send(rssi_value, rem_rssi, 0, noise_value, rem_noise, rxerrors, fixed)
    print(f"ALINK: Sent RADIO_STATUS: rssi={rssi_value}, rem_rssi={rem_rssi}, noise={noise_value}, rem_noise={rem_noise}, rxerrors={rxerrors}, fixed={fixed}")
    
    msg_dict = {
         "mavpackettype": "RADIO_STATUS",
         "rssi": rssi_value,
         "remrssi": rem_rssi,
         "noise": noise_value,
         "remnoise": rem_noise,
         "rxerrors": rxerrors,
         "fixed": fixed,
         "source": "alink"
    }
    add_to_alink_sent_history(msg_dict)

def run_alink_thread(conn):
    """
    Connects to the ALINK JSON/msgpack server, fetches Wi-Fi link data,
    recalculates link health, and sends a RADIO_STATUS message via the provided MAVLink connection.
    """
    global alink_recovered_packets_accumulator, alink_last_accumulator_reset
    while not shutdown_event.is_set():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                print(f"ALINK: Connecting to {ALINK_HOST}:{ALINK_PORT}...")
                client_socket.connect((ALINK_HOST, ALINK_PORT))
                print(f"ALINK: Connected to {ALINK_HOST}:{ALINK_PORT}.")
                while not shutdown_event.is_set():
                    length_prefix = client_socket.recv(4)
                    if not length_prefix:
                        print("ALINK: Connection closed by server.")
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
                            unpacked = msgpack.unpackb(data, use_list=False, strict_map_key=False)
                        except Exception as e:
                            print(f"ALINK: Failed to unpack data: {e}")
                            continue
                        if unpacked.get("type") == "rx" and unpacked.get("id") == "video rx":
                            calculate_alink_link_health(unpacked)
                            now = time.time()
                            if now - alink_last_accumulator_reset >= ACCUMULATOR_PERIOD:
                                recovered_packet_count = alink_recovered_packets_accumulator
                                alink_recovered_packets_accumulator = 0
                                alink_last_accumulator_reset = now
                            else:
                                recovered_packet_count = alink_recovered_packets_accumulator
                            send_radio_status_alink(conn, recovered_packet_count)
                    time.sleep(ALINK_MESSAGE_INTERVAL)
        except Exception as e:
            print(f"ALINK: Connection failed or lost: {e}. Retrying in {ALINK_RETRY_INTERVAL} seconds...")
            time.sleep(ALINK_RETRY_INTERVAL)
    print("Shutdown signal received in ALINK thread. Exiting run_alink_thread.")

#########################################
# Flask Web API (runs in its own thread)
#########################################

def run_flask():
    app = Flask(__name__)

    @app.route("/parameters", methods=["GET"])
    def get_parameters():
        with params_lock:
            return jsonify(available_params)

    @app.route("/parameters/<param_id>", methods=["GET"])
    def get_parameter(param_id):
        with params_lock:
            param = available_params.get(param_id)
        if param is None:
            return jsonify({"error": "Parameter not found"}), 404
        return jsonify({param_id: param})

    @app.route("/parameters/<param_id>", methods=["POST"])
    def set_parameter(param_id):
        if not request.json or "value" not in request.json:
            return jsonify({"error": "Missing new value in request"}), 400
        new_value = request.json["value"]
        with params_lock:
            param = available_params.get(param_id)
            if param is None:
                return jsonify({"error": "Parameter not found"}), 404
            param["value"] = new_value
            available_params[param_id] = param
        global dest_mav_conn
        if dest_mav_conn is None:
            return jsonify({"error": "No MAVLink connection available"}), 500
        try:
            dest_mav_conn.mav.param_set_send(1, 1, param_id.encode('ascii'), float(new_value), param["type"])
        except Exception as e:
            return jsonify({"error": f"Failed to send PARAM_SET: {e}"}), 500
        return jsonify({"status": "Parameter updated", param_id: param})

    @app.route("/stream", methods=["GET"])
    def get_stream():
        msg_type = request.args.get("type")
        with history_lock:
            if msg_type:
                return jsonify(history_by_type.get(msg_type, []))
            else:
                return jsonify(streaming_history)

    @app.route("/alink_stream", methods=["GET"])
    def get_alink_stream():
        with history_lock:
            return jsonify(alink_sent_history)

    # Run Flask in threaded mode without reloader.
    app.run(host="0.0.0.0", port=5000, threaded=True, use_reloader=False)

#########################################
# Main Entry Point and Argument Parsing
#########################################

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
        args.channels_list = parse_channels(args.channels)
    else:
        args.channels_list = []

    # Start the Flask API in its own daemon thread.
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    try:
        if args.sync:
            run_sync_mode(args)
        else:
            run_normal_mode(args)
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
