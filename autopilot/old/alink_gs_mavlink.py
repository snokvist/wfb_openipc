#!/usr/bin/python3
import socket
import msgpack
import struct
import threading
import string
import random
import time
import argparse
import os
import configparser
from pymavlink import mavutil  # New import for MAVLink functionality

VERSION = "0.1.1"

# Global variables
results = []
link_health_score_rssi = 1000  # Default score before calculation for RSSI
link_health_score_snr = 1000  # Default score before calculation for SNR
recovered_packets = 0         # Number of recovered packets (FEC)
lost_packets = 0              # Number of lost packets
best_antennas_rssi = [-105, -105, -105, -105]  # Default values for best antennas RSSI
best_antennas_snr = [-105, -105, -105, -105]   # Default values for best antennas SNR
config = None  # Configuration file object
udp_socket = None  # UDP socket used for sending the original link health data
udp_ip = None      # UDP destination IP (for the original data)
udp_port = None    # UDP destination port (for the original data)
previous_link_health_score_rssi = None
previous_link_health_score_snr = None

# Counting fec_rec
accumulator_period = 1.0
recovered_packets_accumulator = 0
last_accumulator_reset = time.time()

# Global variable for MAVLink connection
mav_conn = None

# Default config values
DEFAULT_CONFIG = {
    'Settings': {
        'version': VERSION,
        'message_interval': '100',  # in milliseconds
        'use_best_rssi': 'True',
        'min_rssi': '-80',
        'max_rssi': '-30',
        'min_snr': '12',
        'max_snr': '38',
        'host': '127.0.0.1',
        'port': '8003',
        'udp_ip': '10.5.0.10',
        'udp_port': '9999',
        'retry_interval': '1',
        # New MAVLink settings:
        'mavlink_ip': '10.0.0.1',
        'mavlink_port': '14551'
    },
    'Descriptions': {
        'version': 'Version of the script',
        'message_interval': 'Time between sending UDP messages in milliseconds',
        'use_best_rssi': 'True to use best available RSSI for health score, or False for average',
        'min_rssi': 'Minimum RSSI value used for link health calculation',
        'max_rssi': 'Maximum RSSI value used for link health calculation',
        'min_snr': 'Minimum SNR value used for link health calculation',
        'max_snr': 'Maximum SNR value used for link health calculation',
        'host': 'Host address for the TCP connection',
        'port': 'Port for the TCP connection',
        'udp_ip': 'UDP IP for sending link health data',
        'udp_port': 'UDP port for sending link health data',
        'retry_interval': 'Time in seconds to wait before retrying TCP connection on failure',
        'mavlink_ip': 'UDP IP to send the MAVLink RADIO_STATUS message',
        'mavlink_port': 'UDP port to send the MAVLink RADIO_STATUS message'
    }
}

def update_recovered_packets_accumulator(new_recovered_packets):
    global recovered_packets_accumulator
    recovered_packets_accumulator += new_recovered_packets

def load_config(config_file='config.ini'):
    global config
    config = configparser.ConfigParser()

    # Create config file with default values if it doesn't exist
    if not os.path.exists(config_file):
        print(f"Config file {config_file} not found. Creating with default values...")
        config.read_dict(DEFAULT_CONFIG)
        with open(config_file, 'w') as f:
            config.write(f)
    else:
        config.read(config_file)

    # Update version if necessary
    if config['Settings'].get('version') != VERSION:
        print(f"Updating version in config file from {config['Settings'].get('version')} to {VERSION}")
        config['Settings']['version'] = VERSION
        update_version_history(config_file)

    # Update config file with any new fields
    updated = False
    for section in DEFAULT_CONFIG:
        if section not in config:
            config[section] = DEFAULT_CONFIG[section]
            updated = True
        else:
            for key, value in DEFAULT_CONFIG[section].items():
                if key not in config[section]:
                    config[section][key] = value
                    updated = True

    if updated:
        with open(config_file, 'w') as f:
            config.write(f)

def update_version_history(config_file):
    if 'Version History' not in config:
        config['Version History'] = {}
    timestamp = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime())
    config['Version History'][VERSION] = f"Version {VERSION} updated on {timestamp}"
    with open(config_file, 'w') as f:
        config.write(f)

def generate_random_code(length=4):
    return ''.join(random.choices(string.ascii_lowercase, k=length))

def request_keyframe():
    num_attempts = 5
    unique_code = generate_random_code()
    special_message = f"special:request_keyframe:{unique_code}"
    for attempt in range(num_attempts):
        send_udp(special_message)
        if verbose_mode:
            print(f"Sent special message: {special_message}, attempt {attempt + 1}/{num_attempts}")
        time.sleep(0.05)

def drop_gop():
    special_message = "special:drop_gop"
    num_attempts = 5
    for attempt in range(num_attempts):
        send_udp(special_message)
        if verbose_mode:
            print(f"Sent special message: {special_message}, attempt {attempt + 1}/{num_attempts}")
        time.sleep(0.1)

def calculate_link_health(video_rx):
    global recovered_packets, lost_packets, link_health_score_rssi, link_health_score_snr, best_antennas_rssi, best_antennas_snr
    global previous_link_health_score_rssi, previous_link_health_score_snr

    try:
        message_interval = int(config['Settings']['message_interval'])
        use_best_rssi = config.getboolean('Settings', 'use_best_rssi')
        min_rssi = float(config['Settings']['min_rssi'])
        max_rssi = float(config['Settings']['max_rssi'])
        min_snr = float(config['Settings']['min_snr'])
        max_snr = float(config['Settings']['max_snr'])

        packets = video_rx.get('packets', {})
        fec_rec = packets.get('fec_rec', [0, 0])
        recovered_packets = fec_rec[0]
        update_recovered_packets_accumulator(recovered_packets)
        lost = packets.get('lost', [0, 0])
        lost_packets = lost[0]

        if 0 < lost_packets < 10:
            request_keyframe()

        rx_ant_stats = video_rx.get('rx_ant_stats', {})
        rssi_values = []
        snr_values = []
        num_antennas = len(rx_ant_stats)
        if num_antennas > 0:
            for antenna in rx_ant_stats.values():
                if len(antenna) >= 6:
                    rssi_avg = antenna[2]
                    snr_avg = antenna[5]
                    rssi_values.append(rssi_avg)
                    snr_values.append(snr_avg)
                else:
                    if verbose_mode:
                        print(f"Warning: Antenna data is incomplete: {antenna}")
        else:
            if verbose_mode:
                print("Warning: No antenna stats available")
            link_health_score_rssi = 1000
            link_health_score_snr = 1000
            best_antennas_rssi = [-105, -105, -105, -105]
            best_antennas_snr = [-105, -105, -105, -105]
            return link_health_score_rssi, link_health_score_snr

        rssi_values.sort(reverse=True)
        snr_values.sort(reverse=True)
        best_antennas_rssi = rssi_values[:4]
        best_antennas_rssi += [-105] * (4 - len(best_antennas_rssi))
        best_antennas_snr = snr_values[:4]
        best_antennas_snr += [-105] * (4 - len(best_antennas_snr))

        if use_best_rssi:
            rssi_to_use = best_antennas_rssi[0]
        else:
            rssi_to_use = sum(best_antennas_rssi) / 4

        if rssi_to_use > max_rssi:
            link_health_score_rssi = 2000
        elif rssi_to_use < min_rssi:
            link_health_score_rssi = 1000
        else:
            link_health_score_rssi = 1000 + ((rssi_to_use - min_rssi) / (max_rssi - min_rssi)) * 1000

        if use_best_rssi:
            avg_best_snr = best_antennas_snr[0]
        else:
            avg_best_snr = sum(best_antennas_snr) / 4
        if avg_best_snr > max_snr:
            link_health_score_snr = 2000
        elif avg_best_snr < min_snr:
            link_health_score_snr = 1000
        else:
            link_health_score_snr = 1000 + ((avg_best_snr - min_snr) / (max_snr - min_snr)) * 1000

        link_health_score_rssi = round(link_health_score_rssi)
        link_health_score_snr = round(link_health_score_snr)

        if verbose_mode:
            print(f"Calculated Health Score RSSI: {link_health_score_rssi}, SNR: {link_health_score_snr}, "
                  f"Recovered: {recovered_packets}, Lost: {lost_packets}, "
                  f"Best Antennas RSSI: {best_antennas_rssi}, Best Antennas SNR: {best_antennas_snr}")

        return link_health_score_rssi, link_health_score_snr

    except Exception as e:
        if verbose_mode:
            print(f"Error calculating link health: {e}, video_rx data: {video_rx}")
        return 1000, 1000

def send_udp(message):
    if verbose_mode:
        print("send_udp function has started")
    message_bytes = message.encode('utf-8')
    message_size = struct.pack('!I', len(message_bytes))
    full_message = message_size + message_bytes
    if verbose_mode:
        print("Preparing UDP message to be sent")
    try:
        udp_socket.sendto(full_message, (udp_ip, udp_port))
        if verbose_mode:
            print(f"UDP Message Sent: {message} (size: {len(message_bytes)} bytes)")
    except Exception as e:
        if verbose_mode:
            print(f"Error sending UDP data: {e}")

def send_radio_status(recovered_packet_count):
    """
    Constructs and sends a MAVLink RADIO_STATUS message using the following mappings:
      - link_health_score_rssi -> rssi (special: 999 -> 0, 2000 -> 254)
      - link_health_score_snr  -> noise (special: 999 -> 0, 2000 -> 254)
      - best_antennas_rssi[0]    -> rem_rssi (mapped from [-128, 128] to 0-254)
      - best_antennas_snr[0]     -> rem_noise (mapped from 0-50 to 0-254)
      - recovered_packets        -> fixed (capped at 254)
      - recovered_packet_count   -> rxerrors (capped at 254)
    """
    def convert_health(x):
        if x == 999:
            return 0
        return int(round((x - 999) * 254 / 1001))  # (2000 - 999) = 1001

    rssi_value = convert_health(link_health_score_rssi)
    noise_value = convert_health(link_health_score_snr)

    # Map best antenna RSSI (assumed in range -128 to 128) to 0-254
    rem_rssi = int(round((best_antennas_rssi[0] + 128) * 254 / 256))

    # Map best antenna SNR (assumed in range 0 to 50) to 0-254; clamp negatives to 0
    rem_snr_raw = best_antennas_snr[0]
    if rem_snr_raw < 0:
        rem_snr_raw = 0
    elif rem_snr_raw > 50:
        rem_snr_raw = 50
    rem_noise = int(round(rem_snr_raw * 254 / 50))

    fixed = min(recovered_packets, 254)
    rxerrors = min(recovered_packet_count, 254)

    # Send the RADIO_STATUS message (fields: rssi, remrssi, noise, remnoise, rxerrors, fixed)
    mav_conn.mav.radio_status_send(rssi_value, rem_rssi, 0, noise_value, rem_noise, rxerrors, fixed)
    if verbose_mode:
        print(f"Sent RADIO_STATUS: rssi={rssi_value}, rem_rssi={rem_rssi}, noise={noise_value}, "
              f"rem_noise={rem_noise}, rxerrors={rxerrors}, fixed={fixed}")

def generate_package():
    """
    Generates the UDP package every message_interval seconds and sends both the original
    UDP link health message and the MAVLink RADIO_STATUS message.
    """
    global recovered_packets_accumulator, last_accumulator_reset
    message_interval = int(config['Settings']['message_interval']) / 1000  # Convert to seconds

    while True:
        timestamp = int(time.time())
        current_time = time.time()
        if current_time - last_accumulator_reset >= accumulator_period:
            recovered_packet_count = recovered_packets_accumulator
            recovered_packets_accumulator = 0
            last_accumulator_reset = current_time
        else:
            recovered_packet_count = recovered_packets_accumulator

        # Construct the original UDP message (if still needed)
        message = f"{timestamp}:{link_health_score_rssi}:{link_health_score_snr}:{recovered_packets}:" \
                  f"{recovered_packet_count}:{best_antennas_rssi[0]}:{best_antennas_snr[0]}:" \
                  f"{best_antennas_snr[1]}:{best_antennas_snr[2]}"
        send_udp(message)

        # Now send the MAVLink RADIO_STATUS message
        send_radio_status(recovered_packet_count)

        time.sleep(message_interval)

def connect_and_receive_msgpack():
    global results, link_health_score_rssi, link_health_score_snr, recovered_packets, lost_packets, best_antennas_rssi, best_antennas_snr
    global udp_socket, udp_ip, udp_port, mav_conn

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    host = config['Settings']['host']
    port = int(config['Settings']['port'])
    udp_ip = config['Settings']['udp_ip']
    udp_port = int(config['Settings']['udp_port'])
    retry_interval = int(config['Settings']['retry_interval'])

    # Initialize the MAVLink connection using new MAVLink settings
    mavlink_ip = config['Settings'].get('mavlink_ip', '10.0.0.1')
    mavlink_port = int(config['Settings'].get('mavlink_port', 14551))
    mav_conn = mavutil.mavlink_connection(f"udpout:{mavlink_ip}:{mavlink_port}")
    # Set the component ID to 100 (camera)
    mav_conn.mav.srcComponent = 100
    if verbose_mode:
        print(f"Initialized MAVLink connection to {mavlink_ip}:{mavlink_port} with component ID 100")

    # Start the UDP/MAVLink sending thread
    udp_thread = threading.Thread(target=generate_package, daemon=True)
    udp_thread.start()

    while True:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                if verbose_mode:
                    print(f"Connecting to {host}:{port}...")
                client_socket.connect((host, port))
                if verbose_mode:
                    print(f"Connected to {host}:{port}")

                while True:
                    length_prefix = client_socket.recv(4)
                    if not length_prefix:
                        if verbose_mode:
                            print("No more data, connection closed.")
                        break

                    msg_length = struct.unpack('!I', length_prefix)[0]
                    data = b""
                    while len(data) < msg_length:
                        chunk = client_socket.recv(min(4096, msg_length - len(data)))
                        if not chunk:
                            if verbose_mode:
                                print("Incomplete data, connection closed.")
                            break
                        data += chunk

                    if len(data) == msg_length:
                        try:
                            unpacked_data = msgpack.unpackb(data, use_list=False, strict_map_key=False)
                            results.append(unpacked_data)
                            if unpacked_data.get("type") == "rx" and unpacked_data.get("id") == "video rx":
                                link_health_score_rssi, link_health_score_snr = calculate_link_health(unpacked_data)
                                if verbose_mode:
                                    print(f"Link Health Score RSSI: {link_health_score_rssi}, SNR: {link_health_score_snr}, "
                                          f"Best Antennas RSSI: {best_antennas_rssi}, Best Antennas SNR: {best_antennas_snr}")
                        except msgpack.UnpackException as e:
                            if verbose_mode:
                                print(f"Failed to unpack data: {e}")
                    else:
                        if verbose_mode:
                            print("Failed to receive full data, closing connection.")
                        break
        except Exception as e:
            if verbose_mode:
                print(f"Connection failed or lost: {e}. Retrying in {retry_interval} seconds...")
            time.sleep(retry_interval)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="TCP MessagePack client with UDP link health reporting and MAVLink RADIO_STATUS messaging.")
    parser.add_argument('--config', type=str, help='Path to configuration file', default='config.ini')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose mode for logging')
    args = parser.parse_args()

    global verbose_mode
    verbose_mode = args.verbose

    load_config(args.config)
    connect_and_receive_msgpack()
