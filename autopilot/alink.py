import time
import socket
import struct
import threading
import msgpack

# ALINK connection settings
ALINK_HOST = "127.0.0.1"
ALINK_PORT = 8003
ALINK_RETRY_INTERVAL = 1      # seconds
ALINK_MESSAGE_INTERVAL = 0.1  # seconds
ACCUMULATOR_PERIOD = 1.0      # seconds

# Module-global variables for ALINK processing.
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

def send_radio_status_alink(conn, recovered_packet_count, shared_state):
    """
    Constructs and sends a MAVLink RADIO_STATUS message based on ALINK link health data.
    Uses the local recovered_packet_count for both fixed and rxerrors.
    Also logs the sent message in shared_state.alink_sent_history.
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
    # Import helper locally to avoid circular imports.
    from mavfwd import add_to_alink_sent_history
    add_to_alink_sent_history(msg_dict, shared_state)

def run_alink_thread(conn, shutdown_event, shared_state):
    """
    Connects to the ALINK JSON/msgpack server, fetches Wi-Fi link data,
    recalculates link health, and sends a RADIO_STATUS message via the provided MAVLink connection.
    """
    global alink_recovered_packets_accumulator, alink_last_accumulator_reset
    while not shutdown_event.is_set():
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
                client_socket.settimeout(1.0)
                print(f"ALINK: Connecting to {ALINK_HOST}:{ALINK_PORT}...")
                client_socket.connect((ALINK_HOST, ALINK_PORT))
                print(f"ALINK: Connected to {ALINK_HOST}:{ALINK_PORT}.")
                while not shutdown_event.is_set():
                    try:
                        length_prefix = client_socket.recv(4)
                    except socket.timeout:
                        continue
                    if not length_prefix:
                        print("ALINK: Connection closed by server.")
                        break
                    msg_length = struct.unpack('!I', length_prefix)[0]
                    data = b""
                    while len(data) < msg_length:
                        try:
                            chunk = client_socket.recv(min(4096, msg_length - len(data)))
                        except socket.timeout:
                            continue
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
                            send_radio_status_alink(conn, recovered_packet_count, shared_state)
                    time.sleep(ALINK_MESSAGE_INTERVAL)
        except Exception as e:
            print(f"ALINK: Connection failed or lost: {e}. Retrying in {ALINK_RETRY_INTERVAL} seconds...")
            time.sleep(ALINK_RETRY_INTERVAL)
    print("Shutdown signal received in ALINK thread. Exiting run_alink_thread.")
