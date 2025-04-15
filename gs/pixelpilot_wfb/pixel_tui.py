#!/usr/bin/env python3
import curses
import subprocess
import re
import time
import socket
import struct

# Regular expressions for matching log lines.
rx_ant_pattern = re.compile(
    r"^(?P<ts>\d+)\s+RX_ANT\s+(?P<freq_info>\d+:\d+:\d+)\s+(?P<antenna_id>\S+)\s+(?P<stats>.+)"
)
pkt_pattern = re.compile(r"^(?P<ts>\d+)\s+PKT\s+(?P<info>[\d:]+)")
session_pattern = re.compile(r"^(?P<ts>\d+)\s+SESSION\s+(?P<info>[\d:]+)")
decrypt_error_pattern = re.compile(r"^Unable to decrypt packet")
lost_packet_pattern = re.compile(r"^(?P<count>\d+)\s+packets lost")

def determine_mode(antenna_id: str) -> str:
    """
    Determine operating mode based on the antenna ID.
    If it contains hexadecimal letters, assume cluster mode; otherwise, local.
    """
    if re.search(r"[a-fA-F]", antenna_id):
        return "Cluster"
    return "Local"

def parse_cluster_antenna(antenna_str: str):
    """
    Parse a cluster-mode antenna id given as:
         ip_address << 32 | wlan_idx << 8 | wlan_antenna_id

    For example, "7f00000100000000" becomes:
      - Upper 32 bits (0x7f000001) -> "127.0.0.1"
      - Lower 8 bits -> antenna id.
      
    Returns a tuple (ip_str, wlan_antenna) or (antenna_str, None) on error.
    """
    try:
        antenna_value = int(antenna_str, 16)
        ip_part = (antenna_value >> 32) & 0xFFFFFFFF  # Top 32 bits
        wlan_antenna = antenna_value & 0xFF             # Lower 8 bits
        ip_str = socket.inet_ntoa(struct.pack("!I", ip_part))
        return (ip_str, wlan_antenna)
    except Exception:
        return (antenna_str, None)

def draw_colored_bar(win, row, col, rssi, bar_length=30):
    """
    Draw a progress bar at the specified (row, col) that represents the RSSI value.
    The bar is divided into three equal segments (red, yellow, green) corresponding to 
    RSSI ranges:
      * Red:   -90 to -60
      * Yellow: -60 to -30
      * Green:  -30 to 0
     
    Filled blocks (using a solid block "█") are drawn in the proper color; unfilled blocks
    are drawn with a light block "░".
    """
    # Clamp RSSI into our expected range.
    rssi = max(-90, min(rssi, 0))
    segment = bar_length // 3  # For bar_length=30, each segment is 10 characters.
    
    # --- Compute fills for each segment using linear interpolation ---
    # Red segment: covers -90 to -60.
    if rssi <= -60:
        red_fill = int((rssi + 90) / 30.0 * segment)
        if red_fill < 1:
            red_fill = 1
    else:
        red_fill = segment

    # Yellow segment: covers -60 to -30.
    if rssi <= -60:
        yellow_fill = 0
    elif rssi <= -30:
        yellow_fill = int((rssi + 60) / 30.0 * segment)
    else:
        yellow_fill = segment

    # Green segment: covers -30 to 0.
    if rssi <= -30:
        green_fill = 0
    elif rssi == 0:
        green_fill = segment
    else:
        green_fill = int((rssi + 30) / 30.0 * segment)

    red_fill = min(red_fill, segment)
    yellow_fill = min(yellow_fill, segment)
    green_fill = min(green_fill, segment)
    
    filled_char = "█"
    empty_char = "░"
    
    # --- Draw the red segment ---
    for i in range(segment):
        ch = filled_char if i < red_fill else empty_char
        win.addstr(row, col + i, ch, curses.color_pair(1) if i < red_fill else 0)
    # --- Draw the yellow segment ---
    for i in range(segment):
        ch = filled_char if i < yellow_fill else empty_char
        win.addstr(row, col + segment + i, ch, curses.color_pair(2) if i < yellow_fill else 0)
    # --- Draw the green segment ---
    for i in range(segment):
        ch = filled_char if i < green_fill else empty_char
        win.addstr(row, col + 2 * segment + i, ch, curses.color_pair(3) if i < green_fill else 0)

def main(stdscr):
    # Initialize curses settings and color pairs.
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_RED, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_GREEN, -1)
    
    stdscr.nodelay(True)
    stdscr.clear()

    # Overall counters.
    rx_ant_count = 0
    pkt_count = 0
    session_count = 0
    decrypt_error_count = 0
    lost_packet_count = 0
    mode = None

    # For chunk management:
    current_chunk_rssi = {}  # RX_ANT data since the most recent PKT line.
    last_chunk_rssi = {}     # Most recent complete chunk.

    # Buffer for recent log lines.
    recent_lines = []

    # Launch the external process using the system path.
    proc = subprocess.Popen(
        ["pixelpilot_wfb.sh"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1  # Line-buffered.
    )

    while True:
        line = proc.stdout.readline()
        if not line:
            if proc.poll() is not None:
                break
            time.sleep(0.1)
            continue

        line = line.strip()
        recent_lines.append(line)
        if len(recent_lines) > 20:
            recent_lines = recent_lines[-20:]

        if decrypt_error_pattern.search(line):
            decrypt_error_count += 1
        lost_match = lost_packet_pattern.match(line)
        if lost_match:
            lost_packet_count += int(lost_match.group("count"))

        # A PKT line marks the start of a new chunk.
        if pkt_pattern.match(line):
            pkt_count += 1
            if current_chunk_rssi:
                last_chunk_rssi = current_chunk_rssi.copy()
            current_chunk_rssi = {}
        
        if session_pattern.match(line):
            session_count += 1

        # Process RX_ANT lines.
        rx_match = rx_ant_pattern.match(line)
        if rx_match:
            rx_ant_count += 1
            antenna_id_raw = rx_match.group("antenna_id")
            if mode is None:
                mode = determine_mode(antenna_id_raw)
            # Format the stats field: count_all:rssi_min:rssi_avg:rssi_max:snr_min:snr_avg:snr_max
            stats = rx_match.group("stats").split(":")
            if len(stats) >= 3:
                try:
                    rssi_avg = int(stats[2])
                except ValueError:
                    rssi_avg = None
            else:
                rssi_avg = None

            if mode == "Cluster":
                ip_str, wlan_antenna = parse_cluster_antenna(antenna_id_raw)
                antenna_key = f"{ip_str} (antenna {wlan_antenna})" if wlan_antenna is not None else antenna_id_raw
            else:
                antenna_key = antenna_id_raw

            if rssi_avg is not None:
                current_chunk_rssi[antenna_key] = rssi_avg

        # Determine which chunk to display.
        chunk_to_display = current_chunk_rssi if current_chunk_rssi else last_chunk_rssi

        stdscr.erase()
        stdscr.addstr(0, 0, "PixelPilot_wfb Summary")
        stdscr.addstr(1, 0, f"Mode: {mode if mode else 'Unknown'}")
        stdscr.addstr(2, 0, f"RX_ANT messages:  {rx_ant_count}")
        stdscr.addstr(3, 0, f"PKT messages:      {pkt_count}")
        stdscr.addstr(4, 0, f"SESSION messages:  {session_count}")
        stdscr.addstr(5, 0, f"Decryption errors: {decrypt_error_count}")
        stdscr.addstr(6, 0, f"Lost packets:      {lost_packet_count}")

        stdscr.addstr(8, 0, "Latest Chunk RSSI Averages:")
        line_offset = 9
        # Sort and print the antennas with a tabulated layout.
        if chunk_to_display:
            for antenna_key in sorted(chunk_to_display.keys()):
                # Truncate antenna name to 20 characters (pad if shorter).
                formatted_key = f"{antenna_key[:20]:20}"
                rssi_avg = chunk_to_display[antenna_key]
                # Format a label combining antenna name and RSSI value.
                label = f"{formatted_key} {rssi_avg:>4} "
                stdscr.addstr(line_offset, 0, label)
                # Compute the column offset: fixed field length of 20 plus a few spaces.
                col_offset = len(label) + 1
                draw_colored_bar(stdscr, line_offset, col_offset, rssi_avg)
                line_offset += 1
        else:
            stdscr.addstr(line_offset, 0, "No RX_ANT data in current chunk.")

        stdscr.addstr(line_offset + 2, 0, "Recent log output:")
        for idx, out_line in enumerate(recent_lines, start=line_offset + 3):
            try:
                stdscr.addstr(idx, 0, out_line)
            except curses.error:
                pass

        stdscr.refresh()

    stdscr.nodelay(False)
    stdscr.addstr(line_offset + 15, 0, "Process finished. Press any key to exit...")
    stdscr.getkey()

if __name__ == "__main__":
    curses.wrapper(main)
