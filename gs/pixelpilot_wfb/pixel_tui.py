#!/usr/bin/env python3
import curses
import subprocess
import re
import time
import socket
import struct

# Regular expressions for log lines
rx_ant_pattern = re.compile(
    r"^(?P<ts>\d+)\s+RX_ANT\s+(?P<freq_info>\d+:\d+:\d+)\s+(?P<antenna_id>\S+)\s+(?P<stats>.+)"
)
pkt_pattern = re.compile(r"^(?P<ts>\d+)\s+PKT\s+(?P<info>[\d:]+)")
session_pattern = re.compile(r"^(?P<ts>\d+)\s+SESSION\s+(?P<info>[\d:]+)")
decrypt_error_pattern = re.compile(r"^Unable to decrypt packet")
lost_packet_pattern = re.compile(r"^(?P<count>\d+)\s+packets lost")

def determine_mode(antenna_id: str) -> str:
    """
    Determines which mode is in use.
    In our case, if the antenna_id contains any hexadecimal letters (a–f, case-insensitive),
    then we assume the antenna_id is in the cluster format.
    """
    if re.search(r"[a-fA-F]", antenna_id):
        return "Cluster"
    return "Local"

def parse_cluster_antenna(antenna_str: str):
    """
    Given an antenna string in cluster mode (composed as:
       ip_address << 32 | wlan_idx << 8 | wlan_antenna_id),
    parse it into its IP address and antenna id.
    
    For example, given "7f00000100000000":
      - The upper 32 bits (0x7f000001) become the IP address (127.0.0.1).
      - The lower 8 bits are taken as the wlan_antenna_id.
    
    Returns a tuple (ip_address, antenna_id). On error returns (antenna_str, None).
    """
    try:
        antenna_value = int(antenna_str, 16)
        # Extract the IP part (upper 32 bits)
        ip_part = (antenna_value >> 32) & 0xFFFFFFFF
        # Extract the antenna portion (least-significant 8 bits)
        wlan_antenna = antenna_value & 0xFF
        ip_str = socket.inet_ntoa(struct.pack("!I", ip_part))
        return (ip_str, wlan_antenna)
    except Exception:
        return (antenna_str, None)

def main(stdscr):
    # Setup curses – hide the cursor and enable non-blocking input.
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.clear()

    # Initialize counters and state.
    rx_ant_count = 0
    pkt_count = 0
    session_count = 0
    decrypt_error_count = 0
    lost_packet_count = 0
    mode = None
    last_cluster_info = None
    recent_lines = []  # To hold recent log lines

    # Start the external process; merge stderr with stdout.
    proc = subprocess.Popen(
        ["pixelpilot_wfb.sh"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1  # Enable line-buffered reading.
    )

    # Main loop: read output lines and update the TUI.
    while True:
        line = proc.stdout.readline()
        if not line:
            # End loop when the process is finished.
            if proc.poll() is not None:
                break
            time.sleep(0.1)
            continue

        line = line.strip()
        recent_lines.append(line)
        # Keep only the last 15 lines to display.
        if len(recent_lines) > 15:
            recent_lines = recent_lines[-15:]

        # Count decryption errors.
        if decrypt_error_pattern.search(line):
            decrypt_error_count += 1

        # Check for lost packet lines.
        lost_match = lost_packet_pattern.match(line)
        if lost_match:
            lost_packet_count += int(lost_match.group("count"))

        # Process RX_ANT messages.
        rx_match = rx_ant_pattern.match(line)
        if rx_match:
            rx_ant_count += 1
            antenna_id_raw = rx_match.group("antenna_id")
            # Determine mode if not known.
            if mode is None:
                mode = determine_mode(antenna_id_raw)
            if mode == "Cluster":
                ip_str, wlan_antenna = parse_cluster_antenna(antenna_id_raw)
                if wlan_antenna is not None:
                    last_cluster_info = f"{ip_str} (antenna {wlan_antenna})"
                else:
                    last_cluster_info = antenna_id_raw
            else:
                last_cluster_info = antenna_id_raw

        # Process PKT and SESSION messages.
        if pkt_pattern.match(line):
            pkt_count += 1
        if session_pattern.match(line):
            session_count += 1

        # Refresh TUI.
        stdscr.erase()
        stdscr.addstr(0, 0, "PixelPilot_wfb Summary")
        stdscr.addstr(1, 0, f"Mode: {mode if mode else 'Unknown'}")
        stdscr.addstr(2, 0, f"RX_ANT messages:  {rx_ant_count}")
        stdscr.addstr(3, 0, f"PKT messages:      {pkt_count}")
        stdscr.addstr(4, 0, f"SESSION messages:  {session_count}")
        stdscr.addstr(5, 0, f"Decryption errors: {decrypt_error_count}")
        stdscr.addstr(6, 0, f"Lost packets:      {lost_packet_count}")
        
        if mode == "Cluster" and last_cluster_info is not None:
            stdscr.addstr(8, 0, f"Last cluster antenna: {last_cluster_info}")
            start_line = 10
        else:
            start_line = 8

        stdscr.addstr(start_line - 1, 0, "Recent log output:")
        for idx, out_line in enumerate(recent_lines, start=start_line):
            try:
                stdscr.addstr(idx, 0, out_line)
            except curses.error:
                pass

        stdscr.refresh()

    # Process finished; wait for user input.
    stdscr.nodelay(False)
    stdscr.addstr(start_line + 16, 0, "Process finished. Press any key to exit...")
    stdscr.getkey()

if __name__ == "__main__":
    curses.wrapper(main)
