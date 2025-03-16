#!/usr/bin/env python3
import argparse
import subprocess
import re
import logging
import sys
import os
import signal
import time

# --- Configuration Constants ---
CONFIG_FILE = "/etc/wifibroadcast.cfg"
DEFAULTS_FILE = "/usr/sbin/wfb-ng.sh"         # file holding default values to update/restore
CHANGE_CMD_FILE = "/usr/sbin/wfb-ng-change.sh"   # command to run on nodes
SSH_TIMEOUT = 5         # general SSH timeout in seconds

# Dedicated constants for killswitch in --sync-vtx mode
KILLSWITCH_DELAY = 5   # seconds to wait before attempting to kill killswitch
KILLSWITCH_TIMEOUT = 5 # seconds timeout for kill command

# Dedicated constants for --find-vtx mode
VTX_FIND_DELAY = 6     # seconds to wait after setting channel in find mode
VTX_FIND_TIMEOUT = 5    # seconds timeout for test command

# Retry timeout for both test and kill commands (try twice)
RETRY_TIMEOUT = 3       # seconds to wait before retrying a command

# Approved channel combinations (easily changed here)
APPROVED_CHANNELS = {
    "HT20": [165, 161, 149, 140, 48],
    "HT40+": [161, 149, 140, 36]
}

# Unified fallback settings for both modes
FALLBACK_CHANNEL = 165
FALLBACK_BANDWIDTH = "HT20"
FALLBACK_REGION = "US"

# --- Utility Functions ---

def read_defaults(filename=DEFAULTS_FILE):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except Exception as e:
        logging.error(f"Failed to read {filename}: {e}")
        sys.exit(1)
    channel_match = re.search(r'^\s*DEFAULT_CHANNEL\s*=\s*(\S+)', content, re.MULTILINE)
    bandwidth_match = re.search(r'^\s*DEFAULT_BANDWIDTH\s*=\s*"?([^"\n]+)"?', content, re.MULTILINE)
    region_match = re.search(r'^\s*DEFAULT_REGION\s*=\s*"?([^"\n]+)"?', content, re.MULTILINE)
    if not (channel_match and bandwidth_match and region_match):
        logging.error("Failed to extract default values from the script file.")
        sys.exit(1)
    return channel_match.group(1), bandwidth_match.group(1), region_match.group(1)

def update_defaults(new_channel, new_bandwidth, new_region, filename=DEFAULTS_FILE):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except Exception as e:
        logging.error(f"Failed to read {filename} for update: {e}")
        sys.exit(1)
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_CHANNEL\s*=\s*).*$',
                         r'\g<prefix>' + str(new_channel),
                         content, flags=re.MULTILINE)
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_BANDWIDTH\s*=\s*).*$',
                         r'\g<prefix>"' + new_bandwidth + '"',
                         content_new, flags=re.MULTILINE)
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_REGION\s*=\s*).*$',
                         r'\g<prefix>"' + new_region + '"',
                         content_new, flags=re.MULTILINE)
    try:
        with open(filename, 'w') as f:
            f.write(content_new)
    except Exception as e:
        logging.error(f"Failed to write updated defaults to {filename}: {e}")
        sys.exit(1)
    logging.info("Default values updated successfully.")

def restore_defaults(new_channel, new_bandwidth, new_region, filename=DEFAULTS_FILE):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except Exception as e:
        logging.error(f"Failed to read {filename} for restore: {e}")
        return False
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_CHANNEL\s*=\s*).*$',
                         r'\g<prefix>' + str(new_channel),
                         content, flags=re.MULTILINE)
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_BANDWIDTH\s*=\s*).*$',
                         r'\g<prefix>"' + new_bandwidth + '"',
                         content_new, flags=re.MULTILINE)
    content_new = re.sub(r'^(?P<prefix>\s*DEFAULT_REGION\s*=\s*).*$',
                         r'\g<prefix>"' + new_region + '"',
                         content_new, flags=re.MULTILINE)
    try:
        with open(filename, 'w') as f:
            f.write(content_new)
    except Exception as e:
        logging.error(f"Failed to write restored defaults to {filename}: {e}")
        return False
    logging.info("Restored default values successfully.")
    return True

def get_server_address(filename=CONFIG_FILE):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except Exception as e:
        logging.error(f"Failed to read {filename} for server_address: {e}")
        sys.exit(1)
    match = re.search(r'^\s*server_address\s*=\s*[\'"]([^\'"]+)[\'"]', content, re.MULTILINE)
    if not match:
        logging.error("Failed to extract server_address from the config file.")
        sys.exit(1)
    return match.group(1)

def extract_nodes_block(content):
    match = re.search(r'nodes\s*=\s*\{', content)
    if not match:
        logging.error("Could not find 'nodes' block in config file.")
        sys.exit(1)
    start_index = match.end()
    brace_count = 1
    i = start_index
    while i < len(content) and brace_count > 0:
        if content[i] == '{':
            brace_count += 1
        elif content[i] == '}':
            brace_count -= 1
        i += 1
    if brace_count != 0:
        logging.error("Braces in 'nodes' block are not balanced.")
        sys.exit(1)
    return content[start_index:i-1]

def parse_nodes(filename=CONFIG_FILE):
    try:
        with open(filename, 'r') as f:
            content = f.read()
    except Exception as e:
        logging.error(f"Failed to read config file {filename}: {e}")
        sys.exit(1)
    nodes_block = extract_nodes_block(content)
    ips = re.findall(r"['\"]((?:\d{1,3}\.){3}\d{1,3})['\"]\s*:", nodes_block)
    if not ips:
        logging.error("No valid IP addresses found in the nodes configuration.")
    else:
        logging.info(f"Found IP addresses: {ips}")
    return ips

def run_command(ip, command, is_local=False, timeout=SSH_TIMEOUT):
    try:
        if is_local:
            logging.info(f"Running local command on {ip}: {command}")
            proc = subprocess.Popen(command, shell=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True)
        else:
            ssh_command = f"ssh root@{ip} '{command}'"
            logging.info(f"Running remote command on {ip}: {ssh_command}")
            proc = subprocess.Popen(ssh_command, shell=True,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                    text=True, preexec_fn=os.setsid)
        try:
            stdout, stderr = proc.communicate(timeout=timeout)
            return subprocess.CompletedProcess(proc.args, proc.returncode, stdout, stderr)
        except subprocess.TimeoutExpired:
            logging.error(f"Command on {ip} timed out after {timeout} seconds.")
            if not is_local:
                try:
                    os.killpg(proc.pid, signal.SIGTERM)
                    logging.info(f"Killed process group for command on {ip}.")
                except Exception as kill_exception:
                    logging.error(f"Failed to kill process group for command on {ip}: {kill_exception}")
            proc.kill()
            stdout, stderr = proc.communicate()
            return subprocess.CompletedProcess(proc.args, 1, stdout, f"Timeout after {timeout} seconds: {stderr}")
    except Exception as e:
        logging.error(f"Error running command on {ip}: {e}")
        return subprocess.CompletedProcess(command, 1, "", str(e))

def cleanup():
    logging.info("Performing cleanup actions...")

# --- New Functionality: Find VTX Mode ---

def find_vtx_mode():
    """
    Cycle through all approved channel and bandwidth combinations (using region "US")
    to find an active VTX.
    For each combination:
      1. Update only the local node with the candidate channel.
      2. Wait for VTX_FIND_DELAY seconds.
      3. Issue a test SSH command to VTX (10.5.0.10) with a timeout of VTX_FIND_TIMEOUT seconds.
          - If the test command fails, wait RETRY_TIMEOUT seconds and retry once.
      4. If the test command returns "VTX Found", update remote nodes with this final setting and exit.
    If no combination yields success, revert all nodes to FALLBACK settings and exit with an error.
    """
    server_address = get_server_address()
    for bandwidth in ["HT40+", "HT20"]:
        if bandwidth not in APPROVED_CHANNELS:
            continue
        for channel in APPROVED_CHANNELS[bandwidth]:
            logging.info(f"Trying channel {channel} with bandwidth {bandwidth} (local update only)...")
            region = "US"
            command_local = f"{CHANGE_CMD_FILE} {channel} {bandwidth} {region}"
            logging.info(f"Setting local node (127.0.0.1) to channel {channel}, {bandwidth}, {region}")
            run_command("127.0.0.1", command_local, is_local=True)
            logging.info(f"Waiting for {VTX_FIND_DELAY} seconds for channel establishment...")
            time.sleep(VTX_FIND_DELAY)
            test_cmd = 'echo "VTX Found"; exit'
            logging.info("Testing for VTX on 10.5.0.10 (first attempt)...")
            test_result = run_command("10.5.0.10", test_cmd, is_local=False, timeout=VTX_FIND_TIMEOUT)
            if not (test_result.returncode == 0 and "VTX Found" in test_result.stdout):
                logging.info(f"Test failed on first attempt. Waiting for {RETRY_TIMEOUT} seconds before retrying...")
                time.sleep(RETRY_TIMEOUT)
                logging.info("Testing for VTX on 10.5.0.10 (second attempt)...")
                test_result = run_command("10.5.0.10", test_cmd, is_local=False, timeout=VTX_FIND_TIMEOUT)
            if test_result.returncode == 0 and "VTX Found" in test_result.stdout:
                print(f"Found VTX on channel {channel} with bandwidth {bandwidth}.")
                remote_ips = [ip for ip in parse_nodes() if ip != "127.0.0.1"]
                command_remote = f"{CHANGE_CMD_FILE} {channel} {bandwidth} {region} {server_address}"
                for ip in remote_ips:
                    logging.info(f"Updating remote node {ip} to channel {channel}, {bandwidth}, {region}")
                    run_command(ip, command_remote, is_local=False)
                sys.exit(0)
            else:
                logging.info(f"VTX not found on channel {channel} with bandwidth {bandwidth}.")
    print("No VTX found on available approved channel/bandwidth combinations.")
    print(f"Reverting all nodes to fallback settings: {FALLBACK_CHANNEL} {FALLBACK_BANDWIDTH} {FALLBACK_REGION} ...")
    restore_defaults(FALLBACK_CHANNEL, FALLBACK_BANDWIDTH, FALLBACK_REGION)
    run_command("127.0.0.1", f"{CHANGE_CMD_FILE} {FALLBACK_CHANNEL} {FALLBACK_BANDWIDTH} {FALLBACK_REGION}", is_local=True)
    remote_ips = [ip for ip in parse_nodes() if ip != "127.0.0.1"]
    command_remote = f"{CHANGE_CMD_FILE} {FALLBACK_CHANNEL} {FALLBACK_BANDWIDTH} {FALLBACK_REGION} {server_address}"
    for ip in remote_ips:
        run_command(ip, command_remote, is_local=False)
    sys.exit(1)

# --- Main Program ---

def main():
    if os.geteuid() != 0:
        logging.error("This program must be run as root.")
        sys.exit(1)
    parser = argparse.ArgumentParser(
        description="Script to change the wireless channel on nodes based on /etc/wifibroadcast.cfg"
    )
    # In --find-vtx mode, the positional arguments are optional.
    parser.add_argument("channel", type=int, nargs="?", help="Channel to set")
    parser.add_argument("bandwidth", type=str, nargs="?", help="Bandwidth to set (e.g., HT20, HT40+, HT40-)")
    parser.add_argument("region", type=str, nargs="?", help="Region to set")
    parser.add_argument("--sync-vtx", action="store_true",
                        help="If set, sync VTX by sending '/usr/bin/sync_channel.sh <channel> <bandwidth> <region>' to root@10.5.0.10")
    parser.add_argument("--find-vtx", action="store_true",
                        help="If set, cycle through approved channel/bandwidth combinations (region 'US') to find an active VTX")
    args = parser.parse_args()

    # If --find-vtx is provided, run find_vtx_mode and exit.
    if args.find_vtx:
        find_vtx_mode()

    # For normal operation, channel, bandwidth, and region are required.
    if args.channel is None or args.bandwidth is None or args.region is None:
        logging.error("Channel, bandwidth, and region must be provided for normal operation.")
        sys.exit(1)

    # --- Approved Channel Validation ---
    approved = APPROVED_CHANNELS
    if args.bandwidth not in approved or args.channel not in approved[args.bandwidth]:
        print("Error: The channel/bandwidth combination you provided is not approved.")
        print("Approved channel/bandwidth combinations:")
        for bw, ch_list in approved.items():
            print(f"  {bw}: {', '.join(str(ch) for ch in ch_list)}")
        sys.exit(1)

    # Read original default values.
    orig_channel, orig_bandwidth, orig_region = read_defaults()
    logging.info(f"Original defaults: CHANNEL={orig_channel}, BANDWIDTH={orig_bandwidth}, REGION={orig_region}")

    # Update defaults file with new values.
    update_defaults(args.channel, args.bandwidth, args.region)

    # Build command strings.
    command_local = f"{CHANGE_CMD_FILE} {args.channel} {args.bandwidth} {args.region}"
    server_address = get_server_address()
    command_remote = f"{CHANGE_CMD_FILE} {args.channel} {args.bandwidth} {args.region} {server_address}"
    logging.info(f"Local command: {command_local}")
    logging.info(f"Remote command: {command_remote}")

    # If --sync-vtx is requested, update only the local node first.
    if args.sync_vtx:
        sync_command = (f'nohup /usr/bin/sync_channel.sh {args.channel} {args.bandwidth} {args.region} '
                        f'> /dev/null 2>&1 & echo "SYNC_STARTED"; exit')
        logging.info(f"Syncing VTX by running command on 10.5.0.10: {sync_command}")
        result = run_command("10.5.0.10", sync_command, is_local=False)
        if result.returncode != 0 or "SYNC_STARTED" not in result.stdout:
            logging.error(f"Sync VTX command failed on 10.5.0.10 with output: {result.stdout.strip()} and error: {result.stderr.strip()}")
            logging.error("Restoring fallback settings due to sync failure...")
            restore_defaults(FALLBACK_CHANNEL, FALLBACK_BANDWIDTH, FALLBACK_REGION)
            sys.exit(1)
        else:
            logging.info(f"Sync VTX command succeeded with output: {result.stdout.strip()}")
        logging.info("Updating local node (127.0.0.1) with channel change...")
        run_command("127.0.0.1", command_local, is_local=True)
    else:
        # Normal operation: update both local and remote nodes.
        ips = parse_nodes()
        local_ips = [ip for ip in ips if ip == "127.0.0.1"]
        remote_ips = [ip for ip in ips if ip != "127.0.0.1"]
        logging.info("Updating local node(s)...")
        for ip in local_ips:
            run_command(ip, command_local, is_local=True)
        logging.info("Updating remote node(s)...")
        for ip in remote_ips:
            run_command(ip, command_remote, is_local=False)

    # In --sync-vtx mode, after updating the local node, wait before killing the killswitch.
    if args.sync_vtx:
        logging.info(f"Waiting for {KILLSWITCH_DELAY} seconds to allow channel to establish...")
        time.sleep(KILLSWITCH_DELAY)
        kill_command = "killall killswitch.sh && echo 'KILLSWITCH_KILLED'; exit"
        logging.info("Killing killswitch on VTX by running command: " + kill_command)
        result_kill = run_command("10.5.0.10", kill_command, is_local=False, timeout=KILLSWITCH_TIMEOUT)
        if not (result_kill.returncode == 0 and "KILLSWITCH_KILLED" in result_kill.stdout):
            logging.error(f"Failed to kill killswitch on VTX on first attempt. Output: {result_kill.stdout.strip()} Error: {result_kill.stderr.strip()}")
            logging.info(f"Waiting for {RETRY_TIMEOUT} seconds before retrying kill command...")
            time.sleep(RETRY_TIMEOUT)
            result_kill = run_command("10.5.0.10", kill_command, is_local=False, timeout=KILLSWITCH_TIMEOUT)
            if not (result_kill.returncode == 0 and "KILLSWITCH_KILLED" in result_kill.stdout):
                logging.error(f"Failed to kill killswitch on VTX on second attempt. Output: {result_kill.stdout.strip()} Error: {result_kill.stderr.strip()}")
                logging.error("Restoring fallback settings due to killswitch kill failure...")
                restore_defaults(FALLBACK_CHANNEL, FALLBACK_BANDWIDTH, FALLBACK_REGION)
                local_restore_command = f"{CHANGE_CMD_FILE} {FALLBACK_CHANNEL} {FALLBACK_BANDWIDTH} {FALLBACK_REGION} 127.0.0.1"
                logging.info(f"Restoring settings on local node (127.0.0.1) with command: {local_restore_command}")
                run_command("127.0.0.1", local_restore_command, is_local=True)
                remote_restore_command = f"{CHANGE_CMD_FILE} {FALLBACK_CHANNEL} {FALLBACK_BANDWIDTH} {FALLBACK_REGION} {server_address}"
                remote_ips = [ip for ip in parse_nodes() if ip != "127.0.0.1"]
                for ip in remote_ips:
                    logging.info(f"Restoring settings on remote node {ip} with command: {remote_restore_command}")
                    run_command(ip, remote_restore_command, is_local=False)
                sys.exit(1)
            else:
                logging.info("Successfully killed killswitch on VTX on second attempt.")
        else:
            logging.info("Successfully killed killswitch on VTX on first attempt.")
        remote_ips = [ip for ip in parse_nodes() if ip != "127.0.0.1"]
        logging.info("Updating remote nodes with final channel settings...")
        for ip in remote_ips:
            run_command(ip, command_remote, is_local=False)

    print("\n=== Summary ===")
    print("Channel change commands have been issued.")
    print("Check logs for details.")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    try:
        main()
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received in main. Exiting gracefully...")
        cleanup()
        sys.exit(1)
