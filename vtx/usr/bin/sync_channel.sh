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
DEFAULTS_FILE = "/usr/sbin/wfb-ng.sh"       # file holding default values to update/restore
CHANGE_CMD_FILE = "/usr/sbin/wfb-ng-change.sh"  # command to run on nodes
SSH_TIMEOUT = 5       # general SSH timeout in seconds

# New dedicated constants for killswitch delay and timeout (now 10 sec)
KILLSWITCH_DELAY = 10  # seconds to wait before attempting to kill killswitch
KILLSWITCH_TIMEOUT = 10  # seconds for the killall killswitch command timeout

# New dedicated constants for find mode:
VTX_FIND_DELAY = 4   # seconds to wait after setting channel in find mode
VTX_FIND_TIMEOUT = 2  # seconds timeout for test command in find mode

# Approved channel combinations (easily changed here)
APPROVED_CHANNELS = {
    "HT40+": [161,157,149,140,48,40],
    "HT20": [165,161,157,153,149,140,48,44,40,36]
}

# Predetermined restore settings (used if killswitch cancellation fails)
RESTORE_CHANNEL = 165
RESTORE_BANDWIDTH = "HT20"
RESTORE_REGION = "00"

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
      1. Send commands to local and remote nodes to change the channel.
      2. Wait for VTX_FIND_DELAY seconds.
      3. Issue a test command to VTX (10.5.0.10) with a timeout of VTX_FIND_TIMEOUT seconds.
      4. If the test command returns the marker "VTX Found", print success and exit.
    If no combination yields success, exit with an error message.
    """
    server_address = get_server_address()
    # We'll use the order: HT20, then HT40+, then HT40-
    for bandwidth in ["HT40+", "HT20"]:
        if bandwidth not in APPROVED_CHANNELS:
            continue
        for channel in APPROVED_CHANNELS[bandwidth]:
            logging.info(f"Trying channel {channel} with bandwidth {bandwidth} on VTX...")
            # In find mode, region is fixed to "US"
            region = "US"
            # Build command strings for local and remote nodes.
            command_local = f"{CHANGE_CMD_FILE} {channel} {bandwidth} {region}"
            command_remote = f"{CHANGE_CMD_FILE} {channel} {bandwidth} {region} {server_address}"
            # Change channel on local node.
            logging.info(f"Setting local node (127.0.0.1) to channel {channel}, {bandwidth}, {region}")
            run_command("127.0.0.1", command_local, is_local=True)
            # Change channel on remote nodes.
            ips = parse_nodes()
            remote_ips = [ip for ip in ips if ip != "127.0.0.1"]
            for ip in remote_ips:
                logging.info(f"Setting remote node {ip} to channel {channel}, {bandwidth}, {region}")
                run_command(ip, command_remote, is_local=False)
            # Wait for the channel to establish.
            logging.info(f"Waiting for {VTX_FIND_DELAY} seconds for channel establishment...")
            time.sleep(VTX_FIND_DELAY)
            # Issue test command to VTX.
            test_cmd = 'echo "VTX Found"; exit'
            logging.info("Testing for VTX on 10.5.0.10...")
            test_result = run_command("10.5.0.10", test_cmd, is_local=False, timeout=VTX_FIND_TIMEOUT)
            if test_result.returncode == 0 and "VTX Found" in test_result.stdout:
                print(f"Found VTX on channel {channel} with bandwidth {bandwidth}.")
                sys.exit(0)
            else:
                logging.info(f"VTX not found on channel {channel} with bandwidth {bandwidth}.")
    print("No VTX found on available approved channel/bandwidth combinations.")
    sys.exit(1)

# --- Main Program ---

def main():
    if os.geteuid() != 0:
        logging.error("This program must be run as root.")
        sys.exit(1)
    parser = argparse.ArgumentParser(
        description="Script to change the wireless channel on nodes based on /etc/wifibroadcast.cfg"
    )
    # In find-vtx mode, the positional arguments are optional.
    parser.add_argument("channel", type=int, nargs="?", help="Channel to set")
    parser.add_argument("bandwidth", type=str, nargs="?", help="Bandwidth to set (e.g., HT20, HT40+, HT40-)")
    parser.add_argument("region", type=str, nargs="?", help="Region to set")
    parser.add_argument("--handle-local-separately", action="store_true",
                        help="If set, handle 127.0.0.1 with special logic")
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

    # Sync VTX if requested.
    if args.sync_vtx:
        sync_command = (f'nohup /usr/bin/sync_channel.sh {args.channel} {args.bandwidth} {args.region} '
                        f'> /dev/null 2>&1 & echo "SYNC_STARTED"; exit')
        logging.info(f"Syncing VTX by running command on 10.5.0.10: {sync_command}")
        result = run_command("10.5.0.10", sync_command, is_local=False)
        if result.returncode != 0 or "SYNC_STARTED" not in result.stdout:
            logging.error(f"Sync VTX command failed on 10.5.0.10 with output: {result.stdout.strip()} and error: {result.stderr.strip()}")
            logging.error("Restoring default values due to sync failure...")
            restore_defaults(orig_channel, orig_bandwidth, orig_region)
            sys.exit(1)
        else:
            logging.info(f"Sync VTX command succeeded with output: {result.stdout.strip()}")

    # Parse node IP addresses.
    ips = parse_nodes()
    if not ips:
        logging.error("No nodes to process. Exiting.")
        sys.exit(1)
    local_ips = [ip for ip in ips if ip == "127.0.0.1"]
    remote_ips = [ip for ip in ips if ip != "127.0.0.1"]

    success = {}
    errors = {}

    # Process local nodes.
    try:
        for ip in local_ips:
            if args.handle_local_separately:
                logging.info("Handling local node (127.0.0.1) with special logic.")
            else:
                logging.info("Handling local node (127.0.0.1) as part of the loop.")
            result = run_command(ip, command_local, is_local=True)
            if result.returncode == 0:
                success[ip] = result.stdout.strip()
                logging.info(f"Command on {ip} succeeded: {result.stdout.strip()}")
            else:
                errors[ip] = result.stderr.strip()
                logging.error(f"Command on {ip} failed with error: {result.stderr.strip()}")
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Exiting gracefully...")
        cleanup()
        sys.exit(1)

    # Process remote nodes.
    try:
        for ip in remote_ips:
            result = run_command(ip, command_remote, is_local=False)
            if result.returncode == 0:
                success[ip] = result.stdout.strip()
                logging.info(f"Command on {ip} succeeded: {result.stdout.strip()}")
            else:
                errors[ip] = result.stderr.strip()
                logging.error(f"Command on {ip} failed with error: {result.stderr.strip()}")
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received. Exiting gracefully...")
        cleanup()
        sys.exit(1)

    # After processing nodes, if --sync-vtx is requested, wait before killing the killswitch.
    if args.sync_vtx:
        logging.info(f"Waiting for {KILLSWITCH_DELAY} seconds to allow channel to establish...")
        time.sleep(KILLSWITCH_DELAY)
        kill_command = "killall killswitch.sh && echo 'KILLSWITCH_KILLED'; exit"
        logging.info("Killing killswitch on VTX by running command: " + kill_command)
        result_kill = run_command("10.5.0.10", kill_command, is_local=False, timeout=KILLSWITCH_TIMEOUT)
        if result_kill.returncode != 0 or "KILLSWITCH_KILLED" not in result_kill.stdout:
            logging.error(f"Failed to kill killswitch on VTX. Output: {result_kill.stdout.strip()} Error: {result_kill.stderr.strip()}")
            logging.error("Restoring predetermined settings due to killswitch kill failure...")
            restore_defaults(RESTORE_CHANNEL, RESTORE_BANDWIDTH, RESTORE_REGION)
            local_restore_command = f"{CHANGE_CMD_FILE} {RESTORE_CHANNEL} {RESTORE_BANDWIDTH} {RESTORE_REGION}"
            logging.info(f"Restoring settings on local node 127.0.0.1 with command: {local_restore_command}")
            result_local_restore = run_command("127.0.0.1", local_restore_command, is_local=True)
            if result_local_restore.returncode != 0:
                logging.error(f"Failed to restore settings on local node 127.0.0.1: {result_local_restore.stderr.strip()}")
            else:
                logging.info(f"Settings restored on local node 127.0.0.1: {result_local_restore.stdout.strip()}")
            remote_restore_command = f"{CHANGE_CMD_FILE} {RESTORE_CHANNEL} {RESTORE_BANDWIDTH} {RESTORE_REGION} {server_address}"
            for ip in remote_ips:
                logging.info(f"Restoring settings on remote node {ip} with command: {remote_restore_command}")
                result_restore = run_command(ip, remote_restore_command, is_local=False)
                if result_restore.returncode != 0:
                    logging.error(f"Failed to restore settings on remote node {ip}: {result_restore.stderr.strip()}")
                else:
                    logging.info(f"Settings restored on remote node {ip}: {result_restore.stdout.strip()}")
            sys.exit(1)
        else:
            logging.info("Successfully killed killswitch on VTX.")

    # Print summary.
    print("\n=== Summary ===")
    print("Success:")
    if success:
        for ip, out in success.items():
            print(f"  {ip}: {out}")
    else:
        print("  None")
    print("\nErrors:")
    if errors:
        for ip, err in errors.items():
            print(f"  {ip}: {err}")
    else:
        print("  None")

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
    try:
        main()
    except KeyboardInterrupt:
        logging.info("KeyboardInterrupt received in main. Exiting gracefully...")
        cleanup()
        sys.exit(1)
