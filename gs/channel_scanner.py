#!/usr/bin/env python3
import argparse
import subprocess
import threading
import time
import re
import sys
from collections import deque

# ──────────────── Configuration ────────────────

TARGET_IP = "10.5.0.10"
CHANNELS = [48, 64, 104, 124, 165]

MONITOR_TIMEOUT = 5     # seconds without a success → trigger scan
SCAN_WINDOW     = 3     # seconds to watch for any success per channel
PING_PERIOD_MS  = 333   # ms between pings in monitor/scan

PERF_PACKET_COUNT = 50
PERF_TIMEOUT_MS   = 250

# SSH jump command template
SSH_JUMP_CMD = [
    "timeout", "-k", "1", "3",
    "sshpass", "-p", "12345", "ssh",
    "-o", "StrictHostKeyChecking=no",
    "-o", "UserKnownHostsFile=/dev/null",
    "-o", "ConnectTimeout=10",
    "-o", "ControlMaster=auto",
    "-o", "ControlPath=/run/ssh_control:%h:%p:%r",
    "-o", "ControlPersist=15s",
    "-o", "ServerAliveInterval=3",
    "-o", "ServerAliveCountMax=2",
    f"root@{TARGET_IP}"
]

# ──────────────── Globals ────────────────

success_times = deque(maxlen=10000)

# ──────────────── Helpers ────────────────

def run_cmd(cmd, verbose=False):
    if verbose:
        print(f"[CMD] {' '.join(cmd)}")
    return subprocess.run(cmd, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, text=True)

def get_adapters():
    res = run_cmd(["grep", "^WFB_NICS=", "/etc/default/wifibroadcast"])
    if res.returncode != 0:
        print("ERROR: unable to read WFB_NICS", file=sys.stderr)
        sys.exit(1)
    m = re.search(r'"([^"]+)"', res.stdout)
    if not m:
        print("ERROR: malformed WFB_NICS", file=sys.stderr)
        sys.exit(1)
    return m.group(1).split()

def get_current_channel(dev, verbose=False):
    res = run_cmd(["iw", "dev", dev, "info"], verbose)
    for line in res.stdout.splitlines():
        m = re.search(r"channel\s+(\d+)", line)
        if m:
            return int(m.group(1))
    return None

def set_channel(dev, ch, verbose=False):
    if verbose:
        print(f"[SET] {dev} → channel {ch}")
    subprocess.run(["iw", "dev", dev, "set", "channel", str(ch)],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# ──────────────── Ping thread ────────────────

def ping_worker(period_ms, verbose=False):
    args = ["fping", "-D", "-e", "-l", "-p", str(period_ms), TARGET_IP]
    if verbose:
        print(f"[PING] {' '.join(args)}")
    proc = subprocess.Popen(args, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True, bufsize=1)
    for line in proc.stdout:
        if verbose:
            print(line, end="")
        if "bytes" in line:
            success_times.append(time.time())

# ──────────────── Scan mode ────────────────

def scan_channels(adapters, verbose=False):
    primary = adapters[0]
    print(f"--- scanning channels for reachability to {TARGET_IP} ---")
    while True:
        for ch in CHANNELS:
            print(f"  trying channel {ch}...", end=" ", flush=True)
            set_channel(primary, ch, verbose)

            before = len(success_times)
            deadline = time.time() + SCAN_WINDOW
            while time.time() < deadline:
                if len(success_times) > before:
                    print("OK")
                    for dev in adapters:
                        set_channel(dev, ch, verbose)
                    return True
                time.sleep(PING_PERIOD_MS / 1000.0)

            print("no reply")
        print("  (no channels replied; retrying full scan…)")

# ──────────────── Main monitor loop ────────────────

def main_loop(adapters, verbose=False, loop_count=0):
    current = get_current_channel(adapters[0], verbose)
    print(f"Current channel on {adapters[0]}: {current}")
    print(f"Waiting up to {SCAN_WINDOW}s for initial ping…")
    start = time.time()
    while time.time() - start < SCAN_WINDOW:
        if success_times:
            break
        time.sleep(0.1)
    if not success_times:
        scan_channels(adapters, verbose)

    scans = 0
    while True:
        time.sleep(1)
        last = success_times[-1] if success_times else 0
        down = time.time() - last
        current = get_current_channel(adapters[0], verbose)
        if down > MONITOR_TIMEOUT:
            ts = time.strftime('%Y-%m-%d %H:%M:%S')
            print(f"{ts} → channel {current}: no success for {down:.1f}s "
                  f"(> {MONITOR_TIMEOUT}s), rescanning")
            scan_channels(adapters, verbose)
            scans += 1
            if loop_count > 0 and scans >= loop_count:
                print(f"Reached loop count limit ({loop_count}); exiting.")
                sys.exit(0)
        else:
            if scans:
                print("Connection restored, resetting loop count.")
            scans = 0

# ──────────────── Performance helpers (unchanged below) ────────────────
#   ... identical perf_for_channel() and perf_jump() functions ...
#   ... (omitted here for brevity – they remain exactly as in the user's code) ...

# ──────────────── Entry point ────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="echo ping and command output")
    parser.add_argument("--loop-count", type=int, default=0,
                        help="max consecutive scan loops before exit (0=infinite)")
    parser.add_argument("--once", action="store_true",
                        help="single run: confirm/scan for working channel then exit")
    parser.add_argument("--perf", action="store_true",
                        help="run local performance test suite and exit")
    parser.add_argument("--perf-jump", action="store_true",
                        help="jump channels + run perf tests remotely then monitor")
    args = parser.parse_args()

    adapters = get_adapters()
    print(f"Adapters: {adapters}")

    # ── Single-shot mode ───────────────────────────────────────────────
    if args.once:
        # start ping thread
        ping_thread = threading.Thread(target=ping_worker,
                                       args=(PING_PERIOD_MS, args.verbose),
                                       daemon=True)
        ping_thread.start()

        # wait a short window for an existing link
        time.sleep(SCAN_WINDOW)
        if success_times:
            print("Initial ping OK – exiting (--once)")
            sys.exit(0)

        # otherwise try one scan pass
        found = scan_channels(adapters, args.verbose)
        if found:
            print("Channel locked – exiting (--once)")
            sys.exit(0)
        else:
            print("No channel found – exiting with error (--once)")
            sys.exit(1)

    # ── Other one-off modes (perf / perf-jump) use previous logic ─────
    if args.perf:
        # ... unchanged perf block from user's latest code ...
        pass  # (omitted for brevity)

    if args.perf_jump:
        # ... unchanged perf_jump call ...
        pass  # (omitted for brevity)

    # ── Normal long-running monitor ───────────────────────────────────
    ping_thread = threading.Thread(target=ping_worker,
                                   args=(PING_PERIOD_MS, args.verbose),
                                   daemon=True)
    ping_thread.start()
    main_loop(adapters, args.verbose, args.loop_count)
