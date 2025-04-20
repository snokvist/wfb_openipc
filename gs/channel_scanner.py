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
CHANNELS = [40, 44, 48, 140, 149, 153, 157, 161, 165]

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
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

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
    subprocess.run(
        ["iw", "dev", dev, "set", "channel", str(ch)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

# ──────────────── Ping thread ────────────────

def ping_worker(ping_period_ms, verbose=False):
    args = ["fping", "-D", "-e", "-l", "-p", str(ping_period_ms), TARGET_IP]
    if verbose:
        print(f"[PING] {' '.join(args)}")
    proc = subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1
    )
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
            found = False
            while time.time() < deadline:
                if len(success_times) > before:
                    found = True
                    break
                time.sleep(PING_PERIOD_MS / 1000.0)

            if found:
                print("OK")
                for dev in adapters:
                    set_channel(dev, ch, verbose)
                return
            else:
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
            if scans != 0:
                print("Connection restored, resetting loop count.")
            scans = 0

# ──────────────── Single‐channel perf test ────────────────

def perf_for_channel(adapters, verbose=False):
    channel = get_current_channel(adapters[0], verbose)
    print(f"--- Performance on channel {channel} ---")
    mtus = [1200 + i*((3000 - 1200)//2) for i in range(3)]
    periods = [10, 20]
    total_tests = len(mtus) * len(periods)
    results = []
    for period in periods:
        for mtu in mtus:
            cmd = [
                "fping", "-b", str(mtu), "-p", str(period),
                "-t", str(PERF_TIMEOUT_MS), "-c", str(PERF_PACKET_COUNT),
                TARGET_IP, "-q"
            ]
            if verbose:
                print(f"[PERF] {' '.join(cmd)}")
            res = subprocess.run(cmd, stdout=subprocess.PIPE,
                                 stderr=subprocess.STDOUT, text=True)
            out = res.stdout.strip()
            print(out)
            m = re.search(r"xmt/rcv/%loss\s*=\s*(\d+)/(\d+)/(\d+)%", out)
            if not m:
                continue
            sent, recv, loss = map(int, m.groups())
            duration_s = PERF_PACKET_COUNT * (period / 1000.0)
            rate_kbps = recv * mtu * 8 / (duration_s * 1000)
            results.append({"mtu": mtu, "period": period,
                            "loss": loss, "rate": rate_kbps})
    count_good = sum(1 for r in results if r["loss"] < 2)
    percent_good = int(count_good / total_tests * 100)
    avg_loss = sum(r["loss"] for r in results) / len(results)
    avg_score = 100 - avg_loss
    results.sort(key=lambda r: (r["loss"], -r["rate"]))
    print("\nSorted Summary (loss ↑, rate ↓):")
    for r in results:
        print(f"  MTU={r['mtu']}   p={r['period']}ms   loss={r['loss']}%   "
              f"rate={r['rate']:.1f} kbps")
    print(f"\nChannel {channel} summary: {percent_good}%+{int(avg_score)}%\n")
    return {"channel": channel, "percent": percent_good, "score": avg_score}

# ──────────────── Perf‐jump across channels ────────────────

def perf_jump(adapters, verbose=False):
    primary = adapters[0]
    summary = []

    # stop majestic on remote first
    print("Stopping majestic on remote host...")
    cmd = SSH_JUMP_CMD + ["/etc/init.d/S95majestic stop"]
    res = run_cmd(cmd, verbose)
    if res.returncode != 0:
        print("ERROR: failed to stop majestic:", res.stdout, file=sys.stderr)
    else:
        print("Remote majestic stopped.")

    for ch in CHANNELS:
        print(f"\n>>> Jumping to channel {ch} <<<")
        cmd = SSH_JUMP_CMD + [f"iw dev wlan0 set channel {ch}"]
        res = run_cmd(cmd, verbose)
        if res.returncode != 0:
            print("ERROR: remote channel jump failed:", res.stdout, file=sys.stderr)
        else:
            print("Remote jump succeeded.")

        set_channel(primary, ch, verbose)
        for dev in adapters[1:]:
            set_channel(dev, ch, verbose)

        time.sleep(2)
        result = perf_for_channel(adapters, verbose)
        summary.append(result)

    summary.sort(key=lambda x: (-x["percent"], -x["score"], x["channel"]))
    print("=== All‐channels performance summary ===")
    for r in summary:
        print(f"Channel {r['channel']}: {r['percent']}%+{int(r['score'])}%")

    winner = summary[0]["channel"]
    print(f"\nSwitching to winning channel {winner}")
    # remote switch first
    cmd = SSH_JUMP_CMD + [f"iw dev wlan0 set channel {winner}"]
    res = run_cmd(cmd, verbose)
    if res.returncode != 0:
        print("ERROR: remote channel switch failed:", res.stdout, file=sys.stderr)
    else:
        print("Remote channel switch succeeded.")
    # local switch next
    set_channel(primary, winner, verbose)
    for dev in adapters[1:]:
        set_channel(dev, winner, verbose)

    # restart majestic remotely
    print("Restarting majestic on remote host...")
    cmd = SSH_JUMP_CMD + ["/etc/init.d/S95majestic restart"]
    res = run_cmd(cmd, verbose)
    if res.returncode != 0:
        print("ERROR: failed to restart majestic:", res.stdout, file=sys.stderr)
    else:
        print("Remote majestic restarted.")

    print("Entering monitor mode...")

# ──────────────── Entry point ────────────────

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="echo ping and command output")
    parser.add_argument("--loop-count", type=int, default=0,
                        help="max consecutive scan loops before exit (0=infinite)")
    parser.add_argument("--perf", action="store_true",
                        help="run local performance test suite and exit")
    parser.add_argument("--perf-jump", action="store_true",
                        help="jump channels + run perf tests remotely then monitor")
    args = parser.parse_args()

    adapters = get_adapters()
    print(f"Adapters: {adapters}")

    if args.perf:
        # stop majestic remotely
        print("Stopping majestic on remote host...")
        cmd = SSH_JUMP_CMD + ["/etc/init.d/S95majestic stop"]
        res = run_cmd(cmd, args.verbose)
        if res.returncode != 0:
            print("ERROR: failed to stop majestic:", res.stdout, file=sys.stderr)
        else:
            print("Remote majestic stopped.")

        # run performance test
        perf_for_channel(adapters, args.verbose)

        # restart majestic remotely
        print("Restarting majestic on remote host...")
        cmd = SSH_JUMP_CMD + ["/etc/init.d/S95majestic restart"]
        res = run_cmd(cmd, args.verbose)
        if res.returncode != 0:
            print("ERROR: failed to restart majestic:", res.stdout, file=sys.stderr)
        else:
            print("Remote majestic restarted.")

        sys.exit(0)

    if args.perf_jump:
        perf_jump(adapters, args.verbose)
        # fall through into monitor mode

    ping_thread = threading.Thread(
        target=ping_worker,
        args=(PING_PERIOD_MS, args.verbose),
        daemon=True
    )
    ping_thread.start()

    main_loop(adapters, args.verbose, args.loop_count)
