#!/usr/bin/env python3
import sys, time, subprocess, re, socket, struct, threading
from collections import defaultdict

# For Flask mode:
from flask import Flask, render_template_string, jsonify, url_for
# For ncurses mode:
import curses

# ====================================================
# Global Regular Expressions
# ====================================================
rx_ant_pattern = re.compile(
    r"^(?P<ts>\d+)\s+RX_ANT\s+(?P<freq_info>\d+:\d+:\d+)\s+(?P<antenna_id>\S+)\s+(?P<stats>.+)"
)
pkt_pattern = re.compile(r"^(?P<ts>\d+)\s+PKT\s+(?P<info>[\d:]+)")
session_pattern = re.compile(r"^(?P<ts>\d+)\s+SESSION\s+(?P<info>[\d:]+)")
decrypt_error_pattern = re.compile(r"^Unable to decrypt packet")
lost_packet_pattern = re.compile(r"^(?P<count>\d+)\s+packets lost")

# ====================================================
# Global Shared Variables
# ====================================================
global_summary = {
    "rx_ant_count": 0,
    "pkt_count": 0,
    "session_count": 0,
    "decrypt_error_count": 0,
    "lost_packet_count": 0,
    "mode": None,
}
global_latest_pkt_info = None  # {uniq, fec_rec, lost_total}

global_current_chunk = {}  # {antenna: {"rssi": int, "count_all": int}}
global_last_chunk = {}

global_log_lines = []  # store latest 300 log lines

global_time_index = 0

# Per-antenna histories (only last 100 points kept in memory)
global_rssi_history_dict = defaultdict(list)
global_lost_history_dict = defaultdict(list)

data_lock = threading.Lock()

# ====================================================
# Helper Functions
# ====================================================
def determine_mode(antenna_id: str) -> str:
    if re.search(r"[a-fA-F]", antenna_id):
        return "Cluster"
    return "Local"

def parse_cluster_antenna(antenna_str: str):
    try:
        val = int(antenna_str, 16)
        ip_part = (val >> 32) & 0xFFFFFFFF
        wlan_antenna = val & 0xFF
        ip_str = socket.inet_ntoa(struct.pack("!I", ip_part))
        return (ip_str, wlan_antenna)
    except Exception:
        return (antenna_str, None)

def generate_bar_components(value, bar_length=30):
    value = max(-90, min(value, 0))
    seg = bar_length // 3
    if value <= -60:
        red_fill = int((value + 90) / 30.0 * seg)
        if red_fill < 1:
            red_fill = 1
    else:
        red_fill = seg
    if value <= -60:
        yellow_fill = 0
    elif value <= -30:
        yellow_fill = int((value + 60) / 30.0 * seg)
    else:
        yellow_fill = seg
    if value <= -30:
        green_fill = 0
    elif value == 0:
        green_fill = seg
    else:
        green_fill = int((value + 30) / 30.0 * seg)
    red_fill = min(red_fill, seg)
    yellow_fill = min(yellow_fill, seg)
    green_fill = min(green_fill, seg)
    return red_fill, yellow_fill, green_fill, seg

def draw_colored_bar(win, row, col, value, bar_length=30):
    red_fill, yellow_fill, green_fill, seg = generate_bar_components(value, bar_length)
    filled = "█"
    empty = "░"
    for i in range(seg):
        ch = filled if i < red_fill else empty
        win.addstr(row, col + i, ch, curses.color_pair(1) if i < red_fill else 0)
    for i in range(seg):
        ch = filled if i < yellow_fill else empty
        win.addstr(row, col + seg + i, ch, curses.color_pair(2) if i < yellow_fill else 0)
    for i in range(seg):
        ch = filled if i < green_fill else empty
        win.addstr(row, col + 2*seg + i, ch, curses.color_pair(3) if i < green_fill else 0)

def generate_html_bar(value, bar_length=30):
    red_fill, yellow_fill, green_fill, seg = generate_bar_components(value, bar_length)
    empty_red = seg - red_fill
    empty_yellow = seg - yellow_fill
    empty_green = seg - green_fill
    red_seg = f'<span style="color:red;">{"█" * red_fill}</span><span style="color:gray;">{"░" * empty_red}</span>'
    yellow_seg = f'<span style="color:orange;">{"█" * yellow_fill}</span><span style="color:gray;">{"░" * empty_yellow}</span>'
    green_seg = f'<span style="color:green;">{"█" * green_fill}</span><span style="color:gray;">{"░" * empty_green}</span>'
    return red_seg + yellow_seg + green_seg

# ====================================================
# Data Collection Loop (with history trimming)
# ====================================================
def data_collection_loop():
    global global_summary, global_latest_pkt_info, global_current_chunk, global_last_chunk
    global global_rssi_history_dict, global_lost_history_dict, global_time_index, global_log_lines

    MAX_POINTS = 100  # Only keep 100 datapoints per antenna

    while True:
        try:
            proc = subprocess.Popen(
                ["pixelpilot_wfb.sh"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1
            )
        except Exception as e:
            with data_lock:
                global_log_lines.append(f"Error starting process: {e}")
                global_log_lines = global_log_lines[-300:]
            time.sleep(3)
            continue

        try:
            while True:
                try:
                    line = proc.stdout.readline()
                except Exception:
                    break
                if not line:
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                    continue
                line = line.strip()
                with data_lock:
                    global_log_lines.append(line)
                    global_log_lines = global_log_lines[-300:]
                if decrypt_error_pattern.search(line):
                    with data_lock:
                        global_summary["decrypt_error_count"] += 1
                lost_match = lost_packet_pattern.match(line)
                if lost_match:
                    with data_lock:
                        global_summary["lost_packet_count"] += int(lost_match.group("count"))
                if pkt_pattern.match(line):
                    with data_lock:
                        global_summary["pkt_count"] += 1
                        parts = line.split("\t")
                        if len(parts) >= 3:
                            pkt_fields = parts[2].split(":")
                            if len(pkt_fields) >= 8:
                                try:
                                    uniq = int(pkt_fields[5])
                                    fec_rec = int(pkt_fields[6])
                                    lost_total = int(pkt_fields[7])
                                    global_latest_pkt_info = {"uniq": uniq, "fec_rec": fec_rec, "lost_total": lost_total}
                                except Exception:
                                    global_latest_pkt_info = None
                            else:
                                global_latest_pkt_info = None
                        else:
                            global_latest_pkt_info = None
                        if global_current_chunk:
                            global_last_chunk = global_current_chunk.copy()
                            # For each antenna in the current chunk, update its history.
                            for ant, data in global_current_chunk.items():
                                c_all = data["count_all"]
                                rssi_val = data["rssi"]
                                if global_latest_pkt_info:
                                    lost_val = global_latest_pkt_info["uniq"] - c_all + global_latest_pkt_info["lost_total"] + global_latest_pkt_info["fec_rec"]
                                else:
                                    lost_val = 0
                                global_rssi_history_dict[ant].append(rssi_val)
                                global_lost_history_dict[ant].append(lost_val)
                                # Trim to only the last MAX_POINTS entries.
                                if len(global_rssi_history_dict[ant]) > MAX_POINTS:
                                    global_rssi_history_dict[ant] = global_rssi_history_dict[ant][-MAX_POINTS:]
                                if len(global_lost_history_dict[ant]) > MAX_POINTS:
                                    global_lost_history_dict[ant] = global_lost_history_dict[ant][-MAX_POINTS:]
                            global_time_index += 1
                        global_current_chunk = {}
                if session_pattern.match(line):
                    with data_lock:
                        global_summary["session_count"] += 1
                rx_match = rx_ant_pattern.match(line)
                if rx_match:
                    with data_lock:
                        global_summary["rx_ant_count"] += 1
                    antenna_id_raw = rx_match.group("antenna_id")
                    with data_lock:
                        if global_summary["mode"] is None:
                            global_summary["mode"] = determine_mode(antenna_id_raw)
                    stats = rx_match.group("stats").split(":")
                    if len(stats) >= 3:
                        try:
                            c_all = int(stats[0])
                            rssi_avg = int(stats[2])
                        except Exception:
                            rssi_avg = None
                            c_all = None
                    else:
                        rssi_avg = None
                        c_all = None
                    with data_lock:
                        if global_summary["mode"] == "Cluster":
                            ip_str, wlan_idx = parse_cluster_antenna(antenna_id_raw)
                            antenna_key = f"{ip_str} (antenna {wlan_idx})" if wlan_idx is not None else antenna_id_raw
                        else:
                            antenna_key = antenna_id_raw
                        if rssi_avg is not None and c_all is not None:
                            global_current_chunk[antenna_key] = {"rssi": rssi_avg, "count_all": c_all}
        except KeyboardInterrupt:
            proc.kill()
            break
        except Exception as e:
            with data_lock:
                global_log_lines.append(f"Error: {e}")
                global_log_lines = global_log_lines[-300:]
            time.sleep(3)
        finally:
            try:
                proc.kill()
            except Exception:
                pass
        time.sleep(3)

# ====================================================
# ncurses Display Mode
# ====================================================
def curses_mode(stdscr):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_RED, -1)
    curses.init_pair(2, curses.COLOR_YELLOW, -1)
    curses.init_pair(3, curses.COLOR_GREEN, -1)
    stdscr.nodelay(True)
    while True:
        with data_lock:
            chunk_copy = global_last_chunk.copy()
            pkt_info = global_latest_pkt_info.copy() if global_latest_pkt_info else None
            summ = global_summary.copy()
        stdscr.erase()
        stdscr.addstr(0, 0, "PixelPilot_wfb Summary (ncurses mode)")
        stdscr.addstr(1, 0, f"Mode: {summ['mode'] if summ['mode'] else 'Unknown'}")
        stdscr.addstr(2, 0, f"RX_ANT: {summ['rx_ant_count']}")
        stdscr.addstr(3, 0, f"PKT:    {summ['pkt_count']}")
        stdscr.addstr(4, 0, f"SESSION:{summ['session_count']}")
        stdscr.addstr(5, 0, f"DecErr: {summ['decrypt_error_count']}")
        stdscr.addstr(6, 0, f"LostPK: {summ['lost_packet_count']}")
        stdscr.addstr(8, 0, "Latest Chunk:")
        header = f"{'Ant':20} {'RSSI':>4}"
        stdscr.addstr(9, 0, header)
        line = 10
        if chunk_copy and pkt_info:
            for ant in sorted(chunk_copy.keys()):
                data = chunk_copy[ant]
                rssi_val = data["rssi"]
                c_all = data["count_all"]
                lost_val = pkt_info["uniq"] - c_all + pkt_info["lost_total"] + pkt_info["fec_rec"]
                rssi_str = f"{rssi_val:>4}"
                lost_str = f"{lost_val:+5d}"
                stdscr.addstr(line, 0, f"{ant[:20]:20} {rssi_str} ")
                draw_colored_bar(stdscr, line, 25, rssi_val, 30)
                stdscr.addstr(line, 25+30+1, lost_str)
                ratio = 0.0
                if c_all > 0:
                    ratio = max(0.0, min(lost_val / c_all, 1.0))
                lost_bar_val = -int(ratio * 90)
                draw_colored_bar(stdscr, line, 25+30+1+len(lost_str)+1, lost_bar_val, 30)
                line += 1
        else:
            stdscr.addstr(line, 0, "No chunk data yet.")
        stdscr.refresh()
        try:
            time.sleep(0.5)
        except KeyboardInterrupt:
            break

# ====================================================
# Flask Web UI Mode (Mobile Friendly)
# ====================================================
def create_flask_app():
    app = Flask(__name__)

    # Endpoint: Latest Chunk Data (for slim display)
    @app.route("/data/chunk")
    def data_chunk():
        chunk_stats = []
        with data_lock:
            chunk_copy = global_last_chunk.copy()
            pkt_info = global_latest_pkt_info.copy() if global_latest_pkt_info else None
        if chunk_copy and pkt_info:
            for ant in sorted(chunk_copy.keys()):
                data = chunk_copy[ant]
                rssi_val = data["rssi"]
                c_all = data["count_all"]
                lost_val = pkt_info["uniq"] - c_all + pkt_info["lost_total"] + pkt_info["fec_rec"]
                lost_str = f"{lost_val:+5d}"
                rssi_bar = generate_html_bar(rssi_val, 30)
                ratio = 0.0
                if c_all > 0:
                    ratio = max(0.0, min(lost_val / c_all, 1.0))
                lost_bar_val = -int(ratio * 90)
                lost_bar = generate_html_bar(lost_bar_val, 30)
                chunk_stats.append({
                    "antenna": ant[:20],
                    "rssi": rssi_val,
                    "lost": lost_str,
                    "rssi_bar": rssi_bar,
                    "lost_bar": lost_bar
                })
        return jsonify(chunk_stats)

    # Endpoint: Per-antenna RSSI history (last 100 points)
    @app.route("/data/rssi")
    def data_rssi():
        datasets = {}
        max_len = 0
        with data_lock:
            for ant, arr in global_rssi_history_dict.items():
                sliced = arr[-100:]
                datasets[ant] = sliced
                if len(sliced) > max_len:
                    max_len = len(sliced)
            labels = list(range(max_len))
        return jsonify({"labels": labels, "datasets": datasets})

    # Endpoint: Per-antenna Lost history (last 100 points)
    @app.route("/data/lost")
    def data_lost():
        datasets = {}
        max_len = 0
        with data_lock:
            for ant, arr in global_lost_history_dict.items():
                sliced = arr[-100:]
                datasets[ant] = sliced
                if len(sliced) > max_len:
                    max_len = len(sliced)
            labels = list(range(max_len))
        return jsonify({"labels": labels, "datasets": datasets})

    # Main HTML template with mobile-friendly layout and dynamic updates.
    template = """
    <!DOCTYPE html>
    <html>
    <head>
      <meta charset="UTF-8">
      <title>PixelPilot_wfb Web UI</title>
      <style>
        body { font-family: Arial, sans-serif; margin: 10px; }
        .refresh-btn { margin: 5px; padding: 5px 10px; font-size: 14px; }
        .stat-row { display: flex; align-items: center; margin-bottom: 5px; font-size: 14px; }
        .stat-row div { margin-right: 5px; }
        .ant-name { width: 80px; }
        .rssi, .lost { width: 40px; text-align: right; }
        .bar { font-family: monospace; }
        canvas { width: 100% !important; max-width: 500px; height: 300px !important; }
      </style>
      <!-- Load Chart.js from local static folder -->
      <script src="{{ url_for('static', filename='chart.min.js') }}"></script>
    </head>
    <body>
      <h1>PixelPilot_wfb Summary</h1>
      <div id="summary">
        <strong>Mode:</strong> <span id="mode">{{ mode }}</span><br>
        <strong>RX_ANT:</strong> <span id="rx_ant_count">{{ rx_ant_count }}</span> &nbsp;
        <strong>PKT:</strong> <span id="pkt_count">{{ pkt_count }}</span> &nbsp;
        <strong>SESSION:</strong> <span id="session_count">{{ session_count }}</span><br>
        <strong>Decryption errors:</strong> <span id="decrypt_error_count">{{ decrypt_error_count }}</span> &nbsp;
        <strong>Lost:</strong> <span id="lost_packet_count">{{ lost_packet_count }}</span>
      </div>
      <div>
        <button class="refresh-btn" onclick="setRefreshRate(200)">200ms</button>
        <button class="refresh-btn" onclick="setRefreshRate(500)">500ms</button>
        <button class="refresh-btn" onclick="setRefreshRate(1000)">1000ms</button>
      </div>
      <h2>Latest Chunk Data</h2>
      <div id="chunkStats" style="font-size:14px;"></div>
      
      <h2>History Charts</h2>
      <canvas id="chartRssi"></canvas>
      <canvas id="chartLost"></canvas>
      
      <script>
        let refreshInterval = 200;
        let refreshTimer;
        
        // Update latest chunk stats.
        async function updateChunkStats() {
          const resp = await fetch("/data/chunk");
          const stats = await resp.json();
          let html = "";
          if (stats.length === 0) {
            html = "<div>No RX_ANT data in current chunk.</div>";
          } else {
            stats.forEach(row => {
              html += `<div class="stat-row">
                          <div class="ant-name">${row.antenna}</div>
                          <div class="rssi bar">${row.rssi_bar} <span>${row.rssi}</span></div>
                          <div class="lost bar">${row.lost_bar} <span>${row.lost}</span></div>
                       </div>`;
            });
          }
          document.getElementById("chunkStats").innerHTML = html;
        }
        
        // Fixed 20-color palette.
        const colorPalette = [
          "#1f77b4","#ff7f0e","#2ca02c","#d62728","#9467bd","#8c564b","#e377c2","#7f7f7f",
          "#bcbd22","#17becf","#228b22","#ff1493","#4169e1","#9acd32","#b03060","#ffd700",
          "#008080","#c71585","#a0522d","#9400d3"
        ];
        function getColorForAntenna(ant, idx) {
          return colorPalette[idx % colorPalette.length];
        }
        
        let chartRssi, chartLost;
        
        async function updateRssiChart() {
          const resp = await fetch("/data/rssi");
          const data = await resp.json();
          chartRssi.data.labels = data.labels;
          const datasets = [];
          const ants = Object.keys(data.datasets).sort();
          ants.forEach((ant, idx) => {
            datasets.push({
              label: ant,
              data: data.datasets[ant],
              borderColor: getColorForAntenna(ant, idx),
              fill: false,
              pointRadius: 0,
              tension: 0
            });
          });
          chartRssi.data.datasets = datasets;
          chartRssi.update(0);
        }
        
        async function updateLostChart() {
          const resp = await fetch("/data/lost");
          const data = await resp.json();
          chartLost.data.labels = data.labels;
          const datasets = [];
          const ants = Object.keys(data.datasets).sort();
          ants.forEach((ant, idx) => {
            datasets.push({
              label: ant,
              data: data.datasets[ant],
              borderColor: getColorForAntenna(ant, idx),
              fill: false,
              pointRadius: 0,
              tension: 0
            });
          });
          chartLost.data.datasets = datasets;
          chartLost.update(0);
        }
        
        async function updateAll() {
          updateChunkStats();
          updateRssiChart();
          updateLostChart();
        }
        
        function setRefreshRate(ms) {
          refreshInterval = ms;
          if (refreshTimer) clearInterval(refreshTimer);
          refreshTimer = setInterval(updateAll, refreshInterval);
        }
        
        window.onload = function() {
          const ctxRssi = document.getElementById("chartRssi").getContext("2d");
          chartRssi = new Chart(ctxRssi, {
            type: 'line',
            data: {
              labels: [],
              datasets: []
            },
            options: {
              responsive: true,
              animation: false,
              plugins: { title: { display: true, text: "RSSI per Antenna" } },
              scales: {
                x: { title: { display: true, text: "Chunk Index" } },
                y: {
                  title: { display: true, text: "RSSI" },
                  min: -110,
                  max: 0,
                  ticks: { stepSize: 1 }
                }
              }
            }
          });
          const ctxLost = document.getElementById("chartLost").getContext("2d");
          chartLost = new Chart(ctxLost, {
            type: 'line',
            data: {
              labels: [],
              datasets: []
            },
            options: {
              responsive: true,
              animation: false,
              plugins: { title: { display: true, text: "Lost per Antenna" } },
              scales: {
                x: { title: { display: true, text: "Chunk Index" } },
                y: { title: { display: true, text: "Lost" }, ticks: { stepSize: 1 } }
              }
            }
          });
          setRefreshRate(200);
        };
      </script>
    </body>
    </html>
    """

    @app.route("/")
    def index():
        with data_lock:
            summ = global_summary.copy()
        return render_template_string(template,
                                      mode=summ["mode"] if summ["mode"] else "Unknown",
                                      rx_ant_count=summ["rx_ant_count"],
                                      pkt_count=summ["pkt_count"],
                                      session_count=summ["session_count"],
                                      decrypt_error_count=summ["decrypt_error_count"],
                                      lost_packet_count=summ["lost_packet_count"])

    return app

# ====================================================
# Main Entry Point
# ====================================================
def main():
    use_flask = "--flask" in sys.argv
    collector = threading.Thread(target=data_collection_loop, daemon=True)
    collector.start()
    try:
        if use_flask:
            app = create_flask_app()
            print("Launching Flask Web UI on http://0.0.0.0:5000")
            app.run(host="0.0.0.0", threaded=True, use_reloader=False)
        else:
            curses.wrapper(curses_mode)
    except KeyboardInterrupt:
        sys.exit(0)

if __name__ == "__main__":
    main()
