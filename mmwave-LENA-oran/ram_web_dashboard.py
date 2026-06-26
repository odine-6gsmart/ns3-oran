#!/usr/bin/env python3
"""
RAM Web Dashboard — NS-3 + xApp monitor (dual-source, unlimited run).

Two independent RAM data sources:
  External (Python)  — /proc/<pid>/status polled every 0.5s wall clock
  Internal (C++)     — ns3_ram_usage.csv written by NS-3 every 100ms sim time

Dashboard: http://localhost:5050
Stops automatically when NS-3 exits. Ctrl+C stops early.
Both CSVs written incrementally — safe to kill at any point.
"""

import os
import sys
import csv
import time
import signal
import threading
import subprocess
from flask import Flask, Response, jsonify

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
SCRIPT_DIR        = os.path.dirname(os.path.abspath(__file__))
XAPP_SCRIPT       = os.path.join(SCRIPT_DIR, "xapp_template.py")
XAPP_OUTPUT_DIR   = os.path.join(SCRIPT_DIR, "xapp_template_outputs")
NS3_RAM_CSV       = os.path.join(XAPP_OUTPUT_DIR, "ns3_ram_usage.csv")
CSV_OUTPUT        = os.path.join(SCRIPT_DIR, "ram_dashboard_results.csv")
SAMPLE_INTERVAL   = 0.5    # wall-clock seconds between external RAM samples
SIMRAM_POLL       = 0.5    # wall-clock seconds between ns3_ram_usage.csv reads
NS3_SEARCH_KEY    = "Differing_Power_Scenerio_HO"
NS3_FIND_TIMEOUT  = 30.0
NS3_EXIT_GRACE    = 5.0
RATE_WINDOW       = 20     # samples for rolling wall-time rate  (~10s)
SIMRATE_WINDOW    = 20     # samples for rolling sim-time rate   (~2s sim)
PORT              = 5053

app = Flask(__name__)

# ---------------------------------------------------------------------------
# /proc helpers
# ---------------------------------------------------------------------------

def get_rss_mb(pid):
    try:
        with open(f"/proc/{pid}/status") as f:
            for line in f:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except Exception:
        pass
    return 0.0


def pid_alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except Exception:
        return False


def find_ns3_pid():
    try:
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as f:
                    cmdline = f.read().decode(errors="replace")
                if NS3_SEARCH_KEY in cmdline:
                    return int(entry)
            except Exception:
                pass
    except Exception:
        pass
    return None


def get_system_ram():
    info = {"total": 0.0, "available": 0.0, "used": 0.0, "pct": 0.0}
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    info["total"] = int(line.split()[1]) / 1024.0
                elif line.startswith("MemAvailable:"):
                    info["available"] = int(line.split()[1]) / 1024.0
    except Exception:
        pass
    info["used"] = info["total"] - info["available"]
    info["pct"]  = (info["used"] / info["total"] * 100.0) if info["total"] > 0 else 0.0
    return info


def read_sim_time():
    """Read latest simulation time from UEPosition.txt."""
    pos_file = os.path.join(XAPP_OUTPUT_DIR, "UEPosition.txt")
    try:
        with open(pos_file, "rb") as f:
            try:
                f.seek(-512, os.SEEK_END)
            except OSError:
                f.seek(0)
            for line in reversed(f.readlines()):
                try:
                    parts = line.decode(errors="replace").strip().split(",")
                    if parts and parts[0].replace(".", "", 1).isdigit():
                        return float(parts[0])
                except Exception:
                    continue
    except Exception:
        pass
    return None


def growth_rate(samples_xy):
    """Linear regression slope over (x, y) pairs."""
    n = len(samples_xy)
    if n < 2:
        return 0.0
    xs = [p[0] for p in samples_xy]
    ys = [p[1] for p in samples_xy]
    xm = sum(xs) / n
    ym = sum(ys) / n
    num = sum((x - xm) * (y - ym) for x, y in zip(xs, ys))
    den = sum((x - xm) ** 2 for x in xs)
    return num / den if den else 0.0


# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.lock          = threading.Lock()
        # External (Python) samples: (wall_elapsed, ns3_mb, xapp_mb, sys_free_mb, sim_time)
        self.samples       = []
        # Internal (C++) samples from ns3_ram_usage.csv: (sim_time_s, ram_mb)
        self.simram        = []
        self.simram_ptr    = 0       # byte offset in ns3_ram_usage.csv
        # Process info
        self.ns3_pid       = None
        self.xapp_pid      = None
        self.ns3_status    = "searching"
        self.xapp_status   = "starting"
        self.ns3_peak      = 0.0
        self.xapp_peak     = 0.0
        self.elapsed       = 0.0
        self.sim_time      = 0.0
        self.done          = False
        self.sys           = {"total": 0.0, "used": 0.0, "available": 0.0, "pct": 0.0}

G = State()

# ---------------------------------------------------------------------------
# SimRAM reader — polls ns3_ram_usage.csv written by the C++ simulation
# ---------------------------------------------------------------------------

def simram_reader_thread():
    """
    Reads new lines appended to ns3_ram_usage.csv (written by NS-3 every
    100ms sim time) and stores them in G.simram.
    Uses a file pointer so only new bytes are read each tick.
    """
    ptr = 0
    header_skipped = False

    while True:
        with G.lock:
            done = G.done

        try:
            if os.path.exists(NS3_RAM_CSV):
                with open(NS3_RAM_CSV, "rb") as f:
                    f.seek(ptr)
                    chunk = f.read()
                    ptr = f.tell()

                if chunk:
                    lines = chunk.decode(errors="replace").splitlines()
                    new_rows = []
                    for line in lines:
                        line = line.strip()
                        if not line:
                            continue
                        if not header_skipped:
                            header_skipped = True   # skip "sim_time_s,ram_mb"
                            continue
                        parts = line.split(",")
                        if len(parts) == 2:
                            try:
                                new_rows.append((float(parts[0]), float(parts[1])))
                            except ValueError:
                                pass

                    if new_rows:
                        with G.lock:
                            G.simram.extend(new_rows)
                            G.simram_ptr = ptr
        except Exception:
            pass

        if done:
            break

        time.sleep(SIMRAM_POLL)


# ---------------------------------------------------------------------------
# External sampler thread — /proc/<pid>/status every 0.5s wall clock
# ---------------------------------------------------------------------------

def sampler_thread(xapp_proc, start_time):
    ns3_find_deadline = start_time + NS3_FIND_TIMEOUT
    ns3_pid           = None
    xapp_pid          = xapp_proc.pid
    ns3_was_running   = False
    ns3_exit_time     = None

    csv_fh     = open(CSV_OUTPUT, "w", newline="")
    csv_writer = csv.writer(csv_fh)
    csv_writer.writerow(["wall_time_s", "ns3_ram_mb", "xapp_ram_mb",
                         "total_ram_mb", "sys_free_mb", "sim_time_s"])

    try:
        while not G.done:
            now     = time.monotonic()
            elapsed = now - start_time

            # Discover NS-3 PID
            if ns3_pid is None:
                ns3_pid = find_ns3_pid()
                if ns3_pid:
                    with G.lock:
                        G.ns3_pid    = ns3_pid
                        G.ns3_status = "running"
                    ns3_was_running = True
                elif now > ns3_find_deadline:
                    with G.lock:
                        if G.ns3_status == "searching":
                            G.ns3_status = "not found"

            # Liveness
            ns3_alive  = ns3_pid is not None and pid_alive(ns3_pid)
            xapp_alive = pid_alive(xapp_pid)

            if ns3_was_running and not ns3_alive:
                if ns3_exit_time is None:
                    ns3_exit_time = now
                    with G.lock:
                        G.ns3_status = "exited"
                if now - ns3_exit_time >= NS3_EXIT_GRACE:
                    with G.lock:
                        G.done = True
                    break

            if not xapp_alive and not ns3_was_running:
                with G.lock:
                    G.done = True
                break

            ns3_st  = "running" if ns3_alive else ("exited" if ns3_was_running else G.ns3_status)
            xapp_st = "running" if xapp_alive else "exited"

            ns3_mb  = get_rss_mb(ns3_pid) if ns3_pid else 0.0
            xapp_mb = get_rss_mb(xapp_pid)
            sys_r   = get_system_ram()
            sim_t   = read_sim_time() or 0.0

            with G.lock:
                G.elapsed      = elapsed
                G.xapp_pid     = xapp_pid
                G.ns3_status   = ns3_st
                G.xapp_status  = xapp_st
                G.sys          = sys_r
                G.sim_time     = sim_t
                if ns3_mb  > G.ns3_peak:  G.ns3_peak  = ns3_mb
                if xapp_mb > G.xapp_peak: G.xapp_peak = xapp_mb
                G.samples.append((elapsed, ns3_mb, xapp_mb, sys_r["available"], sim_t))

            csv_writer.writerow([
                f"{elapsed:.2f}", f"{ns3_mb:.1f}", f"{xapp_mb:.1f}",
                f"{ns3_mb + xapp_mb:.1f}", f"{sys_r['available']:.1f}", f"{sim_t:.3f}"
            ])
            csv_fh.flush()

            time.sleep(SAMPLE_INTERVAL)

    finally:
        csv_fh.close()

    with G.lock:
        G.done = True


# ---------------------------------------------------------------------------
# Flask routes
# ---------------------------------------------------------------------------

@app.route("/")
def index():
    return Response(HTML, mimetype="text/html")


@app.route("/api/data")
def api_data():
    with G.lock:
        samples   = list(G.samples)
        ns3_st    = G.ns3_status
        xapp_st   = G.xapp_status
        ns3_peak  = G.ns3_peak
        xapp_peak = G.xapp_peak
        elapsed   = G.elapsed
        done      = G.done
        sys_r     = dict(G.sys)
        ns3_pid   = G.ns3_pid
        xapp_pid  = G.xapp_pid
        sim_time  = G.sim_time
        simram    = list(G.simram)

    # External rolling rate (per wall second)
    tail      = samples[-RATE_WINDOW:] if len(samples) >= 2 else samples
    ns3_rate  = growth_rate([(s[0], s[1]) for s in tail])
    xapp_rate = growth_rate([(s[0], s[2]) for s in tail])

    ns3_now    = samples[-1][1] if samples else 0.0
    xapp_now   = samples[-1][2] if samples else 0.0
    ns3_delta  = ns3_now  - samples[0][1] if len(samples) > 1 else 0.0
    xapp_delta = xapp_now - samples[0][2] if len(samples) > 1 else 0.0

    wall_per_sim = round(elapsed / sim_time, 1) if sim_time > 0.5 else None

    # Internal (C++) latest value and sim-time rate
    internal_now  = simram[-1][1] if simram else None
    sim_tail      = simram[-SIMRATE_WINDOW:] if len(simram) >= 2 else simram
    rate_per_sim  = growth_rate([(s[0], s[1]) for s in sim_tail])  # MB per sim-second

    # Agreement between external and internal readings
    agreement = None
    if internal_now is not None and ns3_now > 0:
        agreement = round(ns3_now - internal_now, 2)

    MAX_CHART = 600
    chart_samples = samples[-MAX_CHART:]

    return jsonify({
        "elapsed":       round(elapsed, 1),
        "sim_time":      round(sim_time, 2),
        "wall_per_sim":  wall_per_sim,
        "done":          done,
        "total_samples": len(samples),
        "ns3": {
            "pid":          ns3_pid,
            "status":       ns3_st,
            "now":          round(ns3_now, 1),
            "peak":         round(ns3_peak, 1),
            "delta":        round(ns3_delta, 1),
            "rate":         round(ns3_rate, 3),       # MB/wall-sec
            "internal_now": round(internal_now, 1) if internal_now is not None else None,
            "rate_per_sim": round(rate_per_sim, 4),   # MB/sim-sec
            "agreement":    agreement,
        },
        "xapp": {
            "pid":    xapp_pid,
            "status": xapp_st,
            "now":    round(xapp_now, 1),
            "peak":   round(xapp_peak, 1),
            "delta":  round(xapp_delta, 1),
            "rate":   round(xapp_rate, 3),
        },
        "system": {
            "total":     round(sys_r["total"], 0),
            "used":      round(sys_r["used"], 0),
            "available": round(sys_r["available"], 0),
            "pct":       round(sys_r["pct"], 1),
        },
        "chart": {
            "labels": [round(s[0], 1) for s in chart_samples],
            "ns3":    [round(s[1], 1) for s in chart_samples],
            "xapp":   [round(s[2], 1) for s in chart_samples],
            "total":  [round(s[1] + s[2], 1) for s in chart_samples],
        },
        "simram_chart": {
            "labels": [round(s[0], 3) for s in simram[-MAX_CHART:]],
            "ram":    [round(s[1], 2) for s in simram[-MAX_CHART:]],
            "total":  len(simram),
        },
    })


# ---------------------------------------------------------------------------
# HTML + JS
# ---------------------------------------------------------------------------

HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1.0"/>
<title>RAM Dashboard — NS-3 + xApp</title>
<script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.0/dist/chart.umd.min.js"></script>
<style>
  :root {
    --bg:#0d1117; --card:#161b22; --border:#30363d;
    --text:#e6edf3; --muted:#8b949e;
    --green:#3fb950; --yellow:#d29922; --red:#f85149;
    --blue:#58a6ff; --purple:#bc8cff; --cyan:#39d353; --orange:#f0883e;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',system-ui,monospace;}

  header{display:flex;align-items:center;justify-content:space-between;
    padding:14px 24px;background:var(--card);border-bottom:1px solid var(--border);
    flex-wrap:wrap;gap:8px;}
  header h1{font-size:1.1rem;font-weight:600;color:var(--blue);}
  #header-right{display:flex;align-items:center;gap:20px;flex-wrap:wrap;}
  .hstat{display:flex;flex-direction:column;align-items:flex-end;}
  .hstat-label{font-size:.67rem;color:var(--muted);text-transform:uppercase;letter-spacing:.8px;}
  .hstat-val{font-size:.95rem;font-weight:700;font-family:monospace;}
  #pulse{width:10px;height:10px;border-radius:50%;background:var(--green);
    animation:pulse 1.4s ease-in-out infinite;}
  #pulse.stopped{background:var(--red);animation:none;}
  @keyframes pulse{0%,100%{opacity:1;transform:scale(1)}50%{opacity:.4;transform:scale(.7)}}
  #done-badge{display:none;background:var(--green);color:#000;
    padding:3px 12px;border-radius:12px;font-size:.78rem;font-weight:700;}

  .grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));
    gap:16px;padding:20px 20px 0;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:10px;padding:18px;}
  .card-title{font-size:.72rem;text-transform:uppercase;letter-spacing:1px;
    color:var(--muted);margin-bottom:12px;}

  .proc-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:8px;}
  .proc-name{font-size:1rem;font-weight:700;}
  .status-badge{font-size:.7rem;font-weight:700;padding:2px 8px;border-radius:10px;text-transform:uppercase;}
  .status-running  {background:#1a3a1f;color:var(--green); border:1px solid var(--green);}
  .status-exited   {background:#3a1a1a;color:var(--red);   border:1px solid var(--red);}
  .status-searching{background:#2a2a1a;color:var(--yellow);border:1px solid var(--yellow);}

  .metric-row{display:flex;justify-content:space-between;margin:5px 0;}
  .metric-label{color:var(--muted);font-size:.8rem;}
  .metric-val{font-size:.83rem;font-weight:600;font-family:monospace;}
  .big-ram{font-size:2.2rem;font-weight:700;font-family:monospace;
    margin:6px 0 3px;letter-spacing:-1px;}
  .unit{font-size:.95rem;color:var(--muted);}
  .rate-line{font-size:.8rem;margin-top:3px;}

  /* source comparison block */
  .src-block{background:#0d1117;border:1px solid var(--border);border-radius:6px;
    padding:10px 12px;margin-top:10px;}
  .src-title{font-size:.68rem;text-transform:uppercase;letter-spacing:.8px;
    color:var(--muted);margin-bottom:7px;}
  .src-row{display:flex;justify-content:space-between;align-items:center;margin:4px 0;}
  .src-label{font-size:.78rem;color:var(--muted);}
  .src-val{font-size:.82rem;font-weight:700;font-family:monospace;}
  .agree-good{color:var(--green);}
  .agree-warn{color:var(--yellow);}
  .agree-bad {color:var(--red);}

  .sys-bar-outer{background:var(--border);border-radius:4px;height:9px;overflow:hidden;margin:4px 0 2px;}
  .sys-bar-inner{height:100%;border-radius:4px;transition:width .5s;}

  #verdict{margin:14px 20px 0;padding:11px 16px;border-radius:8px;
    font-size:.88rem;font-weight:600;display:none;border-left:4px solid;}
  #verdict.stable {background:#0d2318;border-color:var(--green); color:var(--green);}
  #verdict.leaking{background:#2a0d0d;border-color:var(--red);   color:var(--red);}
  #verdict.slow   {background:#2a1f0d;border-color:var(--yellow);color:var(--yellow);}

  .chart-card{background:var(--card);border:1px solid var(--border);border-radius:10px;
    padding:18px;margin:16px 20px 0;}
  .chart-card:last-of-type{margin-bottom:20px;}
  .chart-header{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;
    flex-wrap:wrap;gap:6px;}
  .chart-legend{display:flex;gap:16px;font-size:.76rem;color:var(--muted);}
  .legend-dot{display:inline-block;width:9px;height:9px;border-radius:50%;margin-right:4px;}
  .chart-tag{font-size:.7rem;color:var(--muted);background:var(--border);
    padding:2px 7px;border-radius:8px;}
  #ramChart,#simChart{max-height:260px;}

  footer{text-align:center;padding:10px;font-size:.72rem;color:var(--muted);
    border-top:1px solid var(--border);}
</style>
</head>
<body>

<header>
  <h1>&#x1F4CA; RAM Dashboard &nbsp;—&nbsp; NS-3 + xApp &nbsp;<span id="pulse"></span></h1>
  <div id="header-right">
    <div class="hstat">
      <span class="hstat-label">Wall Time</span>
      <span class="hstat-val" id="h-elapsed" style="color:var(--blue)">0s</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Sim Time</span>
      <span class="hstat-val" id="h-simtime" style="color:var(--cyan)">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Wall/Sim</span>
      <span class="hstat-val" id="h-ratio" style="color:var(--purple)">—</span>
    </div>
    <div class="hstat">
      <span class="hstat-label">Int. Samples</span>
      <span class="hstat-val" id="h-simrows" style="color:var(--orange)">0</span>
    </div>
    <span id="done-badge">DONE</span>
  </div>
</header>

<div id="verdict"></div>

<div class="grid">

  <!-- NS-3 card -->
  <div class="card">
    <div class="card-title">NS-3 Simulation</div>
    <div class="proc-header">
      <span class="proc-name">&#x1F4E1; NS-3</span>
      <span id="ns3-badge" class="status-badge status-searching">searching</span>
    </div>
    <div class="big-ram" id="ns3-ram">—<span class="unit"> MB</span></div>
    <div class="rate-line">
      Wall rate: <span id="ns3-rate">—</span> MB/s &nbsp;|&nbsp;
      Sim rate: <span id="ns3-simrate">—</span> MB/sim-s
    </div>
    <hr style="border-color:var(--border);margin:10px 0">
    <div class="metric-row"><span class="metric-label">PID</span>          <span class="metric-val" id="ns3-pid">—</span></div>
    <div class="metric-row"><span class="metric-label">Peak RAM</span>     <span class="metric-val" id="ns3-peak">—</span></div>
    <div class="metric-row"><span class="metric-label">Δ from start</span> <span class="metric-val" id="ns3-delta">—</span></div>

    <!-- Dual-source comparison block -->
    <div class="src-block">
      <div class="src-title">&#x1F50D; Dual-Source Comparison</div>
      <div class="src-row">
        <span class="src-label">&#x1F4BB; External (Python /proc)</span>
        <span class="src-val" id="cmp-external" style="color:var(--blue)">—</span>
      </div>
      <div class="src-row">
        <span class="src-label">&#x2699;&#xFE0F; Internal (C++ /proc/self)</span>
        <span class="src-val" id="cmp-internal" style="color:var(--orange)">—</span>
      </div>
      <div class="src-row">
        <span class="src-label">Agreement (Ext − Int)</span>
        <span class="src-val" id="cmp-agree">—</span>
      </div>
    </div>
  </div>

  <!-- xApp card -->
  <div class="card">
    <div class="card-title">xApp (Python)</div>
    <div class="proc-header">
      <span class="proc-name">&#x1F40D; xApp</span>
      <span id="xapp-badge" class="status-badge status-searching">starting</span>
    </div>
    <div class="big-ram" id="xapp-ram">—<span class="unit"> MB</span></div>
    <div class="rate-line">Wall rate: <span id="xapp-rate">—</span> MB/s &nbsp;(10s avg)</div>
    <hr style="border-color:var(--border);margin:10px 0">
    <div class="metric-row"><span class="metric-label">PID</span>          <span class="metric-val" id="xapp-pid">—</span></div>
    <div class="metric-row"><span class="metric-label">Peak RAM</span>     <span class="metric-val" id="xapp-peak">—</span></div>
    <div class="metric-row"><span class="metric-label">Δ from start</span> <span class="metric-val" id="xapp-delta">—</span></div>
  </div>

  <!-- System card -->
  <div class="card">
    <div class="card-title">System Memory</div>
    <div class="metric-row"><span class="metric-label">Total</span> <span class="metric-val" id="sys-total">—</span></div>
    <div class="metric-row"><span class="metric-label">Used</span>  <span class="metric-val" id="sys-used">—</span></div>
    <div>
      <div class="sys-bar-outer"><div id="sys-bar" class="sys-bar-inner" style="width:0%;background:var(--blue)"></div></div>
      <div style="font-size:.73rem;color:var(--muted);text-align:right" id="sys-pct">0%</div>
    </div>
    <div class="metric-row"><span class="metric-label">Free</span>  <span class="metric-val" id="sys-free">—</span></div>
    <hr style="border-color:var(--border);margin:10px 0">
    <div class="metric-row"><span class="metric-label">Process total</span>    <span class="metric-val" id="combined-ram">—</span></div>
    <div class="metric-row"><span class="metric-label">Combined growth</span>  <span class="metric-val" id="combined-rate">—</span></div>
    <div class="metric-row"><span class="metric-label">Page cache drop</span>  <span class="metric-val" id="page-cache">—</span></div>
  </div>

  <!-- Timing card -->
  <div class="card">
    <div class="card-title">Timing &amp; Speed</div>
    <div class="metric-row"><span class="metric-label">Wall time</span>         <span class="metric-val" id="t-wall">—</span></div>
    <div class="metric-row"><span class="metric-label">Sim time</span>          <span class="metric-val" id="t-sim">—</span></div>
    <div class="metric-row"><span class="metric-label">Wall / Sim ratio</span>  <span class="metric-val" id="t-ratio" style="color:var(--yellow)">—</span></div>
    <hr style="border-color:var(--border);margin:10px 0">
    <div class="metric-row"><span class="metric-label">Ext. samples</span>  <span class="metric-val" id="t-samples">—</span></div>
    <div class="metric-row"><span class="metric-label">Int. samples</span>  <span class="metric-val" id="t-simrows" style="color:var(--orange)">—</span></div>
    <div class="metric-row"><span class="metric-label">Int. resolution</span><span class="metric-val" style="color:var(--muted)">100ms sim time</span></div>
    <div class="metric-row"><span class="metric-label">CSVs</span>          <span class="metric-val" style="color:var(--green);font-size:.75rem">live incremental</span></div>
  </div>

</div>

<!-- Chart 1: Wall-time aligned (external Python source) -->
<div class="chart-card">
  <div class="chart-header">
    <div>
      <span style="font-size:.78rem;text-transform:uppercase;letter-spacing:1px;color:var(--muted)">RAM Timeline</span>
      <span class="chart-tag" style="margin-left:8px">&#x1F4BB; External — x: wall clock</span>
    </div>
    <span id="chart1-note" style="font-size:.7rem;color:var(--muted)"></span>
    <div class="chart-legend">
      <span><span class="legend-dot" style="background:var(--blue)"></span>NS-3</span>
      <span><span class="legend-dot" style="background:var(--purple)"></span>xApp</span>
      <span><span class="legend-dot" style="background:var(--cyan)"></span>Total</span>
    </div>
  </div>
  <canvas id="ramChart"></canvas>
</div>

<!-- Chart 2: Sim-time aligned (internal C++ source) -->
<div class="chart-card">
  <div class="chart-header">
    <div>
      <span style="font-size:.78rem;text-transform:uppercase;letter-spacing:1px;color:var(--muted)">NS-3 Internal RAM</span>
      <span class="chart-tag" style="margin-left:8px">&#x2699;&#xFE0F; C++ /proc/self — x: sim time</span>
    </div>
    <span id="chart2-note" style="font-size:.7rem;color:var(--muted)"></span>
    <div class="chart-legend">
      <span><span class="legend-dot" style="background:var(--orange)"></span>NS-3 (internal)</span>
    </div>
  </div>
  <canvas id="simChart"></canvas>
</div>

<footer>
  External: /proc sampled every 500ms wall clock &nbsp;|&nbsp;
  Internal: C++ /proc/self every 100ms sim time → ns3_ram_usage.csv &nbsp;|&nbsp;
  Both CSVs written live &nbsp;|&nbsp; Stops when NS-3 exits
</footer>

<script>
let wallChart, simChart;
let sysFreeStart = null;

function fmtHMS(s) {
  const h = Math.floor(s/3600), m = Math.floor((s%3600)/60), sec = Math.floor(s%60);
  if (h > 0) return `${h}h ${String(m).padStart(2,'0')}m ${String(sec).padStart(2,'0')}s`;
  if (m > 0) return `${m}m ${String(sec).padStart(2,'0')}s`;
  return `${sec}s`;
}
function rateColor(r) {
  const a = Math.abs(r);
  return a >= 5 ? 'var(--red)' : a >= 1 ? 'var(--yellow)' : 'var(--green)';
}
function statusClass(st) {
  return st==='running' ? 'status-running' : st==='exited' ? 'status-exited' : 'status-searching';
}
function set(id, text, color) {
  const el = document.getElementById(id);
  if (!el) return;
  el.textContent = text;
  if (color !== undefined) el.style.color = color;
}
function fmtRate(r) { return (r>=0?'+':'')+r.toFixed(3); }

function initCharts() {
  const commonOpts = (xLabel) => ({
    animation: false, responsive: true,
    interaction: { mode:'index', intersect:false },
    scales: {
      x: { ticks:{ color:'#8b949e', maxTicksLimit:10 }, grid:{ color:'#21262d' } },
      y: { ticks:{ color:'#8b949e', callback: v => v.toFixed(0)+' MB' },
           grid:{ color:'#21262d' }, min:0 }
    },
    plugins: {
      legend: { display:false },
      tooltip: { callbacks: {
        title: items => xLabel + items[0].label,
        label: item  => ` ${item.dataset.label}: ${item.raw.toFixed(1)} MB`
      }}
    }
  });

  wallChart = new Chart(document.getElementById('ramChart').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'NS-3 (MB)',  data:[], borderColor:'#58a6ff', backgroundColor:'rgba(88,166,255,.07)',
          tension:.3, pointRadius:0, fill:true,  borderWidth:2 },
        { label:'xApp (MB)',  data:[], borderColor:'#bc8cff', backgroundColor:'rgba(188,140,255,.05)',
          tension:.3, pointRadius:0, fill:true,  borderWidth:1.5 },
        { label:'Total (MB)', data:[], borderColor:'#39d353', backgroundColor:'transparent',
          tension:.3, pointRadius:0, fill:false, borderWidth:1.5, borderDash:[4,3] },
      ]
    },
    options: {
      ...commonOpts('Wall: '),
      scales: {
        x: {
          ticks:{ color:'#8b949e', maxTicksLimit:10,
            callback:(_,i) => { const v=wallChart.data.labels[i]; return v!==undefined?fmtHMS(v):''; }},
          grid:{ color:'#21262d' }
        },
        y: { ticks:{ color:'#8b949e', callback:v=>v.toFixed(0)+' MB' }, grid:{ color:'#21262d' }, min:0 }
      }
    }
  });

  simChart = new Chart(document.getElementById('simChart').getContext('2d'), {
    type: 'line',
    data: {
      labels: [],
      datasets: [
        { label:'NS-3 internal (MB)', data:[], borderColor:'#f0883e',
          backgroundColor:'rgba(240,136,62,.08)',
          tension:.3, pointRadius:0, fill:true, borderWidth:2 },
      ]
    },
    options: {
      ...commonOpts('Sim: '),
      scales: {
        x: {
          ticks:{ color:'#8b949e', maxTicksLimit:10,
            callback:(_,i) => { const v=simChart.data.labels[i]; return v!==undefined?v.toFixed(1)+'s':''; }},
          grid:{ color:'#21262d' }
        },
        y: { ticks:{ color:'#8b949e', callback:v=>v.toFixed(0)+' MB' }, grid:{ color:'#21262d' }, min:0 }
      }
    }
  });
}

function updateUI(d) {
  // Header
  set('h-elapsed', fmtHMS(d.elapsed), 'var(--blue)');
  set('h-simtime', d.sim_time > 0 ? d.sim_time.toFixed(2)+'s' : '—', 'var(--cyan)');
  set('h-ratio',   d.wall_per_sim ? d.wall_per_sim+'x' : '—', 'var(--purple)');
  set('h-simrows', d.simram_chart.total.toLocaleString(), 'var(--orange)');
  if (d.done) {
    document.getElementById('done-badge').style.display = 'inline';
    document.getElementById('pulse').classList.add('stopped');
  }

  // NS-3 card
  const n = d.ns3;
  document.getElementById('ns3-badge').textContent = n.status;
  document.getElementById('ns3-badge').className   = 'status-badge ' + statusClass(n.status);
  document.getElementById('ns3-ram').innerHTML     = n.now.toFixed(1) + '<span class="unit"> MB</span>';

  const nr = document.getElementById('ns3-rate');
  nr.textContent = fmtRate(n.rate); nr.style.color = rateColor(n.rate);
  const sr = document.getElementById('ns3-simrate');
  sr.textContent = fmtRate(n.rate_per_sim); sr.style.color = rateColor(n.rate_per_sim);

  set('ns3-pid',   n.pid || '—');
  set('ns3-peak',  n.peak.toFixed(1)+' MB');
  set('ns3-delta', (n.delta>=0?'+':'')+n.delta.toFixed(1)+' MB');

  // Dual-source comparison
  set('cmp-external', n.now.toFixed(1)+' MB', 'var(--blue)');
  if (n.internal_now !== null && n.internal_now !== undefined) {
    set('cmp-internal', n.internal_now.toFixed(1)+' MB', 'var(--orange)');
    const ag = n.agreement;
    const agEl = document.getElementById('cmp-agree');
    agEl.textContent = (ag>=0?'+':'')+ag.toFixed(1)+' MB';
    const absAg = Math.abs(ag);
    agEl.className = 'src-val ' + (absAg < 5 ? 'agree-good' : absAg < 20 ? 'agree-warn' : 'agree-bad');
  } else {
    set('cmp-internal', 'waiting…', 'var(--muted)');
    set('cmp-agree', '—');
  }

  // xApp card
  const x = d.xapp;
  document.getElementById('xapp-badge').textContent = x.status;
  document.getElementById('xapp-badge').className   = 'status-badge ' + statusClass(x.status);
  document.getElementById('xapp-ram').innerHTML     = x.now.toFixed(1) + '<span class="unit"> MB</span>';
  const xr = document.getElementById('xapp-rate');
  xr.textContent = fmtRate(x.rate); xr.style.color = rateColor(x.rate);
  set('xapp-pid',   x.pid || '—');
  set('xapp-peak',  x.peak.toFixed(1)+' MB');
  set('xapp-delta', (x.delta>=0?'+':'')+x.delta.toFixed(1)+' MB');

  // System card
  const s = d.system;
  if (sysFreeStart === null && s.available > 0) sysFreeStart = s.available;
  const drop = sysFreeStart !== null ? sysFreeStart - s.available : 0;
  set('sys-total', s.total.toFixed(0)+' MB');
  set('sys-used',  s.used.toFixed(0)+' MB');
  set('sys-free',  s.available.toFixed(0)+' MB');
  set('sys-pct',   s.pct.toFixed(1)+'%');
  const bar = document.getElementById('sys-bar');
  bar.style.width      = s.pct+'%';
  bar.style.background = s.pct>80?'var(--red)':s.pct>60?'var(--yellow)':'var(--blue)';
  const combined = n.now + x.now;
  const combRate = n.rate + x.rate;
  set('combined-ram',  combined.toFixed(1)+' MB');
  const cr = document.getElementById('combined-rate');
  cr.textContent = fmtRate(combRate)+' MB/s'; cr.style.color = rateColor(combRate);
  set('page-cache', drop > 0 ? '−'+drop.toFixed(0)+' MB' : '—');

  // Timing card
  set('t-wall',    fmtHMS(d.elapsed));
  set('t-sim',     d.sim_time > 0 ? d.sim_time.toFixed(2)+'s' : '—');
  set('t-ratio',   d.wall_per_sim ? d.wall_per_sim+'x real' : '—');
  set('t-samples', d.total_samples.toLocaleString());
  set('t-simrows', d.simram_chart.total.toLocaleString());

  // Chart 1 — wall-time
  wallChart.data.labels           = d.chart.labels;
  wallChart.data.datasets[0].data = d.chart.ns3;
  wallChart.data.datasets[1].data = d.chart.xapp;
  wallChart.data.datasets[2].data = d.chart.total;
  wallChart.update('none');
  const c1note = document.getElementById('chart1-note');
  c1note.textContent = d.total_samples > 600
    ? `Showing last 600 of ${d.total_samples.toLocaleString()} samples` : '';

  // Chart 2 — sim-time
  simChart.data.labels           = d.simram_chart.labels;
  simChart.data.datasets[0].data = d.simram_chart.ram;
  simChart.update('none');
  const c2note = document.getElementById('chart2-note');
  c2note.textContent = d.simram_chart.total > 600
    ? `Showing last 600 of ${d.simram_chart.total.toLocaleString()} samples` : '';

  // Verdict (after 60s wall time)
  if (d.elapsed >= 60) {
    const v   = document.getElementById('verdict');
    const abs = Math.abs(n.rate);
    v.style.display = 'block';
    if (abs >= 5.0) {
      v.className   = 'verdict leaking';
      v.textContent = '⚠️  LEAK DETECTED — NS-3 wall rate: ' + fmtRate(n.rate) +
        ' MB/s | Sim rate: ' + fmtRate(n.rate_per_sim) +
        ' MB/sim-s. Expected <1 MB/s with fix applied.';
    } else if (abs >= 1.0) {
      v.className   = 'verdict slow';
      v.textContent = '⚡  Slow growth — wall rate: ' + fmtRate(n.rate) +
        ' MB/s | sim rate: ' + fmtRate(n.rate_per_sim) +
        ' MB/sim-s. Normal trace accumulation is 0.2–0.5 MB/s.';
    } else {
      v.className   = 'verdict stable';
      v.textContent = '✅  RAM stable — wall rate: ' + fmtRate(n.rate) +
        ' MB/s | sim rate: ' + fmtRate(n.rate_per_sim) +
        ' MB/sim-s. Leak fix confirmed working.';
    }
  }
}

async function poll() {
  while (true) {
    try {
      const resp = await fetch('/api/data');
      const data = await resp.json();
      updateUI(data);
      if (data.done) break;
    } catch(e) { console.warn('poll:', e); }
    await new Promise(r => setTimeout(r, 500));
  }
  document.getElementById('done-badge').style.display = 'inline';
  document.getElementById('pulse').classList.add('stopped');
}

initCharts();
poll();
</script>
</body>
</html>
"""

# ---------------------------------------------------------------------------
# Cleanup / summary
# ---------------------------------------------------------------------------

_xapp_proc = None

def terminate_xapp():
    global _xapp_proc
    if _xapp_proc and _xapp_proc.poll() is None:
        try:
            pgid = os.getpgid(_xapp_proc.pid)
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(1.0)
            if _xapp_proc.poll() is None:
                os.killpg(pgid, signal.SIGKILL)
        except Exception:
            try:
                _xapp_proc.terminate()
            except Exception:
                pass


def print_summary():
    with G.lock:
        samples  = list(G.samples)
        simram   = list(G.simram)
        ns3_peak = G.ns3_peak

    if not samples:
        return

    ns3_rate_wall = growth_rate([(s[0], s[1]) for s in samples])
    ns3_rate_sim  = growth_rate([(s[0], s[1]) for s in simram]) if len(simram) >= 2 else 0.0
    total_s       = samples[-1][0]
    sim_s         = samples[-1][4]

    print(f"\n{'='*58}")
    print(f"  RAM DASHBOARD — FINAL SUMMARY")
    print(f"{'='*58}")
    print(f"  Wall time      : {total_s:.1f}s  ({len(samples)} ext. samples)")
    print(f"  Sim time       : {sim_s:.2f}s  ({len(simram)} int. samples)")
    print(f"  Wall/Sim ratio : {total_s/sim_s:.1f}x" if sim_s > 0 else "  Wall/Sim ratio : —")
    print(f"  NS-3 start     : {samples[0][1]:>8.1f} MB")
    print(f"  NS-3 end       : {samples[-1][1]:>8.1f} MB   peak: {ns3_peak:.1f} MB")
    print(f"  Growth (wall)  : {ns3_rate_wall:>+8.3f} MB/wall-s")
    print(f"  Growth (sim)   : {ns3_rate_sim:>+8.3f} MB/sim-s")
    print(f"  Ext. CSV       : {CSV_OUTPUT}")
    print(f"  Int. CSV       : {NS3_RAM_CSV}")
    print(f"{'='*58}")
    if abs(ns3_rate_wall) < 1.0:
        print("  VERDICT: RAM stable — leak fix confirmed.")
    elif ns3_rate_wall >= 5.0:
        print("  VERDICT: LEAK DETECTED — NS-3 RAM growing fast!")
    else:
        print("  VERDICT: Slow growth — normal trace accumulation.")
    print(f"{'='*58}\n")


def shutdown_handler(sig, frame):
    print("\n[RAM-DASH] Shutting down...")
    with G.lock:
        G.done = True
    terminate_xapp()
    print_summary()
    sys.exit(0)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    global _xapp_proc

    if not os.path.exists(XAPP_SCRIPT):
        print(f"ERROR: xapp_template.py not found at {XAPP_SCRIPT}")
        sys.exit(1)

    signal.signal(signal.SIGINT,  shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print(f"[RAM-DASH] Launching xapp_template.py ...")
    _xapp_proc = subprocess.Popen(
        [sys.executable, XAPP_SCRIPT],
        cwd=SCRIPT_DIR,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    print(f"[RAM-DASH] xApp PID = {_xapp_proc.pid}")

    start_time = time.monotonic()

    # External sampler
    sampler = threading.Thread(
        target=sampler_thread, args=(_xapp_proc, start_time), daemon=True
    )
    sampler.start()

    # Internal C++ CSV reader
    simreader = threading.Thread(target=simram_reader_thread, daemon=True)
    simreader.start()

    # Auto-shutdown watcher
    def watch_done():
        sampler.join()
        print("\n[RAM-DASH] NS-3 simulation finished.")
        print_summary()
        terminate_xapp()

    threading.Thread(target=watch_done, daemon=True).start()

    print(f"\n[RAM-DASH] Dashboard → http://localhost:{PORT}")
    print(f"[RAM-DASH] External CSV : {CSV_OUTPUT}")
    print(f"[RAM-DASH] Internal CSV : {NS3_RAM_CSV}")
    print(f"[RAM-DASH] Monitoring until NS-3 exits. Ctrl+C to stop.\n")

    import logging
    logging.getLogger("werkzeug").setLevel(logging.ERROR)

    app.run(host="0.0.0.0", port=PORT, threaded=True, use_reloader=False)


if __name__ == "__main__":
    main()
