#!/usr/bin/env python3
"""
Offline replay test for xapp_template.py metrics (RSRP, PER, SINR, TP, etc.)
Uses a completed sim output directory — no ns-3 needed.

Usage:
    python3 test_xapp_metrics.py [sim_dir]
    python3 test_xapp_metrics.py /path/to/sim_0000
"""

import sys
import os
import math
import importlib.util
from collections import deque
from pathlib import Path

# ── Load xapp_template as a module without running main() ────────────────────
XAPP_PATH = Path(__file__).parent / "xapp_template.py"
spec = importlib.util.spec_from_file_location("xapp", XAPP_PATH)
xapp = importlib.util.module_from_spec(spec)

# Patch BUILD_DIR before executing module so os.makedirs doesn't create a stray dir
import unittest.mock as mock
with mock.patch("os.makedirs"):
    spec.loader.exec_module(xapp)

# ── Sim directory ─────────────────────────────────────────────────────────────
SIM_DIR = sys.argv[1] if len(sys.argv) > 1 else \
    "/mnt/storage/celtic/app_files/oran_native/MLC_TEST/celtic/oran_native_github_prep/celtic_rl/data/sims/sim_0000"

print(f"[TEST] Replaying metrics from: {SIM_DIR}")

# ── Read all file data in one shot (completed sim) ────────────────────────────
# Override BUILD_DIR in the module so read_new_lines points at our sim dir
xapp.BUILD_DIR = SIM_DIR

# Reset all accumulators / history in the module
for cid in [1, 2]:
    xapp.sinr_history[cid].clear()
    xapp.tp_history[cid].clear()
    xapp.load_history[cid].clear()
    xapp.dist_history[cid].clear()
    xapp.latency_history[cid].clear()
    xapp.rsrp_history[cid].clear()
    xapp.per_history[cid].clear()
    xapp._per_accum[cid]  = {'tot': 0.0, 'err': 0.0, 'ts': -1}
    xapp._rsrp_accum[cid] = {'sum': 0.0, 'n': 0,   'ts': -1}
xapp.handover_history.clear()
for k in xapp.file_pointers['du']:    xapp.file_pointers['du'][k] = 0
for k in xapp.file_pointers['cu']:    xapp.file_pointers['cu'][k] = 0
xapp.file_pointers['rlc'] = 0
xapp.file_pointers['pos'] = 0
xapp.file_pointers['ho']  = 0

# Ingest everything
sim_time = xapp.get_current_sim_time(SIM_DIR) or 120.0
xapp.process_metrics(SIM_DIR, sim_time)

# Flush last partial PER epoch if any
for cid in [1, 2]:
    acc = xapp._per_accum[cid]
    if acc['ts'] > 0 and acc['tot'] > 0:
        xapp.per_history[cid].append({'t': acc['ts'] / 1000.0, 'v': acc['err'] / acc['tot']})

# Flush last partial RSRP epoch if any
for cid in [1, 2]:
    acc = xapp._rsrp_accum[cid]
    if acc['ts'] > 0 and acc['n'] > 0:
        xapp.rsrp_history[cid].append({'t': acc['ts'] / 1000.0, 'v': acc['sum'] / acc['n']})

# ── Summary stats ─────────────────────────────────────────────────────────────
LOOKBACK = sim_time  # use full sim as lookback to get overall averages

print(f"\n{'='*70}")
print(f"  REPLAY RESULTS  (sim_time={sim_time:.1f}s, {LOOKBACK:.1f}s lookback)")
print(f"{'='*70}")

for cid in [1, 2]:
    sinr  = xapp.get_avg_sinr(xapp.sinr_history[cid], sim_time, LOOKBACK)
    tp    = xapp.get_avg_metric(xapp.tp_history[cid],      sim_time, LOOKBACK)
    lat   = xapp.get_avg_metric(xapp.latency_history[cid], sim_time, LOOKBACK)
    load  = xapp.get_avg_metric(xapp.load_history[cid],    sim_time, LOOKBACK)
    dist  = xapp.get_avg_metric(xapp.dist_history[cid],    sim_time, LOOKBACK)
    rsrp  = xapp.get_avg_metric(xapp.rsrp_history[cid],    sim_time, LOOKBACK)
    per   = xapp.get_avg_metric(xapp.per_history[cid],     sim_time, LOOKBACK)

    print(f"\n  Cell {cid}:")
    print(f"    SINR     : {sinr:>7.2f} dB"   if sinr is not None else "    SINR     : N/A")
    print(f"    TP       : {tp:>7.0f} kbps"   if tp   is not None else "    TP       : N/A")
    print(f"    Latency  : {lat:>7.2f} ms"    if lat  is not None else "    Latency  : N/A")
    print(f"    Load     : {load:>7.1f} UEs"  if load is not None else "    Load     : N/A")
    print(f"    Distance : {dist:>7.1f} m"    if dist is not None else "    Distance : N/A")
    print(f"    RSRP     : {rsrp:>7.2f} dBm"  if rsrp is not None else "    RSRP     : N/A  <-- check position file")
    print(f"    PER      : {per*100:>7.2f} %"  if per  is not None else "    PER      : N/A  <-- check du-cell cols 32/37")

    print(f"\n    -- Sample history sizes --")
    print(f"    sinr_history  : {len(xapp.sinr_history[cid])} entries")
    print(f"    rsrp_history  : {len(xapp.rsrp_history[cid])} entries")
    print(f"    per_history   : {len(xapp.per_history[cid])} entries")
    print(f"    dist_history  : {len(xapp.dist_history[cid])} entries")

    # Show last 5 RSRP and PER values for spot-check
    print(f"\n    -- Last 5 RSRP epochs (cell {cid}) --")
    for entry in list(xapp.rsrp_history[cid])[-5:]:
        print(f"      t={entry['t']:6.1f}s  rsrp={entry['v']:6.2f} dBm")

    print(f"\n    -- Last 5 PER epochs (cell {cid}) --")
    for entry in list(xapp.per_history[cid])[-5:]:
        print(f"      t={entry['t']:6.1f}s  per={entry['v']*100:5.1f}%")

print(f"\n{'='*70}")
print("  If RSRP/PER show N/A, check column indices in du/position files.")
print(f"{'='*70}\n")
