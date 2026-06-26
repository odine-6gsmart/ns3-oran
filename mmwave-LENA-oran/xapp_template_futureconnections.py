#!/usr/bin/env python3
"""
xApp Orchestrator for FutureConnections 4-gNB Scenario
4 gNBs, irregular layout, 4600x4600m, 20 UEs, 3.5 GHz
"""
import os
import time
import math
import subprocess
import threading
from collections import deque

# --- Configuration ---
BUILD_DIR     = "xapp_futureconnections_outputs"
POLL_INTERVAL = 0.5    # wall-clock seconds between polls
LOOKBACK_WINDOW = 2.0  # seconds for rolling metric averages

N_CELLS = 4
CELL_IDS = [1, 2, 3, 4]

# RSRP / RSRQ / PER thresholds — calibrated against observed 4-gNB scenario ranges:
#   RSRP: -95 to -109 dBm  |  RSRQ: -8.7 to -18.8 dB  |  PER: 56-68 %
RSRP_LOW_DBM  = -103.0   # dBm — boost power (very weak coverage: targets Cells 2,3)
RSRP_HIGH_DBM =  -88.0   # dBm — reduce power (strong signal; headroom in this layout)
RSRQ_LOW_DB   =  -14.0   # dB  — downtilt (interference dominant: targets Cells 1,3)
RSRQ_HIGH_DB  =   -9.0   # dB  — uptilt   (clean channel: targets Cell 4)
PER_HIGH      =    0.64   # 64% — lower A3, encourage HO (bad link: targets Cells 1,3)
PER_LOW       =    0.58   # 58% — raise A3, stable link   (relatively ok: targets Cell 4)

os.makedirs(BUILD_DIR, exist_ok=True)

# NS-3 command — 4-gNB irregular layout scenario
NS3_CMD = [
    "../build/scratch/ns3.42-FutureConnections4gNBScenerio-optimized",
    "--simTime=20.0",
    "--N_Ues=20",
    "--useHybrid=true",
    "--enableTiltTwoRay=true",
    "--reducedPmValues=true",
    "--enableE2FileLogging=true",
    "--enableTraces=false",
    "--Mobility=true",
    "--exportUEPositions=true",
    "--enableRuntimeControl=true",
    "--ControlPollInterval=10",
    "--RngRun=155",
    "--TxPower1=38.0",
    "--TxPower2=38.0",
    "--TxPower3=38.0",
    "--TxPower4=38.0",
    "--Tilt=10.0",
]

def run_ns3():
    """Runs the NS-3 simulation in BUILD_DIR."""
    print(f"[NS-3] Starting 4-gNB simulation inside ./{BUILD_DIR}/ ...")
    log_path = os.path.join(BUILD_DIR, "ns3_stdout.log")
    with open(log_path, "w") as out:
        process = subprocess.Popen(NS3_CMD, cwd=BUILD_DIR, stdout=out, stderr=subprocess.STDOUT)
        process.wait()
    print(f"[NS-3] Simulation finished. Outputs in ./{BUILD_DIR}/")


# =========================================================================
# TOPOLOGY CONSTANTS — must match FutureConnections4gNBScenerio.cc exactly
# =========================================================================

# Irregular gNB positions (x, y) in metres — 2-D for distance calc
BS_POS = {
    1: (900.0,  3200.0),
    2: (3500.0, 3600.0),
    3: (1800.0,  800.0),
    4: (3800.0, 1600.0),
}

# 3-D positions (x, y, z) — z=5.0m for useHybrid=true
BS_POS_3D = {
    1: (900.0,  3200.0, 5.0),
    2: (3500.0, 3600.0, 5.0),
    3: (1800.0,  800.0, 5.0),
    4: (3800.0, 1600.0, 5.0),
}

# RSRP formula constants — exact match to CalculateRSRPRealistic() in C++ scenario
LOG_DIST_EXP = 3.8
LOG_DIST_REF = 43.3   # dB at d0=1m
HPBW         = 10.0   # half-power beamwidth in degrees (UniformPlanarArray)


# =========================================================================
# STATE & HISTORY TRACKING
# =========================================================================

file_pointers = {
    'du':  {cid: 0 for cid in CELL_IDS},
    'rlc': 0,
    'cu':  {cid: 0 for cid in CELL_IDS},
    'pos': 0,
    'ho':  0,
    'rx':  0,   # RxPacketTrace.txt — RSRQ source
}

sinr_history    = {cid: deque(maxlen=200) for cid in CELL_IDS}
tp_history      = {cid: deque(maxlen=200) for cid in CELL_IDS}
load_history    = {cid: deque(maxlen=200) for cid in CELL_IDS}
dist_history    = {cid: deque(maxlen=500) for cid in CELL_IDS}
latency_history = {cid: deque(maxlen=200) for cid in CELL_IDS}
rsrp_history    = {cid: deque(maxlen=200) for cid in CELL_IDS}
per_history     = {cid: deque(maxlen=200) for cid in CELL_IDS}
rsrq_history    = {cid: deque(maxlen=500) for cid in CELL_IDS}
handover_history = {}  # {ue_id: deque of handover events}

# Per-epoch accumulators — reset when timestamp changes
_per_accum  = {cid: {'tot': 0.0, 'err': 0.0, 'ts': -1} for cid in CELL_IDS}
_rsrp_accum = {cid: {'sum': 0.0, 'n':   0,   'ts': -1} for cid in CELL_IDS}
# RSRQ: 100 ms epochs (ep = int(round(t * 10)))
_rsrq_accum = {cid: {'sum': 0.0, 'n':   0,   'ep': -1} for cid in CELL_IDS}

# RxPacketTrace.txt header — resolved once to get column indices by name
_rx_col = None  # dict: col_name → index; None = not yet parsed

# Per-algorithm cooldown timers
# POWER/TILT/A3 → existing SINR+TP / load+dist / ping-pong managers
# RSRP/RSRQ/PER_A3 → new RSRP-power / RSRQ-tilt / PER-A3 managers
last_action_t = {
    'POWER': 0.0, 'TILT': 0.0, 'A3': 0.0,
    'RSRP': 0.0,  'RSRQ': 0.0, 'PER_A3': 0.0,
}


# =========================================================================
# UTILITIES & FILE READING
# =========================================================================

def read_new_lines(filepath, pointer_type, cid=None):
    """Reads only newly appended lines using byte-offset pointers."""
    if not os.path.exists(filepath):
        return []
    ptr = file_pointers[pointer_type][cid] if cid is not None else file_pointers[pointer_type]
    lines = []
    try:
        with open(filepath, 'r') as f:
            f.seek(ptr)
            lines = f.readlines()
            new_ptr = f.tell()
        if cid is not None:
            file_pointers[pointer_type][cid] = new_ptr
        else:
            file_pointers[pointer_type] = new_ptr
    except Exception as e:
        print(f"[ERR] read_new_lines {filepath}: {e}")
    return lines


def get_current_sim_time(build_dir):
    """Extracts latest simulation time from UEPosition.txt."""
    pos_file = os.path.join(build_dir, "UEPosition.txt")
    if not os.path.exists(pos_file):
        return None
    try:
        with open(pos_file, 'rb') as f:
            try:
                f.seek(-1024, os.SEEK_END)
            except OSError:
                f.seek(0)
            for line in reversed(f.readlines()):
                try:
                    parts = line.decode('utf-8').strip().split(',')
                    if parts[0].replace('.', '', 1).isdigit():
                        return float(parts[0])
                except (ValueError, IndexError):
                    continue
    except Exception:
        pass
    return None


def get_current_network_state(build_dir):
    """
    Reads ground-truth cell config from NetworkConfigurations.txt.
    Header: Time,Cell1_TxPower,Cell1_Tilt,Cell1_A3, ..., Cell4_TxPower,Cell4_Tilt,Cell4_A3
    """
    state = {cid: {'power': 38.0, 'tilt': 10.0, 'a3': 0.0} for cid in CELL_IDS}
    netconf_file = os.path.join(build_dir, "NetworkConfigurations.txt")
    if not os.path.exists(netconf_file):
        return state
    try:
        with open(netconf_file, 'rb') as f:
            try:
                f.seek(-1024, os.SEEK_END)
            except OSError:
                f.seek(0)
            lines = f.readlines()
            if lines:
                p = lines[-1].decode('utf-8').strip().split(',')
                # 13 columns: Time + 3 per cell × 4 cells
                if len(p) >= 13:
                    for i, cid in enumerate(CELL_IDS):
                        base = 1 + i * 3
                        state[cid]['power'] = float(p[base])
                        state[cid]['tilt']  = float(p[base + 1])
                        state[cid]['a3']    = float(p[base + 2])
    except Exception:
        pass
    return state


def get_avg_metric(history_deque, current_time, lookback_window):
    """Rolling average of a scalar metric within the lookback window."""
    vals = [x['v'] for x in history_deque
            if current_time - lookback_window <= x['t'] <= current_time]
    return sum(vals) / len(vals) if vals else None


def get_avg_sinr(history_deque, current_time, lookback_window):
    """Weighted SINR average using SINR bin counts (3GPP formula)."""
    counts_list = [x['counts'] for x in history_deque
                   if current_time - lookback_window <= x['t'] <= current_time]
    if not counts_list:
        return None
    total = {b: 0.0 for b in [34, 46, 58, 70, 82, 94, 127]}
    for c in counts_list:
        for b, v in c.items():
            if b in total:
                total[b] += v
    tot = sum(total.values())
    if tot == 0:
        return None
    return sum(((b / 2.0) - 23.0) * v for b, v in total.items()) / tot


def compute_rsrp(ue_x, ue_y, ue_z, cell_id, tx_power, tilt):
    """
    Exact replica of CalculateRSRPRealistic() from FutureConnections4gNBScenerio.cc.
    LogDistance pathloss (exp=3.8, ref=43.3 dB at 1m) + parabolic antenna gain.
    """
    bx, by, bz = BS_POS_3D[cell_id]
    dx, dy, dz = ue_x - bx, ue_y - by, ue_z - bz
    dist = max(math.sqrt(dx**2 + dy**2 + dz**2), 1.0)
    pathloss  = LOG_DIST_REF + 10.0 * LOG_DIST_EXP * math.log10(dist)
    theta_deg = math.acos(max(min(dz / dist, 1.0), -1.0)) * 180.0 / math.pi
    boresight = 90.0 + tilt
    gain = max(18.0 - 12.0 * ((theta_deg - boresight) / HPBW)**2, -30.0)
    return tx_power + gain - pathloss


def write_commands(build_dir, commands):
    """Atomically writes control commands to runtime_control.txt."""
    if not commands:
        return
    control_file = os.path.join(build_dir, "runtime_control.txt")
    temp_file    = control_file + ".tmp"
    try:
        with open(temp_file, 'w') as f:
            for cmd in commands:
                f.write(f"{cmd['type']} {cmd['cell']} {cmd['value']:.2f}\n")
        os.rename(temp_file, control_file)
    except Exception as e:
        print(f"[ERR] write_commands: {e}")


def _parse_rx_header(build_dir):
    """
    Reads the header line of RxPacketTrace.txt once and builds a name→index map.
    Advances file_pointers['rx'] past the header so read_new_lines never re-reads it.
    Resolves the SINR column by searching for 'SINR' in the column name.
    """
    global _rx_col
    rx_file = os.path.join(build_dir, "RxPacketTrace.txt")
    if not os.path.exists(rx_file):
        return
    try:
        with open(rx_file, 'r') as f:
            header_line = f.readline()
            header_end  = f.tell()
        cols    = [c.strip() for c in header_line.split('\t')]
        col_map = {c: i for i, c in enumerate(cols)}
        sinr_key = next((k for k in col_map if 'SINR' in k), None)
        _rx_col  = {
            'dir':  0,
            'time': col_map.get('time'),
            'cell': col_map.get('cellId'),
            'sinr': col_map.get(sinr_key) if sinr_key else None,
        }
        file_pointers['rx'] = header_end
    except Exception as e:
        print(f"[ERR] _parse_rx_header: {e}")


# =========================================================================
# METRICS PROCESSING
# =========================================================================

def process_metrics(build_dir, sim_time):
    """Reads all new NS-3 output lines and updates metric histories."""

    # 1. DU Metrics (SINR bins + PER) — one file per cell
    for cid in CELL_IDS:
        du_file = os.path.join(build_dir, f"du-cell-{cid}.txt")
        for line in read_new_lines(du_file, 'du', cid):
            p = line.strip().split(',')
            if len(p) < 38:
                continue
            try:
                counts = {
                    34: float(p[23]), 46: float(p[24]), 58: float(p[25]),
                    70: float(p[26]), 82: float(p[27]), 94: float(p[28]),
                    127: float(p[29])
                }
                if sum(counts.values()) > 0:
                    sinr_history[cid].append({'t': sim_time, 'counts': counts})

                # PER: col 32 = TB.TotNbrDl, col 37 = TB.ErrTotalNbrDl
                ts_ms  = int(float(p[0]))
                tb_tot = float(p[32])
                tb_err = float(p[37])
                acc = _per_accum[cid]
                if ts_ms != acc['ts']:
                    if acc['ts'] > 0 and acc['tot'] > 0:
                        per_history[cid].append({'t': sim_time,
                                                 'v': acc['err'] / acc['tot']})
                    acc['tot'] = tb_tot
                    acc['err'] = tb_err
                    acc['ts']  = ts_ms
                else:
                    acc['tot'] += tb_tot
                    acc['err'] += tb_err
            except (ValueError, IndexError):
                continue

    # 2. RLC Metrics (Throughput, Latency) — single shared file, CellId in col 2
    rlc_file = os.path.join(build_dir, "DlE2RlcStats.txt")
    for line in read_new_lines(rlc_file, 'rlc'):
        p = line.strip().split()
        if len(p) < 11:
            continue
        try:
            t1, t2, cid = float(p[0]), float(p[1]), int(p[2])
            if cid not in CELL_IDS:
                continue
            rb    = float(p[9])   # RxBytes
            delay = float(p[10])  # delay (seconds)
            if (t2 - t1) > 0:
                tp_history[cid].append({'t': t2, 'v': (rb * 8) / ((t2 - t1) * 1000)})  # kbps
                latency_history[cid].append({'t': t2, 'v': delay * 1000.0})              # ms
        except (ValueError, IndexError):
            continue

    # 3. CU-CP Metrics (Active UE load) — one file per cell
    for cid in CELL_IDS:
        cu_file = os.path.join(build_dir, f"cu-cp-cell-{cid}.txt")
        for line in read_new_lines(cu_file, 'cu', cid):
            p = line.strip().split(',')
            if len(p) >= 3:
                try:
                    load_history[cid].append({'t': sim_time, 'v': int(p[2])})
                except ValueError:
                    continue

    # 4. UE Position → Distance + RSRP per cell
    pos_file   = os.path.join(build_dir, "UEPosition.txt")
    state_snap = get_current_network_state(build_dir)
    for line in read_new_lines(pos_file, 'pos'):
        p = line.strip().split(',')
        if len(p) < 7 or p[1] != 'UE':
            continue
        try:
            t   = float(p[0])
            ux  = float(p[3])
            uy  = float(p[4])
            uz  = float(p[5])
            cid = int(p[6])
            if cid not in CELL_IDS:
                continue

            bx, by = BS_POS[cid]
            dist_history[cid].append({'t': t, 'v': math.sqrt((ux - bx)**2 + (uy - by)**2)})

            # RSRP: accumulate per epoch, flush when timestamp changes
            tx_power = state_snap[cid]['power']
            tilt     = state_snap[cid]['tilt']
            rsrp_val = compute_rsrp(ux, uy, uz, cid, tx_power, tilt)
            acc   = _rsrp_accum[cid]
            ts_ms = int(round(t * 1000))
            if ts_ms != acc['ts']:
                if acc['ts'] > 0 and acc['n'] > 0:
                    rsrp_history[cid].append({'t': acc['ts'] / 1000.0,
                                              'v': acc['sum'] / acc['n']})
                acc['sum'] = rsrp_val
                acc['n']   = 1
                acc['ts']  = ts_ms
            else:
                acc['sum'] += rsrp_val
                acc['n']   += 1
        except (ValueError, KeyError):
            continue

    # 5. Handover Log — ping-pong detection
    ho_file = os.path.join(build_dir, "HandoverLog.txt")
    for line in read_new_lines(ho_file, 'ho'):
        p = line.strip().split(',')
        if len(p) < 5:
            continue
        try:
            ho = {
                'time':   float(p[0]),
                'ue_id':  int(p[2]),
                'source': int(p[3]),
                'target': int(p[4]),
                'is_pp':  False,
            }
            uid = ho['ue_id']
            if uid not in handover_history:
                handover_history[uid] = deque(maxlen=100)
            hist = list(handover_history[uid])
            if hist:
                last = hist[-1]
                if (ho['time'] - last['time'] <= 2.0
                        and last['source'] == ho['target']
                        and last['target'] == ho['source']):
                    ho['is_pp'] = True
            handover_history[uid].append(ho)
        except (ValueError, IndexError):
            continue

    # 6. RSRQ from RxPacketTrace.txt — DL rows only, 100 ms epochs
    #    Formula: RSRQ_dB = 10·log10(SINR_lin / (SINR_lin + 1))
    #    Same computation as extract_cell_level_metrics_future_connections.py
    if _rx_col is None:
        _parse_rx_header(build_dir)
    if _rx_col is not None:
        time_idx = _rx_col['time']
        cell_idx = _rx_col['cell']
        sinr_idx = _rx_col['sinr']
        if time_idx is not None and cell_idx is not None and sinr_idx is not None:
            rx_file = os.path.join(build_dir, "RxPacketTrace.txt")
            for line in read_new_lines(rx_file, 'rx'):
                parts = line.strip().split('\t')
                if len(parts) <= max(time_idx, cell_idx, sinr_idx):
                    continue
                if parts[_rx_col['dir']].strip() != 'DL':
                    continue
                try:
                    t       = float(parts[time_idx])
                    cid     = int(parts[cell_idx])
                    sinr_dB = float(parts[sinr_idx])
                    if cid not in CELL_IDS:
                        continue
                    sinr_lin = 10.0 ** (max(min(sinr_dB, 80.0), -80.0) / 10.0)
                    rsrq_dB  = 10.0 * math.log10(max(sinr_lin / (sinr_lin + 1.0), 1e-15))
                    ep  = int(round(t * 10))  # 100 ms epoch index
                    acc = _rsrq_accum[cid]
                    if ep != acc['ep']:
                        if acc['ep'] >= 0 and acc['n'] > 0:
                            rsrq_history[cid].append({'t': acc['ep'] / 10.0,
                                                      'v': acc['sum'] / acc['n']})
                        acc['sum'] = rsrq_dB
                        acc['n']   = 1
                        acc['ep']  = ep
                    else:
                        acc['sum'] += rsrq_dB
                        acc['n']   += 1
                except (ValueError, IndexError):
                    continue


# =========================================================================
# XAPP ALGORITHMS — adapted for 4-cell irregular layout
# =========================================================================

def run_power_manager(sim_time, state):
    """Per-cell TX power control based on SINR + throughput thresholds."""
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['POWER'] + 1.0:
        return []

    commands = []
    for cid in CELL_IDS:
        avg_sinr = get_avg_sinr(sinr_history[cid], sim_time, LOOKBACK_WINDOW)
        avg_tp   = get_avg_metric(tp_history[cid],  sim_time, LOOKBACK_WINDOW)
        if avg_sinr is None or avg_tp is None:
            continue

        cur_p = state[cid]['power']
        # SINR / TP thresholds per power level
        s_thresh = {30: 3.0, 34: 5.0, 38: 6.5, 42: 9.0,  46: 11.0}.get(cur_p, 6.5)
        t_thresh = {30: 800, 34: 1000, 38: 1200, 42: 1600, 46: 2000}.get(cur_p, 1200)
        s_exc    = {30: 6.0, 34: 8.0, 38: 10.0, 42: 12.0, 46: 14.0}.get(cur_p, 10.0)
        t_exc    = {30: 1800, 34: 2200, 38: 2800, 42: 3200, 46: 3800}.get(cur_p, 2800)

        new_p  = cur_p
        reason = ""
        if avg_sinr < s_thresh or avg_tp < t_thresh:
            new_p  = min(46.0, cur_p + 4.0)
            reason = "POOR_LINK (increase power)"
        elif cur_p > 38.0 and avg_sinr > s_exc and avg_tp > t_exc:
            new_p  = max(30.0, cur_p - 4.0)
            reason = "EXCELLENT (reduce power)"

        if new_p != cur_p:
            commands.append({'type': 'POWER', 'cell': cid, 'value': new_p})
            print(f"[{sim_time:.1f}s][POWER] Cell {cid}: {cur_p:.0f}->{new_p:.0f} dBm | "
                  f"{reason} | SINR={avg_sinr:.1f} dB TP={avg_tp:.0f} kbps")

    if commands:
        last_action_t['POWER'] = sim_time
    return commands


def run_tilt_manager(sim_time, state):
    """
    Load-balance across all 4 cells by adjusting tilts.
    Finds the most-loaded and least-loaded cell — if imbalance > threshold,
    downtilt the heavy cell and uptilt the light cell.
    Falls back to distance-based tilt tuning per cell when balanced.
    """
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['TILT'] + 3.0:
        return []

    commands  = []
    triggered = False

    loads = {cid: get_avg_metric(load_history[cid], sim_time, LOOKBACK_WINDOW)
             for cid in CELL_IDS}
    dists = {cid: get_avg_metric(dist_history[cid], sim_time, LOOKBACK_WINDOW)
             for cid in CELL_IDS}

    valid_loads = {cid: v for cid, v in loads.items() if v is not None}

    if len(valid_loads) >= 2:
        heavy_cid = max(valid_loads, key=valid_loads.get)
        light_cid = min(valid_loads, key=valid_loads.get)
        load_diff = valid_loads[heavy_cid] - valid_loads[light_cid]

        if load_diff > 5:  # at least 5-UE imbalance across cells
            cur_h = state[heavy_cid]['tilt']
            cur_l = state[light_cid]['tilt']
            new_h = min(15.0, cur_h + 2.5)  # downtilt heavy → shrink coverage
            new_l = max(5.0,  cur_l - 2.5)  # uptilt  light → expand coverage

            if new_h != cur_h:
                commands.append({'type': 'TILT', 'cell': heavy_cid, 'value': new_h})
                print(f"[{sim_time:.1f}s][TILT] Cell {heavy_cid}: {cur_h}->{new_h} | "
                      f"LOAD_HEAVY (load={valid_loads[heavy_cid]:.0f} UEs)")
                triggered = True

            if new_l != cur_l:
                commands.append({'type': 'TILT', 'cell': light_cid, 'value': new_l})
                print(f"[{sim_time:.1f}s][TILT] Cell {light_cid}: {cur_l}->{new_l} | "
                      f"LOAD_LIGHT (load={valid_loads[light_cid]:.0f} UEs)")
                triggered = True

    # Distance-based fallback (one cell per epoch, not during load balancing)
    # Thresholds scaled for 4600×4600m area (~800–1500m average distances)
    if not triggered:
        for cid in CELL_IDS:
            d_avg = dists[cid]
            if d_avg is None:
                continue
            cur_tilt = state[cid]['tilt']
            new_tilt = cur_tilt
            reason   = ""
            if d_avg < 700.0 and cur_tilt < 15.0:
                new_tilt = min(15.0, cur_tilt + 2.5)
                reason   = "NEAR_PROXIMITY"
            elif d_avg > 1400.0 and cur_tilt > 5.0:
                new_tilt = max(5.0,  cur_tilt - 2.5)
                reason   = "FAR_COVERAGE"
            if new_tilt != cur_tilt:
                commands.append({'type': 'TILT', 'cell': cid, 'value': new_tilt})
                print(f"[{sim_time:.1f}s][TILT] Cell {cid}: {cur_tilt}->{new_tilt} | "
                      f"{reason} | avg_dist={d_avg:.0f}m")
                triggered = True
                break  # one cell per epoch in fallback mode

    if triggered:
        last_action_t['TILT'] = sim_time
    return commands


def run_handover_manager(sim_time, state):
    """
    Per-cell A3 offset control based on ping-pong handover rate.
    High PP rate → raise A3 (make HO harder).
    Low PP rate  → lower A3 (allow faster HO).
    Constraint: at most N_CELLS-1 cells can simultaneously be at -3.0.
    """
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['A3'] + 2.5:
        return []

    commands  = []
    decisions = {}

    for cid in CELL_IDS:
        tot_ho, pp_ho = 0, 0
        for uid, hist in handover_history.items():
            for ho in hist:
                if sim_time - LOOKBACK_WINDOW < ho['time'] <= sim_time and ho['source'] == cid:
                    tot_ho += 1
                    if ho['is_pp']:
                        pp_ho += 1
        if tot_ho == 0:
            continue

        rate   = (pp_ho / tot_ho) * 100.0
        new_a3 = 0.0
        reason = "MODERATE_PP"
        if rate > 70.0:
            new_a3 = 3.0
            reason = "HIGH_PP"
        elif rate < 50.0:
            new_a3 = -3.0
            reason = "LOW_PP"
        decisions[cid] = {'a3': new_a3, 'rate': rate, 'reason': reason}

    # Constraint: don't set ALL cells to -3.0 simultaneously
    neg3_cells = [c for c, d in decisions.items() if d['a3'] == -3.0]
    if len(neg3_cells) == N_CELLS:
        # Exempt the one with lowest PP rate (it needs -3.0 the least)
        weakest = min(neg3_cells, key=lambda c: decisions[c]['rate'])
        decisions[weakest]['a3']    = 0.0
        decisions[weakest]['reason'] = "CONSTRAINED_NOT_ALL_NEG3"

    for cid, d in decisions.items():
        cur_a3 = state[cid]['a3']
        if d['a3'] != cur_a3:
            commands.append({'type': 'A3', 'cell': cid, 'value': d['a3']})
            print(f"[{sim_time:.1f}s][A3] Cell {cid}: {cur_a3:.1f}->{d['a3']:.1f} dB | "
                  f"{d['reason']} | PP={d['rate']:.1f}%")

    if commands:
        last_action_t['A3'] = sim_time
    return commands


def run_rsrp_power_control(sim_time, state):
    """
    RSRP → TxPower.  Coverage-quality complement to run_power_manager.
    Targets cells with genuinely weak received signal (independent of SINR bin counts).
    Cooldown 2s — offset from run_power_manager (1s) to avoid same-tick collisions.
    Calibrated: RSRP_LOW=-103 dBm targets Cells 2,3; RSRP_HIGH=-88 dBm rarely fires.
    """
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['RSRP'] + 2.0:
        return []
    commands = []
    for cid in CELL_IDS:
        avg_rsrp = get_avg_metric(rsrp_history[cid], sim_time, LOOKBACK_WINDOW)
        if avg_rsrp is None:
            continue
        cur_p = state[cid]['power']
        new_p = cur_p
        if avg_rsrp < RSRP_LOW_DBM:
            new_p = min(46.0, cur_p + 4.0)
            print(f"[{sim_time:.1f}s][RSRP→PWR] Cell {cid}: {cur_p:.0f}->{new_p:.0f} dBm | "
                  f"RSRP={avg_rsrp:.1f} dBm (weak coverage)")
        elif avg_rsrp > RSRP_HIGH_DBM and cur_p > 38.0:
            new_p = max(30.0, cur_p - 4.0)
            print(f"[{sim_time:.1f}s][RSRP→PWR] Cell {cid}: {cur_p:.0f}->{new_p:.0f} dBm | "
                  f"RSRP={avg_rsrp:.1f} dBm (strong, save power)")
        if new_p != cur_p:
            commands.append({'type': 'POWER', 'cell': cid, 'value': new_p})
    if commands:
        last_action_t['RSRP'] = sim_time
    return commands


def run_rsrq_tilt_control(sim_time, state):
    """
    RSRQ → Tilt.  Interference-aware tilt complement to run_tilt_manager.
    RSRQ reflects channel quality ratio (signal vs interference+noise).
    Cooldown 4s — offset from run_tilt_manager (3s) to avoid same-tick collisions.
    Calibrated: RSRQ_LOW=-14 dB targets Cells 1,3; RSRQ_HIGH=-9 dB targets Cell 4.
    """
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['RSRQ'] + 4.0:
        return []
    commands = []
    for cid in CELL_IDS:
        avg_rsrq = get_avg_metric(rsrq_history[cid], sim_time, LOOKBACK_WINDOW)
        if avg_rsrq is None:
            continue
        cur_t = state[cid]['tilt']
        new_t = cur_t
        if avg_rsrq < RSRQ_LOW_DB:
            new_t = min(15.0, cur_t + 2.5)
            print(f"[{sim_time:.1f}s][RSRQ→TILT] Cell {cid}: {cur_t:.1f}->{new_t:.1f} deg | "
                  f"RSRQ={avg_rsrq:.1f} dB (interference, downtilt)")
        elif avg_rsrq > RSRQ_HIGH_DB and cur_t > 5.0:
            new_t = max(5.0, cur_t - 2.5)
            print(f"[{sim_time:.1f}s][RSRQ→TILT] Cell {cid}: {cur_t:.1f}->{new_t:.1f} deg | "
                  f"RSRQ={avg_rsrq:.1f} dB (clean channel, uptilt)")
        if new_t != cur_t:
            commands.append({'type': 'TILT', 'cell': cid, 'value': new_t})
    if commands:
        last_action_t['RSRQ'] = sim_time
    return commands


def run_per_a3_control(sim_time, state):
    """
    PER → A3 Handover Offset.  Link-error complement to run_handover_manager.
    High PER → lower A3 (easier HO, get UE to a better cell).
    Low  PER → raise A3 (link is stable, avoid unnecessary churn).
    Cooldown 3s — offset from run_handover_manager (2.5s) to avoid same-tick collisions.
    Calibrated: PER_HIGH=64% targets Cells 1,3; PER_LOW=58% targets Cell 4.
    """
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['PER_A3'] + 3.0:
        return []
    commands = []
    for cid in CELL_IDS:
        avg_per = get_avg_metric(per_history[cid], sim_time, LOOKBACK_WINDOW)
        if avg_per is None:
            continue
        cur_a3 = state[cid]['a3']
        new_a3 = cur_a3
        if avg_per > PER_HIGH:
            new_a3 = -3.0
            print(f"[{sim_time:.1f}s][PER→A3] Cell {cid}: {cur_a3:.1f}->{new_a3:.1f} dB | "
                  f"PER={avg_per*100:.1f}% (bad link, easier HO)")
        elif avg_per < PER_LOW:
            new_a3 = 3.0
            print(f"[{sim_time:.1f}s][PER→A3] Cell {cid}: {cur_a3:.1f}->{new_a3:.1f} dB | "
                  f"PER={avg_per*100:.1f}% (stable link, harder HO)")
        if new_a3 != cur_a3:
            commands.append({'type': 'A3', 'cell': cid, 'value': new_a3})
    if commands:
        last_action_t['PER_A3'] = sim_time
    return commands


# =========================================================================
# MAIN LOOP
# =========================================================================

def _pp_stats(cid, sim_time):
    """Returns (total_HO, pp_HO) for a cell within the lookback window."""
    tot, pp = 0, 0
    for uid, hist in handover_history.items():
        for ho in hist:
            if sim_time - LOOKBACK_WINDOW < ho['time'] <= sim_time and ho['source'] == cid:
                tot += 1
                if ho['is_pp']:
                    pp += 1
    return tot, pp


def xapp_loop():
    print("=" * 70)
    print("   FutureConnections 4-gNB xApp Orchestrator")
    print(f"   Cells: {N_CELLS}  |  Layout: 4600x4600m irregular")
    print(f"   Output dir: {BUILD_DIR}")
    print("=" * 70)

    last_sim_time   = 0.0
    last_print_time = 0.0
    wall_start      = time.time()

    try:
        while True:
            sim_time = get_current_sim_time(BUILD_DIR)

            if sim_time is not None and sim_time > last_sim_time:
                last_sim_time = sim_time

                # 1. READ ALL NEW METRICS
                process_metrics(BUILD_DIR, sim_time)

                # 2. PRINT STATUS EVERY 1s
                if sim_time >= last_print_time + 1.0:
                    elapsed   = time.time() - wall_start
                    remaining = max(0.0, 120.0 - sim_time)
                    print(f"\n--- sim={sim_time:.1f}s | wall={elapsed:.0f}s | ~{remaining:.0f}s left ---")
                    for cid in CELL_IDS:
                        sinr  = get_avg_sinr(sinr_history[cid],    sim_time, LOOKBACK_WINDOW) or 0.0
                        tp    = get_avg_metric(tp_history[cid],     sim_time, LOOKBACK_WINDOW) or 0.0
                        load  = get_avg_metric(load_history[cid],   sim_time, LOOKBACK_WINDOW) or 0.0
                        dist  = get_avg_metric(dist_history[cid],   sim_time, LOOKBACK_WINDOW) or 0.0
                        lat   = get_avg_metric(latency_history[cid],sim_time, LOOKBACK_WINDOW) or 0.0
                        rsrp  = get_avg_metric(rsrp_history[cid],   sim_time, LOOKBACK_WINDOW)
                        per   = get_avg_metric(per_history[cid],    sim_time, LOOKBACK_WINDOW)
                        rsrq  = get_avg_metric(rsrq_history[cid],   sim_time, LOOKBACK_WINDOW)
                        tot_ho, pp_ho = _pp_stats(cid, sim_time)
                        pp_rate  = (pp_ho / tot_ho * 100.0) if tot_ho > 0 else 0.0
                        rsrp_str = f"{rsrp:>6.1f}" if rsrp is not None else "   N/A"
                        per_str  = f"{per * 100.0:>4.1f}%" if per is not None else " N/A%"
                        rsrq_str = f"{rsrq:>5.1f}" if rsrq is not None else "  N/A"
                        print(f" Cell{cid} | SINR:{sinr:>5.1f}dB | TP:{tp:>5.0f}kbps | "
                              f"Lat:{lat:>4.1f}ms | Load:{load:>2.0f}UE | "
                              f"Dist:{dist:>6.0f}m | RSRP:{rsrp_str}dBm | "
                              f"RSRQ:{rsrq_str}dB | PER:{per_str} | PP:{pp_rate:>4.1f}%")
                    print("-" * 122)
                    last_print_time = math.floor(sim_time)

                # 3. GET GROUND TRUTH STATE
                state = get_current_network_state(BUILD_DIR)

                # 4. RUN ALGORITHMS
                all_commands = []
                # --- existing managers (SINR+TP / load+dist / ping-pong) ---
                all_commands.extend(run_power_manager(sim_time, state))
                all_commands.extend(run_tilt_manager(sim_time, state))
                all_commands.extend(run_handover_manager(sim_time, state))
                # --- new managers (RSRP / RSRQ / PER) ---
                all_commands.extend(run_rsrp_power_control(sim_time, state))
                all_commands.extend(run_rsrq_tilt_control(sim_time, state))
                all_commands.extend(run_per_a3_control(sim_time, state))

                # 5. SEND COMMANDS BACK TO NS-3
                write_commands(BUILD_DIR, all_commands)

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("\n[xApp] Stopped by user.")


def main():
    ns3_thread = threading.Thread(target=run_ns3, daemon=True)
    ns3_thread.start()

    # Let NS-3 initialise and create output files before polling
    time.sleep(3)
    xapp_loop()

    ns3_thread.join()


if __name__ == "__main__":
    main()
