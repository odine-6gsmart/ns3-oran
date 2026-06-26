#!/usr/bin/env python3
import os
import time
import math
import subprocess
import threading
from collections import deque

# --- Configuration ---
BUILD_DIR = "xapp_template_outputs"  # Path to the NS-3 build directory where text files are logged
POLL_INTERVAL = 0.5    # How often to check for new data (in seconds)
LOOKBACK_WINDOW = 2.0  # Lookback window for averaging metrics (seconds)

os.makedirs(BUILD_DIR, exist_ok=True)

# NS-3 command
NS3_CMD = [
    "../build/scratch/ns3.42-Differing_Power_Scenerio_HO-optimized",
    "--simTime=10.0",
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
    "--Tilt=10.0"
]

def run_ns3():
    """Runs the NS-3 simulation in the background inside BUILD_DIR"""
    print(f"[NS-3] Starting simulation inside ./{BUILD_DIR}/ ...")
    log_file_path = os.path.join(BUILD_DIR, "ns3_stdout.log")
    with open(log_file_path, "w") as out:
        process = subprocess.Popen(NS3_CMD, cwd=BUILD_DIR, stdout=out, stderr=subprocess.STDOUT)
        process.wait()
        print(f"\n[NS-3] Simulation finished! Outputs saved in ./{BUILD_DIR}/")

# --- Constants ---
BS_POS    = {1: [750.0, 1000.0],        2: [1250.0, 1000.0]}
BS_POS_3D = {1: [750.0, 1000.0, 5.0],  2: [1250.0, 1000.0, 5.0]}  # z=5m (useHybrid=true)

# RSRP formula constants — exact match to CalculateRSRPRealistic() in scenario script
# Propagation: LogDistance (Exponent=3.8, ReferenceLoss=43.3 dB at 1m), useHybrid+TwoRay mode
LOG_DIST_EXP = 3.8
LOG_DIST_REF = 43.3   # dB at d0=1m
HPBW         = 10.0   # half-power beamwidth in degrees (UniformPlanarArray approximation)

# --- State & History Tracking ---
# Dictionaries to remember file reading positions to avoid re-reading
file_pointers = {
    'du': {1: 0, 2: 0},
    'rlc': 0,
    'cu': {1: 0, 2: 0},
    'pos': 0,
    'ho': 0
}

# Time-series deques for metrics
sinr_history    = {1: deque(maxlen=200), 2: deque(maxlen=200)}
tp_history      = {1: deque(maxlen=200), 2: deque(maxlen=200)}
load_history    = {1: deque(maxlen=200), 2: deque(maxlen=200)}
dist_history    = {1: deque(maxlen=500), 2: deque(maxlen=500)}
latency_history = {1: deque(maxlen=200), 2: deque(maxlen=200)}
rsrp_history    = {1: deque(maxlen=200), 2: deque(maxlen=200)}  # cell-avg RSRP (dBm)
per_history     = {1: deque(maxlen=200), 2: deque(maxlen=200)}  # MAC BLER [0,1]
handover_history = {}  # {ue_id: deque of handovers}

# Per-epoch accumulators for PER and RSRP (reset on each new 100ms epoch)
_per_accum  = {1: {'tot': 0.0, 'err': 0.0, 'ts': -1},
               2: {'tot': 0.0, 'err': 0.0, 'ts': -1}}
_rsrp_accum = {1: {'sum': 0.0, 'n': 0, 'ts': -1},
               2: {'sum': 0.0, 'n': 0, 'ts': -1}}

# Timers to prevent rapid back-to-back actions
last_action_t = {'POWER': 0.0, 'TILT': 0.0, 'A3': 0.0}


# =========================================================================
# UTILITIES & FILE READING
# =========================================================================

def read_new_lines(filepath, pointer_type, cid=None):
    """Reads only newly appended lines from a file using pointers."""
    if not os.path.exists(filepath):
        return []
        
    # Get current pointer
    if cid is not None:
        ptr = file_pointers[pointer_type][cid]
    else:
        ptr = file_pointers[pointer_type]
        
    lines = []
    try:
        with open(filepath, 'r') as f:
            f.seek(ptr)
            lines = f.readlines()
            
            # Update pointer
            if cid is not None:
                file_pointers[pointer_type][cid] = f.tell()
            else:
                file_pointers[pointer_type] = f.tell()
    except Exception as e:
        print(f"Error reading {filepath}: {e}")
        
    return lines

def get_current_sim_time(build_dir):
    """Extracts the current simulation time from UEPosition.txt"""
    pos_file = os.path.join(build_dir, "UEPosition.txt")
    if not os.path.exists(pos_file):
        return None
        
    try:
        with open(pos_file, 'rb') as f:
            try: f.seek(-1024, os.SEEK_END)
            except OSError: f.seek(0)
            
            last_lines = f.readlines()
            if not last_lines: return None
                
            for line in reversed(last_lines):
                try:
                    parts = line.decode('utf-8').strip().split(',')
                    if len(parts) > 0 and parts[0].replace('.','',1).isdigit():
                        return float(parts[0])
                except (ValueError, IndexError): continue
    except Exception: pass
    return None

def get_current_network_state(build_dir):
    """Reads the ACTUAL current ground truth configuration of the network."""
    state = {
        1: {'power': 38.0, 'tilt': 10.0, 'a3': 0.0},
        2: {'power': 38.0, 'tilt': 10.0, 'a3': 0.0}
    }
    netconf_file = os.path.join(build_dir, "NetworkConfigurations.txt")
    if os.path.exists(netconf_file):
        try:
            with open(netconf_file, 'rb') as f:
                try: f.seek(-500, os.SEEK_END)
                except OSError: f.seek(0)
                lines = f.readlines()
                if lines:
                    p = lines[-1].decode('utf-8').strip().split(',')
                    if len(p) >= 7:
                        state[1]['power'], state[1]['tilt'], state[1]['a3'] = float(p[1]), float(p[2]), float(p[3])
                        state[2]['power'], state[2]['tilt'], state[2]['a3'] = float(p[4]), float(p[5]), float(p[6])
        except Exception: pass
    return state

def get_avg_metric(history_deque, current_time, lookback_window):
    """Calculates the average of a metric over the lookback window."""
    valid_vals = [x['v'] for x in history_deque if current_time - lookback_window <= x['t'] <= current_time]
    return sum(valid_vals) / len(valid_vals) if valid_vals else None

def get_avg_sinr(history_deque, current_time, lookback_window):
    """Calculates true weighted average SINR over the lookback window using 3GPP formula."""
    valid_counts = [x['counts'] for x in history_deque if current_time - lookback_window <= x['t'] <= current_time]
    if not valid_counts: return None
    
    total_counts = {b: 0.0 for b in [34, 46, 58, 70, 82, 94, 127]}
    for c in valid_counts:
        for b, v in c.items():
            if b in total_counts:
                total_counts[b] += v
            
    tot = sum(total_counts.values())
    if tot == 0: return None
    
    avg_sinr_db = sum(((b / 2.0) - 23.0) * val for b, val in total_counts.items()) / tot
    return avg_sinr_db

def compute_rsrp(ue_x, ue_y, ue_z, cell_id, tx_power, tilt):
    """
    Exact replica of CalculateRSRPRealistic() from Differing_Power_Scenerio_HO.cc.
    LogDistance pathloss (exp=3.8, ref=43.3 dB at 1m) + parabolic antenna gain.
    Returns RSRP in dBm.
    """
    bx, by, bz = BS_POS_3D[cell_id]
    dx, dy, dz = ue_x - bx, ue_y - by, ue_z - bz
    dist = max(math.sqrt(dx**2 + dy**2 + dz**2), 1.0)
    pathloss = LOG_DIST_REF + 10.0 * LOG_DIST_EXP * math.log10(dist)
    theta_deg = math.acos(max(min(dz / dist, 1.0), -1.0)) * 180.0 / math.pi
    boresight = 90.0 + tilt
    gain = max(18.0 - 12.0 * ((theta_deg - boresight) / HPBW)**2, -30.0)
    return tx_power + gain - pathloss


def write_commands(build_dir, commands):
    """Atomically writes commands to runtime_control.txt"""
    if not commands: return
    control_file = os.path.join(build_dir, "runtime_control.txt")
    temp_file = f"{control_file}.tmp"
    try:
        with open(temp_file, 'w') as f:
            for cmd in commands:
                f.write(f"{cmd['type']} {cmd['cell']} {cmd['value']:.2f}\n")
        os.rename(temp_file, control_file)
    except Exception as e:
        print(f"Error writing commands: {e}")


# =========================================================================
# METRICS PROCESSING
# =========================================================================

def process_metrics(build_dir, sim_time):
    """Reads all new lines from NS-3 outputs and updates metric histories."""
    
    # 1. DU Metrics (SINR + PER)
    for cid in [1, 2]:
        du_file = os.path.join(build_dir, f"du-cell-{cid}.txt")
        for line in read_new_lines(du_file, 'du', cid):
            p = line.strip().split(',')
            if len(p) < 38: continue
            try:
                ts = sim_time
                counts = {
                    34: float(p[23]), 46: float(p[24]), 58: float(p[25]),
                    70: float(p[26]), 82: float(p[27]), 94: float(p[28]), 127: float(p[29])
                }
                if sum(counts.values()) > 0:
                    sinr_history[cid].append({'t': ts, 'counts': counts})

                # PER: TB.TotNbrDl.1.UEID (col 32) and TB.ErrTotalNbrDl.1.UEID (col 37)
                # Multiple UE rows share the same du-file timestamp — accumulate per epoch,
                # flush to per_history when the timestamp changes.
                ts_ms = int(float(p[0]))
                tb_tot = float(p[32])
                tb_err = float(p[37])
                acc = _per_accum[cid]
                if ts_ms != acc['ts']:
                    if acc['ts'] > 0 and acc['tot'] > 0:
                        per_history[cid].append({
                            't': sim_time,
                            'v': acc['err'] / acc['tot']
                        })
                    acc['tot'] = tb_tot
                    acc['err'] = tb_err
                    acc['ts']  = ts_ms
                else:
                    acc['tot'] += tb_tot
                    acc['err'] += tb_err
            except (ValueError, IndexError):
                continue

    # 2. RLC Metrics (Throughput, Latency)
    rlc_file = os.path.join(build_dir, "DlE2RlcStats.txt")
    for line in read_new_lines(rlc_file, 'rlc'):
        p = line.strip().split()
        if len(p) < 11: continue
        try:
            t1, t2, cid = float(p[0]), float(p[1]), int(p[2])
            rb = float(p[9])  # RxBytes
            delay = float(p[10]) # delay
            
            if (t2 - t1) > 0:
                tp_history[cid].append({'t': t2, 'v': (rb * 8) / ((t2 - t1) * 1000)}) # kbps
                latency_history[cid].append({'t': t2, 'v': delay * 1000.0}) # ms
        except ValueError:
            continue

    # 3. CU Metrics (Load)
    for cid in [1, 2]:
        cu_file = os.path.join(build_dir, f"cu-cp-cell-{cid}.txt")
        for line in read_new_lines(cu_file, 'cu', cid):
            p = line.strip().split(',')
            if len(p) >= 3:
                try:
                    load_history[cid].append({'t': sim_time, 'v': int(p[2])})
                except ValueError:
                    continue

    # 4. Position Metrics (Distance + RSRP)
    pos_file = os.path.join(build_dir, "UEPosition.txt")
    state_snap = get_current_network_state(build_dir)
    for line in read_new_lines(pos_file, 'pos'):
        p = line.strip().split(',')
        if len(p) < 7 or p[1] != 'UE': continue
        try:
            t   = float(p[0])
            ux  = float(p[3])
            uy  = float(p[4])
            uz  = float(p[5])          # UE height (always 1.5 m)
            cid = int(p[6])
            bx, by = BS_POS[cid]
            dist = math.sqrt((ux - bx)**2 + (uy - by)**2)
            dist_history[cid].append({'t': t, 'v': dist})

            # RSRP: accumulate per-epoch per-cell, flush when timestamp changes
            tx_power = state_snap[cid]['power'] if state_snap else 38.0
            tilt     = state_snap[cid]['tilt']  if state_snap else 10.0
            rsrp_val = compute_rsrp(ux, uy, uz, cid, tx_power, tilt)
            acc = _rsrp_accum[cid]
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

    # 5. Handover Metrics (Ping-Pong Rate)
    ho_file = os.path.join(build_dir, "HandoverLog.txt")
    for line in read_new_lines(ho_file, 'ho'):
        p = line.strip().split(',')
        if len(p) >= 5:
            try:
                ho = {'time': float(p[0]), 'ue_id': int(p[2]), 'source': int(p[3]), 'target': int(p[4]), 'is_pp': False}
                
                if ho['ue_id'] not in handover_history:
                    handover_history[ho['ue_id']] = deque(maxlen=100)
                
                ue_hist = list(handover_history[ho['ue_id']])
                if ue_hist:
                    last_ho = ue_hist[-1]
                    # Detect Ping-Pong (return within 2.0s)
                    if ho['time'] - last_ho['time'] <= 2.0 and last_ho['source'] == ho['target'] and last_ho['target'] == ho['source']:
                        ho['is_pp'] = True
                        
                handover_history[ho['ue_id']].append(ho)
            except ValueError:
                continue


# =========================================================================
# XAPP LOGIC ALGORITHMS
# =========================================================================

def run_power_manager(sim_time, state):
    """Adjusts TX Power based on SINR and Throughput."""
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['POWER'] + 1.0:
        return []
        
    commands = []
    for cid in [1, 2]:
        avg_sinr = get_avg_sinr(sinr_history[cid], sim_time, LOOKBACK_WINDOW)
        avg_tp = get_avg_metric(tp_history[cid], sim_time, LOOKBACK_WINDOW)
        
        if avg_sinr is None or avg_tp is None: continue
        
        cur_p = state[cid]['power']
        
        # Threshold Logic
        s_thresh = {30: 5, 34: 7, 38: 8.3, 42: 11, 46: 13.5}.get(cur_p, 8.3)
        t_thresh = {30: 1450, 34: 1700, 38: 1788, 42: 2400, 46: 2750}.get(cur_p, 1788)
        s_exc = {30: 7.9, 34: 9.9, 38: 12.75, 42: 13.5, 46: 16.0}.get(cur_p, 12.75)
        t_exc = {30: 2460, 34: 2970, 38: 3713, 42: 3400, 46: 3800}.get(cur_p, 3713)
        
        new_p = cur_p
        reason = ""
        
        if avg_sinr < s_thresh or avg_tp < t_thresh:
            new_p = min(46.0, cur_p + 4.0)
            reason = "EMERGENCY (Increase Power)"
        elif cur_p > 38.0 and avg_sinr > s_exc and avg_tp > t_exc:
            new_p = max(30.0, cur_p - 4.0)
            reason = "EXCELLENT (Decrease Power)"
            
        if new_p != cur_p:
            commands.append({'type': 'POWER', 'cell': cid, 'value': new_p})
            print(f"[{sim_time:.1f}s][POWER] Cell {cid}: {cur_p}->{new_p} | {reason} | SINR: {avg_sinr:.1f}, TP: {avg_tp:.0f}")
            
    if commands: last_action_t['POWER'] = sim_time
    return commands

def run_tilt_manager(sim_time, state):
    """Adjusts Tilt based on Load balancing and UE Distance."""
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['TILT'] + 3.0:
        return []
        
    commands = []
    l1 = get_avg_metric(load_history[1], sim_time, LOOKBACK_WINDOW)
    l2 = get_avg_metric(load_history[2], sim_time, LOOKBACK_WINDOW)
    d1 = get_avg_metric(dist_history[1], sim_time, LOOKBACK_WINDOW)
    d2 = get_avg_metric(dist_history[2], sim_time, LOOKBACK_WINDOW)
    
    triggered = False
    
    if l1 is not None and l2 is not None:
        # Priority 1: Load Balancing
        if abs(l1 - l2) > 8:
            h, l = (1, 2) if l1 > l2 else (2, 1)
            cur_tilt_h = state[h]['tilt']
            cur_tilt_l = state[l]['tilt']
            
            new_tilt_h = min(15.0, cur_tilt_h + 2.5) # Downtilt heavily loaded cell
            new_tilt_l = max(5.0, cur_tilt_l - 2.5)  # Uptilt lightly loaded cell
            
            if new_tilt_h != cur_tilt_h:
                commands.append({'type': 'TILT', 'cell': h, 'value': new_tilt_h})
                print(f"[{sim_time:.1f}s][TILT] Cell {h}: {cur_tilt_h}->{new_tilt_h} | LOAD_BALANCE (Heavy)")
                triggered = True
                
            if new_tilt_l != cur_tilt_l:
                commands.append({'type': 'TILT', 'cell': l, 'value': new_tilt_l})
                print(f"[{sim_time:.1f}s][TILT] Cell {l}: {cur_tilt_l}->{new_tilt_l} | LOAD_BALANCE (Light)")
                triggered = True
                
        # Priority 2: Near/Far Proximity
        if not triggered:
            for cid in [1, 2]:
                d_avg = d1 if cid == 1 else d2
                if d_avg is None: continue
                
                cur_tilt = state[cid]['tilt']
                new_tilt = cur_tilt
                reason = ""
                
                if d_avg < 190.2 and cur_tilt < 15.0:
                    new_tilt = cur_tilt + 2.5
                    reason = "NEAR_PROXIMITY"
                elif d_avg > 152.9 and cur_tilt > 5.0:
                    new_tilt = cur_tilt - 2.5
                    reason = "FAR_COVERAGE"
                    
                if new_tilt != cur_tilt:
                    commands.append({'type': 'TILT', 'cell': cid, 'value': new_tilt})
                    print(f"[{sim_time:.1f}s][TILT] Cell {cid}: {cur_tilt}->{new_tilt} | {reason} | Avg Dist: {d_avg:.1f}m")
                    triggered = True
                    break # Only one at a time if not load balancing
                    
    if triggered: last_action_t['TILT'] = sim_time
    return commands

def run_handover_manager(sim_time, state):
    """Adjusts A3 Offset based on Handover Ping-Pong Rates."""
    global last_action_t
    if sim_time < LOOKBACK_WINDOW or sim_time < last_action_t['A3'] + 2.5:
        return []
        
    commands = []
    decisions = {}
    
    for cid in [1, 2]:
        tot_ho, pp_ho = 0, 0
        for ue_id, hist in handover_history.items():
            for ho in hist:
                if sim_time - LOOKBACK_WINDOW < ho['time'] <= sim_time and ho['source'] == cid:
                    tot_ho += 1
                    if ho['is_pp']: pp_ho += 1
                    
        if tot_ho == 0: continue
        
        rate = (pp_ho / tot_ho) * 100.0
        new_a3 = 0.0
        reason = "MODERATE_PP_RATE"
        
        if rate > 72.0:
            new_a3 = 3.0
            reason = "HIGH_PP_RATE"
        elif rate < 53.0:
            new_a3 = -3.0
            reason = "LOW_PP_RATE"
            
        decisions[cid] = {'a3': new_a3, 'rate': rate, 'reason': reason}
        
    # Apply constraint to ensure both cells don't simultaneously hit -3.0
    if (1 in decisions and 2 in decisions and decisions[1]['a3'] == -3.0 and decisions[2]['a3'] == -3.0):
        weaker = 1 if decisions[1]['rate'] < decisions[2]['rate'] else 2
        decisions[weaker]['a3'] = 0.0
        decisions[weaker]['reason'] = "CONSTRAINT_A3_BOTH_NEG3"
        
    for cid, d in decisions.items():
        cur_a3 = state[cid]['a3']
        if d['a3'] != cur_a3:
            commands.append({'type': 'A3', 'cell': cid, 'value': d['a3']})
            print(f"[{sim_time:.1f}s][A3] Cell {cid}: {cur_a3}->{d['a3']} | {d['reason']} | PP Rate: {d['rate']:.1f}%")
            
    if commands: last_action_t['A3'] = sim_time
    return commands


# =========================================================================
# MAIN LOOP
# =========================================================================

def xapp_loop():
    print("========================================")
    print("      NS-3 O-RAN FULL xApp Orchestrator ")
    print("========================================")
    print(f"Listening to directory: {BUILD_DIR}")
    
    last_sim_time  = 0.0
    last_print_time = 0.0
    wall_start      = time.time()

    try:
        while True:
            sim_time = get_current_sim_time(BUILD_DIR)

            if sim_time is not None and sim_time > last_sim_time:
                last_sim_time = sim_time

                # 1. READ ALL METRICS
                process_metrics(BUILD_DIR, sim_time)

                # 2. PRINT METRICS EVERY 1s
                if sim_time >= last_print_time + 1.0:
                    elapsed   = time.time() - wall_start
                    remaining = max(0.0, 120.0 - sim_time)
                    print(f"\n--- sim={sim_time:.1f}s | wall={elapsed:.0f}s | ~{remaining:.0f}s left ---")
                    for cid in [1, 2]:
                        sinr  = get_avg_sinr(sinr_history[cid], sim_time, LOOKBACK_WINDOW) or 0.0
                        tp    = get_avg_metric(tp_history[cid], sim_time, LOOKBACK_WINDOW) or 0.0
                        load  = get_avg_metric(load_history[cid], sim_time, LOOKBACK_WINDOW) or 0.0
                        dist  = get_avg_metric(dist_history[cid], sim_time, LOOKBACK_WINDOW) or 0.0
                        lat   = get_avg_metric(latency_history[cid], sim_time, LOOKBACK_WINDOW) or 0.0
                        rsrp  = get_avg_metric(rsrp_history[cid], sim_time, LOOKBACK_WINDOW)
                        per   = get_avg_metric(per_history[cid], sim_time, LOOKBACK_WINDOW)
                        rsrp_str = f"{rsrp:>6.1f} dBm" if rsrp is not None else "   N/A dBm"
                        per_str  = f"{per * 100.0:>5.1f}%"  if per  is not None else "  N/A%"

                        # Ping-Pong Rate
                        tot_ho, pp_ho = 0, 0
                        for ue_id, hist in handover_history.items():
                            for ho in hist:
                                if sim_time - LOOKBACK_WINDOW < ho['time'] <= sim_time and ho['source'] == cid:
                                    tot_ho += 1
                                    if ho['is_pp']: pp_ho += 1
                        pp_rate = (pp_ho / tot_ho * 100.0) if tot_ho > 0 else 0.0

                        print(f" Cell {cid} | SINR: {sinr:>5.1f} dB | TP: {tp:>5.0f} kbps | "
                              f"Lat: {lat:>4.1f} ms | Load: {load:>3.0f} UEs | "
                              f"Dist: {dist:>5.1f} m | RSRP: {rsrp_str} | "
                              f"PER: {per_str} | PP Rate: {pp_rate:>4.1f}%")
                    print("-" * 120)
                    last_print_time = math.floor(sim_time)

                # 3. GET CURRENT GROUND TRUTH STATE
                state = get_current_network_state(BUILD_DIR)
                
                # 4. RUN ALGORITHMS
                all_commands = []
                all_commands.extend(run_power_manager(sim_time, state))
                all_commands.extend(run_tilt_manager(sim_time, state))
                all_commands.extend(run_handover_manager(sim_time, state))
                
                # 5. SEND BACK TO NS-3
                write_commands(BUILD_DIR, all_commands)
                
            time.sleep(POLL_INTERVAL)
            
    except KeyboardInterrupt:
        print("\nxApp Orchestrator stopped by user.")

def main():
    ns3_thread = threading.Thread(target=run_ns3)
    ns3_thread.start()
    
    # Wait a bit for NS-3 to start creating files
    time.sleep(2)
    xapp_loop()
    
    ns3_thread.join()

if __name__ == "__main__":
    main()
