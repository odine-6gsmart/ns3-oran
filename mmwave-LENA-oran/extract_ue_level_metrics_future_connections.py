#!/usr/bin/env python3
"""
Extract UE-level metrics from NS-3 O-RAN FutureConnections 4-gNB simulation outputs.
Specifically designed to process the outputs of xapp_template_futureconnections.py.
"""

import os
import math
import argparse
import pandas as pd
import numpy as np
from pathlib import Path

N_CELLS = 4

BS_POS_3D = {
    1: (900.0,  3200.0, 5.0),
    2: (3500.0, 3600.0, 5.0),
    3: (1800.0,  800.0, 5.0),
    4: (3800.0, 1600.0, 5.0),
}
LOG_DIST_EXP = 3.8
LOG_DIST_REF = 43.3
HPBW         = 10.0

def compute_rsrp(ue_x, ue_y, ue_z, cell_id, tx_power, tilt):
    bx, by, bz = BS_POS_3D[cell_id]
    dx, dy, dz = ue_x - bx, ue_y - by, ue_z - bz
    dist = max(math.sqrt(dx**2 + dy**2 + dz**2), 1.0)
    pathloss = LOG_DIST_REF + 10.0 * LOG_DIST_EXP * math.log10(dist)
    theta_deg = math.acos(max(min(dz / dist, 1.0), -1.0)) * 180.0 / math.pi
    boresight = 90.0 + tilt
    gain = max(18.0 - 12.0 * ((theta_deg - boresight) / HPBW)**2, -30.0)
    return tx_power + gain - pathloss

def normalize_timestamp_series(ts: pd.Series) -> pd.Series:
    ts = pd.to_numeric(ts, errors="coerce")
    if ts.isna().all():
        return ts
    ts0 = ts.min()
    normalized = (ts - ts0) * 0.001
    return (normalized + 0.1).round(1)

def sinr_bin_index_to_dB(bin_index: float) -> float:
    return (bin_index / 2.0) - 23.0

def load_network_config(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df.rename(columns={'Time(s)': 'time'}, inplace=True)
        return df.sort_values('time').reset_index(drop=True)
    except Exception as e:
        print(f"Error loading network config: {e}")
        return None

def load_ue_positions(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df = df[df['Type'] == 'UE'].copy()
        df.rename(columns={'Time(s)': 'time', 'ID': 'ue_id'}, inplace=True)
        return df[['time', 'ue_id', 'X(m)', 'Y(m)', 'Z(m)', 'CellID']]
    except Exception as e:
        print(f"Error loading UE positions: {e}")
        return None

def load_du_ue_metrics(path, cell_id):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df['time'] = normalize_timestamp_series(df['timestamp'])

        sinr_cols_ueid = [c for c in df.columns if "L1M.RS-SINR.Bin" in c and c.endswith(".UEID")]
        if not sinr_cols_ueid:
            return None

        bin_col_pairs = []
        for col in sinr_cols_ueid:
            part = col.strip().split("Bin")[-1].replace(".UEID", "").replace(",", "").strip()
            part = "".join(ch for ch in part if (ch.isdigit() or ch == "." or ch == "-"))
            try:
                bin_idx = float(part)
            except ValueError:
                bin_idx = 0.0
            bin_col_pairs.append((bin_idx, col))

        bin_col_pairs.sort()
        sinr_cols_ueid = [c for _, c in bin_col_pairs]
        centers_db = np.array([sinr_bin_index_to_dB(b) for b, _ in bin_col_pairs], dtype=float)

        if 'ueImsiComplete' not in df.columns:
            return None

        df['ueImsiComplete'] = (
            df['ueImsiComplete'].astype(str).str.strip().str.lstrip("0").replace("", "0").astype(int)
        )

        tb_tot_col = next((c for c in df.columns if c.strip() == 'TB.TotNbrDl.1.UEID'), None)
        tb_err_col = next((c for c in df.columns if c.strip() == 'TB.ErrTotalNbrDl.1.UEID'), None)

        ue_sinr_data = []
        for _, row in df.iterrows():
            counts = np.array([row.get(col, 0) for col in sinr_cols_ueid], dtype=float)
            counts[np.isnan(counts)] = 0.0
            total = counts.sum()
            avg_sinr_db = (counts * centers_db).sum() / total if total > 0 else np.nan

            tb_tot = float(row[tb_tot_col]) if tb_tot_col else np.nan
            tb_err = float(row[tb_err_col]) if tb_err_col else np.nan
            per = float(np.clip(tb_err / tb_tot, 0, 1)) if (tb_tot_col and tb_err_col and tb_tot > 0) else np.nan

            ue_sinr_data.append({
                'time':    row['time'],
                'ue_id':   row['ueImsiComplete'],
                'sinr_db': avg_sinr_db,
                'per':     per,
            })

        return pd.DataFrame(ue_sinr_data)
    except Exception as e:
        print(f"Error loading DU metrics from {path}: {e}")
        return None

def load_rlc_ue_metrics(path, cell_id):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path, sep=r'\s+', comment='%', header=None,
                         names=['start', 'end', 'CellId', 'IMSI', 'RNTI', 'LCID',
                                'nTxPDUs', 'TxBytes', 'nRxPDUs', 'RxBytes', 'delay',
                                'stdDev', 'min', 'max', 'PduSize', 'stdDev2', 'min2', 'max2'])

        df = df[df['CellId'] == cell_id].copy()
        df['time'] = df['end']
        df['window'] = df['end'] - df['start']
        df['throughput_kbps'] = ((df['RxBytes'] * 8 / 1000) / df['window']).clip(upper=10000.0)
        df['latency_ms'] = df['delay'] * 1000
        df['pkt_loss'] = ((df['TxBytes'] - df['RxBytes']) / df['TxBytes'].clip(lower=1)).clip(0, 1)
        df.rename(columns={'IMSI': 'ue_id'}, inplace=True)

        return df[['time', 'ue_id', 'throughput_kbps', 'latency_ms', 'pkt_loss']]
    except Exception as e:
        print(f"Error loading RLC stats from {path}: {e}")
        return None


def load_rxtrace_rsrq(rxtrace_path, rlc_path=None):
    """
    Parse RxPacketTrace.txt (DL rows only) to compute RSRQ per UE and per cell
    over 100 ms epochs. Returns (ue_rsrq_dict, cell_rsrq_dict) keyed by int cell_id.
    RSRQ formula: RSRQ_dB = 10*log10(SINR_lin / (SINR_lin + 1))
    """
    if not os.path.exists(rxtrace_path):
        return {}, {}
    try:
        df = pd.read_csv(rxtrace_path, sep='\t')
        df.columns = df.columns.str.strip()
        dir_col = df.columns[0]
        dl = df[df[dir_col].astype(str).str.strip() == 'DL'].copy()
        if dl.empty:
            return {}, {}

        sinr_col = next((c for c in dl.columns if 'SINR' in c), None)
        if sinr_col is None:
            return {}, {}

        dl['time']    = (dl['time'].astype(float) * 10).round().astype(int) / 10.0
        sinr_lin      = 10 ** (dl[sinr_col].astype(float).clip(-80, 80) / 10.0)
        dl['rsrq_dB'] = 10 * np.log10((sinr_lin / (sinr_lin + 1)).clip(1e-15))
        dl['cellId']  = dl['cellId'].astype(int)
        dl['rnti']    = dl['rnti'].astype(int)

        # Build (CellId, RNTI) → IMSI map from RLC stats to correctly label UEs
        rnti_imsi_map = {}
        if rlc_path and os.path.exists(rlc_path):
            try:
                rlc = pd.read_csv(
                    rlc_path, sep=r'\s+', comment='%', header=None,
                    names=['start','end','CellId','IMSI','RNTI','LCID',
                           'nTxPDUs','TxBytes','nRxPDUs','RxBytes','delay',
                           'stdDev','min','max','PduSize','stdDev2','min2','max2'],
                )
                rnti_imsi_map = (
                    rlc.groupby([rlc['CellId'].astype(int), rlc['RNTI'].astype(int)])['IMSI']
                    .first().to_dict()
                )
            except Exception:
                pass

        ue_rsrq_dict   = {}
        cell_rsrq_dict = {}

        for cell_id, cell_dl in dl.groupby('cellId'):
            cell_dl = cell_dl.copy()
            if rnti_imsi_map:
                keys = list(zip(cell_dl['cellId'], cell_dl['rnti']))
                cell_dl['ue_id'] = [rnti_imsi_map.get(k, k[1]) for k in keys]
            else:
                cell_dl['ue_id'] = cell_dl['rnti']
            ue_rsrq_dict[int(cell_id)] = (
                cell_dl.groupby(['time', 'ue_id'])
                .agg(rsrq_dB=('rsrq_dB', 'mean'))
                .reset_index()
            )
            cell_rsrq_dict[int(cell_id)] = (
                cell_dl.groupby('time')['rsrq_dB'].mean()
                .reset_index().rename(columns={'rsrq_dB': 'avg_rsrq_dB'})
            )

        return ue_rsrq_dict, cell_rsrq_dict
    except Exception as e:
        print(f"Error loading RxPacketTrace RSRQ from {rxtrace_path}: {e}")
        return {}, {}


def extract_metrics_from_dir(input_dir):
    run_path = Path(input_dir)
    print(f"Extracting metrics from: {run_path}")

    ue_positions = load_ue_positions(run_path / "UEPosition.txt")
    netconf      = load_network_config(run_path / "NetworkConfigurations.txt")
    ue_rsrq, _   = load_rxtrace_rsrq(run_path / "RxPacketTrace.txt", run_path / "DlE2RlcStats.txt")

    metrics_dict = {}

    for cell_id in range(1, N_CELLS + 1):
        du_metrics  = load_du_ue_metrics(run_path / f"du-cell-{cell_id}.txt", cell_id)
        rlc_metrics = load_rlc_ue_metrics(run_path / "DlE2RlcStats.txt", cell_id)

        if ue_positions is None:
            continue

        cell_pos = ue_positions[ue_positions['CellID'] == cell_id].copy()
        if cell_pos.empty:
            print(f"  No UEs found in cell {cell_id} — skipping.")
            continue

        merged_df = cell_pos.copy()

        if du_metrics is not None:
            merged_df = merged_df.merge(du_metrics, on=['time', 'ue_id'], how='left')
        else:
            merged_df['sinr_db'] = np.nan
            merged_df['per']     = np.nan

        if rlc_metrics is not None:
            merged_df = merged_df.merge(rlc_metrics, on=['time', 'ue_id'], how='left')
        else:
            merged_df['throughput_kbps'] = np.nan
            merged_df['latency_ms']      = np.nan
            merged_df['pkt_loss']        = np.nan

        if netconf is not None:
            p_col = f'Cell{cell_id}_TxPower'
            t_col = f'Cell{cell_id}_Tilt'
            if p_col in netconf.columns and t_col in netconf.columns:
                nc = netconf[['time', p_col, t_col]].rename(columns={p_col: 'tx_power', t_col: 'tilt'})
                merged_df = pd.merge_asof(merged_df.sort_values('time'), nc, on='time', direction='backward')
                merged_df['tx_power'] = merged_df['tx_power'].fillna(38.0)
                merged_df['tilt']     = merged_df['tilt'].fillna(10.0)
            else:
                merged_df['tx_power'] = 38.0
                merged_df['tilt']     = 10.0
        else:
            merged_df['tx_power'] = 38.0
            merged_df['tilt']     = 10.0

        merged_df['rsrp_dbm'] = merged_df.apply(
            lambda r: compute_rsrp(r['X(m)'], r['Y(m)'], r['Z(m)'], cell_id, r['tx_power'], r['tilt']),
            axis=1
        )

        # Merge RxPacketTrace RSRQ per UE per epoch
        rsrq_df = ue_rsrq.get(cell_id)
        if rsrq_df is not None:
            merged_df = merged_df.merge(rsrq_df, on=['time', 'ue_id'], how='left')
        else:
            merged_df['rsrq_dB'] = np.nan

        final_cols = ['time', 'ue_id', 'sinr_db', 'throughput_kbps', 'latency_ms',
                      'pkt_loss', 'per', 'rsrp_dbm', 'rsrq_dB',
                      'X(m)', 'Y(m)', 'Z(m)']
        available_cols = [col for col in final_cols if col in merged_df.columns]
        merged_df = merged_df[available_cols].sort_values(['time', 'ue_id']).reset_index(drop=True)

        metrics_dict[f"cell{cell_id}"] = merged_df

    return metrics_dict


def main():
    parser = argparse.ArgumentParser(description="Extract UE-Level metrics from FutureConnections 4-gNB outputs")
    parser.add_argument('--input-dir',     type=str, default='xapp_futureconnections_outputs')
    parser.add_argument('--output-dir',    type=str, default='ns3_sim_timeseries_data')
    parser.add_argument('--output-prefix', type=str, default='fc_ue_level_metrics')
    args = parser.parse_args()

    base_dir    = Path(__file__).parent
    input_path  = base_dir / args.input_dir
    output_dir  = base_dir / args.output_dir
    output_prefix = args.output_prefix

    if not input_path.exists():
        print(f"Error: Input directory {input_path} does not exist.")
        return

    output_dir.mkdir(parents=True, exist_ok=True)

    metrics_dict = extract_metrics_from_dir(input_path)

    for cell_name, df in metrics_dict.items():
        if df is not None and not df.empty:
            output_file = output_dir / f"{output_prefix}_{cell_name}.csv"
            df.to_csv(output_file, index=False, float_format='%.3f')
            print(f"Successfully generated {output_file} ({len(df)} records)")
        else:
            print(f"No data found for {cell_name}.")

if __name__ == "__main__":
    main()
