#!/usr/bin/env python3
"""
Extract cell-level simulation metrics from NS-3 O-RAN FutureConnections 4-gNB outputs.
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

def load_network_config(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df.rename(columns={'Time(s)': 'time'}, inplace=True)
        return df
    except Exception as e:
        print(f"Error loading network config: {e}")
        return None

def load_cucp_metrics(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df['time'] = normalize_timestamp_series(df['timestamp'])
        df = df[['time', 'numActiveUes']].copy()
        df = df.groupby('time')['numActiveUes'].first().reset_index()
        return df
    except Exception as e:
        print(f"Error loading CU-CP metrics from {path}: {e}")
        return None

def sinr_bin_index_to_dB(bin_index: float) -> float:
    return (bin_index / 2.0) - 23.0

def load_du_metrics(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df['time'] = normalize_timestamp_series(df['timestamp'])

        metrics = {
            'L1M.RS-SINR.Bin34': 'sum',
            'L1M.RS-SINR.Bin46': 'sum',
            ' L1M.RS-SINR.Bin58': 'sum',
            'L1M.RS-SINR.Bin70': 'sum',
            'L1M.RS-SINR.Bin82': 'sum',
            'L1M.RS-SINR.Bin94': 'sum',
            'L1M.RS-SINR.Bin127': 'sum',
            'RRU.PrbUsedDl': 'sum',
            'dlPrbUsage': 'mean',
            'ulPrbUsage': 'mean',
            'TB.TotNbrDl.1.UEID': 'sum',
            'TB.ErrTotalNbrDl.1.UEID': 'sum',
        }

        agg_dict = {}
        for k, v in metrics.items():
            if k in df.columns:
                agg_dict[k] = v
            elif k.strip() in df.columns:
                agg_dict[k.strip()] = v

        df_agg = df.groupby('time').agg(agg_dict).reset_index()

        tot_col = next((c for c in df_agg.columns if c.strip() == 'TB.TotNbrDl.1.UEID'), None)
        err_col = next((c for c in df_agg.columns if c.strip() == 'TB.ErrTotalNbrDl.1.UEID'), None)
        if tot_col and err_col:
            mask = df_agg[tot_col] > 0
            df_agg['per'] = np.nan
            df_agg.loc[mask, 'per'] = (df_agg.loc[mask, err_col] / df_agg.loc[mask, tot_col]).clip(0, 1)

        prb_col = next((c for c in df_agg.columns if c.strip() == 'dlPrbUsage'), None)
        if prb_col:
            df_agg['prb_util'] = (df_agg[prb_col] / 100.0).clip(0, 1)

        return df_agg
    except Exception as e:
        print(f"Error loading DU metrics from {path}: {e}")
        return None

def compute_cell_avg_sinr(du_df):
    if du_df is None or du_df.empty:
        return None

    sinr_cols = [c for c in du_df.columns if "L1M.RS-SINR.Bin" in c and not c.endswith(".UEID")]
    if not sinr_cols:
        return None

    sinr_cols = sorted(sinr_cols)
    centers_dB = []
    for c in sinr_cols:
        part = c.split("Bin")[-1].replace(",", "").strip()
        part = "".join(ch for ch in part if (ch.isdigit() or ch == "." or ch == "-"))
        try:
            centers_dB.append(sinr_bin_index_to_dB(float(part)))
        except ValueError:
            centers_dB.append(0.0)

    centers_arr = np.array(centers_dB, dtype=float)

    result_times, result_sinr = [], []
    for _, row in du_df.iterrows():
        counts = np.array([row[c] for c in sinr_cols], dtype=float)
        counts[np.isnan(counts)] = 0.0
        total = counts.sum()
        result_times.append(row['time'])
        result_sinr.append((counts * centers_arr).sum() / total if total > 0 else np.nan)

    return pd.DataFrame({'time': result_times, 'avg_sinr_db': result_sinr})

def load_rlc_stats(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path, sep=r'\s+', comment='%', header=None,
                         names=['start', 'end', 'CellId', 'IMSI', 'RNTI', 'LCID',
                                'nTxPDUs', 'TxBytes', 'nRxPDUs', 'RxBytes', 'delay',
                                'stdDev', 'min', 'max', 'PduSize', 'stdDev2', 'min2', 'max2'])

        df['_time'] = df['end']
        df['window'] = df['end'] - df['start']
        df['rx_throughput_kbps'] = ((df['RxBytes'] * 8 / 1000) / df['window']).clip(upper=10000.0)
        df['latency_ms'] = df['delay'] * 1000
        df['pkt_loss'] = ((df['TxBytes'] - df['RxBytes']) / df['TxBytes'].clip(lower=1)).clip(0, 1)

        result = {}
        for cell_id in range(1, N_CELLS + 1):
            cell_data = df[df['CellId'] == cell_id].copy()
            if not cell_data.empty:
                cell_agg = cell_data.groupby('_time').agg({
                    'rx_throughput_kbps': 'mean',
                    'latency_ms': 'mean',
                    'pkt_loss': 'mean',
                }).reset_index().rename(columns={'_time': 'time'})
                result[f'cell{cell_id}'] = cell_agg
            else:
                result[f'cell{cell_id}'] = None
        return result
    except Exception as e:
        print(f"Error loading RLC stats from {path}: {e}")
        return None

def load_cell_rsrp(pos_path, netconf_path):
    if not os.path.exists(pos_path):
        return None
    try:
        pos_df = pd.read_csv(pos_path)
        pos_df = pos_df[pos_df['Type'] == 'UE'].copy()
        pos_df.rename(columns={'Time(s)': 'time', 'ID': 'ue_id'}, inplace=True)
        pos_df = pos_df.sort_values('time').reset_index(drop=True)

        netconf = None
        if os.path.exists(netconf_path):
            netconf = pd.read_csv(netconf_path)
            netconf.rename(columns={'Time(s)': 'time'}, inplace=True)
            netconf = netconf.sort_values('time').reset_index(drop=True)

        result = {}
        for cell_id in range(1, N_CELLS + 1):
            cell_pos = pos_df[pos_df['CellID'] == cell_id].copy()
            if cell_pos.empty:
                result[f'cell{cell_id}'] = None
                continue

            if netconf is not None:
                p_col = f'Cell{cell_id}_TxPower'
                t_col = f'Cell{cell_id}_Tilt'
                nc = netconf[['time', p_col, t_col]].rename(columns={p_col: 'tx_power', t_col: 'tilt'})
                cell_pos = pd.merge_asof(cell_pos.sort_values('time'), nc, on='time', direction='backward')
                cell_pos['tx_power'] = cell_pos['tx_power'].fillna(38.0)
                cell_pos['tilt']     = cell_pos['tilt'].fillna(10.0)
            else:
                cell_pos['tx_power'] = 38.0
                cell_pos['tilt']     = 10.0

            cell_pos['rsrp_dbm'] = cell_pos.apply(
                lambda r: compute_rsrp(r['X(m)'], r['Y(m)'], r['Z(m)'], cell_id, r['tx_power'], r['tilt']),
                axis=1
            )
            avg = cell_pos.groupby('time')['rsrp_dbm'].mean().reset_index()
            result[f'cell{cell_id}'] = avg

        return result
    except Exception as e:
        print(f"Error computing cell RSRP: {e}")
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

    net_config = load_network_config(run_path / "NetworkConfigurations.txt")
    rlc_stats  = load_rlc_stats(run_path / "DlE2RlcStats.txt")
    rsrp_stats = load_cell_rsrp(run_path / "UEPosition.txt", run_path / "NetworkConfigurations.txt")
    _, cell_rsrq = load_rxtrace_rsrq(run_path / "RxPacketTrace.txt", run_path / "DlE2RlcStats.txt")

    cucp_per_cell = {}
    du_per_cell   = {}
    sinr_per_cell = {}
    for cid in range(1, N_CELLS + 1):
        cucp_per_cell[cid] = load_cucp_metrics(run_path / f"cu-cp-cell-{cid}.txt")
        du_df = load_du_metrics(run_path / f"du-cell-{cid}.txt")
        du_per_cell[cid]   = du_df
        sinr_per_cell[cid] = compute_cell_avg_sinr(du_df) if du_df is not None else None

    max_time = 50.0
    if net_config is not None and not net_config.empty:
        max_time = max(max_time, net_config['time'].max() + 1.0)
    time_index = pd.DataFrame({'time': np.arange(0.1, max_time, 0.1).round(1)})

    merged_df = time_index.copy()

    if net_config is not None:
        merged_df = merged_df.merge(net_config, on='time', how='left')

    for cid in range(1, N_CELLS + 1):
        if cucp_per_cell[cid] is not None:
            merged_df = merged_df.merge(
                cucp_per_cell[cid].rename(columns={'numActiveUes': f'numActiveUes_cell{cid}'}),
                on='time', how='left'
            )
        if sinr_per_cell[cid] is not None:
            merged_df = merged_df.merge(
                sinr_per_cell[cid].rename(columns={'avg_sinr_db': f'avg_sinr_db_cell{cid}'}),
                on='time', how='left'
            )
        if rlc_stats is not None and rlc_stats.get(f'cell{cid}') is not None:
            rlc_df = rlc_stats[f'cell{cid}'].rename(columns={
                'rx_throughput_kbps': f'avg_ue_throughput_kbps_cell{cid}',
                'latency_ms':         f'avg_ue_latency_ms_cell{cid}',
                'pkt_loss':           f'pkt_loss_cell{cid}',
            })
            merged_df = merged_df.merge(rlc_df, on='time', how='left')

        du_df = du_per_cell[cid]
        if du_df is not None:
            extra_cols = ['time']
            if 'per'      in du_df.columns: extra_cols.append('per')
            if 'prb_util' in du_df.columns: extra_cols.append('prb_util')
            if len(extra_cols) > 1:
                extra_df = du_df[extra_cols].copy()
                extra_df.rename(columns={c: f'{c}_cell{cid}' for c in extra_cols if c != 'time'}, inplace=True)
                merged_df = merged_df.merge(extra_df, on='time', how='left')

        if rsrp_stats is not None and rsrp_stats.get(f'cell{cid}') is not None:
            rsrp_df = rsrp_stats[f'cell{cid}'].rename(columns={'rsrp_dbm': f'rsrp_dbm_cell{cid}'})
            merged_df = merged_df.merge(rsrp_df, on='time', how='left')

        # Merge RxPacketTrace-based RSRQ (cell average per 100 ms epoch)
        rsrq_df = cell_rsrq.get(cid)
        if rsrq_df is not None:
            merged_df = merged_df.merge(
                rsrq_df.rename(columns={'avg_rsrq_dB': f'avg_rsrq_dB_cell{cid}'}),
                on='time', how='left'
            )

    data_cols = [c for c in merged_df.columns if c != 'time']
    merged_df = merged_df.dropna(subset=data_cols, how='all')

    essential_cols = ['time']
    for cid in range(1, N_CELLS + 1):
        essential_cols += [
            f'Cell{cid}_TxPower', f'Cell{cid}_Tilt', f'Cell{cid}_A3',
            f'numActiveUes_cell{cid}',
            f'avg_sinr_db_cell{cid}',
            f'avg_ue_throughput_kbps_cell{cid}',
            f'avg_ue_latency_ms_cell{cid}',
            f'pkt_loss_cell{cid}',
            f'per_cell{cid}',
            f'prb_util_cell{cid}',
            f'rsrp_dbm_cell{cid}',
            f'avg_rsrq_dB_cell{cid}',
        ]

    available_cols = [col for col in essential_cols if col in merged_df.columns]
    merged_df = merged_df[available_cols]

    result = {}
    for cid in range(1, N_CELLS + 1):
        cell_cols = ['time'] + [c for c in merged_df.columns if f'Cell{cid}_' in c or f'_cell{cid}' in c]
        df_cell = merged_df[cell_cols].copy()
        df_cell.rename(columns=lambda x: x.replace(f'Cell{cid}_', '').replace(f'_cell{cid}', ''), inplace=True)
        df_cell = df_cell.dropna(subset=[c for c in df_cell.columns if c != 'time'], how='all')
        result[f'cell{cid}'] = df_cell

    return result


def main():
    parser = argparse.ArgumentParser(description="Extract Cell-Level metrics from FutureConnections 4-gNB outputs")
    parser.add_argument('--input-dir',     type=str, default='xapp_futureconnections_outputs')
    parser.add_argument('--output-dir',    type=str, default='ns3_sim_timeseries_data')
    parser.add_argument('--output-prefix', type=str, default='fc_cell_level_metrics')
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
            print(f"Successfully generated {output_file} ({len(df)} rows)")
        else:
            print(f"No data found for {cell_name}.")

if __name__ == "__main__":
    main()
