#!/usr/bin/env python3
"""
Extract cell-level simulation metrics from NS-3 O-RAN simulation output files.
Specifically designed to process the outputs of xapp_template.py.
Converts raw simulation data into a time-series CSV format.
"""

import os
import argparse
import pandas as pd
import numpy as np
from pathlib import Path

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
        }
        
        # Filter metrics to only those that exist in the DataFrame
        agg_dict = {}
        for k, v in metrics.items():
            if k in df.columns:
                agg_dict[k] = v
            elif k.strip() in df.columns:
                agg_dict[k.strip()] = v
                
        df_agg = df.groupby('time').agg(agg_dict).reset_index()
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
    
    result_times = []
    result_sinr = []
    
    for idx, row in du_df.iterrows():
        counts = np.array([row[c] for c in sinr_cols], dtype=float)
        counts[np.isnan(counts)] = 0.0
        total = counts.sum()
        
        if total > 0:
            result_times.append(row['time'])
            result_sinr.append((counts * centers_arr).sum() / total)
        else:
            result_times.append(row['time'])
            result_sinr.append(np.nan)
            
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
        df['rx_throughput_kbps'] = (df['RxBytes'] * 8 / 1000) / df['window']
        df['latency_ms'] = df['delay'] * 1000
        
        result = {}
        for cell_id in [1, 2]:
            cell_data = df[df['CellId'] == cell_id].copy()
            if not cell_data.empty:
                metrics = {
                    'rx_throughput_kbps': 'mean',
                    'latency_ms': 'mean',
                }
                cell_agg = cell_data.groupby('_time').agg(metrics).reset_index()
                cell_agg.rename(columns={'_time': 'time'}, inplace=True)
                result[f'cell{cell_id}'] = cell_agg
            else:
                result[f'cell{cell_id}'] = None
        return result
    except Exception as e:
        print(f"Error loading RLC stats from {path}: {e}")
        return None

def extract_metrics_from_dir(input_dir):
    run_path = Path(input_dir)
    print(f"Extracting metrics from: {run_path}")
    
    net_config = load_network_config(run_path / "NetworkConfigurations.txt")
    cucp_cell1 = load_cucp_metrics(run_path / "cu-cp-cell-1.txt")
    cucp_cell2 = load_cucp_metrics(run_path / "cu-cp-cell-2.txt")
    du_cell1 = load_du_metrics(run_path / "du-cell-1.txt")
    du_cell2 = load_du_metrics(run_path / "du-cell-2.txt")
    rlc_stats = load_rlc_stats(run_path / "DlE2RlcStats.txt")
    
    sinr_cell1 = compute_cell_avg_sinr(du_cell1) if du_cell1 is not None else None
    sinr_cell2 = compute_cell_avg_sinr(du_cell2) if du_cell2 is not None else None
    
    # Establish a time index
    max_time = 50.0
    if net_config is not None and not net_config.empty:
        max_time = max(max_time, net_config['time'].max() + 1.0)
    time_index = pd.DataFrame({'time': np.arange(0.1, max_time, 0.1).round(1)})
    
    merged_df = time_index.copy()
    
    if net_config is not None:
        merged_df = merged_df.merge(net_config, on='time', how='left')
        
    if cucp_cell1 is not None:
        merged_df = merged_df.merge(cucp_cell1.rename(columns={'numActiveUes': 'numActiveUes_cell1'}), on='time', how='left')
    if cucp_cell2 is not None:
        merged_df = merged_df.merge(cucp_cell2.rename(columns={'numActiveUes': 'numActiveUes_cell2'}), on='time', how='left')
        
    if sinr_cell1 is not None:
        merged_df = merged_df.merge(sinr_cell1.rename(columns={'avg_sinr_db': 'avg_sinr_db_cell1'}), on='time', how='left')
    if sinr_cell2 is not None:
        merged_df = merged_df.merge(sinr_cell2.rename(columns={'avg_sinr_db': 'avg_sinr_db_cell2'}), on='time', how='left')
        
    if rlc_stats is not None:
        if rlc_stats.get('cell1') is not None:
            rlc_cell1 = rlc_stats['cell1'].copy()
            rlc_cell1.rename(columns={'rx_throughput_kbps': 'avg_ue_throughput_kbps_cell1', 'latency_ms': 'avg_ue_latency_ms_cell1'}, inplace=True)
            merged_df = merged_df.merge(rlc_cell1, on='time', how='left')
        if rlc_stats.get('cell2') is not None:
            rlc_cell2 = rlc_stats['cell2'].copy()
            rlc_cell2.rename(columns={'rx_throughput_kbps': 'avg_ue_throughput_kbps_cell2', 'latency_ms': 'avg_ue_latency_ms_cell2'}, inplace=True)
            merged_df = merged_df.merge(rlc_cell2, on='time', how='left')

    # Drop totally empty rows
    data_cols = [c for c in merged_df.columns if c != 'time']
    merged_df = merged_df.dropna(subset=data_cols, how='all')

    # Select only our target columns
    essential_cols = [
        'time',
        'Cell1_TxPower', 'Cell1_Tilt', 'Cell1_A3',
        'Cell2_TxPower', 'Cell2_Tilt', 'Cell2_A3',
        'numActiveUes_cell1', 'numActiveUes_cell2',
        'avg_sinr_db_cell1', 'avg_sinr_db_cell2',
        'avg_ue_throughput_kbps_cell1', 'avg_ue_throughput_kbps_cell2',
        'avg_ue_latency_ms_cell1', 'avg_ue_latency_ms_cell2'
    ]
    
    available_cols = [col for col in essential_cols if col in merged_df.columns]
    merged_df = merged_df[available_cols]
    
    # Split into two dataframes
    cell1_cols = ['time'] + [c for c in merged_df.columns if 'Cell1' in c or 'cell1' in c]
    cell2_cols = ['time'] + [c for c in merged_df.columns if 'Cell2' in c or 'cell2' in c]
    
    df_cell1 = merged_df[cell1_cols].copy()
    df_cell1.rename(columns=lambda x: x.replace('Cell1_', '').replace('_cell1', ''), inplace=True)
    df_cell1 = df_cell1.dropna(subset=[c for c in df_cell1.columns if c != 'time'], how='all')
    
    df_cell2 = merged_df[cell2_cols].copy()
    df_cell2.rename(columns=lambda x: x.replace('Cell2_', '').replace('_cell2', ''), inplace=True)
    df_cell2 = df_cell2.dropna(subset=[c for c in df_cell2.columns if c != 'time'], how='all')
    
    return {'cell1': df_cell1, 'cell2': df_cell2}

def main():
    parser = argparse.ArgumentParser(description="Extract Cell-Level metrics from xApp outputs")
    parser.add_argument('--input-dir', type=str, 
                        default='xapp_template_outputs',
                        help='Input directory containing simulation outputs (default: xapp_template_outputs)')
    parser.add_argument('--output-dir', type=str,
                        default='ns3_sim_timeseries_data',
                        help='Output directory for CSV files (default: ns3_sim_timeseries_data)')
    parser.add_argument('--output-prefix', type=str,
                        default='cell_level_metrics',
                        help='Output CSV file prefix (default: cell_level_metrics)')
                        
    args = parser.parse_args()
    
    base_dir = Path(__file__).parent
    input_path = base_dir / args.input_dir
    output_dir = base_dir / args.output_dir
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
