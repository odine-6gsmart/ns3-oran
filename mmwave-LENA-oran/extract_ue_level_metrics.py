#!/usr/bin/env python3
"""
Extract UE-level metrics from NS-3 O-RAN simulation output files.
Specifically designed to process the outputs of xapp_template.py.
Creates per-cell CSV files with UE-level SINR, throughput, latency, and location.
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

def sinr_bin_index_to_dB(bin_index: float) -> float:
    return (bin_index / 2.0) - 23.0

def load_ue_positions(path):
    if not os.path.exists(path):
        return None
    try:
        df = pd.read_csv(path)
        df = df[df['Type'] == 'UE'].copy()
        df.rename(columns={'Time(s)': 'time'}, inplace=True)
        df.rename(columns={'ID': 'ue_id'}, inplace=True)
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
        bin_indices = [b for b, c in bin_col_pairs]
        sinr_cols_ueid = [c for b, c in bin_col_pairs]
        centers_db = np.array([sinr_bin_index_to_dB(x) for x in bin_indices], dtype=float)
        
        if 'ueImsiComplete' not in df.columns:
            return None
            
        df['ueImsiComplete'] = df['ueImsiComplete'].astype(str).str.strip().str.lstrip("0").replace("", "0").astype(int)
        
        ue_sinr_data = []
        for idx, row in df.iterrows():
            time_val = row['time']
            ue_id = row['ueImsiComplete']
            
            counts = np.array([row.get(col, 0) for col in sinr_cols_ueid], dtype=float)
            counts[np.isnan(counts)] = 0.0
            total = counts.sum()
            
            if total > 0:
                avg_sinr_db = (counts * centers_db).sum() / total
            else:
                avg_sinr_db = np.nan
                
            ue_sinr_data.append({
                'time': time_val,
                'ue_id': ue_id,
                'sinr_db': avg_sinr_db
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
        df['throughput_kbps'] = (df['RxBytes'] * 8 / 1000) / df['window']
        df['latency_ms'] = df['delay'] * 1000
        df.rename(columns={'IMSI': 'ue_id'}, inplace=True)
        
        return df[['time', 'ue_id', 'throughput_kbps', 'latency_ms']]
    except Exception as e:
        print(f"Error loading RLC stats from {path}: {e}")
        return None

def extract_metrics_from_dir(input_dir):
    run_path = Path(input_dir)
    print(f"Extracting metrics from: {run_path}")
    
    ue_positions = load_ue_positions(run_path / "UEPosition.txt")
    
    metrics_dict = {}
    
    for cell_id in [1, 2]:
        du_metrics = load_du_ue_metrics(run_path / f"du-cell-{cell_id}.txt", cell_id)
        rlc_metrics = load_rlc_ue_metrics(run_path / "DlE2RlcStats.txt", cell_id)
        
        if ue_positions is not None:
            cell_pos = ue_positions[ue_positions['CellID'] == cell_id].copy()
            merged_df = cell_pos.copy()
        else:
            continue
            
        if du_metrics is not None:
            merged_df = merged_df.merge(du_metrics, on=['time', 'ue_id'], how='left')
        else:
            merged_df['sinr_db'] = np.nan
            
        if rlc_metrics is not None:
            merged_df = merged_df.merge(rlc_metrics, on=['time', 'ue_id'], how='left')
        else:
            merged_df['throughput_kbps'] = np.nan
            merged_df['latency_ms'] = np.nan
            
        final_cols = ['time', 'ue_id', 'sinr_db', 'throughput_kbps', 'latency_ms', 'X(m)', 'Y(m)', 'Z(m)']
        available_cols = [col for col in final_cols if col in merged_df.columns]
        merged_df = merged_df[available_cols]
        merged_df = merged_df.sort_values(['time', 'ue_id']).reset_index(drop=True)
        
        metrics_dict[f"cell{cell_id}"] = merged_df
        
    return metrics_dict

def main():
    parser = argparse.ArgumentParser(description="Extract UE-Level metrics from xApp outputs")
    parser.add_argument('--input-dir', type=str, 
                        default='xapp_template_outputs',
                        help='Input directory containing simulation outputs (default: xapp_template_outputs)')
    parser.add_argument('--output-dir', type=str,
                        default='ns3_sim_timeseries_data',
                        help='Output directory for CSV files (default: ns3_sim_timeseries_data)')
    parser.add_argument('--output-prefix', type=str,
                        default='ue_level_metrics',
                        help='Output CSV file prefix (default: ue_level_metrics)')
                        
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
            print(f"Successfully generated {output_file} ({len(df)} records)")
        else:
            print(f"No data found for {cell_name}.")

if __name__ == "__main__":
    main()
