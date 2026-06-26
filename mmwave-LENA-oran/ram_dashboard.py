#!/usr/bin/env python3
"""
RAM Usage Dashboard for NS-3 + xApp monitoring.
Launches xapp_template.py, tracks RAM of both the NS-3 process and the
xApp Python process for 50 wall-clock seconds, and shows a live curses
terminal dashboard. Saves results to ram_dashboard_results.csv.

Usage:
    cd mmwave-LENA-oran/
    python3 ram_dashboard.py

Press 'q' to quit early.
"""

import os
import sys
import csv
import time
import signal
import threading
import subprocess
import curses
from collections import deque

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
XAPP_SCRIPT   = os.path.join(SCRIPT_DIR, "xapp_template.py")
CSV_OUTPUT    = os.path.join(SCRIPT_DIR, "ram_dashboard_results.csv")
DURATION_S    = 50.0          # wall-clock seconds to monitor
SAMPLE_INTERVAL = 0.5         # seconds between samples
NS3_SEARCH_KEY  = "Differing_Power_Scenerio_HO"  # substring in NS-3 cmdline
NS3_FIND_TIMEOUT = 15.0       # seconds to wait for NS-3 to start
MAX_SAMPLES   = int(DURATION_S / SAMPLE_INTERVAL) + 10

# ---------------------------------------------------------------------------
# Low-level helpers — pure /proc, no psutil
# ---------------------------------------------------------------------------

def get_rss_mb(pid: int) -> float:
    """Read Resident Set Size from /proc/<pid>/status in MB."""
    try:
        with open(f"/proc/{pid}/status") as fh:
            for line in fh:
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024.0
    except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
        pass
    return 0.0


def pid_alive(pid: int) -> bool:
    """Return True if the process with given PID is still running."""
    try:
        os.kill(pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


def find_ns3_pid() -> int | None:
    """Scan /proc/*/cmdline for the NS-3 binary name."""
    try:
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/cmdline", "rb") as fh:
                    cmdline = fh.read().decode(errors="replace")
                if NS3_SEARCH_KEY in cmdline:
                    return int(entry)
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                pass
    except PermissionError:
        pass
    return None


def get_system_ram_mb() -> dict:
    """Read MemTotal and MemAvailable from /proc/meminfo, return MB values."""
    info = {"total": 0.0, "available": 0.0}
    try:
        with open("/proc/meminfo") as fh:
            for line in fh:
                if line.startswith("MemTotal:"):
                    info["total"] = int(line.split()[1]) / 1024.0
                elif line.startswith("MemAvailable:"):
                    info["available"] = int(line.split()[1]) / 1024.0
    except Exception:
        pass
    info["used"] = info["total"] - info["available"]
    return info


def growth_rate_mb_per_s(samples: list[tuple[float, float]]) -> float:
    """
    Linear regression slope (MB/s) over the last N (time, value) samples.
    Returns 0.0 if fewer than 2 samples.
    """
    n = len(samples)
    if n < 2:
        return 0.0
    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    x_mean = sum(xs) / n
    y_mean = sum(ys) / n
    num = sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys))
    den = sum((x - x_mean) ** 2 for x in xs)
    return num / den if den != 0 else 0.0


# ---------------------------------------------------------------------------
# Shared state (written by sampler thread, read by curses thread)
# ---------------------------------------------------------------------------

class State:
    def __init__(self):
        self.lock = threading.Lock()
        # Time-series: list of (wall_time, ns3_mb, xapp_mb, sys_free_mb)
        self.samples: list[tuple] = []
        self.ns3_pid: int | None  = None
        self.xapp_pid: int | None = None
        self.ns3_status: str      = "searching"
        self.xapp_status: str     = "starting"
        self.ns3_peak: float      = 0.0
        self.xapp_peak: float     = 0.0
        self.elapsed: float       = 0.0
        self.done: bool           = False          # signals both threads to stop


STATE = State()


# ---------------------------------------------------------------------------
# Sampler thread
# ---------------------------------------------------------------------------

def sampler_thread(xapp_proc: subprocess.Popen, start_time: float):
    """Runs in background: samples RAM every SAMPLE_INTERVAL seconds."""
    global STATE

    ns3_find_deadline = start_time + NS3_FIND_TIMEOUT
    ns3_pid = None
    xapp_pid = xapp_proc.pid

    while True:
        now = time.monotonic()
        elapsed = now - start_time

        # Stop condition
        if elapsed >= DURATION_S or STATE.done:
            with STATE.lock:
                STATE.done = True
            break

        # Try to find NS-3 PID if not yet found
        if ns3_pid is None:
            ns3_pid = find_ns3_pid()
            if ns3_pid:
                with STATE.lock:
                    STATE.ns3_pid = ns3_pid
                    STATE.ns3_status = "running"
            elif now > ns3_find_deadline:
                with STATE.lock:
                    STATE.ns3_status = "not found"

        # Read RAM
        ns3_mb   = get_rss_mb(ns3_pid)  if ns3_pid   else 0.0
        xapp_mb  = get_rss_mb(xapp_pid) if xapp_pid  else 0.0
        sys_info = get_system_ram_mb()

        # Determine live status
        ns3_alive  = (ns3_pid  is not None) and pid_alive(ns3_pid)
        xapp_alive = pid_alive(xapp_pid)

        ns3_st  = "running" if ns3_alive  else ("exited" if ns3_pid else "searching")
        xapp_st = "running" if xapp_alive else "exited"

        with STATE.lock:
            STATE.elapsed      = elapsed
            STATE.ns3_status   = ns3_st
            STATE.xapp_status  = xapp_st
            STATE.xapp_pid     = xapp_pid
            if ns3_mb  > STATE.ns3_peak:  STATE.ns3_peak  = ns3_mb
            if xapp_mb > STATE.xapp_peak: STATE.xapp_peak = xapp_mb
            STATE.samples.append((elapsed, ns3_mb, xapp_mb, sys_info["available"]))

        time.sleep(SAMPLE_INTERVAL)

    with STATE.lock:
        STATE.done = True


# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------

def flush_csv():
    """Write all collected samples to CSV after the run."""
    with STATE.lock:
        samples = list(STATE.samples)
    with open(CSV_OUTPUT, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(["wall_time_s", "ns3_ram_mb", "xapp_ram_mb",
                         "total_ram_mb", "sys_free_mb"])
        for (t, ns3, xapp, free) in samples:
            writer.writerow([f"{t:.2f}", f"{ns3:.1f}", f"{xapp:.1f}",
                             f"{ns3+xapp:.1f}", f"{free:.1f}"])


# ---------------------------------------------------------------------------
# Curses dashboard
# ---------------------------------------------------------------------------

C_NORMAL  = 0
C_GREEN   = 1
C_YELLOW  = 2
C_RED     = 3
C_CYAN    = 4
C_BOLD    = 5


def rate_color(rate_mb_s: float) -> int:
    if rate_mb_s >= 5.0:
        return C_RED
    if rate_mb_s >= 1.0:
        return C_YELLOW
    return C_GREEN


def draw_bar(win, y: int, x: int, width: int, fraction: float, char="█"):
    filled = max(0, min(width, int(fraction * width)))
    empty  = width - filled
    try:
        win.addstr(y, x, char * filled)
        win.addstr(y, x + filled, "░" * empty)
    except curses.error:
        pass


def draw_dashboard(stdscr):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(int(SAMPLE_INTERVAL * 1000))

    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(C_GREEN,  curses.COLOR_GREEN,  -1)
    curses.init_pair(C_YELLOW, curses.COLOR_YELLOW, -1)
    curses.init_pair(C_RED,    curses.COLOR_RED,    -1)
    curses.init_pair(C_CYAN,   curses.COLOR_CYAN,   -1)
    curses.init_pair(C_BOLD,   curses.COLOR_WHITE,  -1)

    while True:
        key = stdscr.getch()
        if key in (ord("q"), ord("Q")):
            with STATE.lock:
                STATE.done = True
            break

        with STATE.lock:
            samples   = list(STATE.samples)
            ns3_pid   = STATE.ns3_pid
            xapp_pid  = STATE.xapp_pid
            ns3_st    = STATE.ns3_status
            xapp_st   = STATE.xapp_status
            ns3_peak  = STATE.ns3_peak
            xapp_peak = STATE.xapp_peak
            elapsed   = STATE.elapsed
            done      = STATE.done

        if done and elapsed >= DURATION_S:
            break

        rows, cols = stdscr.getmaxyx()
        if rows < 20 or cols < 60:
            stdscr.clear()
            try:
                stdscr.addstr(0, 0, "Terminal too small — resize to at least 60x20")
            except curses.error:
                pass
            stdscr.refresh()
            continue

        stdscr.erase()

        # --- Header ---
        title = " RAM USAGE DASHBOARD  |  NS-3 + xApp  |  50s Run "
        try:
            stdscr.attron(curses.color_pair(C_CYAN) | curses.A_BOLD)
            stdscr.addstr(0, 0, "─" * cols)
            stdscr.addstr(0, max(0, (cols - len(title)) // 2), title)
            stdscr.attroff(curses.color_pair(C_CYAN) | curses.A_BOLD)
        except curses.error:
            pass

        # --- Latest values ---
        ns3_mb  = samples[-1][1] if samples else 0.0
        xapp_mb = samples[-1][2] if samples else 0.0
        sys_free = samples[-1][3] if samples else 0.0
        sys_info = get_system_ram_mb()
        ns3_delta  = ns3_mb  - samples[0][1] if samples else 0.0
        xapp_delta = xapp_mb - samples[0][2] if samples else 0.0

        # growth rate over last 10 samples
        tail = samples[-10:] if len(samples) >= 2 else samples
        ns3_rate  = growth_rate_mb_per_s([(s[0], s[1]) for s in tail])
        xapp_rate = growth_rate_mb_per_s([(s[0], s[2]) for s in tail])

        # --- Left panel: Processes ---
        row = 2
        try:
            stdscr.attron(curses.A_BOLD)
            stdscr.addstr(row, 2, "PROCESSES")
            stdscr.attroff(curses.A_BOLD)
        except curses.error:
            pass
        row += 1

        for label, mb, peak, delta, rate, st, pid in [
            ("NS-3 ", ns3_mb,  ns3_peak,  ns3_delta,  ns3_rate,  ns3_st,  ns3_pid),
            ("xApp ", xapp_mb, xapp_peak, xapp_delta, xapp_rate, xapp_st, xapp_pid),
        ]:
            st_color = C_GREEN if st == "running" else (C_RED if st in ("exited", "not found") else C_YELLOW)
            st_str   = f"[{st.upper()[:8]:8s}]"
            pid_str  = f"  PID {pid}" if pid else ""

            try:
                stdscr.addstr(row, 2, f"  {label}")
                stdscr.attron(curses.color_pair(st_color))
                stdscr.addstr(row, 9, st_str)
                stdscr.attroff(curses.color_pair(st_color))
                stdscr.addstr(row, 19, f"  RAM: {mb:>7.1f} MB{pid_str}")
            except curses.error:
                pass
            row += 1

            rate_col = rate_color(abs(rate))
            delta_sign = "+" if delta >= 0 else ""
            try:
                stdscr.addstr(row, 9, f"  Peak: {peak:>7.1f} MB")
                stdscr.addstr(row, 28, f"  Δ: {delta_sign}{delta:>+6.1f} MB")
                stdscr.addstr(row, 46, f"  Rate: ")
                stdscr.attron(curses.color_pair(rate_col))
                stdscr.addstr(row, 54, f"{rate:>+7.2f} MB/s")
                stdscr.attroff(curses.color_pair(rate_col))
            except curses.error:
                pass
            row += 2

        # --- Right panel: System RAM ---
        sys_col = cols // 2
        sys_total = sys_info["total"]
        sys_used  = sys_info["used"]
        sys_pct   = (sys_used / sys_total * 100.0) if sys_total > 0 else 0.0
        bar_w     = max(10, min(30, cols - sys_col - 20))

        try:
            stdscr.attron(curses.A_BOLD)
            stdscr.addstr(2, sys_col, "SYSTEM MEMORY")
            stdscr.attroff(curses.A_BOLD)
            stdscr.addstr(3, sys_col, f"  Total : {sys_total:>8.0f} MB")
            stdscr.addstr(4, sys_col, f"  Used  : ")
            bar_col = rate_color(sys_pct / 10.0)
            stdscr.attron(curses.color_pair(bar_col))
            draw_bar(stdscr, 4, sys_col + 10, bar_w, sys_used / sys_total if sys_total else 0)
            stdscr.attroff(curses.color_pair(bar_col))
            stdscr.addstr(4, sys_col + 10 + bar_w + 1, f"{sys_used:>7.0f} MB ({sys_pct:.1f}%)")
            stdscr.addstr(5, sys_col, f"  Free  : {sys_info['available']:>8.0f} MB")

            stdscr.addstr(7, sys_col, "GROWTH RATE (last 5s)")
            ns3_rc = rate_color(abs(ns3_rate))
            stdscr.addstr(8, sys_col, f"  NS-3 : ")
            stdscr.attron(curses.color_pair(ns3_rc))
            stdscr.addstr(8, sys_col + 9, f"{ns3_rate:>+8.2f} MB/s")
            stdscr.attroff(curses.color_pair(ns3_rc))
            xapp_rc = rate_color(abs(xapp_rate))
            stdscr.addstr(9, sys_col, f"  xApp : ")
            stdscr.attron(curses.color_pair(xapp_rc))
            stdscr.addstr(9, sys_col + 9, f"{xapp_rate:>+8.2f} MB/s")
            stdscr.attroff(curses.color_pair(xapp_rc))

            combined_rate = ns3_rate + xapp_rate
            combined_rc = rate_color(abs(combined_rate))
            stdscr.addstr(10, sys_col, f"  Total: ")
            stdscr.attron(curses.color_pair(combined_rc) | curses.A_BOLD)
            stdscr.addstr(10, sys_col + 9, f"{combined_rate:>+8.2f} MB/s")
            stdscr.attroff(curses.color_pair(combined_rc) | curses.A_BOLD)
        except curses.error:
            pass

        # --- Divider ---
        chart_top = 13
        try:
            stdscr.attron(curses.color_pair(C_CYAN))
            stdscr.addstr(chart_top, 0, "─" * cols)
            legend = "  RAM TIMELINE — NS-3 [█]  xApp [▒]  (MB)"
            stdscr.addstr(chart_top, 0, legend)
            stdscr.attroff(curses.color_pair(C_CYAN))
        except curses.error:
            pass

        # --- Chart ---
        chart_rows  = rows - chart_top - 4   # rows available for the chart body
        chart_start = chart_top + 1
        y_label_w   = 6                       # width of the Y-axis label column
        chart_w     = cols - y_label_w - 2   # usable columns

        if chart_rows >= 3 and chart_w >= 10 and samples:
            all_vals = [s[1] + s[2] for s in samples]
            max_val  = max(max(all_vals), 1.0)
            # round up to a nice ceiling
            ceil_val = max_val * 1.15

            # Y-axis labels (top, mid, 0)
            for offset, label_val in [
                (0,              ceil_val),
                (chart_rows // 2, ceil_val / 2),
                (chart_rows - 1, 0.0),
            ]:
                try:
                    stdscr.addstr(chart_start + offset, 0,
                                  f"{label_val:>5.0f}┤" if label_val > 0 else f"{'0':>5s}┼")
                except curses.error:
                    pass

            # Draw bars column by column (most-recent on the right)
            n_cols   = min(len(samples), chart_w)
            displayed = samples[-n_cols:]

            for col_i, (t, ns3, xapp, _) in enumerate(displayed):
                x_pos = y_label_w + col_i
                total = ns3 + xapp

                # NS-3 bar (solid block)
                ns3_frac  = ns3  / ceil_val if ceil_val > 0 else 0
                xapp_frac = xapp / ceil_val if ceil_val > 0 else 0

                ns3_rows  = int(ns3_frac  * chart_rows)
                xapp_rows = int(xapp_frac * chart_rows)

                for row_i in range(chart_rows):
                    chart_y = chart_start + chart_rows - 1 - row_i
                    char = " "
                    color = C_NORMAL
                    if row_i < ns3_rows:
                        char  = "█"
                        color = C_GREEN if ns3_rate < 1.0 else (C_YELLOW if ns3_rate < 5.0 else C_RED)
                    elif row_i < xapp_rows:
                        char  = "▒"
                        color = C_CYAN
                    try:
                        if color != C_NORMAL:
                            stdscr.attron(curses.color_pair(color))
                        stdscr.addstr(chart_y, x_pos, char)
                        if color != C_NORMAL:
                            stdscr.attroff(curses.color_pair(color))
                    except curses.error:
                        pass

            # X-axis time labels
            x_axis_y = chart_start + chart_rows
            try:
                stdscr.addstr(x_axis_y, y_label_w - 1, "└" + "─" * chart_w)
                # tick marks every ~10s
                tick_cols = int(10.0 / DURATION_S * chart_w)
                for t_mark in range(0, int(DURATION_S) + 1, 10):
                    tick_x = y_label_w + int(t_mark / DURATION_S * chart_w)
                    label  = f"{t_mark}s"
                    try:
                        stdscr.addstr(x_axis_y + 1, tick_x, label)
                    except curses.error:
                        pass
            except curses.error:
                pass

        # --- Status bar ---
        status_y = rows - 1
        n_samples = len(samples)
        pct = min(100.0, elapsed / DURATION_S * 100)
        bar_fill = int(pct / 100 * 20)
        prog_bar = "[" + "=" * bar_fill + ">" + " " * (20 - bar_fill) + "]"
        status = (f"  {prog_bar} {elapsed:>5.1f}s / {DURATION_S:.0f}s"
                  f"  │  Samples: {n_samples}"
                  f"  │  CSV: ram_dashboard_results.csv"
                  f"  │  q=quit")
        try:
            stdscr.attron(curses.color_pair(C_CYAN))
            stdscr.addstr(status_y, 0, "─" * cols)
            stdscr.addstr(status_y, 0, status[:cols - 1])
            stdscr.attroff(curses.color_pair(C_CYAN))
        except curses.error:
            pass

        stdscr.refresh()

    # Final state after loop
    with STATE.lock:
        STATE.done = True


# ---------------------------------------------------------------------------
# Fallback: plain-print mode (tiny terminal)
# ---------------------------------------------------------------------------

def plain_mode(xapp_proc: subprocess.Popen, start_time: float):
    """Simple scrolling output when curses is unavailable or terminal too small."""
    print(f"[RAM-DASH] Plain mode — monitoring for {DURATION_S:.0f}s")
    while True:
        with STATE.lock:
            samples = STATE.samples
            elapsed = STATE.elapsed
            ns3_st  = STATE.ns3_status
            xapp_st = STATE.xapp_status
            done    = STATE.done

        if samples:
            _, ns3, xapp, free = samples[-1]
            tail = samples[-10:]
            ns3_rate  = growth_rate_mb_per_s([(s[0], s[1]) for s in tail])
            xapp_rate = growth_rate_mb_per_s([(s[0], s[2]) for s in tail])
            print(f"[{elapsed:>5.1f}s] NS-3={ns3:>7.1f}MB ({ns3_rate:>+6.2f}MB/s) [{ns3_st}]"
                  f"  xApp={xapp:>6.1f}MB ({xapp_rate:>+6.2f}MB/s) [{xapp_st}]"
                  f"  SysFree={free:>7.1f}MB")

        if done or elapsed >= DURATION_S:
            break
        time.sleep(SAMPLE_INTERVAL)


# ---------------------------------------------------------------------------
# Cleanup
# ---------------------------------------------------------------------------

def terminate_subprocess(proc: subprocess.Popen):
    """Kill the xapp subprocess and its process group."""
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        time.sleep(1.0)
        if proc.poll() is None:
            os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError, OSError):
        try:
            proc.terminate()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    if not os.path.exists(XAPP_SCRIPT):
        print(f"ERROR: xapp_template.py not found at {XAPP_SCRIPT}")
        sys.exit(1)

    print(f"[RAM-DASH] Launching xapp_template.py ...")
    xapp_proc = subprocess.Popen(
        [sys.executable, XAPP_SCRIPT],
        cwd=SCRIPT_DIR,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,   # new process group for clean kill
    )
    print(f"[RAM-DASH] xApp PID = {xapp_proc.pid}")

    start_time = time.monotonic()

    # Start sampler in background
    sampler = threading.Thread(
        target=sampler_thread,
        args=(xapp_proc, start_time),
        daemon=True,
    )
    sampler.start()

    # Wait briefly for NS-3 to start before opening curses
    time.sleep(2.0)

    # Launch curses dashboard (or fallback)
    try:
        curses.wrapper(draw_dashboard)
    except Exception as exc:
        print(f"[RAM-DASH] curses unavailable ({exc}), switching to plain mode.")
        plain_mode(xapp_proc, start_time)

    # Signal sampler to stop and wait
    with STATE.lock:
        STATE.done = True
    sampler.join(timeout=3.0)

    # Terminate the xapp + NS-3 process tree
    if xapp_proc.poll() is None:
        print("\n[RAM-DASH] Stopping xapp_template.py ...")
        terminate_subprocess(xapp_proc)

    # Write CSV
    flush_csv()
    print(f"[RAM-DASH] Results saved to: {CSV_OUTPUT}")

    # Print summary
    with STATE.lock:
        samples = STATE.samples
        ns3_peak  = STATE.ns3_peak
        xapp_peak = STATE.xapp_peak

    if samples:
        total_elapsed = samples[-1][0]
        ns3_start  = samples[0][1]
        xapp_start = samples[0][2]
        ns3_end    = samples[-1][1]
        xapp_end   = samples[-1][2]
        rate_ns3   = growth_rate_mb_per_s([(s[0], s[1]) for s in samples])
        rate_xapp  = growth_rate_mb_per_s([(s[0], s[2]) for s in samples])

        print("\n" + "=" * 60)
        print("  RAM DASHBOARD — FINAL SUMMARY")
        print("=" * 60)
        print(f"  Duration     : {total_elapsed:.1f}s  ({len(samples)} samples)")
        print(f"  NS-3  start  : {ns3_start:>8.1f} MB")
        print(f"  NS-3  end    : {ns3_end:>8.1f} MB   peak: {ns3_peak:.1f} MB")
        print(f"  NS-3  growth : {rate_ns3:>+8.3f} MB/s  (full-run avg)")
        print(f"  xApp  start  : {xapp_start:>8.1f} MB")
        print(f"  xApp  end    : {xapp_end:>8.1f} MB   peak: {xapp_peak:.1f} MB")
        print(f"  xApp  growth : {rate_xapp:>+8.3f} MB/s  (full-run avg)")
        print("=" * 60)
        if abs(rate_ns3) < 1.0:
            print("  VERDICT: RAM stable — leak fix appears effective.")
        elif rate_ns3 >= 5.0:
            print("  VERDICT: LEAK DETECTED — NS-3 RAM growing fast!")
        else:
            print("  VERDICT: Slow growth — monitor further.")
        print("=" * 60)


if __name__ == "__main__":
    main()
