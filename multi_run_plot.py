#!/usr/bin/env python3
"""
multi_run_plot.py
Reads:
  ~/results/phaqm/run_N/queue-length.csv   cols: time, queue
  ~/results/mpc/run_N/mpc-results.csv      cols: time_s, queue_pkts, drop_prob, ...
Output:
  ~/results/multi_run_comparison.pdf / .png
"""
import csv, math, os, glob, statistics
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np

HOME      = os.path.expanduser("~")
RESULTS   = os.path.join(HOME, "results_ieee")
QREF      = 500.0
TRANSIENT = 30.0
T_END     = 120.0
STEP      = 0.5   # resample grid (s)
WIN       = 6     # moving avg window on resampled grid

# ── Helpers ──────────────────────────────────────────────────

def resample(t_raw, y_raw, t_grid):
    out, j = [], 0
    for tg in t_grid:
        while j < len(t_raw)-1 and t_raw[j+1] <= tg:
            j += 1
        out.append(y_raw[j])
    return out

def load_queue_csv(path):
    """queue-length.csv: col0=time, col1=queue"""
    t, q = [], []
    with open(path) as f:
        next(f)
        for line in f:
            p = line.strip().split(',')
            if len(p) >= 2:
                try: t.append(float(p[0])); q.append(float(p[1]))
                except: pass
    return t, q

def load_mpc_results_csv(path):
    """mpc-results.csv: header time_s,queue_pkts,drop_prob,..."""
    t, q = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t.append(float(row['time_s']))
                q.append(float(row['queue_pkts']))
            except: pass
    return t, q

def moving_avg(q, win=WIN):
    return [statistics.mean(q[max(0,i-win):i+1]) for i in range(len(q))]

def perf_stats(t_grid, q_res):
    q_ss = [v for tv,v in zip(t_grid,q_res) if tv >= TRANSIENT]
    if not q_ss: return None
    mean = statistics.mean(q_ss)
    std  = statistics.stdev(q_ss) if len(q_ss)>1 else 0
    rmse = math.sqrt(statistics.mean([(v-QREF)**2 for v in q_ss]))
    mae  = statistics.mean([abs(v-QREF) for v in q_ss])
    return {'mean':mean,'std':std,'rmse':rmse,'mae':mae,'offset':mean-QREF}

def mean_std_bands(all_runs):
    if not all_runs: return np.array([]), np.array([]), np.array([])
    arr  = np.array(all_runs)
    mean = arr.mean(axis=0)
    std  = arr.std(axis=0)
    return mean, mean-std, mean+std

def all_stats(all_runs, t_grid):
    stats = [perf_stats(t_grid, r) for r in all_runs]
    stats = [s for s in stats if s]
    if not stats: return {}
    return {
        'mean_mean':   statistics.mean([s['mean']   for s in stats]),
        'mean_rmse':   statistics.mean([s['rmse']   for s in stats]),
        'mean_std':    statistics.mean([s['std']    for s in stats]),
        'mean_mae':    statistics.mean([s['mae']    for s in stats]),
        'mean_offset': statistics.mean([s['offset'] for s in stats]),
        'std_rmse':    statistics.stdev([s['rmse']  for s in stats]) if len(stats)>1 else 0,
        'nruns': len(stats),
        'all_rmses': [s['rmse'] for s in stats]
    }

# ── Load data ─────────────────────────────────────────────────

t_grid = list(np.arange(0, T_END+STEP, STEP))

print("Loading results...")

# PHAQM — queue-length.csv (time, queue)
phaqm_files = sorted(glob.glob(os.path.join(RESULTS,"phaqm","run_*","queue-length.csv")))
print(f"  PHAQM: found {len(phaqm_files)} runs")
phaqm_runs = []
for f in phaqm_files:
    t_r, q_r = load_queue_csv(f)
    if t_r:
        phaqm_runs.append(resample(t_r, q_r, t_grid))

# MPC — mpc-results.csv (time_s, queue_pkts, drop_prob, ...)
mpc_files = sorted(glob.glob(os.path.join(RESULTS,"mpc","run_*","mpc-results.csv")))
print(f"  MPC  : found {len(mpc_files)} runs")
mpc_runs = []
for f in mpc_files:
    t_r, q_r = load_mpc_results_csv(f)
    if t_r:
        mpc_runs.append(resample(t_r, q_r, t_grid))

if not phaqm_runs: print("ERROR: No PHAQM data found"); exit(1)
if not mpc_runs:   print("ERROR: No MPC data found");   exit(1)

# ── Compute bands ─────────────────────────────────────────────

p_mean, p_lo, p_hi = mean_std_bands(phaqm_runs)
m_mean, m_lo, m_hi = mean_std_bands(mpc_runs)
p_mavg = moving_avg(list(p_mean))
m_mavg = moving_avg(list(m_mean))

ps = all_stats(phaqm_runs, t_grid)
ms = all_stats(mpc_runs,   t_grid)

nruns_p = len(phaqm_runs)
nruns_m = len(mpc_runs)
improvement = ps['mean_rmse'] / ms['mean_rmse'] if ms['mean_rmse'] > 0 else 0

# ── Print table ───────────────────────────────────────────────
print(f"\n{'═'*72}")
print(f"  {'Metric':<35} {'PHAQM':>15}  {'Hybrid MPC':>15}")
print(f"{'═'*72}")
print(f"  {'Runs completed':<35} {ps['nruns']:>15}  {ms['nruns']:>15}")
print(f"  {'Mean queue (pkts)':<35} {ps['mean_mean']:>15.1f}  {ms['mean_mean']:>15.1f}")
print(f"  {'Mean σ per run (pkts)':<35} {ps['mean_std']:>15.1f}  {ms['mean_std']:>15.1f}")
print(f"  {'Mean RMSE ± std (pkts)':<35} {ps['mean_rmse']:>7.1f}±{ps['std_rmse']:<7.1f}  {ms['mean_rmse']:>7.1f}±{ms['std_rmse']:<7.1f}")
print(f"  {'Mean MAE (pkts)':<35} {ps['mean_mae']:>15.1f}  {ms['mean_mae']:>15.1f}")
print(f"  {'Mean offset from qRef':<35} {ps['mean_offset']:>+15.1f}  {ms['mean_offset']:>+15.1f}")
print(f"  {'RMSE improvement':<35} {'—':>15}  {improvement:>14.1f}×")
print(f"{'═'*72}\n")

# ── Figure ────────────────────────────────────────────────────
fig = plt.figure(figsize=(16, 13))
fig.patch.set_facecolor('white')
fig.suptitle(
    f"AQM Multi-Run Statistical Comparison  —  {max(nruns_p,nruns_m)} Independent Runs per Controller\n"
    "200 Flows  (120 BulkTCP + 60 WebTCP + 20 UDP)  ·  45 Mbps Bottleneck  ·  Access: 5/20/50ms (ITU-T G.114)  ·  Backbone: 50ms",
    fontsize=13, fontweight='bold', y=0.99)

gs = gridspec.GridSpec(3, 2, figure=fig,
                       hspace=0.48, wspace=0.28,
                       top=0.91, bottom=0.06, left=0.07, right=0.97)

ax_all = fig.add_subplot(gs[0, :])   # top full-width
ax_pl  = fig.add_subplot(gs[1, 0])   # PHAQM all runs
ax_ml  = fig.add_subplot(gs[1, 1])   # MPC all runs
ax_box = fig.add_subplot(gs[2, 0])   # RMSE boxplot
ax_tab = fig.add_subplot(gs[2, 1])   # stats table

XLIM = (0, T_END)

def shade_bands(ax):
    ax.fill_between([0, TRANSIENT], 0, 1400,
                    alpha=0.06, color='silver', label='Startup transient (0–30s)')
    ax.fill_between(t_grid, [QREF*0.90]*len(t_grid), [QREF*1.10]*len(t_grid),
                    alpha=0.13, color='limegreen', label='±10% target band')
    ax.axhline(QREF, color='crimson', lw=2.0, ls='--', zorder=4,
               label=f'qRef = {QREF:.0f}')

# ── Panel 1: Overlaid mean curves ────────────────────────────
shade_bands(ax_all)
ax_all.fill_between(t_grid, p_lo, p_hi, alpha=0.22, color='tomato')
ax_all.plot(t_grid, p_mean, color='firebrick', lw=2.5,
            label=f'PHAQM mean (n={nruns_p})  RMSE={ps["mean_rmse"]:.0f} pkts')
ax_all.fill_between(t_grid, m_lo, m_hi, alpha=0.22, color='steelblue')
ax_all.plot(t_grid, m_mean, color='navy', lw=2.5,
            label=f'Hybrid MPC mean (n={nruns_m})  RMSE={ms["mean_rmse"]:.0f} pkts')

# Annotations
ax_all.annotate(f'PHAQM: high variance\nRMSE={ps["mean_rmse"]:.0f} pkts',
                xy=(80, float(p_mean[int(80/STEP)])),
                xytext=(60, 800),
                arrowprops=dict(arrowstyle='->', color='firebrick', lw=1.5),
                fontsize=10, color='firebrick',
                bbox=dict(boxstyle='round,pad=0.3', fc='mistyrose', ec='firebrick', alpha=0.9))
ax_all.annotate(f'Hybrid MPC: stable\nRMSE={ms["mean_rmse"]:.0f} pkts  ({improvement:.0f}× better)',
                xy=(80, float(m_mean[int(80/STEP)])),
                xytext=(55, 280),
                arrowprops=dict(arrowstyle='->', color='navy', lw=1.5),
                fontsize=10, color='navy',
                bbox=dict(boxstyle='round,pad=0.3', fc='aliceblue', ec='navy', alpha=0.9))

ax_all.set_title("Mean Queue Occupancy  (shaded band = ±1σ across runs)",
                 fontsize=11, fontweight='bold')
ax_all.set_ylabel("Queue (packets)", fontsize=10)
ax_all.set_ylim(0, 1100)
ax_all.set_xlim(*XLIM)
ax_all.legend(fontsize=9, framealpha=0.92, loc='upper right', ncol=2)
ax_all.grid(True, alpha=0.30, ls='--')
ax_all.set_xticklabels([])

# ── Panel 2: PHAQM individual runs ───────────────────────────
shade_bands(ax_pl)
for k, run in enumerate(phaqm_runs):
    ax_pl.plot(t_grid, run, color='tomato', lw=0.8, alpha=0.40,
               label='Individual run' if k==0 else '')
ax_pl.plot(t_grid, p_mean, color='firebrick', lw=2.5, label=f'Mean (n={nruns_p})')
ax_pl.fill_between(t_grid, p_lo, p_hi, alpha=0.25, color='firebrick', label='±1σ band')
ax_pl.set_title(f"PHAQM — All {nruns_p} Runs  [RMSE={ps['mean_rmse']:.0f}±{ps['std_rmse']:.0f}]",
                fontsize=10, color='firebrick')
ax_pl.set_ylabel("Queue (packets)", fontsize=10)
ax_pl.set_xlabel("Simulation time (s)", fontsize=10)
ax_pl.set_ylim(0, 1100)
ax_pl.set_xlim(*XLIM)
ax_pl.legend(fontsize=8, framealpha=0.9)
ax_pl.grid(True, alpha=0.30, ls='--')

# ── Panel 3: MPC individual runs ─────────────────────────────
shade_bands(ax_ml)
for k, run in enumerate(mpc_runs):
    ax_ml.plot(t_grid, run, color='steelblue', lw=0.8, alpha=0.40,
               label='Individual run' if k==0 else '')
ax_ml.plot(t_grid, m_mean, color='navy', lw=2.5, label=f'Mean (n={nruns_m})')
ax_ml.fill_between(t_grid, m_lo, m_hi, alpha=0.25, color='navy', label='±1σ band')
ax_ml.set_title(f"Hybrid MPC+PID+RLS — All {nruns_m} Runs  [RMSE={ms['mean_rmse']:.0f}±{ms['std_rmse']:.0f}]",
                fontsize=10, color='navy')
ax_ml.set_ylabel("Queue (packets)", fontsize=10)
ax_ml.set_xlabel("Simulation time (s)", fontsize=10)
ax_ml.set_ylim(0, 1100)
ax_ml.set_xlim(*XLIM)
ax_ml.legend(fontsize=8, framealpha=0.9)
ax_ml.grid(True, alpha=0.30, ls='--')

# ── Panel 4: RMSE boxplot ─────────────────────────────────────
p_rmses = ps.get('all_rmses', [])
m_rmses = ms.get('all_rmses', [])
if p_rmses and m_rmses:
    bp = ax_box.boxplot([p_rmses, m_rmses],
                        labels=['PHAQM', 'Hybrid MPC'],
                        patch_artist=True, widths=0.5,
                        medianprops=dict(color='white', lw=2.5),
                        whiskerprops=dict(lw=1.5),
                        capprops=dict(lw=1.5))
    bp['boxes'][0].set_facecolor('tomato')
    bp['boxes'][1].set_facecolor('steelblue')
    # Individual points
    for i, (vals, xpos, col) in enumerate([(p_rmses,1,'firebrick'),(m_rmses,2,'navy')]):
        ax_box.scatter([xpos]*len(vals), vals, color=col, zorder=5,
                       s=60, alpha=0.8)
    ax_box.set_ylabel("RMSE (packets)", fontsize=10)
    ax_box.set_title("RMSE Distribution Across 5 Runs", fontsize=10, fontweight='bold')
    ax_box.grid(True, alpha=0.30, ls='--', axis='y')
    ymax = max(max(p_rmses), max(m_rmses))
    ax_box.text(1.5, ymax * 0.55,
                f"{improvement:.1f}× better",
                ha='center', fontsize=14, fontweight='bold', color='navy',
                bbox=dict(boxstyle='round', fc='aliceblue', ec='navy', lw=1.5))

# ── Panel 5: Stats table ──────────────────────────────────────
ax_tab.axis('off')
rows = [
    ["Metric",                 "PHAQM",                              "Hybrid MPC"],
    ["Runs (n)",               str(ps['nruns']),                     str(ms['nruns'])],
    ["Mean queue (pkts)",      f"{ps['mean_mean']:.1f}",             f"{ms['mean_mean']:.1f}"],
    ["Mean RMSE (pkts)",       f"{ps['mean_rmse']:.1f}±{ps['std_rmse']:.1f}",
                                                                     f"{ms['mean_rmse']:.1f}±{ms['std_rmse']:.1f}"],
    ["Mean MAE (pkts)",        f"{ps['mean_mae']:.1f}",              f"{ms['mean_mae']:.1f}"],
    ["Mean σ (pkts)",          f"{ps['mean_std']:.1f}",              f"{ms['mean_std']:.1f}"],
    ["Offset from qRef",       f"{ps['mean_offset']:+.1f}",          f"{ms['mean_offset']:+.1f}"],
    ["RMSE improvement",       "baseline",                           f"{improvement:.1f}×  ✓"],
]
table = ax_tab.table(cellText=rows[1:], colLabels=rows[0],
                     cellLoc='center', loc='center',
                     bbox=[0.0, 0.0, 1.0, 1.0])
table.auto_set_font_size(False)
table.set_fontsize(10)
for (r,c), cell in table.get_celld().items():
    if r == 0:
        cell.set_facecolor('#2c3e50')
        cell.set_text_props(color='white', fontweight='bold')
    elif r % 2 == 0:
        cell.set_facecolor('#f5f5f5')
    else:
        cell.set_facecolor('white')
    if r > 0 and c == 1:
        cell.set_facecolor('#ffe8e8')
    if r > 0 and c == 2:
        cell.set_facecolor('#e8f0ff')
    cell.set_edgecolor('#cccccc')
ax_tab.set_title("Statistical Summary (steady-state t≥30s)", fontsize=10, fontweight='bold')

# ── Save ──────────────────────────────────────────────────────
os.makedirs(RESULTS, exist_ok=True)
out_pdf = os.path.join(RESULTS, "multi_run_comparison.pdf")
out_png = os.path.join(RESULTS, "multi_run_comparison.png")
plt.savefig(out_pdf, bbox_inches='tight', facecolor='white')
plt.savefig(out_png, dpi=300, bbox_inches='tight', facecolor='white')
print(f"Saved → {out_pdf}")
print(f"Saved → {out_png}")
