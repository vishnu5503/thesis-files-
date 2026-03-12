#!/bin/bash
# =============================================================================
# run_experiments.sh
# Runs PHAQM and Hybrid MPC simulations multiple times with different seeds
# Enables pcap capture on bottleneck for Wireshark analysis
#
# Usage:
#   chmod +x run_experiments.sh
#   ./run_experiments.sh
#
# Output folders:
#   ~/results/phaqm/run_1/   ~/results/phaqm/run_2/  ...
#   ~/results/mpc/run_1/     ~/results/mpc/run_2/    ...
#   ~/results/pcap/          -- all .pcap files (open in Wireshark)
# =============================================================================

NS3_DIR="$HOME/ns-3.45"
RESULTS_DIR="$HOME/results_ieee"
NRUNS=5          # number of independent runs per controller
SIM_TIME=120     # seconds per run

mkdir -p "$RESULTS_DIR/phaqm"
mkdir -p "$RESULTS_DIR/mpc"
mkdir -p "$RESULTS_DIR/pcap"

echo "=============================================="
echo "  AQM Multi-Run Experiment"
echo "  Runs per controller : $NRUNS"
echo "  Simulation time     : ${SIM_TIME}s each"
echo "  Results dir         : $RESULTS_DIR"
echo "=============================================="

cd "$NS3_DIR"

# ── Build once ────────────────────────────────────────────────
echo ""
echo "[BUILD] Building NS-3..."
./ns3 build 2>&1 | tail -3
if [ $? -ne 0 ]; then echo "BUILD FAILED"; exit 1; fi
echo "[BUILD] Done."

# ── Run PHAQM ─────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Running PHAQM  ($NRUNS runs)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

for RUN in $(seq 1 $NRUNS); do
    OUTDIR="$RESULTS_DIR/phaqm/run_$RUN"
    mkdir -p "$OUTDIR"
    echo ""
    echo "  [PHAQM Run $RUN/$NRUNS] seed=$RUN ..."

    ./ns3 run "scratch/phaqm-dumbbell \
        --simTime=$SIM_TIME \
        --seed=$RUN \
        --outDir=$OUTDIR \
        --enablePcap=true" 2>&1 | grep -E "PHAQM|Done|Error|Assert"

    # Move output files into run folder
    [ -f "$HOME/phaqm-queue-length.csv" ] && \
        mv "$HOME/phaqm-queue-length.csv" "$OUTDIR/queue-length.csv"
    [ -f "$HOME/phaqm-results.csv" ] && \
        mv "$HOME/phaqm-results.csv"      "$OUTDIR/drop-prob.csv"

    # Move pcap files
    find "$NS3_DIR" -name "phaqm-bottleneck*.pcap" 2>/dev/null | while read f; do
        mv "$f" "$RESULTS_DIR/pcap/phaqm_run${RUN}_$(basename $f)"
    done

    echo "  [PHAQM Run $RUN] → $OUTDIR"
done

# ── Run MPC ───────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Running Hybrid MPC  ($NRUNS runs)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

for RUN in $(seq 1 $NRUNS); do
    OUTDIR="$RESULTS_DIR/mpc/run_$RUN"
    mkdir -p "$OUTDIR"
    echo ""
    echo "  [MPC Run $RUN/$NRUNS] seed=$RUN ..."

    ./ns3 run "scratch/hybrid-mpc-dumbbell \
        --simTime=$SIM_TIME \
        --seed=$RUN \
        --enablePcap=true" 2>&1 | grep -E "MPC|Flows|Done|Error|Assert"

    # Move output files
    [ -f "$HOME/mpc-hybrid-results.csv" ] && \
        mv "$HOME/mpc-hybrid-results.csv" "$OUTDIR/mpc-results.csv"

    # Move pcap files
    find "$NS3_DIR" -name "mpc-bottleneck*.pcap" 2>/dev/null | while read f; do
        mv "$f" "$RESULTS_DIR/pcap/mpc_run${RUN}_$(basename $f)"
    done

    echo "  [MPC Run $RUN] → $OUTDIR"
done

# ── Summary ───────────────────────────────────────────────────
echo ""
echo "=============================================="
echo "  All runs complete!"
echo ""
echo "  CSV results:"
ls "$RESULTS_DIR/phaqm/"*/queue-length.csv 2>/dev/null | head -5
ls "$RESULTS_DIR/mpc/"*/mpc-results.csv 2>/dev/null | head -5
echo ""
echo "  PCAP files for Wireshark:"
ls "$RESULTS_DIR/pcap/"*.pcap 2>/dev/null | head -10
echo ""
echo "  Next step:"
echo "    python3 ~/multi_run_plot.py"
echo "=============================================="
