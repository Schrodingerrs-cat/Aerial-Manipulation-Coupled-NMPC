#!/usr/bin/env bash
#SBATCH -N 1
#SBATCH -c 32
#SBATCH -t 12:00:00
#SBATCH --mem 32G
#SBATCH --job-name="am3_bench"
#SBATCH -o am3bench_%j.out
#SBATCH -e am3bench_%j.err
#SBATCH -p academic

# Writing benchmark (am3_write_bench.py), every mismatch case in parallel, PROCS processes per case (one core each:
# sharing cores slows acados and trips the episode's 400 s wall-clock guard, found 2026-11-12).
# Submit from the repo root:  CTRL=ND:10,10 NAME=ND [NOISE=real|lab|nominal] [FORCE=0.6] sbatch slurm/am3_bench_turing.sh   (FORCE: force-controlled writing)
# Results: Code/results/am3_write/bench/<NAME>_real[_case].{log,json}
set -uo pipefail
CTRL=${CTRL:-ND:10,10}; NAME=${NAME:-ND}; PROCS=${PROCS:-3}; NOISE=${NOISE:-real}; FORCE=${FORCE:-}
CASES=${CASES:-"nominal payload100g payload200g motor_slow damping3x kf90 gust_y wind_sine"}
REPO=$(cd "${SLURM_SUBMIT_DIR:-$(dirname "$0")/..}" && pwd); cd "$REPO/Code" || exit 1
type module >/dev/null 2>&1 || { source /etc/profile.d/modules.sh 2>/dev/null; source /etc/profile.d/30-set_modulepath.sh 2>/dev/null; }
module load slurm; eval "$($HOME/miniconda3/bin/conda shell.bash hook)"; conda activate am3rl || exit 1
unset MUJOCO_GL; export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=.:scripts
FL=acados_job${SLURM_JOB_ID:-local}; ln -sfn acados third_party/$FL; export OCRL_ACADOS=$FL          # per-job solver build (-march=native), see am3_hybrid_turing.sh
echo "job ${SLURM_JOB_ID:-local} on $(hostname) at $(date)   ctrl $CTRL   name $NAME   procs/case $PROCS"

# build the solver once, before the parallel runs
python -c "from stable_baselines3 import PPO; import sys; sys.path.insert(0,'scripts'); from am3_write_eval import make; make('$CTRL'); print('solver built')" 2>&1 | tail -1

mkdir -p results/am3_write/bench
for c in $CASES; do
  if [ "$c" = nominal ]; then arg=""; log="${NAME}_${NOISE}"; else arg="--case $c"; log="${NAME}_${NOISE}_$c"; fi
  python scripts/am3_write_bench.py "$CTRL" --noise "$NOISE" --procs "$PROCS" $arg ${FORCE:+--force $FORCE} --name "$NAME" > "results/am3_write/bench/$log.log" 2>&1 &
done
wait
rm -f third_party/$FL
echo "finished $(date)"
for f in results/am3_write/bench/${NAME}_${NOISE}*.log; do echo "$f  $(grep SUMMARY "$f" | cut -c9-220)"; done
