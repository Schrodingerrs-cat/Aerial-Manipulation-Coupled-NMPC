#!/usr/bin/env bash
#SBATCH -N 1
#SBATCH -c 20
#SBATCH -t 4:00:00
#SBATCH --mem 16G
#SBATCH --job-name="am3_tail_eval"
#SBATCH -o am3tail_%j.out
#SBATCH -e am3tail_%j.err
#SBATCH -p academic
# Tail study RL evaluation on the square route (Code/scripts/am3_tail_rl_eval.py).  Submit from the repo root:
#   NAME=<stamp>_tail_rl_T2 MODELS="path/model_final.zip:free path/model_final.zip:rigid" sbatch slurm/am3_tail_eval_turing.sh
set -uo pipefail
REPO=$(cd "${SLURM_SUBMIT_DIR:-$(dirname "$0")/..}" && pwd); cd "$REPO/Code" || exit 1
type module >/dev/null 2>&1 || { source /etc/profile.d/modules.sh 2>/dev/null; source /etc/profile.d/30-set_modulepath.sh 2>/dev/null; }
module load slurm
eval "$($HOME/miniconda3/bin/conda shell.bash hook)"; conda activate am3rl
unset MUJOCO_GL; export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=.:scripts
FL=acados_job${SLURM_JOB_ID:-local}; ln -sfn acados third_party/$FL; export OCRL_ACADOS=$FL          # per-job solver build, as am3_bench_turing.sh
echo "job ${SLURM_JOB_ID:-local} on $(hostname) at $(date)   $NAME   $MODELS"
python scripts/am3_tail_rl_eval.py "$NAME" $MODELS --T 2.0 1.5 1.2 1.0 0.9 --mb 0 0.15 0.2 0.25 0.3 --seeds 3 --procs 20
rm -f third_party/$FL; echo "finished $(date)"
