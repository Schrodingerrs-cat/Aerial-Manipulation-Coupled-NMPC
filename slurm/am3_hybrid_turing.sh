#!/usr/bin/env bash
#SBATCH -N 1
#SBATCH -c 32
#SBATCH -t 24:00:00
#SBATCH --mem 48G
#SBATCH --job-name="am3_L"
#SBATCH -o am3L_%j.out
#SBATCH -e am3L_%j.err
#SBATCH -p academic
#SBATCH --signal=B:TERM@120
# NMPC + learned model correction L (Code/scripts/am3_train_hybrid.py), 2026-11-19.  CPU only; needs acados (Code/third_party/acados).
# Submit from the repo root:  TAG=2026-11-19_1200_L2_hybrid_s0 NCH=2 sbatch slurm/am3_hybrid_turing.sh      (RESUME=1 to continue, WALL=1 to randomize wall stiffness/friction, LAGMAX=motor-lag upper bound, NOISE=real|lab)
# acados code is compiled with -march=native, so each job builds its own solver: a per-job link third_party/acados_job<ID> -> acados changes the
# build-directory hash (ocrl/am3_nmpc_dob.py hashes the OCRL_ACADOS flavour) without touching any code.
set -uo pipefail
TAG=${TAG:?set TAG}; NCH=${NCH:?set NCH}; STEPS=${STEPS:-10000000}; ENVS=${ENVS:-30}; SEED=${SEED:-0}; RESUME=${RESUME:-0}; WALL=${WALL:-0}; LAGMAX=${LAGMAX:-2.5}; NOISE=${NOISE:-real}
fail () { echo "FATAL: $1"; exit 1; }
REPO=${SLURM_SUBMIT_DIR:-$(cd "$(dirname "$0")/.." && pwd)}
echo "job ${SLURM_JOB_ID:-local} on $(hostname) at $(date); tag $TAG nch $NCH steps $STEPS envs $ENVS"
type module >/dev/null 2>&1 || { source /etc/profile.d/modules.sh 2>/dev/null; source /etc/profile.d/30-set_modulepath.sh 2>/dev/null; }
module load slurm || fail "module load slurm"
eval "$($HOME/miniconda3/bin/conda shell.bash hook)" && conda activate am3rl || fail "conda env am3rl"
cd "$REPO/Code" || fail "no $REPO/Code"
[ -d third_party/acados/lib ] || fail "acados not built in Code/third_party/acados"
FL=acados_job${SLURM_JOB_ID:-local}; ln -sfn acados third_party/$FL; export OCRL_ACADOS=$FL
unset MUJOCO_GL; export OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 PYTHONPATH=.:scripts
python scripts/am3_train_hybrid.py "$TAG" --nch "$NCH" --steps "$STEPS" --envs "$ENVS" --seed "$SEED" --noise "$NOISE" --preflight-only || fail "preflight failed, training not started"
EXTRA=""; [ "$RESUME" = "1" ] && EXTRA="--resume"; [ "$WALL" = "1" ] && EXTRA="$EXTRA --wall"; EXTRA="$EXTRA --lag-max $LAGMAX --noise $NOISE"
python scripts/am3_train_hybrid.py "$TAG" --nch "$NCH" --steps "$STEPS" --envs "$ENVS" --seed "$SEED" $EXTRA; RC=$?
rm -f third_party/$FL
echo "finished $(date) exit $RC"; tail -5 "results/am3_rl/$TAG/train.log" 2>/dev/null; exit $RC
