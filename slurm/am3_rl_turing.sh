#!/usr/bin/env bash
#SBATCH -N 1
#SBATCH -c 32
#SBATCH -t 24:00:00
#SBATCH --mem 32G
#SBATCH --job-name="am3_rl"
#SBATCH -o am3rl_%j.out
#SBATCH -e am3rl_%j.err
#SBATCH -p academic
#SBATCH --signal=B:TERM@120          # SIGTERM 120 s before the wall-clock kill, so the trainer can checkpoint

# ---------------------------------------------------------------------------
# End-to-end RL (track 2), any rung (default R1 hover; see ARGS).  CPU ONLY - no GPU, no CUDA, no acados.
# Submit FROM THE REPO ROOT:   cd ~/sem3/Aerial-Manipulation-Coupled-NMPC && sbatch slurm/am3_rl_turing.sh
# Override anything:           TAG=myrun STEPS=5000000 ENVS=16 sbatch slurm/am3_rl_turing.sh
# Other rounds:               ARGS="--env track5" TAG=R5_s0 sbatch slurm/am3_rl_turing.sh
# Continue a timed-out run:    RESUME=1 sbatch slurm/am3_rl_turing.sh
# Logs land as am3rl_<jobid>.out/.err in the directory you submitted from.
# ---------------------------------------------------------------------------
set -uo pipefail                     # deliberately NOT -e: we want to reach the diagnostics at the end

TAG=${TAG:-R1_hover_turing_s0}
STEPS=${STEPS:-20000000}
ENVS=${ENVS:-30}
SEED=${SEED:-0}
RESUME=${RESUME:-0}
ENVNAME=${ENVNAME:-am3rl}
ARGS=${ARGS:-}                       # extra trainer options, e.g. ARGS="--env track5 --lr 5e-5 --lr-decay --target-kl 0.02"

fail () { echo; echo "FATAL: $1"; echo "job aborted at $(date)"; exit 1; }

# ---- locate the repo from THIS script's own location: no hard-coded paths ----
if [ -n "${SLURM_SUBMIT_DIR:-}" ]; then SRC="$SLURM_SUBMIT_DIR/slurm/am3_rl_turing.sh"; else SRC="${BASH_SOURCE[0]}"; fi
REPO=$(cd "$(dirname "$SRC")/.." 2>/dev/null && pwd) || fail "cannot resolve the repo directory from $SRC"

echo "=========================================================================="
echo "job        : ${SLURM_JOB_ID:-local}   node $(hostname)"
echo "started    : $(date)"
echo "repo       : $REPO"
echo "tag        : $TAG   steps $STEPS   envs $ENVS   seed $SEED   resume $RESUME"
echo "cpus       : ${SLURM_CPUS_ON_NODE:-$(nproc)}"
echo "=========================================================================="

# ---- modules: slurm is REQUIRED on this cluster -----------------------------
echo "--- loading modules ---"
type module >/dev/null 2>&1 || { source /etc/profile.d/modules.sh 2>/dev/null; source /etc/profile.d/30-set_modulepath.sh 2>/dev/null; }   # batch shells do not define `module` (found 2026-11-03)
module load slurm || fail "'module load slurm' failed - without it srun/sbatch tooling is not on PATH"
echo "module load slurm OK"

# ---- conda env --------------------------------------------------------------
[ -x "$HOME/miniconda3/bin/conda" ] || fail "conda not found at $HOME/miniconda3/bin/conda"
eval "$($HOME/miniconda3/bin/conda shell.bash hook)" || fail "could not initialise conda"
conda activate "$ENVNAME" || fail "conda env '$ENVNAME' not found - run 'bash slurm/setup_turing.sh' on the login node first"
PY=$(which python) || fail "no python after activating '$ENVNAME'"
echo "python     : $PY ($(python --version 2>&1))"

# ---- files ------------------------------------------------------------------
[ -f "$REPO/Code/scripts/am3_train_direct.py" ]    || fail "missing $REPO/Code/scripts/am3_train_direct.py - the copied repo is incomplete"
[ -f "$REPO/Code/ocrl/am3_rl_direct.py" ]          || fail "missing $REPO/Code/ocrl/am3_rl_direct.py"
[ -f "$REPO/Code/model/aerial_manipulator_v3.xml" ] || fail "missing the robot model XML"
cd "$REPO/Code" || fail "cannot cd to $REPO/Code"

unset MUJOCO_GL          # training renders nothing; osmesa makes `import mujoco` fail on the compute nodes (found 2026-11-03)
export OMP_NUM_THREADS=1       # parallelism comes from the workers, not from BLAS
export MKL_NUM_THREADS=1

# ---- preflight: never burn queue time on a run that cannot start ------------
echo; echo "--- PREFLIGHT ---"
python scripts/am3_train_direct.py "$TAG" --preflight-only --steps "$STEPS" --envs "$ENVS" $ARGS
PF=$?
[ $PF -eq 0 ] || fail "preflight exited $PF - see above. Training NOT started."

echo; echo "--- TRAINING ---"
EXTRA=""; [ "$RESUME" = "1" ] && EXTRA="--resume"
python scripts/am3_train_direct.py "$TAG" --steps "$STEPS" --envs "$ENVS" --seed "$SEED" \
       --save-every 500000 --torch-threads 1 --device cpu $ARGS $EXTRA
RC=$?

echo; echo "=========================================================================="
echo "finished   : $(date)   exit $RC"
RES="$REPO/Code/results/am3_rl/$TAG"
if [ -d "$RES" ]; then
  echo "checkpoints:"; ls -la "$RES" | tail -12
  echo; echo "last log lines:"; tail -20 "$RES/train.log" 2>/dev/null
else
  echo "WARNING: nothing at $RES - training produced no output"
fi
case $RC in
  0)   echo "STATUS: completed normally" ;;
  143) echo "STATUS: SIGTERM at wall clock. model_interrupt.zip was saved - resubmit with RESUME=1 to continue." ;;
  1)   echo "STATUS: unhandled python exception - traceback is in $RES/train.log" ;;
  *)   echo "STATUS: exit $RC. Check am3rl_${SLURM_JOB_ID:-}.err and: sacct -j ${SLURM_JOB_ID:-} --format=JobID,State,ExitCode,MaxRSS,Elapsed" ;;
esac
echo "=========================================================================="
exit $RC
