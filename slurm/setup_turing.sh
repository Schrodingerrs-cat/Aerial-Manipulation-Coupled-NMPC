#!/usr/bin/env bash
# One-time environment setup on WPI Turing.  Run this ON THE LOGIN NODE before submitting any job:
#     bash ~/arm_optimal/Aerial-Manipulation-Coupled-NMPC/slurm/setup_turing.sh
# It is safe to re-run.  It does NOT need acados, CUDA or a GPU: the end-to-end RL track is pure MuJoCo + PyTorch (CPU).
set -euo pipefail

ENVNAME=${1:-am3rl}
echo "=== creating conda env '$ENVNAME' (python 3.10) ==="
eval "$(~/miniconda3/bin/conda shell.bash hook)"
conda create -y -n "$ENVNAME" python=3.10 || echo "env exists, continuing"
conda activate "$ENVNAME"

echo "=== installing pinned dependencies ==="
pip install --upgrade pip
pip install "torch==2.5.1" --index-url https://download.pytorch.org/whl/cpu
pip install "mujoco==3.10.0" "casadi==3.8.1" "numpy==2.2.6" "scipy==1.15.3" \
            "gymnasium==1.2.3" "stable-baselines3==2.7.0" "matplotlib==3.10.9"

echo "=== verifying ==="
python - <<'PY'
import numpy, mujoco, casadi, gymnasium, torch, stable_baselines3, scipy
print("numpy", numpy.__version__, "| mujoco", mujoco.__version__, "| casadi", casadi.__version__)
print("gymnasium", gymnasium.__version__, "| torch", torch.__version__, "| sb3", stable_baselines3.__version__)
PY

echo
echo "=== SETUP OK ==="
echo "Now run the preflight on a COMPUTE node before submitting the real job:"
echo "    srun -c 8 --mem 16G -t 00:20:00 -p academic \\"
echo "         ~/miniconda3/envs/$ENVNAME/bin/python \\"
echo "         ~/arm_optimal/Aerial-Manipulation-Coupled-NMPC/Code/scripts/am3_train_direct.py PREFLIGHT --preflight-only"
echo "If that prints 'PREFLIGHT PASSED', submit with:  sbatch slurm/am3_rl_turing.sh"
