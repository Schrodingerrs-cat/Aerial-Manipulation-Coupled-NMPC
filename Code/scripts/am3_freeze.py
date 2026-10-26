"""Freeze manifest for the held-out evaluation: sha256 of every source/config/model file that defines the candidate controllers, the acceptance criteria (copied from the draft and never edited afterwards), library versions.
A code edit after freezing requires a new manifest version (am3_freeze_v2.json) with equivalence evidence; existing manifests are never overwritten.  Usage: am3_freeze.py VERSION [EXTRA_FILE ...]   (extra files, e.g. EKF residual levels or SAC checkpoints, are hashed relative to Code/)"""
import sys, os, json, glob, hashlib, shutil, platform
HERE = os.path.dirname(os.path.abspath(__file__)); CODE = os.path.join(HERE, "..")
def sha(p): return hashlib.sha256(open(p, "rb").read()).hexdigest()
ver = sys.argv[1]; extra = sys.argv[2:]; out = os.path.join(CODE, "config", f"am3_freeze_{ver}.json"); acc = os.path.join(CODE, "config", f"am3_acceptance_{ver}.json")
assert not os.path.exists(out) and not os.path.exists(acc), "manifest exists: create a new version instead"
shutil.copy(os.path.join(CODE, "config", "am3_acceptance_draft.json"), acc)
files = sorted(glob.glob(os.path.join(CODE, "ocrl", "am3_*.py"))) + [os.path.join(CODE, "config", f) for f in ("am3_design_v3.json", "am3_task_v1.json", "am3_b_gains_v3.json", acc.split(os.sep)[-1], "am3_domain_heldout.json", "am3_domain_val.json", "am3_domain_dev.json", "am3_domain_mm_dev.json", "am3_domain_mm_val.json", "am3_domain_mm_heldout.json", "am3_domain_mm_train.json")] + [os.path.join(CODE, "model", "aerial_manipulator_v3.xml")] + [os.path.join(CODE, e) for e in extra]
import mujoco, casadi, numpy
json.dump(dict(version=ver, files={os.path.relpath(f, CODE): sha(f) for f in files}, python=platform.python_version(), mujoco=mujoco.__version__, casadi=casadi.__version__, numpy=numpy.__version__, machine=platform.platform(), cpus=os.cpu_count(),
               candidates=dict(B="ocrl/am3_ctrl_b.py with config/am3_b_gains_v3.json", N="ocrl/am3_ctrl_n.py + am3_nmpc.py (N=10, h=0.02, nsub=4, acados SQP_RTI, weights in am3_nmpc.W)", Nd="same with coupling='decoupled'")), open(out, "w"), indent=1); print("wrote", out, acc)
