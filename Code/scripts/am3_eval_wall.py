"""Score a controller on the held-out wall-line scenarios with the NMPC harness (Episode) and am3_acceptance_v2.  CTRL is N, B, or a policy .zip.
Usage: am3_eval_wall.py CTRL --block 0 --blocks 6 --out wall_N_0.json"""
import sys, os, json, argparse
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_episode import Episode, initial_state
from ocrl.am3_eval import metrics, passes
from am3_run import make_ctrl

CFG = os.path.join(HERE, "..", "config")
SC = [s for d in ("heldout", "heldout2") for s in json.load(open(os.path.join(CFG, f"am3_domain_{d}.json")))["scenarios"] if s["path"].startswith("line")]
CRIT = json.load(open(os.path.join(CFG, "am3_acceptance_v2.json")))["criteria"]


def run(name, s, record=None, ff=False):
    if name.endswith(".zip"):
        from ocrl.am3_rl_wall import RLCtrl
        cfg = os.path.join(os.path.dirname(name), "config.json"); ff = ff or (os.path.exists(cfg) and json.load(open(cfg)).get("env") == "wall_ff")
        ctrl = RLCtrl(PPO.load(name, device="cpu"), vel_ff=ff)
    else: ctrl = make_ctrl(name)
    ep = Episode(ctrl, s["path"], seed=s["seed"], T_max=40.0, x0=initial_state(s["gap0"], (s["dyz"][0], 1.04 + s["dyz"][1])))
    if hasattr(ctrl, "attach"): ctrl.attach(ep)
    if record is not None: record.attach(ep)
    ep.run(); m = metrics(ep); p = passes(m, CRIT); return ep, m, p


def main(a):
    rows = []
    for s in SC[a.block::a.blocks]:
        _, m, p = run(a.ctrl, s, ff=a.ff); t = m.get("trace", {}); k = m["contact"]
        rows.append(dict(id=s["id"], path=s["path"], completed=m["completed"], abort=str(m["abort"]), passed=p["all"], failed=[f for f, v in p.items() if not v and f != "all"], track_rms_mm=t.get("track_rms_mm"), coverage=t.get("coverage_5mm"), endpoint_mm=t.get("endpoint_err_mm"), base_max_mm=m["base"]["max_mm"], tilt_max=m["attitude"]["tilt_max_deg"], contact=k["valid_fraction"], losses=k["losses"], peak_Fn=k["peak_Fn_N"]))
        print(rows[-1], flush=True)
    json.dump(rows, open(os.path.join(HERE, "..", "results", "am3_rl", "eval", a.out), "w"), indent=1, default=str)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("--block", type=int, default=0); ap.add_argument("--blocks", type=int, default=1); ap.add_argument("--out", default="wall_eval.json"); ap.add_argument("--ff", action="store_true", help="force joint velocity feedforward (otherwise read from the run config)"); main(ap.parse_args())
