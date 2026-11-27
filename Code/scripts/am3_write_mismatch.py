"""Writing under model mismatch the controller does not know (am3_plant.apply_mismatch), to find where a controller degrades.
Usage: am3_write_mismatch.py CTRL CASE [--text OM] [--height 0.10]   (CASE from CASES below)"""
import sys, os, json, argparse, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_write_eval import write_metrics, make
from ocrl.am3_write import WriteEpisode

CASES = dict(nominal={}, payload100g=dict(payload=dict(kg=0.10, pos_hand_m=[0.0, 0.0, 0.0])), payload200g=dict(payload=dict(kg=0.20, pos_hand_m=[0.0, 0.0, 0.0])),
             kf90=dict(kf_scale=0.90), motor_slow=dict(motor_tau_scale=2.0), damping3x=dict(joint_damping_scale=3.0),
             gust_y=dict(disturbance=[dict(kind="step", axis=1, amp=1.0, t0=12.0, dur=3.0)]), wind_sine=dict(disturbance=[dict(kind="sine", axis=1, amp=0.8, freq=0.5, t0=3.0), dict(kind="sine", axis=2, amp=0.5, freq=0.3, t0=3.0)]),
             # Stress set (2026-11-19, the proposal's untested conditions): wall contact model, approach (impact) speed, and outside the RL/L training ranges
             # (training: motor lag <= 2.5x, thrust 0.88-1.08, payload <= 250-280 g).  v_approach (m/s) and v_final (m/s; task 0.02 and 0.01) override the approach and the final touch-down speed for that run only.
             wall_soft=dict(k_scale=0.5), wall_stiff=dict(k_scale=3.0), wall_k02=dict(k_scale=0.2), wall_k6=dict(k_scale=6.0), wall_k10=dict(k_scale=10.0),          # 2026-11-27: outside the L2W/L9W wall training range (k x0.4-3.5); R-series never randomised the wall
             wall_slippery=dict(mu_scale=0.5), wall_grippy=dict(mu_scale=2.0),
             approach_fast=dict(v_approach=0.06, v_final=0.03), motor_lag15=dict(motor_tau_scale=1.5), motor_lag3=dict(motor_tau_scale=3.0), kf85=dict(kf_scale=0.85),
             combo=dict(payload=dict(kg=0.10, pos_hand_m=[0.0, 0.0, 0.0]), motor_tau_scale=1.5, disturbance=[dict(kind="step", axis=1, amp=1.0, t0=12.0, dur=3.0)]))


def main(a):
    t0 = time.time(); ep = WriteEpisode(make(a.ctrl), a.text, height=a.height, mismatch=CASES[a.case]); ep.run(); m = dict(ctrl=os.path.basename(os.path.dirname(a.ctrl)) or a.ctrl, case=a.case, text=a.text, **write_metrics(ep), wall_s=round(time.time() - t0))
    print(m, flush=True); os.makedirs(os.path.join(HERE, "..", "results", "am3_write"), exist_ok=True)
    json.dump(m, open(os.path.join(HERE, "..", "results", "am3_write", f"mm_{m['ctrl']}_{a.case}_{a.text}.json"), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("case", choices=sorted(CASES)); ap.add_argument("--text", default="OM"); ap.add_argument("--height", type=float, default=0.10); main(ap.parse_args())
