"""Writing under model mismatch the controller does not know (am3_plant.apply_mismatch), to find where a controller degrades.
Usage: am3_write_mismatch.py CTRL CASE [--text OM] [--height 0.10]   (CASE from CASES below)"""
import sys, os, json, argparse, time
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
from am3_write_eval import write_metrics, make
from ocrl.am3_write import WriteEpisode

CASES = dict(nominal={}, payload100g=dict(payload=dict(kg=0.10, pos_hand_m=[0.0, 0.0, 0.0])), payload200g=dict(payload=dict(kg=0.20, pos_hand_m=[0.0, 0.0, 0.0])),
             kf90=dict(kf_scale=0.90), motor_slow=dict(motor_tau_scale=2.0), damping3x=dict(joint_damping_scale=3.0),
             gust_y=dict(disturbance=[dict(kind="step", axis=1, amp=1.0, t0=12.0, dur=3.0)]), wind_sine=dict(disturbance=[dict(kind="sine", axis=1, amp=0.8, freq=0.5, t0=3.0), dict(kind="sine", axis=2, amp=0.5, freq=0.3, t0=3.0)]))


def main(a):
    t0 = time.time(); ep = WriteEpisode(make(a.ctrl), a.text, height=a.height, mismatch=CASES[a.case]); ep.run(); m = dict(ctrl=os.path.basename(os.path.dirname(a.ctrl)) or a.ctrl, case=a.case, text=a.text, **write_metrics(ep), wall_s=round(time.time() - t0))
    print(m, flush=True); os.makedirs(os.path.join(HERE, "..", "results", "am3_write"), exist_ok=True)
    json.dump(m, open(os.path.join(HERE, "..", "results", "am3_write", f"mm_{m['ctrl']}_{a.case}_{a.text}.json"), "w"), indent=1)


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("ctrl"); ap.add_argument("case", choices=sorted(CASES)); ap.add_argument("--text", default="OM"); ap.add_argument("--height", type=float, default=0.10); main(ap.parse_args())
