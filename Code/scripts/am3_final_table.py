"""Final writing table at lab sensor noise (2026-11-26): every controller x 17 conditions, 15 runs each (5 sentences x 3 seeds).
Reads Code/results/am3_write/bench_turing_1126/ (rsynced from Turing).  A '_rerun_' file (400 s wall-clock guard raised to 1500 s, 2026-11-21) replaces the original.
Cell: completed/15, trace RMS mm (mean over completed runs).  Writes <name>.md and <name>.json next to the bench folder.
Usage: am3_final_table.py NAME"""
import sys, os, json, glob, re

HERE = os.path.dirname(os.path.abspath(__file__)); B = os.path.join(HERE, "..", "results", "am3_write", "bench_turing_1126")
CASES = ["nominal", "payload100g", "payload200g", "motor_slow", "motor_lag15", "motor_lag3", "kf90", "kf85", "damping3x", "gust_y", "wind_sine",
         "wall_soft", "wall_stiff", "wall_slippery", "wall_grippy", "approach_fast", "combo"]
CTRLS = [("N", "2026-11-20_1959", "NMPC"), ("NDF", "2026-11-20_1959", "NMPC + disturbance estimate"), ("R21", "2026-11-20_1959", "learned controller (R21)"),
         ("R22", "2026-11-20_1959", "learned controller, wider randomisation (R22)"), ("R23", "2026-11-22_0849", "R21 retrained at lab noise (R23)"),
         ("R24", "2026-11-23_1827", "R22 retrained at lab noise (R24)"), ("L2", "2026-11-20_1959", "NMPC + 2-channel correction, lag to 2.5x (L2)"),
         ("L2W", "2026-11-21_1645", "NMPC + 2-channel correction, wall rand. (L2W)"), ("L9W", "2026-11-22_0849", "NMPC + 9-channel correction (L9W)"),
         ("L9W22", "2026-11-25_1548", "L9W fine-tuned to 2.2x motor lag (L9W_lag22)"),
         ("R29", "2026-11-27_1009", "learned controller trained with take-off (R29)")]


def path(stamp, ctrl, case):
    stress = case in CASES[11:] or case in ("motor_lag15", "motor_lag3", "kf85")
    suffix = "" if case == "nominal" else "_" + case
    rerun = glob.glob(os.path.join(B, f"2026-11-21_1646_rerun_{ctrl}_lab{suffix}.json"))
    return rerun[0] if rerun else os.path.join(B, f"{stamp}_{'stress_' if stress else ''}{ctrl}_lab{suffix}.json")


def main(name):
    rows = {}
    for ctrl, stamp, _ in CTRLS:
        rows[ctrl] = {}
        for c in CASES:
            s = json.load(open(path(stamp, ctrl, c)))["summary"]
            rows[ctrl][c] = dict(completed=s["completed"], n=s["n"], rms_mm=s["trace_rms_mean"], F_peak_N=s.get("F_peak_max"), rotor_tv=s.get("rotor_tv"))
    md = ["| controller | " + " | ".join(CASES) + " | all 15/15 | 0/15 |", "|---" * (len(CASES) + 3) + "|"]
    for ctrl, _, label in CTRLS:
        r = rows[ctrl]
        cells = [f"{v['completed']}, {v['rms_mm']:.2f}" if v["completed"] else "**0**" for v in r.values()]
        md.append(f"| {label} | " + " | ".join(cells) + f" | {sum(v['completed'] == v['n'] for v in r.values())} | {sum(v['completed'] == 0 for v in r.values())} |")
    out = os.path.join(B, "..", name)
    open(out + ".md", "w").write(__doc__.splitlines()[0] + "\nCell: completed/15, trace RMS mm. **0** = no run finished.\n\n" + "\n".join(md) + "\n")
    json.dump(rows, open(out + ".json", "w"), indent=1); print("\n".join(md))


if __name__ == "__main__":
    main(sys.argv[1])
