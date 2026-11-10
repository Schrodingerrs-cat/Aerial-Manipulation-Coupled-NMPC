"""Why do R10 wall episodes end?  Rolls a policy (or 'pd') in WallEnv and prints the end phase, time and which limit fired."""
import sys, numpy as np
from stable_baselines3 import PPO
sys.path.insert(0, ".")
from ocrl.am3_rl_wall import WallEnv
from ocrl.am3_rl_direct import BASE
pol = sys.argv[1]; p = "pd" if pol == "pd" else PPO.load(pol, device="cpu")
for s in range(int(sys.argv[2]) if len(sys.argv) > 2 else 8):
    env = WallEnv(seed=s); o, _ = env.reset(seed=s); R = 0; n = 0; mx = 0
    while True:
        a = env.expert_action(env.sens(env.pl.state())) if p == "pd" else p.predict(o, deterministic=True)[0]
        o, r, term, trunc, i = env.step(a); R += r; n += 1; x = env.pl.state(); mx = max(mx, i["pad_err"])
        if term or trunc: break
    tilt = np.degrees(2 * np.arcsin(min(1.0, np.hypot(x[4], x[5]))))
    print(s, env.path, f"t={n*env.dt_c:.2f}s", "phase", i["phase"], "done", i["completed"], "term", term, f"R/step {R/n:.2f}", f"tilt {tilt:.1f}", f"base {np.linalg.norm(x[:3]-BASE)*1e3:.0f}mm", f"F {i['F']:.2f}", "failed", env.task.failed, "other", len(env.pl.other_contacts()), f"pad_err_max {mx*1e3:.1f}mm", flush=True)
