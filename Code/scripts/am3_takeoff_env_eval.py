"""Take-off check of an RL checkpoint in the training env (2026-11-26): N episodes that start ON THE STAND (no mid-take-off starts), nominal model, 'real' sensing as in training.
Success = reaches the writing approach without termination.  Usage: am3_takeoff_env_eval.py MODEL_ZIP [N=10]"""
import sys, numpy as np
from stable_baselines3 import PPO
from ocrl.am3_rl_wall import WallEnv, KIND_KW, SHAPES

pol = PPO.load(sys.argv[1], device="cpu"); n = int(sys.argv[2]) if len(sys.argv) > 2 else 10
kw = dict(KIND_KW["write15_drjt_to"]); kw["takeoff_p"] = 1.0
env = WallEnv(seed=1000, max_steps=1000, vel_ff=True, paths=SHAPES, write_p=0.7, dr=False, **kw); ok = 0
for i in range(n):
    o, _ = env.reset()
    for k in range(env.max_steps):
        o, r, te, tr, info = env.step(pol.predict(o, deterministic=True)[0])
        if te or tr or info["phase"] == "approach": break
    x = env.pl.state(); good = info["phase"] == "approach" and not te; ok += good
    print(f"ep{i}: {'OK  ' if good else 'FAIL'} t {(k + 1) * env.dt_c:.2f} s phase {info['phase']} base err {info['e'] * 1e3:.0f} mm tilt {np.degrees(2 * np.arcsin(min(1, np.hypot(x[4], x[5])))):.0f} deg contacts {env.pl.other_contacts()[:1]}")
print(f"TAKEOFF SUCCESS {ok}/{n}  ({sys.argv[1]})")
