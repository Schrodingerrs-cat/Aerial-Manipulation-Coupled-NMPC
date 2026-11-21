"""Train the learned model correction L (ocrl/am3_hybrid_l.py) with PPO.  2026-11-19.
Preflight (imports, solver build, zero-correction rollout) runs before training; checkpoints every --save-every steps; SIGTERM saves model_interrupt.zip; --resume continues.
Usage: am3_train_hybrid.py TAG --nch 2|9 [--steps 10000000] [--envs 30] [--seed 0] [--ramp 300] [--wall] [--lag-max 2.5] [--noise real|lab] [--resume] [--preflight-only]"""
import sys, os, json, time, signal, argparse, glob, datetime
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import numpy as np
from stable_baselines3 import PPO                     # torch before acados
from stable_baselines3.common.vec_env import SubprocVecEnv, VecMonitor
from stable_baselines3.common.callbacks import BaseCallback

OUT = None


def log(msg):
    line = f"[{datetime.datetime.now():%Y-%m-%d %H:%M:%S}] {msg}"; print(line, flush=True)
    if OUT: open(os.path.join(OUT, "train.log"), "a").write(line + "\n")


def make_env(nch, seed, ramp, wall, lag_max, noise):
    def f():
        from ocrl.am3_hybrid_l import HybridEnv
        return HybridEnv(nch, seed=seed, ramp=ramp, wall=wall, lag_max=lag_max, noise=noise)
    return f


class Saver(BaseCallback):
    def __init__(self, every):
        super().__init__(); self.every, self.next, self.t0, self.ok, self.n, self.slow = every, every, time.time(), [], 0, 0

    def _on_step(self):
        for i in self.locals["infos"]:
            if "episode" in i: self.ok.append(bool(i.get("completed", False))); self.n += 1; self.slow += bool(i.get("wall_timeout", False))
        if self.num_timesteps >= self.next:
            self.model.save(os.path.join(OUT, f"model_{self.num_timesteps // 1000}k.zip")); self.next += self.every
            ep = [e["r"] for e in self.model.ep_info_buffer]
            log(f"steps {self.num_timesteps}  episodes {self.n}  ep_rew_mean {np.mean(ep) if ep else float('nan'):.1f}  completed(last 200) {np.mean(self.ok[-200:]) if self.ok else 0:.2f}  wall-clock truncations {self.slow}  {self.num_timesteps / (time.time() - self.t0):.0f} steps/s")
        return True


def preflight(a):
    from ocrl.am3_hybrid_l import HybridEnv
    log(f"PREFLIGHT nch={a.nch} cpus={os.cpu_count()} job={os.environ.get('SLURM_JOB_ID', '-')} acados={os.environ.get('OCRL_ACADOS', 'acados')}")
    env = HybridEnv(a.nch, seed=12345, ramp=1, noise=a.noise); o, _ = env.reset(); assert o.shape == env.observation_space.shape and np.isfinite(o).all(), "bad observation"
    t0 = time.time(); k = 0
    while k < 150:
        o, r, te, tr, _ = env.step(np.zeros(a.nch)); k += 1; assert np.isfinite(o).all() and np.isfinite(r), f"non-finite at step {k}"
        if te or tr: env.reset()
    log(f"PREFLIGHT OK: obs {o.shape}, {(time.time() - t0) / k * 1e3:.1f} ms/step (solver built)")


def main(a):
    global OUT
    OUT = os.path.join(HERE, "..", "results", "am3_rl", a.tag); os.makedirs(OUT, exist_ok=True); preflight(a)
    if a.preflight_only: return
    json.dump(dict(env="hybrid", nch=a.nch, algo="ppo", steps=a.steps, envs=a.envs, seed=a.seed, ramp=a.ramp, wall=a.wall, lag_max=a.lag_max, noise=a.noise, base="NDF:0.5,f", started=f"{datetime.datetime.now():%Y-%m-%d %H:%M}"), open(os.path.join(OUT, "config.json"), "w"), indent=1)
    venv = VecMonitor(SubprocVecEnv([make_env(a.nch, a.seed * 1000 + i, a.ramp, a.wall, a.lag_max, a.noise) for i in range(a.envs)], start_method="forkserver"), info_keywords=("completed", "wall_timeout"))
    ck = sorted(glob.glob(os.path.join(OUT, "model_*k.zip")), key=os.path.getmtime) + glob.glob(os.path.join(OUT, "model_interrupt.zip"))
    if a.resume and ck:
        last = max(ck, key=os.path.getmtime); model = PPO.load(last, env=venv, device="cpu"); log(f"resumed from {last} at {model.num_timesteps}")
    else:
        model = PPO("MlpPolicy", venv, n_steps=256, batch_size=1920, n_epochs=5, learning_rate=1e-4, gamma=0.995, gae_lambda=0.95, clip_range=0.2, target_kl=0.03, ent_coef=0.0,
                    policy_kwargs=dict(net_arch=[256, 256], log_std_init=-1.5), seed=a.seed, device="cpu", verbose=0)
    signal.signal(signal.SIGTERM, lambda *_: (model.save(os.path.join(OUT, "model_interrupt.zip")), log("SIGTERM: saved model_interrupt.zip"), sys.exit(143)))
    log(f"TRAIN {a.steps} steps, {a.envs} envs"); model.learn(a.steps - model.num_timesteps, callback=Saver(a.save_every), reset_num_timesteps=not a.resume)
    model.save(os.path.join(OUT, "model_final.zip")); log("done: model_final.zip")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--nch", type=int, choices=(2, 9), required=True); ap.add_argument("--steps", type=int, default=10_000_000)
    ap.add_argument("--envs", type=int, default=30); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--ramp", type=int, default=300); ap.add_argument("--save-every", type=int, default=500_000)
    ap.add_argument("--wall", action="store_true"); ap.add_argument("--lag-max", type=float, default=2.5); ap.add_argument("--noise", choices=("real", "lab"), default="real"); ap.add_argument("--resume", action="store_true"); ap.add_argument("--preflight-only", action="store_true"); main(ap.parse_args())
