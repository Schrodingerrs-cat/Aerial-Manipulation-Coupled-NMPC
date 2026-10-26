"""SAC training of the learned model correction L (two-dimensional action, NMPC unchanged, production NMPC settings).  Episodes are drawn from the mismatch TRAINING list (config/am3_domain_mm_train.json, one error
family per episode), truncated at MAX_STEPS control cycles.  Usage: am3_train_sac.py TAG --seed S [--envs 5 --steps 200000]   -> results/am3_rl/TAG/{config.json, episodes.csv, model_*.zip}"""
import sys, os, json, time, argparse
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
def make_env(seed, max_steps):
    from ocrl.am3_rl import CorrectionEnv; from stable_baselines3.common.monitor import Monitor
    pool = json.load(open(os.path.join(HERE, "..", "config", "am3_domain_mm_train.json")))["scenarios"]; return Monitor(CorrectionEnv(seed=seed, max_steps=max_steps, pool=pool))
def main():
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--envs", type=int, default=5); ap.add_argument("--steps", type=int, default=200000); ap.add_argument("--max-steps", type=int, default=900); a = ap.parse_args()
    import torch; torch.set_num_threads(2)
    from stable_baselines3 import SAC
    from stable_baselines3.common.vec_env import SubprocVecEnv
    from stable_baselines3.common.callbacks import BaseCallback
    out = os.path.join(HERE, "..", "results", "am3_rl", a.tag); os.makedirs(out, exist_ok=True); from functools import partial
    venv = SubprocVecEnv([partial(make_env, a.seed * 1000 + i, a.max_steps) for i in range(a.envs)], start_method="spawn")
    class Log(BaseCallback):
        def __init__(self): super().__init__(); self.f = open(os.path.join(out, "episodes.csv"), "w"); self.f.write("steps,ep_return,ep_len,wall_s\n"); self.t0 = time.time(); self.last = 0
        def _on_step(self):
            for info in self.locals["infos"]:
                if "episode" in info: self.f.write(f"{self.num_timesteps},{info['episode']['r']:.3f},{info['episode']['l']},{time.time() - self.t0:.0f}\n"); self.f.flush()
            if self.num_timesteps - self.last >= 20000: self.last = self.num_timesteps; self.model.save(os.path.join(out, f"model_{self.num_timesteps // 1000}k"))
            return True
    model = SAC("MlpPolicy", venv, learning_rate=3e-4, buffer_size=300000, learning_starts=3000, batch_size=256, gamma=0.98, tau=0.005, train_freq=1, gradient_steps=1, ent_coef="auto_0.1", policy_kwargs=dict(net_arch=[128, 128]), seed=a.seed, verbose=0, device="cpu")
    json.dump(dict(vars(a), algo="SAC", net_arch=[128, 128], lr=3e-4, gamma=0.98, ent_coef="auto_0.1", obs="ocrl/am3_rl.py build_obs", reward="ocrl/am3_rl.py CorrectionEnv.step"), open(os.path.join(out, "config.json"), "w"), indent=1)
    model.learn(total_timesteps=a.steps, callback=Log()); model.save(os.path.join(out, "model_final")); venv.close(); print("done")
if __name__ == "__main__": main()
