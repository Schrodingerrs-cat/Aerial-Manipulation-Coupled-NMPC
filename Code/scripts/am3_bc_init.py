"""Behaviour-cloning warm start for the trajectory policy: regress the PPO actor onto the scripted PD+feedforward expert (and the critic onto discounted returns), so PPO starts
at expert level and only has to improve on it.  Data: the expert's action labels on states visited while executing the expert plus Gaussian action noise (covers off-nominal states).
The result is an ordinary PPO model; deployment is the network only.  Usage: am3_bc_init.py TAG [--episodes 150] [--noise 0.2] [--epochs 40]  ->  results/am3_rl/TAG/model_bc.zip"""
import sys, os, json, argparse, time
import numpy as np
from stable_baselines3 import PPO          # first: torch must load before anything else heavy
import torch
from stable_baselines3.common.vec_env import DummyVecEnv
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_rl_direct import Track3Env


def main(a):
    out = os.path.join(HERE, "..", "results", "am3_rl", a.tag); os.makedirs(out, exist_ok=True)
    env = Track3Env(seed=a.seed); rng = np.random.default_rng(a.seed); O, A, G = [], [], []; t0 = time.time()
    for ep in range(a.episodes):
        o, _ = env.reset(seed=50000 + a.seed * 1000 + ep); ob, ac, rw = [], [], []
        while True:
            ae = env.expert_action(env.pl.state()); ob.append(o); ac.append(ae)
            o, r, term, trunc, _ = env.step(np.clip(ae + a.noise * rng.standard_normal(10) * (np.arange(10) < 3), -1, 1)); rw.append(r)
            if term or trunc: break
        g, ret = 0.0, np.zeros(len(rw))
        for i in range(len(rw) - 1, -1, -1): g = rw[i] + 0.99 * g; ret[i] = g
        O += ob; A += ac; G += list(ret)
        if ep % 25 == 0: print(f"collected episode {ep}/{a.episodes}  samples {len(O)}  {time.time() - t0:.0f}s", flush=True)
    O, A, G = np.array(O, np.float32), np.array(A, np.float32), np.array(G, np.float32); print(f"dataset {O.shape}, return mean {G.mean():.1f} std {G.std():.1f}")
    venv = DummyVecEnv([lambda: Track3Env(seed=0)])
    model = PPO("MlpPolicy", venv, policy_kwargs=dict(net_arch=[256, 256], activation_fn=torch.nn.ELU, log_std_init=a.log_std_init), seed=a.seed, device="cpu")
    pol = model.policy; opt = torch.optim.Adam(pol.parameters(), lr=a.lr); To, Ta, Tg = map(torch.as_tensor, (O, A, G / G.std()))
    for epoch in range(a.epochs):
        perm = torch.randperm(len(To)); la = lv = 0.0
        for i in range(0, len(To), 2048):
            idx = perm[i:i + 2048]; dist = pol.get_distribution(To[idx]); v = pol.predict_values(To[idx]).squeeze(-1)
            loss_a = ((dist.distribution.mean - Ta[idx]) ** 2).mean(); loss_v = ((v - Tg[idx]) ** 2).mean(); opt.zero_grad(); (loss_a + 0.5 * loss_v).backward(); opt.step(); la += loss_a.item(); lv += loss_v.item()
        nb = (len(To) + 2047) // 2048; print(f"epoch {epoch + 1}/{a.epochs}  actor mse {la / nb:.5f}  critic mse {lv / nb:.4f}", flush=True)
    model.save(os.path.join(out, "model_bc.zip")); json.dump(dict(vars(a), samples=len(O)), open(os.path.join(out, "bc_config.json"), "w"), indent=1); print("saved model_bc.zip")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("tag"); ap.add_argument("--episodes", type=int, default=150); ap.add_argument("--noise", type=float, default=0.2)
    ap.add_argument("--epochs", type=int, default=40); ap.add_argument("--lr", type=float, default=1e-3); ap.add_argument("--seed", type=int, default=0); ap.add_argument("--log-std-init", type=float, default=-2.0); main(ap.parse_args())
