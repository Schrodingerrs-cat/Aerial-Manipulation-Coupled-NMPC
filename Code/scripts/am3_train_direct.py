"""Train the end-to-end RL controller (track 2).  R1 = hover.  PPO, parallel MuJoCo environments, no NMPC and no acados.

Built to survive an unattended cluster run:
  * a PREFLIGHT stage that checks every dependency, config file and one full rollout BEFORE any training starts, and exits non-zero with a named reason if anything is wrong
  * timestamped logging to both stdout and <out>/train.log, flushed every line
  * checkpoints every --save-every steps, plus an interrupt checkpoint on SIGTERM/SIGINT (SLURM sends SIGTERM before it kills a job), so a wall-clock kill never loses the run
  * --resume continues from the newest checkpoint in the output directory
  * periodic evaluation against two fixed references - the zero action and a scripted PD expert - so the curve is interpretable, not just a number that goes up
  * any exception is written to the log with a full traceback before exiting non-zero

Usage:
  am3_train_direct.py TAG --preflight-only            check the machine, run nothing
  am3_train_direct.py TAG --steps 3000000 --envs 16
  am3_train_direct.py TAG --resume
"""
import sys, os, json, time, signal, argparse, traceback, platform, datetime
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, ".."))
import numpy as np

LOG_F = None


def log(msg, level="INFO"):
    line = f"[{datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}] [{level}] {msg}"
    print(line, flush=True)
    if LOG_F is not None:
        LOG_F.write(line + "\n"); LOG_F.flush()


def preflight(a):
    """Everything that could fail on an unfamiliar machine, checked before a single training step. Raises with a named reason."""
    log("=" * 72); log("PREFLIGHT")
    log(f"python {platform.python_version()} on {platform.platform()}")
    log(f"cpus visible: {os.cpu_count()}  SLURM_CPUS_ON_NODE={os.environ.get('SLURM_CPUS_ON_NODE', '-')}  JOB={os.environ.get('SLURM_JOB_ID', '-')}")
    mods = {}
    for m in ("numpy", "mujoco", "casadi", "gymnasium", "torch", "stable_baselines3", "scipy"):
        try:
            mod = __import__(m); mods[m] = getattr(mod, "__version__", "?"); log(f"  import {m:20s} OK  {mods[m]}")
        except Exception as e:
            raise RuntimeError(f"PREFLIGHT FAIL: cannot import '{m}': {e}")
    import torch
    log(f"  torch threads {torch.get_num_threads()}  cuda available {torch.cuda.is_available()}")
    for f in ("config/am3_design_v3.json", "config/am3_task_v1.json", "model/aerial_manipulator_v3.xml"):
        p = os.path.join(HERE, "..", f)
        if not os.path.exists(p): raise RuntimeError(f"PREFLIGHT FAIL: missing required file {f}")
        log(f"  file {f:40s} OK  {os.path.getsize(p)} bytes")
    log("  constructing environment ...")
    env = make_raw(a.env, 0, 200, a.fine_w, a.fine_s, a.lin_w, a.tilt_w, a.bad_pen, a.p0_sigma, a.v0_sigma)
    o, _ = env.reset(seed=0)
    if o.shape != env.observation_space.shape: raise RuntimeError(f"PREFLIGHT FAIL: obs shape {o.shape} != space {env.observation_space.shape}")
    if not np.isfinite(o).all(): raise RuntimeError("PREFLIGHT FAIL: non-finite observation on reset")
    t0 = time.perf_counter(); n = 0
    for _ in range(200):
        o, r, term, trunc, info = env.step(env.action_space.sample()); n += 1
        if not np.isfinite(o).all() or not np.isfinite(r): raise RuntimeError(f"PREFLIGHT FAIL: non-finite obs/reward at step {n}")
        if term or trunc: env.reset()
    dt = (time.perf_counter() - t0) / 200
    log(f"  random rollout OK: {dt * 1e3:.2f} ms/step")
    env_b = env
    if a.env in ("write_dr", "write_drc"):                                      # same reward without randomization: the scripted PD cannot fly the randomized plant, so it is no reward check there
        env_b = make_raw("write", 0, 200, a.fine_w, a.fine_s, a.lin_w, a.tilt_w, a.bad_pen, a.p0_sigma, a.v0_sigma); log("  reward check on env 'write' (write_dr = write + domain randomization, same reward)")
    if a.env == "hybrid":                                        # the reference controller is the plain NMPC (zero residual): it must earn positive reward on the nominal task
        from ocrl.am3_hybrid import HybridEnv
        nm = baseline_returns(HybridEnv(seed=0, dr=False), n_ep=2)[0]; log(f"  reference reward/step   plain NMPC (zero residual, nominal) {nm:8.3f}")
        if nm <= 0.0: raise RuntimeError(f"PREFLIGHT FAIL: plain NMPC earns {nm:.3f}/step")
        log("PREFLIGHT PASSED"); log("=" * 72); return dict(modules=mods, step_ms=dt * 1e3, ret_zero=nm, ret_pd=nm)
    z, pd = baseline_returns(env_b, n_ep=2)
    log(f"  reference reward/step   zero-action {z:8.3f}   scripted PD {pd:8.3f}")
    if pd <= z: raise RuntimeError(f"PREFLIGHT FAIL: scripted PD ({pd:.3f}/step) does not beat zero action ({z:.3f}/step); the task or reward is mis-specified")
    if pd <= 0.0: raise RuntimeError(f"PREFLIGHT FAIL: a competent controller earns {pd:.3f} reward/step. Surviving must pay: otherwise the agent learns to terminate the episode")
    est = a.steps * dt / max(a.envs, 1) / 3600
    log(f"  estimated rollout wall time for {a.steps:,} steps on {a.envs} envs: {est:.2f} h (plus PPO updates)")
    log("PREFLIGHT PASSED"); log("=" * 72)
    return dict(modules=mods, step_ms=dt * 1e3, ret_zero=z, ret_pd=pd)


def baseline_returns(env, n_ep=3):
    """Reward PER STEP of the two fixed references on the same episodes: zero action, and the environment's own scripted PD.  Per step, so the numbers compare with
    training episodes of any length (an earlier version printed whole-episode returns from 200-step episodes next to 1000-step training returns, which misled)."""
    out = []
    for policy in ("zero", "pd"):
        R, N = 0.0, 0
        for k in range(n_ep):
            env.reset(seed=1000 + k)
            while True:
                a = np.zeros(env.action_space.shape[0]) if policy == "zero" else env.expert_action(env.pl.state())
                _, r, term, trunc, _ = env.step(a); R += r; N += 1
                if term or trunc: break
        out.append(R / max(N, 1))
    return out[0], out[1]


def make_raw(kind, seed, max_steps, fine_w=None, fine_s=0.01, lin_w=None, tilt_w=None, bad_pen=None, p0_sigma=None, v0_sigma=None):
    from ocrl.am3_rl_direct import HoverEnv, TrackEnv, Track3Env, Track4Env, Track5Env, Track6Env, Track7Env, Track8Env, Track9Env
    kw = dict(fine_s=fine_s)
    if fine_w is not None: kw["fine_w"] = fine_w
    if lin_w is not None: kw["lin_w"] = lin_w
    if tilt_w is not None: kw["tilt_w"] = tilt_w
    if bad_pen is not None: kw["bad_pen"] = bad_pen
    if p0_sigma is not None: kw["p0_sigma"] = p0_sigma
    if v0_sigma is not None: kw["v0_sigma"] = v0_sigma
    if kind == "hybrid":
        from ocrl.am3_hybrid import HybridEnv
        return HybridEnv(seed=seed, dr=fine_w != -1.0)                            # NMPC + residual policy, writing/shapes, domain randomization
    if kind in ("wall", "wall_ff", "wall_shapes", "write", "write_dr", "write_drc", "write15"):
        from ocrl.am3_rl_wall import WallEnv, SHAPES, KIND_KW
        if kind in ("wall_shapes", "write", "write_dr", "write_drc", "write15"): kw["paths"] = SHAPES
        kw.update(KIND_KW.get(kind, {}))                                          # R15 (Phase 1): history 4, disturbance observer, realistic sensing, fine pad reward
        if kind == "write_drc": kw["dr_ramp"] = 300                               # R14: randomization ramped in over the first 300 episodes per env
        if kind in ("write", "write_dr", "write_drc", "write15"): kw["write_p"] = 0.7                     # 70 % writing episodes (moving base), 30 % fixed-base shapes
        if kind in ("write_dr", "write_drc"): kw["dr"] = True                                    # plus domain randomization (payload, thrust, motor lag, damping, wind)
        return WallEnv(seed=seed, max_steps=max_steps, vel_ff=kind != "wall", **kw)
    return {"track": TrackEnv, "track3": Track3Env, "track4": Track4Env, "track5": Track5Env, "track6": Track6Env, "track7": Track7Env, "track8": Track8Env, "track9": Track9Env}.get(kind, HoverEnv)(seed=seed, max_steps=max_steps, **kw)


def make_env(seed, max_steps, kind="hover", fine_w=None, fine_s=0.01, lin_w=None, tilt_w=None, bad_pen=None, p0_sigma=None, v0_sigma=None):
    from stable_baselines3.common.monitor import Monitor
    return Monitor(make_raw(kind, seed, max_steps, fine_w, fine_s, lin_w, tilt_w, bad_pen, p0_sigma, v0_sigma))


def main(a):
    global LOG_F
    out = os.path.join(HERE, "..", "results", "am3_rl", a.tag); os.makedirs(out, exist_ok=True)
    LOG_F = open(os.path.join(out, "train.log"), "a")
    log(f"tag={a.tag}  args={vars(a)}")
    info = preflight(a)
    if a.preflight_only:
        json.dump(info, open(os.path.join(out, "preflight.json"), "w"), indent=1); log("preflight-only: exiting 0"); return 0

    import torch; torch.set_num_threads(max(1, a.torch_threads))
    from stable_baselines3 import PPO
    from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
    from stable_baselines3.common.callbacks import BaseCallback
    from functools import partial

    mk = [partial(make_env, a.seed * 1000 + i, a.max_steps, a.env, a.fine_w, a.fine_s, a.lin_w, a.tilt_w, a.bad_pen, a.p0_sigma, a.v0_sigma) for i in range(a.envs)]
    venv = SubprocVecEnv(mk, start_method="spawn") if a.envs > 1 else DummyVecEnv(mk)
    log(f"vec env ready: {a.envs} workers")

    ck = [f for f in os.listdir(out) if f.startswith("model_") and f.endswith(".zip")]
    model, done_steps = None, 0
    if a.resume and ck:
        newest = max(ck, key=lambda f: os.path.getmtime(os.path.join(out, f)))
        model = PPO.load(os.path.join(out, newest), env=venv, device=a.device)
        done_steps = model.num_timesteps; log(f"RESUMED from {newest} at {done_steps:,} steps")
    if model is None and a.init_from and PPO.load(a.init_from, device="cpu").observation_space.shape != venv.observation_space.shape:
        # the new observation entries are appended at the end: zero first-layer weights for them, so the warm-started policy acts exactly as the source at the start
        src = PPO.load(a.init_from, device="cpu").policy.state_dict()
        model = PPO("MlpPolicy", venv, learning_rate=(lambda p: a.lr * max(0.1, p)) if a.lr_decay else a.lr, n_steps=a.rollout, batch_size=a.batch, n_epochs=a.epochs,
                    gamma=a.gamma, gae_lambda=0.95, clip_range=0.2, ent_coef=a.ent, target_kl=a.target_kl,
                    policy_kwargs=dict(net_arch=[256, 256], activation_fn=torch.nn.ELU, log_std_init=a.log_std_init), seed=a.seed, verbose=0, device=a.device)
        sd = model.policy.state_dict()
        for k, v in src.items():
            if v.shape == sd[k].shape: sd[k] = v.clone()
            else: sd[k] = torch.zeros_like(sd[k]); sd[k][:, :v.shape[1]] = v
        model.policy.load_state_dict(sd); log(f"WARM START (widened input {src['mlp_extractor.policy_net.0.weight'].shape[1]} -> {venv.observation_space.shape[0]}) from {a.init_from}; lr {a.lr}")
    if model is None and a.init_from:
        model = PPO.load(a.init_from, env=venv, device=a.device, custom_objects=dict(learning_rate=a.lr, target_kl=a.target_kl))
        model.set_random_seed(a.seed)      # PPO.load restores the saved seed (0), which would give every warm start identical torch and env streams
        log(f"WARM START from {a.init_from} (counting steps from 0; lr {a.lr})")
    if model is None and a.algo == "sac":                                         # Phase 1 algorithm comparison (2026-11-11): off-policy SAC on the same env, from scratch
        from stable_baselines3 import SAC
        model = SAC("MlpPolicy", venv, learning_rate=a.lr, buffer_size=1_000_000, batch_size=512, train_freq=1, gradient_steps=8, gamma=a.gamma, learning_starts=20_000,
                    policy_kwargs=dict(net_arch=[256, 256], activation_fn=torch.nn.ELU), seed=a.seed, verbose=0, device=a.device)
        log(f"new SAC model: net [256,256] ELU, lr {a.lr}, batch 512, {a.envs} envs x 8 gradient steps per vec step, device {a.device}")
    if model is None:
        model = PPO("MlpPolicy", venv, learning_rate=(lambda p: a.lr * max(0.1, p)) if a.lr_decay else a.lr, n_steps=a.rollout, batch_size=a.batch, n_epochs=a.epochs,
                    gamma=a.gamma, gae_lambda=0.95, clip_range=0.2, ent_coef=a.ent, target_kl=a.target_kl,
                    policy_kwargs=dict(net_arch=[256, 256], activation_fn=torch.nn.ELU, log_std_init=a.log_std_init), seed=a.seed, verbose=0, device=a.device)
        log(f"new PPO model: net [256,256] ELU, lr {a.lr}, rollout {a.rollout}, batch {a.batch}, device {a.device}")
    json.dump(dict(vars(a), preflight=info), open(os.path.join(out, "config.json"), "w"), indent=1, default=str)

    class Cb(BaseCallback):
        def __init__(self):
            super().__init__(); self.f = open(os.path.join(out, "episodes.csv"), "a"); self.t0 = time.time(); self.last = done_steps; self.n_ep = 0; self.lens = []
            if self.f.tell() == 0: self.f.write("steps,ep_return,ep_len,wall_s\n")
        def _on_step(self):
            for i in self.locals.get("infos", []):
                if "episode" in i:
                    self.n_ep += 1; self.lens.append(i['episode']['l']); self.lens = self.lens[-500:]
                    try: self.f.write(f"{self.num_timesteps},{i['episode']['r']:.3f},{i['episode']['l']},{time.time() - self.t0:.0f}\n"); self.f.flush()
                    except BlockingIOError: pass          # Turing NFS hiccup killed R10_wall_s0 at 4M steps; losing one log line is fine
            if self.num_timesteps - self.last >= a.save_every:
                self.last = self.num_timesteps
                p = os.path.join(out, f"model_{self.num_timesteps // 1000}k"); self.model.save(p)
                import csv as _c
                try:
                    rows = list(_c.DictReader(open(os.path.join(out, "episodes.csv")))); R = [float(r["ep_return"]) / max(1.0, float(r["ep_len"])) for r in rows][-40:]
                    mr = float(np.mean(R)) if R else float("nan")
                except Exception: mr = float("nan")
                ml = float(np.mean(self.lens)) if self.lens else float('nan')
                if self.num_timesteps >= a.collapse_after and len(self.lens) >= 200 and ml < a.collapse_len:
                    log(f"COLLAPSE: mean episode length {ml:.1f} < {a.collapse_len} after {self.num_timesteps:,} steps. The policy is learning to terminate, not to hover. Stopping.", "ERROR")
                    self.model.save(os.path.join(out, 'model_collapsed')); raise SystemExit(2)
                log(f"step {self.num_timesteps:>9,}  episodes {self.n_ep:>6}  mean len {ml:6.1f}  reward/step (last 40) {mr:7.2f}  "
                    f"ref: zero {info['ret_zero']:.2f} / PD {info['ret_pd']:.2f}  saved {os.path.basename(p)}.zip  elapsed {(time.time() - self.t0) / 60:.1f} min")
            return True

    cb = Cb()

    def on_signal(sig, frame):
        log(f"received signal {sig}: saving interrupt checkpoint", "WARN")
        model.save(os.path.join(out, "model_interrupt")); LOG_F.flush(); sys.exit(143)
    signal.signal(signal.SIGTERM, on_signal); signal.signal(signal.SIGINT, on_signal)

    log(f"training to {a.steps:,} steps ...")
    t0 = time.time()
    model.learn(total_timesteps=max(0, a.steps - done_steps), callback=cb, reset_num_timesteps=not a.resume)
    model.save(os.path.join(out, "model_final"))
    log(f"TRAINING COMPLETE: {model.num_timesteps:,} steps in {(time.time() - t0) / 3600:.2f} h -> model_final.zip")
    venv.close()
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("tag"); ap.add_argument("--steps", type=int, default=3_000_000); ap.add_argument("--algo", choices=("ppo", "sac"), default="ppo"); ap.add_argument("--envs", type=int, default=8)
    ap.add_argument("--seed", type=int, default=0); ap.add_argument("--max-steps", type=int, default=1000)
    ap.add_argument("--lr", type=float, default=3e-4); ap.add_argument("--lr-decay", action="store_true"); ap.add_argument("--rollout", type=int, default=512); ap.add_argument("--batch", type=int, default=2048)
    ap.add_argument("--epochs", type=int, default=5); ap.add_argument("--gamma", type=float, default=0.99); ap.add_argument("--ent", type=float, default=0.0)
    ap.add_argument("--target-kl", type=float, default=0.02); ap.add_argument("--save-every", type=int, default=100_000)
    ap.add_argument("--log-std-init", type=float, default=-1.5); ap.add_argument("--fine-w", type=float, default=None); ap.add_argument("--env", choices=("hover", "track", "track3", "track4", "track5", "track6", "track7", "track8", "track9", "wall", "wall_ff", "wall_shapes", "write", "write_dr", "write_drc", "write15", "hybrid"), default="hover"); ap.add_argument("--fine-s", type=float, default=0.01); ap.add_argument("--lin-w", type=float, default=None); ap.add_argument("--tilt-w", type=float, default=None); ap.add_argument("--bad-pen", type=float, default=None); ap.add_argument("--p0-sigma", type=float, default=None); ap.add_argument("--v0-sigma", type=float, default=None); ap.add_argument("--init-from", default=None); ap.add_argument("--device", default="cpu"); ap.add_argument("--torch-threads", type=int, default=2)
    ap.add_argument("--collapse-after", type=int, default=400_000); ap.add_argument("--collapse-len", type=float, default=60.0); ap.add_argument("--resume", action="store_true"); ap.add_argument("--preflight-only", action="store_true")
    a = ap.parse_args()
    try:
        sys.exit(main(a))
    except Exception:
        log("UNHANDLED EXCEPTION — training aborted", "ERROR")
        for ln in traceback.format_exc().splitlines(): log("    " + ln, "ERROR")
        sys.exit(1)
