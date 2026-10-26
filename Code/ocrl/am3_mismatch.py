"""Named plant-model mismatch families, unknown to every controller (Plan s9.1/s9.2): payload at the hand, contact stiffness/damping/friction, propulsion (thrust coefficient, motor lag), slow and sudden base disturbances.
Each scenario carries ONE family with a sampled magnitude; the plant realises it (am3_plant.apply_mismatch and contact scales in am3_episode)."""
import numpy as np
FAM = ("payload", "contact", "propulsion", "disturbance_slow", "disturbance_sudden")
PATHS = ("line", "circle", "rounded_triangle")

def sample_mismatch(rng, fam):
    u = lambda a, b: round(float(rng.uniform(a, b)), 4)
    if fam == "payload": return dict(payload=dict(kg=u(0.03, 0.10), pos_hand_m=[0.0, u(-0.05, 0.05), u(-0.06, -0.02)]))
    if fam == "contact": return dict(k_scale=u(0.6, 1.6), c_scale=u(0.5, 2.0), mu_scale=u(0.5, 2.0))
    if fam == "propulsion": return dict(kf_scale=u(0.9, 1.1), motor_tau_scale=u(0.7, 1.4))
    ax = int(rng.choice([0, 1, 2])); sg = float(rng.choice([-1, 1]))
    if fam == "disturbance_slow": return dict(disturbance=[dict(kind="sine", axis=ax, amp=sg * u(0.1, 0.3), t0=u(2.0, 6.0), freq=u(0.2, 0.5))])
    return dict(disturbance=[dict(kind="step", axis=ax, amp=sg * u(0.2, 0.5), t0=u(7.0, 9.0), dur=u(0.5, 1.5))])

def sample_scenario(rng, i, sid, seed, fam=None):
    fam = fam or FAM[i % len(FAM)]
    return dict(id=sid, family=fam, path=PATHS[int(rng.integers(0, 3))], seed=int(seed), gap0=round(float(rng.uniform(0.05, 0.08)), 4), dyz=[round(float(v), 4) for v in rng.uniform(-0.01, 0.01, 2)], mismatch=sample_mismatch(rng, fam))
