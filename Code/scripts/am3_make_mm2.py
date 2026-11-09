"""Larger mismatch scenario lists (v2).  The v1 lists carry 4 scenarios per family in dev/val and 8 in held-out, which is too few to support a per-family claim - and the project's question
("which kinds of model error need a learned correction") is exactly a per-family claim.  These lists use the same frozen sampler (ocrl/am3_mismatch.py, untouched) with fresh seeds and more draws.

Written as NEW files: the v1 lists and every result produced against them stay valid.
  mm2_dev      50 scenarios (10 per family)   diagnosis and tuning
  mm2_val      50 scenarios (10 per family)   checkpoint selection for L
  mm2_heldout 100 scenarios (20 per family)   the final paired N/D/L comparison, used once

Also writes mm2_dist_strong: a separate probe that scales the external-disturbance amplitudes by 4x and 8x.  The v1 disturbance amplitudes are 0.1-0.5 N against a 15 N vehicle (1-3 % of weight)
and N passes 4/4 on them, so "disturbances need no correction" may be an artefact of the magnitude rather than a property of the controller.  This probe tests that, and is reported separately -
it is a diagnostic, not a change to the agreed experiment.

Usage: am3_make_mm2.py [--force]      (refuses to overwrite existing files without --force)
"""
import os, sys, json, copy, argparse, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_mismatch import sample_scenario, FAM

CFG_DIR = os.path.join(HERE, "..", "config")


def make(name, n, seed, force):
    path = os.path.join(CFG_DIR, f"am3_domain_{name}.json")
    if os.path.exists(path) and not force: raise SystemExit(f"{path} exists; pass --force to overwrite")
    rng = np.random.default_rng(seed)
    sc = [sample_scenario(rng, i, f"{name}_{i:03d}", int(rng.integers(1, 10 ** 6))) for i in range(n)]
    json.dump(dict(name=name, seed=seed, scenarios=sc), open(path, "w"), indent=1)
    fam = {f: sum(1 for s in sc if s["family"] == f) for f in FAM}
    print(f"{name:16s} {n:4d} scenarios   per family: {fam}")
    return sc


def make_dist_strong(seed, force):
    """Disturbance families only, amplitudes scaled 4x and 8x.  The sampler is untouched; the amplitude is scaled afterwards."""
    path = os.path.join(CFG_DIR, "am3_domain_mm2_dist_strong.json")
    if os.path.exists(path) and not force: raise SystemExit(f"{path} exists; pass --force to overwrite")
    rng = np.random.default_rng(seed); sc = []
    for scale in (4.0, 8.0):
        for k, fam in enumerate(("disturbance_slow", "disturbance_sudden")):
            for j in range(10):
                s = sample_scenario(rng, 0, f"ds{int(scale)}_{fam[-4:]}_{j:02d}", int(rng.integers(1, 10 ** 6)), fam=fam)
                s = copy.deepcopy(s); s["mismatch"]["disturbance"][0]["amp"] *= scale; s["amp_scale"] = scale
                sc.append(s)
    json.dump(dict(name="mm2_dist_strong", seed=seed, note="disturbance amplitudes scaled 4x and 8x; diagnostic only", scenarios=sc), open(path, "w"), indent=1)
    amps = [abs(s["mismatch"]["disturbance"][0]["amp"]) for s in sc]
    print(f"{'mm2_dist_strong':16s} {len(sc):4d} scenarios   |amp| {min(amps):.2f}-{max(amps):.2f} N (vehicle weight 14.9 N)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--force", action="store_true"); a = ap.parse_args()
    make("mm2_dev", 50, 101, a.force)
    make("mm2_val", 50, 202, a.force)
    make("mm2_heldout", 100, 303, a.force)
    make_dist_strong(404, a.force)
    print("\nwritten to Code/config/ — the v1 mm_* lists are untouched")


def make_family(name, fam, n, seed, force):
    """A single-family training pool.  The projection analysis (Findings/RL/01) identifies `contact` as the only family where the two-channel
    interface both matters and is expressible, so L is trained there rather than on a mixture that is mostly unreachable."""
    path = os.path.join(CFG_DIR, f"am3_domain_{name}.json")
    if os.path.exists(path) and not force: raise SystemExit(f"{path} exists; pass --force to overwrite")
    rng = np.random.default_rng(seed)
    sc = [sample_scenario(rng, 0, f"{name}_{i:03d}", int(rng.integers(1, 10 ** 6)), fam=fam) for i in range(n)]
    json.dump(dict(name=name, seed=seed, family=fam, scenarios=sc), open(path, "w"), indent=1)
    print(f"{name:20s} {n:4d} scenarios, family={fam}")
