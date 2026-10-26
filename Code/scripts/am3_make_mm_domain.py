"""Mismatch scenario lists for N/D/L (shared by all three): one family per scenario, sampled magnitude, noise seed, initial offsets.  Separate seeds for train (SAC), development, validation, held-out."""
import os, sys, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, ".."))
from ocrl.am3_mismatch import sample_scenario
def make(name, n, seed):
    rng = np.random.default_rng(seed); sc = [sample_scenario(rng, i, f"{name}_{i:03d}", int(rng.integers(1, 10 ** 6))) for i in range(n)]
    json.dump(dict(name=name, seed=seed, scenarios=sc), open(os.path.join(HERE, "..", "config", f"am3_domain_{name}.json"), "w"), indent=1)
make("mm_dev", 20, 11); make("mm_val", 20, 22); make("mm_heldout", 40, 33); make("mm_train", 300, 44)
