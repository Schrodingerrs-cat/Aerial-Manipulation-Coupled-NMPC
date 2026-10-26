"""Scenario lists (nominal model): path/speed variant, sensor-noise seed, initial hover gap and initial pad offset.  Separate seeds for development, validation and held-out lists.  Usage: am3_make_domain.py"""
import os, json, numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); PATHS = ["line", "line_fast", "line_T1", "circle", "circle_T4", "rounded_triangle", "rounded_triangle_T5", "free"]
def make(name, n, seed):
    rng = np.random.default_rng(seed); sc = [dict(id=f"{name}_{i:02d}", path=PATHS[i % len(PATHS)], seed=int(rng.integers(1, 10 ** 6)), gap0=float(rng.uniform(0.05, 0.08)), dyz=[float(rng.uniform(-0.01, 0.01)), float(rng.uniform(-0.01, 0.01))]) for i in range(n)]
    json.dump(dict(name=name, seed=seed, scenarios=sc), open(os.path.join(HERE, "..", "config", f"am3_domain_{name}.json"), "w"), indent=1)
make("dev", 16, 101); make("val", 16, 202); make("heldout", 30, 303)
