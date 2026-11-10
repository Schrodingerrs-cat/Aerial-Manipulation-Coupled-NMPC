A [0.0, 0.0, 1.5] -> B [2.0, 0.0, 1.5], 0.2 kg load on 0.6 m string, quad 1.05 kg. Video = simulation time 1.0x. Green = A, red = B, blue dots = load trail.
U_fast: Normal drone (load ignored), 1.8 s move: settled at 9.83 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 65.2 deg, overshoot 8.9 cm
U_best: Normal drone (load ignored), best: 4.75 s move: settled at 4.35 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 5.2 deg, overshoot 0.2 cm
S: Load-aware, swing suppressed (<3 deg), 4.3 s plan: settled at 4.02 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 4.4 deg, overshoot 1.8 cm
E: Load-aware, swing exploited, 1.8 s plan: settled at 2.79 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 41.0 deg, overshoot 4.1 cm
