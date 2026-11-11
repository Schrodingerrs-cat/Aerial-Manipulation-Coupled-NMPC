A [0.0, 0.0, 1.5] -> B [2.0, 0.0, 1.5], 0.2 kg load on 0.6 m string, quad 1.05 kg. Video = simulation time 1.0x. Sensing: {'seed': 0, 'wind': False} (qp_run.SIG). Green = A, red = B, blue dots = load trail.
U_fast: Normal drone (load ignored), 1.8 s move: settled at 9.70 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 65.9 deg, overshoot 10.4 cm
U_best: Normal drone (load ignored), best: 4.75 s move: settled at 5.39 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 5.3 deg, overshoot 0.5 cm
S: Load-aware, swing suppressed (<3 deg), 4.3 s plan: settled at 4.04 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 3.9 deg, overshoot 0.5 cm
E: Load-aware, swing exploited, 1.8 s plan: settled at 1.99 s (drone within 5 cm of B and swing < 2 deg from then on), max swing 41.7 deg, overshoot 2.1 cm
