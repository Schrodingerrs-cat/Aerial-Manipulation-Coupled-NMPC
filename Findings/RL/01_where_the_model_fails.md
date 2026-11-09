# Where the model fails, and what a two-channel correction can reach (30 Oct 2026)

Diagnostic step before training L. Two questions: **where does N actually fail**, and **how much of that failure can the proposal's correction interface even express**. Scripts: `Code/scripts/am3_projection.py`; data `Code/results/am3_projection/projection.json`, `Code/results/am3_runs/mmdev_{N,D,B}/gate.json`.

## 1. Failures by error family (mismatch development list, 4 scenarios each)

| family | N | D (EKF) | B | effect of the classical correction |
|---|---|---|---|---|
| payload | **0/4** | **0/4** | 0/4 | none |
| contact | 2/4 | 2/4 | 0/4 | none |
| propulsion | 1/4 | **3/4** | 0/4 | **fixes it** |
| disturbance_slow | 4/4 | 4/4 | 0/4 | not needed |
| disturbance_sudden | 4/4 | 4/4 | 0/4 | not needed |
| **total** | 11/20 | 13/20 | 0/20 | |

Failure signatures (N): payload fails `path_tracking_rms`/`coverage`/`endpoint` in **all four** scenarios - a persistent tracking bias, not a stability loss. Propulsion fails `contact_fraction`/`recovery` - it loses contact. Contact fails a mix.

D's entire advantage over N comes from the propulsion family. It changes nothing elsewhere.

## 2. Oracle projection error

For the prediction to match reality the correction must supply the generalised force `dtau = M_nom (nudot_true - nudot_nom)` (9-vector). The interface can only supply vectors in the 2-D span `D(x) = [J_C^T n, E_pitch]`. Whatever lies outside that span is unreachable by **any** corrector, classical or learned.

Metric: kinetic (`M^-1`) norm, which is unit-consistent for generalised forces. 40 states sampled from the contact phases of a nominal `line` run, 6 mismatch draws per family. Bounded column additionally respects `|d| <= D_MAX = [0.5 N, 0.10 N m]`.

| family | unrepresentable (span) | unrepresentable (bounded) | error lives in: base force / base moment / joint torque |
|---|---|---|---|
| payload | **98.3 %** | 98.3 % | 64 / 17 / 19 % |
| contact | **26.9 %** | 26.9 % | 76 / 14 / 9 % |
| propulsion | 75.9 % | 75.9 % | 97 / 3 / 0 % |
| disturbance_slow | 100 % | 100 % | 100 / 0 / 0 % |
| disturbance_sudden | 100 % | 100 % | 100 / 0 / 0 % |

Bounded equals unbounded everywhere, so `D_MAX` is **not** the binding limit - the span is.

Why these numbers are physically sensible:
* **payload**: a mass at the hand with a lateral offset adds a gravity force (world −z), a roll moment and a joint torque. The interface has a contact-normal force (−x) and a **pitch** moment, and **no joint channel at all**. Almost nothing projects.
* **contact**: a stiffness/damping error is a force along the contact normal, which is exactly channel 1. The ~27 % residual is the friction (`mu_scale`) part, which is tangential.
* **propulsion**: a thrust-coefficient error acts along the body thrust axis, not the wall normal.
* **disturbances**: a world-frame force on the base; channel 1 acts at the contact point, so it carries an unavoidable moment with it.

## 3. The important nuance

**Unrepresentable does not imply the task fails, and representable does not imply it succeeds.** Both disturbance families are 100 % unrepresentable yet N passes 4/4 - the disturbances (0.1-0.5 N against a 15 N vehicle) are small enough that ordinary feedback absorbs them. Conversely propulsion is 76 % unrepresentable yet D fixes it, because the ~24 % that *is* representable happens to lie in the task-critical direction: `F_hat_n` directly sets how hard the pad presses, and the propulsion failure mode is loss of contact.

So the projection bound answers "can the interface express this error", not "will the task fail". Task outcome = representable fraction x task-criticality of that direction x what feedback already handles. `Plan.md` s8 warns exactly against reading unrepresentability as proof against learning, and this is why.

## 4. What this implies for L

| family | representable | N fails? | D exploits it? | headroom for L |
|---|---|---|---|---|
| contact | **73 %** | partly (2/4) | **no** (2/4) | **largest - this is the target** |
| propulsion | 24 % | yes (1/4) | yes (3/4) | small, already taken |
| payload | 2 % | yes (0/4) | no | ~none; structural limit |
| disturbance_* | 0 % | no | n/a | none needed |

**The contact family is where L should be trained and evaluated.** It is the one place where the correction channels match the physics (73 % representable) and the classical estimator fails to use it (no improvement over N). It is also the "non-parametric" column of the proposal's hypothesis, so a win there is directly on-thesis.

**Payload is a structural finding, not a tuning failure.** At 98 % unrepresentable, no corrector on this interface can fix it. Reporting that with this number is a stronger result than reporting a learned policy that fails for unexplained reasons.

## 5. Caveats
* 4 scenarios per family. The payload result (0/4 for three different controllers with identical failure criteria) is a strong signal; the rest are thin. Regenerate the dev/val/held-out sets at 15-20 per family before any claim is published.
* The projection is evaluated at frozen states sampled from a *nominal* trajectory, not in closed loop. It bounds what the interface can express at those states; it does not simulate what feedback would do.
* Disturbance magnitudes (0.1-0.5 N, about 1-3 % of weight) may simply be too gentle to challenge N. Re-run harder before concluding that disturbances need no correction.

---

# UPDATE (30 Oct 2026): the v1 result does not replicate at larger sample size

Section 1 above used the v1 development list: 20 scenarios, 4 per family. Repeating it on `mm2_dev` (50 scenarios, 10 per family, same frozen sampler, fresh seeds, identical acceptance criteria) **reverses the headline**.

| family | N (of 10) | D (of 10) | v1 was (of 4) |
|---|---|---|---|
| payload | **0** | **0** | N 0, D 0 — confirmed |
| contact | **5** | **3** | N 2, D 2 — D now *worse* |
| propulsion | 8 | 8 | N 1, D 3 — **D's advantage disappears** |
| disturbance_slow | 10 | 10 | 4, 4 — confirmed |
| disturbance_sudden | 10 | 10 | 4, 4 — confirmed |
| **total** | **33/50** | **31/50** | N 11/20, D 13/20 |

## The classical correction does not improve on nominal NMPC

Paired on the same 50 scenarios:

| | count |
|---|---|
| both pass | 29 |
| both fail | 15 |
| N passes, D fails | **4** |
| D passes, N fails | **2** |

Exact McNemar on the 6 discordant pairs: **p = 0.688**. The difference is **not statistically significant**, and the point estimate now favours N.

The v1 "D 13/20 vs N 11/20" was a 2-scenario difference on 20 paired samples — it was never significant either, and at 50 samples it does not replicate. **The claim that the disturbance observer improves on nominal NMPC is not supported by the evidence.** `Findings/STATUS.md` and the README should be corrected; the one-scenario demonstration in `Media_results/AM3_NvsD_propulsion_mm007_2026-10-30` is a real case where D rescues N, but it is one of only two such cases in 50, against four where D breaks what N could do.

## What this changes for L

It makes the research question **cleaner**, not harder:

* The honest baseline for the report is now "classical correction gives no measurable benefit on this task", which is itself a result worth reporting — the proposal explicitly asks whether classical methods suffice, and the answer so far is that they neither help nor hurt.
* L no longer has to beat a strong classical baseline; it has to beat **nominal N**, and D's failure to do so is the context that makes an RL win interesting if it happens.
* The target is unchanged and now better supported: **contact** is the family with headroom (N 5/10, so half the scenarios fail) *and* the family where the correction channels are 73 % representable. Propulsion is nearly saturated (8/10) and the disturbance families are fully solved, so neither offers room.

## Payload is a hard wall, and worse than it looked

8 of 10 payload scenarios **abort with loss of control** — the vehicle is lost, not merely mistracking (the v1 sample showed only tracking failures, which understated it). Combined with the 98.3 % projection residual, payload mismatch is outside what this correction interface can address, by construction. This should be reported as a structural limitation of the proposal's two-channel design, with the projection number as the evidence.

## Method note

Both N and D ran the identical 50 scenarios with identical seeds, plants and criteria; the only difference is the corrector. The batch used 6 parallel workers, so **timing from these runs is invalid** (the repository already flags this for parallel batches) — only pass/fail is used here.

---

# UPDATE 2 (30 Oct 2026): the disturbance result was an artefact of magnitude

The caveat in section 5 was correct. `mm2_dist_strong` repeats the two disturbance families with amplitudes scaled 4x and 8x (the sampler itself is untouched; the amplitude is scaled afterwards). N only, same acceptance criteria.

| scale | family | N passes | amplitude | as % of the 14.9 N vehicle weight |
|---|---|---|---|---|
| 4x | disturbance_slow | 6/10 | 0.70-1.18 N | 8 % |
| 4x | disturbance_sudden | 8/10 | 0.85-1.93 N | 13 % |
| 8x | disturbance_slow | 4/10 | 0.85-2.36 N | 16 % |
| 8x | disturbance_sudden | 4/10 | 1.61-3.97 N | 27 % |
| | **total** | **22/40** | | |

At the v1 amplitudes (0.1-0.5 N, 1-3 % of weight) N passes 10/10. At 8x it passes 8/20. **"Disturbances need no correction" is only true for disturbances small enough that it is trivially true.** The dominant failure is `base_max` - the vehicle is pushed off station beyond the 20 mm tolerance - with contact loss and a few aborts at 8x.

This does **not** reopen the disturbance families as a target for L, because scaling an amplitude does not change its direction: the projection residual stays at 100 %. A world-frame force on the base is not in the span of (contact-normal force, body pitch moment) whatever its size. So the honest statement for that cell of the proposal's 2x2 table is **"a correction is needed at realistic magnitudes, but this interface cannot provide it"** - a structural finding, not an RL-versus-classical one.

## Consolidated picture

| family | does N fail? | representable by the interface | verdict |
|---|---|---|---|
| payload | yes, severely (0/10, 8 aborts) | 2 % | structural wall |
| **contact** | **half (5/10)** | **73 %** | **the only target with both need and headroom** |
| propulsion | rarely (8/10) | 24 % | nearly saturated, little room |
| disturbance, small | no (10/10) | 0 % | no correction needed |
| disturbance, large | yes (22/40) | 0 % | structural wall |

Contact is the single cell where a correction is both **needed** and **expressible**. L is trained and evaluated there.
