# Training L: the design, and why each piece is what it is (30 Oct 2026)

Read `01_where_the_model_fails.md` first — it is the reason this experiment is aimed where it is.

## What L actually is

**Not** a flight controller. The NMPC still produces every command. L outputs **two numbers per control cycle** that are injected into the NMPC's *prediction model* before it optimises:

```
d = (F̂ₙ, τ̂_pitch)          clipped to |d| ≤ [0.5 N, 0.10 N·m], rate-limited to [0.25, 0.05] per cycle
```

`F̂ₙ` adds a force along the wall normal at the contact point; `τ̂_pitch` adds a moment about the body pitch axis. The policy can change what the controller *believes*, never what it is allowed to do — all constraint handling, rotor limits and tilt limits stay with the NMPC.

**Verified**: action ≡ 0 reproduces nominal N **bit-for-bit** (`results/am3_runs/zparity_Z`). So any difference L makes is the policy's doing and nothing else. This is the property that makes the comparison clean, and it is checked, not assumed.

## Why contact, and only contact

From the projection analysis:

| family | N fails? | representable by the two channels | trained on? |
|---|---|---|---|
| payload | 0/10, 8 aborts | 2 % | **no** — outside the interface by construction |
| **contact** | **5/10** | **73 %** | **yes** |
| propulsion | 8/10 | 24 % | no — little room left |
| disturbance | 10/10 small, 22/40 strong | 0 % | no — not expressible at any magnitude |

Training on a mixture of all five families, as the original script did, spends most of its samples on errors the policy **cannot represent no matter what it learns**. That is not a fair test of whether learning helps; it is a test of whether the interface spans the error, and we already answered that analytically.

Contact mismatch (`k_scale` 0.6–1.6, `c_scale` 0.5–2.0, `mu_scale` 0.5–2.0) is the one family where the channels line up with the physics: a stiffness or damping error *is* a force along the contact normal, which is exactly channel 1. The ~27 % residual is the friction term, which is tangential and stays unreachable.

Pools (new files, nothing overwritten):

| pool | n | use |
|---|---|---|
| `mm2_train_contact` | 400 | training episodes |
| `mm2_val_contact` | 40 | checkpoint selection |
| `mm2_dev_contact` | 40 | diagnosis during development |
| `mm2_heldout` | 100 (20/family) | the final paired N/D/L comparison, used once |

## Observation (48 numbers, `ocrl/am3_rl.py: build_obs`)

| group | what | why it is there |
|---|---|---|
| phase one-hot (11) | which task phase | the right correction during approach differs from during trace |
| contact flag, progress (2) | measured contact, path fraction | |
| forces (3) | measured Fₙ, predicted Fₙ, **their difference** | the residual is the direct evidence of a contact-model error |
| gap, closing speed (2) | normalised | pre-contact information — lets the policy act *before* impact |
| pad error, pad velocity error (6) | vs reference | the thing being tracked |
| base error, base velocity (6) | vs fixed hover reference | the other thing being tracked |
| attitude, angular rate, joint rates (9) | | vehicle state |
| previous correction (2) | | continuity; the action is rate-limited, so the policy needs to know where it is |
| solver diagnostics (3) | slack, status, fallback | tells the policy when it is pushing the NMPC into trouble |
| force-residual history (4) | last 4 samples | a slow drift and a sudden impact look different over time |

All measured or nominal-model quantities. **No privileged information**: the policy never sees the true `k_scale`, `mu_scale` or any plant parameter. D cannot see them either, so the comparison is fair.

## Reward (`CorrectionEnv.step`)

```
r = +1.0                                             alive, still in the task
    − 0.3 · min(e_base / 5 mm,  2)                   base position error
    − 0.3 · min(e_pad  / 3 mm,  2)                   pad tracking error
    − 0.05 · ‖a‖²                                    correction effort
    − 1.5 · clip((peak Fₙ − 2 N) / 4 N, 0, 2)        only above half the 4 N force cap
    − 2.0 per new contact loss
    − 20 on failure,  + 10 on completing the task
```

The scales are the gate thresholds, not free parameters: 5 mm and 3 mm are the distances the acceptance criteria care about, 4 N is the pad compression limit. The `+1.0` per step means an early failure forfeits several hundred points of future reward, so the `−20` is a nudge rather than the main signal. No orientation or force-setpoint term — those were removed with the task change.

## Algorithm

SAC, 5 parallel environments, `lr` 3e-4, buffer 300k, batch 256, γ 0.98, `ent_coef` auto, net [128, 128], CPU. SAC rather than PPO because each environment step costs **15.7 ms** (it contains a full acados solve), so sample efficiency matters more than throughput.

Measured: 200k steps ≈ 20 min per seed on this machine. Three seeds is under an hour.

## Protocol, fixed before running

1. **Pilot** — 40k steps, one seed. Does return improve at all? If not, fix the setup before spending more.
2. **Full** — 200k steps, seeds 0/1/2.
3. **Checkpoint selection on `mm2_val_contact`.** Never on held-out data.
4. **Paired held-out comparison** N vs D vs L on `mm2_heldout`, reported per family.
5. Freeze before step 4 and record the manifest.

## What counts as a result — decided now, not after

* **L must beat its own zero-action baseline.** Zero action *is* N, so if L does not beat N on contact scenarios it has learned nothing useful, and no amount of framing changes that.
* A **negative** result is publishable, but only with evidence that L was trained fairly: learning curves, three seeds, validation-based selection. "Undertrained policy loses" is not a finding.
* Because D gives no significant benefit (p = 0.688 on 50 paired scenarios), the meaningful comparison is **L vs N**, with D as context.

## Known risks

* **Contact mismatch may be partly unlearnable too.** 27 % of it is friction, which is tangential and outside the span. If L plateaus well short of fixing everything, check that ceiling before blaming the policy.
* **Episodes are long** (~750 control steps), so 200k steps is only ~270 episodes. Credit assignment over that horizon is hard. If learning stalls, shortening episodes to the contact phase is the first thing to try.
* **A contact-specialist may hurt other families.** The held-out comparison covers all five, so this will show up rather than hide. If it does, report it — a correction that helps one error and harms others is an important property of the method, not a result to suppress.
