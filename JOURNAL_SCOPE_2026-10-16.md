# Journal scope: what to put in the paper (2026-10-16)

Sources read: `Probing_for_Properties_Journal_Outline.pdf` (approved), `main.tex`, `OLDmain.tex`,
`runtime_logs/` (paper_scope_notes, tipping_mode_* notes, ADAPTIVE_PRESS_IDEAS, tipT_full_suite/*,
tipT_bias_audit/*), `UNCERTAINTY_CHECKS_PLAN.md`, `src/irb120_ros2` notes and object GT.

Constraints: about two weeks to finish experiments and write half the paper. Nothing from the outline
is dropped unless it no longer applies. Each item below is marked **keep**, **reshape** or **defer**,
with a reason.

---

## 1. The pitch

**One sentence (paper):** A robot can decide how to touch an object it has never seen, recover the
object's mass, CoM height and friction from one touch, and from those numbers predict, before it
moves, whether a fast shove will land the object in a goal or knock it over.

**One sentence (for a talk or a non-specialist):** *Two boxes look identical, but one has its weight
up front. The robot pokes each once, then shoves one into the goal and refuses to shove the other
that way, and it predicted which one would fall before it touched either.*

That demo works with what you already have. The tipT CoM series has one exterior and five
interiors. With the measured μ_k ≈ 0.16 and z_c = 122 mm, the coast-phase gate μ_k·z_c ≤ d_f
(OLDmain eq. coast_gate) needs d_f ≥ ~19.5 mm:

| Config | d_f shoving +x | d_f shoving −x | Prediction |
|---|---:|---:|---|
| tipT_c | 30.0 mm | 30.0 mm | shovable either way |
| tipT_e | **7.4 mm** | 52.6 mm | topples during coast if shoved +x, whatever the push height. The robot should pick −x |
| tipT_d | 52.6 mm | **7.4 mm** | the mirror case |

A geometry-only policy cannot tell these apart, and it is the strongest figure and video available.

**Title options** (keeping "Probing for Properties"):
- *Probing for Properties: Touch Once, Then Shove*
- *Probing for Properties: Non-Prehensile Estimation that Predicts Its Own Failures*
- *Feel Before You Shove: Choosing, Executing and Trusting Non-Prehensile Probes of Mass, CoM and Friction*

**Thesis line from the outline, kept:** more affordance from less information. Point cloud,
wrench and a partial view; closed form at every step; nothing learned.

---

## 2. What changed since the outline (and why it helps)

1. **Forward tipping now runs on hardware** (`fwd_tip`, tipT a–e, c1–c3). Both modes are real, so
   the mode-choice contribution (C1) is a real result and not just a plan.
2. **One estimator for both modes** (`tip_core.py`). Only the tilt source differs, and press-pull
   results reproduce bit for bit across 47 trials. This is a clean, publishable piece of
   engineering.
3. **An identifiability result.** One pivot identifies only the first moments m·d and m·z_c (the
   λ-invariance (m, d, z) → (λm, d/λ, z/λ)). This is why the planar CoM is a known input. It also
   answers "why not estimate d?" in one line and links to Atkeson/An/Hollerbach. On tipT, m·r is
   within 0–2 %, even where m alone is 5–9 % off. Reviewers respect this kind of statement.
4. **Vision for the mode, touch for the amount.** The side camera classified 5/5 outcomes (tip,
   slide-then-tip, pure slide, a tip too light for the F/T). As a tilt sensor it had a 3–20 % scale
   error, which method D can't absorb. Quotable: *"The camera tells us what happened; touch tells
   us how much."*
5. **A mass-free slide-vs-tip predictor (CoST).**
   CoST = μ_t(h − μ_f L) / (d(1 − μ_t μ_f)) contains neither m nor z_c. It flagged tipT_d as a
   slide (0.56) and a/b/c as marginal (0.97–1.02; a and b did slide first). This is the formal
   "sliding limit" step of the outline's mode heuristic.
6. **A reactive guard that needs no prior** (robotlab07 4b29503). The slide probe (back off 2 mm
   at 10 mm) and the `too_easy` gate at 0.30 N together act as an observability gate:
   0.30 N ⇔ τ_on ≳ 0.047 N·m ⇔ d ≳ 12 mm. The paper can pair it with the predictive CoST as
   *predict, then verify in contact*.
7. **A bounded recovery cascade** (`tip_cascade.py`, at most 4 attempts, reasons logged). This is
   the outline's "mode choice held up?" logic, implemented.
8. **The error budget is mostly characterized.** Bias audit plus O1–O6: a systematic −6 % mass
   deficit. Confirmed causes: onset smoothing and a late zero (c1 video). The rest is unresolved
   and bounded. F/T recalibrated 2026-10-09. The monitor's press-pull failure (fingertip slip) is
   known and fits the outline's "monitor → forward" story.
9. **Not built yet:** the shove primitive (no release/shove code under `src`), the predictive
   η_slip margin (no code), and the press-pull "usability" interval (planned in
   `tipping_mode_selector_plan.md`).
10. **A correction to the old shove story.** Under velocity control, which the arm uses, mass
    cancels out of the tip window and the coast distance. Under force control, travel depends only
    on S = μ_k m g, so the errors in m and μ cancel. **Don't claim mass is needed for placement.**
    The honest and still interesting version: *each estimate gates a different failure*:
    - μ sets the distance.
    - z_c and d set the tip window and coast gate.
    - m (as S and τ) sets force feasibility. This is how "unshovable" objects like the monitor are
      predicted.

---

## 3. Scope, mapped onto the outline's four pieces

### C1 — Choosing the contact and the tipping mode (keep; mostly built)

| Outline item | Status | What goes in the paper |
|---|---|---|
| Press/push point as an optimization (Table II) | Torque-score argmax is drafted in `main.tex` | Keep. **Reshape the open "height vs pivot distance" question:** pick the press point that minimizes the normalized press demand p_min = d / (μ_f H − a), subject to the table constraint. H rewards height, the inward offset a penalizes distance from the pivot, and the trade-off is set by μ_f rather than by hand. This resolves the outline's red SH note in closed form. Forward tip stays as the pure torque argmax. |
| Constraints (inside top with margin, reachable, fingertip holds) | Camera view check (`top_support_v1`), MoveIt reachability, μ_f in p_min | Keep. Each constraint now maps to a specific implemented check. |
| JX's conditions (normal tilted away from pull; contact nearer the robot) | Normal screen ε_n is implemented | Mention ε_n. Nearer-to-base: one sentence as a reach tie-breaker. |
| Three-step mode heuristic | (1) roundness = `pivot_shape` straight/curved (implemented); (2) sliding limit = CoST before contact plus slide probe in contact (implemented); (3) press-pull usability = p_min interval (offline, ~1 day) | Keep all three. This is the flowchart figure. |
| Roundness measure and threshold | Straight vs curved from the cam3 SAM fit | Keep JX's definition ("edge = sharp line contact"). Set the threshold from the fillet sweep (C3). |
| Contact ablation (centroid, random, poor) | Not run | **Small version:** 3 placements × 2 objects × 3 trials. |
| Mode choice vs outcome for every object | Logged per episode (`tip_choice`, cascade reasons) | Free with the campaign. Decision: run it autonomously on every configuration, sweeps included. Its cost is zero. |

### C2 — Keeping the interaction safe (keep, reshaped to fit two weeks)

| Outline item | Status | Proposal |
|---|---|---|
| One governing hazard (slip), safe press band | Formulated in OLDmain | Keep the text. |
| Predictive η_slip margin, corrected within a tip | **Not implemented** | Reshape. Compute λ_slip(θ) = μ_t N_t − |f_t| from the measured wrench and the push-measured S, offline on every press-pull trial (margin traces), and show it matches observed slips and retries. Make it online only if time allows. Keep the rename η_slip vs η_safety. |
| Tip past the balance point | Press-pull already crosses θ* (fx sign flip counts as success) | Keep. Make sure the campaign includes **one clean crossing trace**, which the old draft called thesis-defining. |
| Forward-tip stopping rule; why only press-pull crosses safely | 10 %-of-peak reversal plus slide probe and `too_easy` are implemented | Keep. Add the observability gate (finding 6) as part of forward-tip safety. |
| Safety–accuracy curve | Not run | **Cheap version:** sweep the press-pull arc limit (e.g. −10°, −17°, −23°, past θ*) on 2 objects × 3 trials. Plot estimation error against min λ_slip. Optionally add the forward-tip reversal fraction (30 / 10 / 5 %). |
| Retry/failure rate vs the ISRR reactive scheme | Cascade logs reasons | Report the cascade's retries per object against the ISRR adaptive-press logs. |

### C3 — Saying where it stops working (keep)

| Axis | Objects | Status | Notes |
|---|---|---|---|
| CoM offset | tipT a–e (GT in `irb120_models`) | 1 run each plus c1–c3 | Repeat ×5. d → 0 (tipT_e) is the observability breakdown point and d large (tipT_d) is the slide breakdown point, which gives a two-sided envelope. |
| Base rounding (fillet) | cylinder_e, heart_a? | **Check:** are fillet bases printed? | If not, print 3 radii for the tipT base (0, ~5, ~15 mm). This sets the roundness threshold. |
| Base friction (bottom material) | tipT_c with felt / bare PLA / PTFE tape / rubber sheet | Not run; cheap | Weigh after adding the material. This sets the sliding-limit margin, and the CoST prediction is checked per material. |
| Fingertip friction | (outline lists it in condition studies) | — | **Defer.** One rubber tip is in scope per the outline. Cite the monitor fingertip slip as the observed limit. |

Output: one degradation figure (error vs axis, a breakpoint on each) plus the **sensitivity map**
σ(z_c)/z_c over (d, z_c). This single figure ties together the edge cases (d, e), the selector
thresholds and the error bars.

### V — Showing the numbers are worth having: the shove (keep; biggest risk)

- **Primitive (new, ~2–3 days):** a velocity-controlled stroke at height h, then a sharp finger
  deceleration (faster than μg ≈ 1.6 m/s²) so the object separates and coasts. Log the wrench
  (S directly), the release time, the final pose (cameras) and tip/no-tip.
- **Formulation (written in OLDmain; needs the correction in finding 10):** the push-height window
  h ∈ [h_min, h_max], the intensity σ, the coast gate μ_k z_c ≤ d_f, the direction choice, and a
  force-feasibility check for "unshovable".
- **Policies (four, as outlined):** ours (estimate-planned h, direction and σ); base push; top push;
  centroid-height push. Add a GT-parameter oracle as an extra if time allows.
- **Objects:** tipT_c, tipT_e (the identical-exterior pair), one everyday object (box or
  flashlight), and the monitor as the predicted-unshovable case (2–3 trials, stopped by the force
  check).
- **Trials:** 10 for ours on the key pair, 5 per baseline policy. That is about 75 shoves, roughly
  1.5 robot days.
- **Error tolerance:** perturb estimates jointly (m and μ correlated, per the cancellation) through
  the gate and report how much error flips the decision. This is the outline's "how wrong can it
  be" question.
- **One-tip sufficiency gate** (from TASK_SUFFICIENCY): implement it only as an offline pure
  function, `assess_task(evidence, spec) → READY / ABSTAIN(reason)`, and report the fraction of
  episodes where one tip was enough. **Defer** the full 180-episode policy comparison.

### Uncertainty (outline Plan A; it answers JX's NLS question)

- Analytic first-order terms (already derived): δz_c/z_c = δd/d, δm/m = δτ_on/τ_on − δd/d,
  scale error → ẑ_c ≈ k·z_c.
- Monte Carlo through the existing `uncertainty_budget()`, extended to forward tip.
- Coverage check on the ×5 repeats: does |est − GT| fall inside 2σ at the right rate?
- Carry it into the shove gate.
- Report the residual −6 % mass bias as a **validated error envelope** with its known contributions
  (smoothing, late onset, gravity-tilt candidate), not as solved.

---

## 4. Objects (outline wants 10–15)

| Group | Configurations | Count |
|---|---|---:|
| Designed, exact GT | tipT a–e (CoM), tipT_c × 3 materials (friction), 3 fillet bases | 11 |
| Everyday, measured GT | box, heart, flashlight, monitor (ISRR continuity), soda | 5 |
| New, hard to grasp (JX) | one large item: desktop printer or toaster | 1–2 |
| **Total** | | **17–18 configurations** |

**Named hard case** (outline VI-B): a light object that slides and has a poor top. Use the soda
bottle (domed top) or a light round-top container. Write the prediction (CoST < 1, p_min infeasible
or large, low SNR) **before** running it. The two-slug tipT stays out, as decided.

**Deliberate failures** (JX: yes):
- tipT_d slides (CoST).
- tipT_e is unobservable / too easy.
- The largest fillet loses the pivot.
- Monitor press-pull slips at the fingertip.
- The monitor is unshovable in the shove task.

That covers one failure per condition.

---

## 5. Two-week plan (Oct 17 → Oct 30)

| Days | Robot | Desk (agent-assisted, in parallel) |
|---|---|---|
| 1–2 | Commit robotlab07 code (results must point to a hash). 5-min F/T known-weight gain check. Attach friction materials and weigh; print fillets if missing. | **Freeze the estimator:** remove the extra onset smoothing (a confirmed bias), version it. Implement the offline p_min interval and CoST in the selector log. Start writing the Methods sections. |
| 3–4 | Build and test the shove primitive on tipT_c. | Write the contact and mode section (finish `main.tex` §IV) and the estimator/identifiability section. |
| 5–6 | **Estimation campaign:** tipT a–e ×5, friction ×5, fillet ×5, everyday ×3–5. About 95 episodes, roughly 2 days. | Batch-estimate each night. Monte Carlo, coverage. |
| 7 | Safety sweep (arc limits) plus contact ablation, about 40 episodes. | λ_slip traces offline. |
| 8–9 | **Shove campaign**, about 75 shoves. | Shove formulation section (corrected). |
| 10 | Hard case, reruns, video capture. | Figures: flowchart, envelope, sensitivity map, safety–accuracy, shove. |
| 11–14 | Buffer and writing | Results drafted from data. Intro and abstract last. |

Robot time is about 6 days, with roughly 4 days of slack. If the shove primitive slips past day 5,
cut the shove to the tipT pair plus ours vs top push (the most informative comparison), not the
estimation campaign.

**Writable now (≈50 %):** Intro framing, Prior work (OLDmain plus comparison table; re-check the
Phys2Real marks), Background, Contact & mode (drafted), Estimator & identifiability (notes are
written), Safety formulation (OLDmain), Shove formulation (OLDmain plus the correction), Scope and
assumptions table (`paper_scope_notes.md`).

---

## 6. Reviewer and advisor concerns, and where each is answered

| Concern | Where it's answered |
|---|---|
| ISRR rules were given without reasoning or validation (JX) | C1 p_min objective plus the contact ablation |
| "Why assume the planar CoM?" | Identifiability (first moments) plus the dual-tip being prior literature |
| Too few objects / one trial each | 17–18 configurations, ×5 repeats, failures reported |
| Where does it break? | C3 breakpoints plus the sensitivity map |
| Is the error bar honest? | Monte Carlo, coverage on the repeats, the bias reported as an envelope |
| Is the shove real? (the ISRR video was simulated) | V hardware, 4 policies |
| What's the baseline? | Shove: uninformed policies. Estimation: mode ablation (forward-only vs press-pull-only vs selected) plus the Yu et al. interaction-count comparison in the table. A full Yu reproduction is deferred. |
| Does mass matter for the task? | Each estimate gates a different failure (finding 10), so the claim is not overstated |
| Large, hard-to-grasp objects (JX) | Monitor plus printer or toaster |

---

## 7. Deferred (each with a reason)

- **Liquids, compliance tolerance, articulated objects, dual-contact or tactile tips:** already
  excluded in the outline.
- **Online predictive η_slip control:** replaced by measured margin traces. Online is a stretch goal.
- **Fingertip-friction sweep:** one tip is in scope. The monitor slip is the reported limit.
- **Full Yu et al. reproduction and the two-edge estimator:** the dual-tip was dropped as prior
  literature. Cite it and compare interaction counts.
- **The 180-episode sufficiency-policy comparison and learned/LLM selectors:** the offline gate
  plus the single-tip sufficiency rate cover the claim.
- **Online vision monitor:** stays an offline outcome check. Shadow mode only if free.
- **Planar CoM:** cited from prior work; one end-to-end demo as in the outline.

---

## 8. Decisions needed from you

1. Are the fillet bases (and cylinder_e / heart_a) already printed, or do we print 3 tipT fillet
   bases?
2. Which large object: printer or toaster? Which hard case: soda or a new light container?
3. Trials: 5 per estimation configuration and 10 per shove (key pair)? I recommend yes.
4. On the robotlab07 slide probe: keep "abort on slide" for the campaign (it's simpler and already
   validated), or allow re-probing (it keeps tipT_a-style slide-then-tip runs)? I recommend abort,
   and report the slide as a friction sample.
