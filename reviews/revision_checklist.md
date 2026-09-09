# Revision checklist — Multi-sensor CR dataset (Nature Scientific Data)

All 61 items (Editor + Reviewers 1–5), ranked by difficulty/tedium rather than grouped by reviewer. Within each tier, order doesn't matter much — pick whatever's convenient.

**Status legend**
- `[x]` Drafted & verified — confirmed present in your live `main.tex` and/or authors-response document
- `[~]` Verified against code/data, not yet drafted into text
- `[ ]` Not yet addressed
- `[!]` Blocked / needs a decision or teammate input before drafting

**Tracked files**: manuscript is `main.tex`; the real response letter is your **"Nature Scientific Data CR dataset authors response"** document (`\TODO{}`, `\section*{Reviewer N}` structure) — **not** `reviews/response_reviewer5.tex`, which is a separate file I generated early on and isn't the one you're actively editing. Manuscript edits are tagged `\AS[label: ]{...}`.

---

## ⚠ Known bugs in the live documents — fix before submitting

- **`main.tex` — duplicated 5.31 sentence.** The identical `\AS[5.31: ]{...}` block (system_clock vs. high_resolution_clock) appears twice in a row, straddling "The motor process is launched last...". Delete the second occurrence.
- **`main.tex` — broken orphan fragment tagged 5.36.** In the Usage Notes "Getting started" paragraph, `\AS[5.36: ]{The filtering }` is an incomplete sentence fragment ("...aligned timestamps. The filtering Users who wish to regenerate..." — ungrammatical). Separate from the real, correctly-drafted 5.36 edit later in "Limitations of applicability". Remove it or finish the sentence.
- **Authors response — Comment 5.36 response is in the wrong slot.** `\paragraph{Response to Comment 5.36}` is commented out; its text landed under `\paragraph{Changes made to the article}` instead. Fix:
  ```latex
  \paragraph{Response to Comment 5.36}
  We agree. We have added an explicit statement to the Usage Notes that \texttt{filtfilt} is acausal and cannot be reproduced in a causal, online or real-time setting, and that users requiring causal filtering should apply their own causal filter instead.

  \paragraph{Changes made to the article}
  Usage Notes was revised to state this explicitly.
  ```
- **Authors response — Comment 5.31 is blank** even though `main.tex` already has the fix:
  ```latex
  \paragraph{Response to Comment 5.31}
  We agree. The manuscript incorrectly named \texttt{std::chrono::high\_resolution\_clock} as the source of the FBG timestamp. In the released C++ client, \texttt{high\_resolution\_clock} is used only to bound the acquisition loop's runtime; each frame's actual timestamp is taken from \texttt{std::chrono::system\_clock}, which is epoch-anchored and consistent with the wall-clock timestamps used by the other acquisition processes. We have corrected the manuscript text accordingly.

  \paragraph{Changes made to the article}
  The Data collection interfaces section was corrected to reference \texttt{std::chrono::system\_clock} instead of \texttt{std::chrono::high\_resolution\_clock}.
  ```
- **Authors response — Comment 5.40 response omits the coordinate-frame point**, the specific thing the reviewer flagged:
  ```latex
  \paragraph{Response to Comment 5.40}
  We agree; this statement was inaccurate. As the reviewer notes, and as Table 3 itself shows (curvature reported in $\text{mm}^{-1}$ in the raw file versus $\text{m}^{-1}$ in the processed file), the FBG curvature and bending angle are unit-converted, timestamp-corrected, low-pass filtered, and resampled identically to the other processed streams; the reconstructed 3-D shape additionally undergoes a coordinate-frame change into the robot body frame during this processing. We have corrected the text accordingly.
  ```
- **Dependency, not a bug**: 5.33's response claims the timestamp fix is "applied in the same reprocessing pass used to address... Comment 5.19" — but 5.19 (below, Larger tier) isn't done yet. Don't submit 5.33 as-is until it's true.
- **Cosmetic**: authors-response document ends with two `\end{document}` in a row — delete the stray second one.

---

## Trivial (14) — one-line / mechanical fixes

- [x] **5.6** (R5) — Table 1: "Weight" → "Mass" (g, not N). *Drafted, confirmed in both documents.*
- [x] **5.11** (R5) — "minimize" → "reduce" (multiple occurrences). *Drafted, confirmed in both documents.*
- [x] **5.13** (R5) — Transpose vs. inverse for $g_\text{base}$. *Response drafted — explains no change was needed (homogeneous transform requires the actual inverse).*
- [x] **5.15** (R5) — Merge Data Availability and Code Availability sections. *Drafted, confirmed in both documents.*
- [x] **5.29** (R5) — State FBG delay value (13.4 ms) + sign convention in Methods. *Drafted, confirmed in both documents.*
- [x] **5.31** (R5) — Fix clock name: `system_clock`, not `high_resolution_clock`. *Manuscript drafted (⚠ duplicated, see bugs); response letter blank (fix given above).*
- [x] **4.3** (R4) — State CC-BY 4.0 in the paper text. *Drafted, confirmed in both documents.*
- [X] **E3** (Editor) — Remove section numbering.
- [ ] **E5** (Editor) — Add data citation (DOI) at start of Data Record.
- [X] **2.3** (R2) — Typo "measuraments" → "measurements".
- [ ] **3.2** (R3) — Figure caption notes (calibration sweeps, zero baselines).
- [ ] **4.6** (R4) — Dataset versioning / long-term maintenance note.
- [ ] **5.38** (R5) — Rename "Euclidean distance" → "planar distance" (or compute full 3-D).
- [~] **5.16** (R5) — 502 vs. 503 FBG point indices. Verified: should read 0–501. Not yet drafted.

## Easy (15) — short, self-contained writing/edits

- [x] **5.36** (R5) — State `filtfilt` is acausal in Usage Notes; filtered data still valid offline. *Manuscript drafted (⚠ orphan fragment to remove, see bugs); response letter structural bug (fix given above).*
- [x] **5.39** (R5) — Clarify FBG "raw" file already contains Shape CORE–reconstructed data. *Drafted, confirmed in both documents.*
- [x] **5.40** (R5) — Drop "without further modification"; state actual modifications. *Manuscript drafted correctly; response letter incomplete (fix given above).*
- [ ] **E1** (Editor) — Reviewer-suggested refs: include only where relevant.
- [ ] **E4** (Editor) — Check Background & Summary for out-of-scope results/conclusions.
- [ ] **1.2** (R1) — Soften "clear physical correspondent in the governing equations" claim.
- [ ] **2.2** (R2) — Tendon–disk contact force sensor limitation statement. Owner: Chengnan (rationale).
- [ ] **4.4** (R4) — Clarify 2.2.4 "pose of selected..." sentence. Cluster with 1.1, 5.7.
- [ ] **4.5** (R4) — Share recording videos if available. Owner: Greg.
- [ ] **5.5** (R5) — Fix Table 1 description (contains more than geometrical parameters).
- [ ] **5.7** (R5) — $g_\text{fix}$/$g_\text{wand}$ terminology consistency pass. Cluster with 1.1, 4.4. *(Core clarification for $g_\text{fix}$ itself already drafted under 5.7's own comment.)*
- [ ] **5.12** (R5) — Justify 3D-printed pulleys; check roundness/eccentricity. Owner: Chengnan + Greg.
- [ ] **5.14** (R5) — General prose polish.
- [ ] **5.28** (R5) — Retain OptiTrack validity mask (`disk_k_is_valid`) in processed output. Owner: Spencer.
- [ ] **5.32** (R5) — State/quantify 4-motor sequential-read timestamp latency. Owner: Tongjia.

## Moderate (21) — real content/code work, single-person scope

- [x] **5.33** (R5) — ATI timestamp end→center correction (−2.5 ms). *Manuscript + `process_data.m` fix + response drafted; bundled with 5.19 reprocessing (⚠ not done yet, see bugs).*
- [~] **5.17** (R5) — `mocap_frames.csv` column order. Verified: actual order is `[roll,pitch,yaw,x,y,z]`. Not yet drafted. Spencer sign-off suggested.
- [~] **5.18** (R5) — `"circle" or "Lissajous"` logic bug. Verified harmless (hold phase within 0.2–0.3°). Not yet drafted. Owner: Tongjia.
- [~] **5.24** (R5) — Quasi-static protocol not reproducible from released code. Underlying data verified fine. Owner: Tongjia.
- [ ] **1.1** (R1) — Figure/diagram clarity (contact wand, actuator-sensor labeling). Cluster with 4.4, 5.7.
- [ ] **2.1** (R2) — Embodied intelligence / morphological computation paragraph. Owner: Chengnan input.
- [ ] **3.1** (R3) — Generalization-limitations discussion. Same E2 tension as 5.8/GVS.
- [ ] **4.1** (R4) — Related robots/actuation-architecture paragraph. Combine with 5.1. Owner: Chengnan input.
- [ ] **4.2** (R4) — FBG-vs-MoCap divergence under contact (new analysis on existing data). Owner: Spencer.
- [ ] **5.4** (R5) — Quantify contact-force validation (RMSE, bias, max error, correlation).
- [ ] **5.20** (R5) — Reconcile contact-processing inconsistencies (15 Hz vs. 30 Hz filter, filenames, `writetable`).
- [ ] **5.21** (R5) — `runAllDataCollection.bat` missing Resense/HEX12 process. Owner: Tongjia (or Angela, who wrote it).
- [ ] **5.22** (R5) — Confirm which OptiTrack calibration procedure generated the released data. Owner: Spencer.
- [ ] **5.23** (R5) — Rewrite trajectory description to match actual discrete-waypoint implementation. Owner: Tongjia.
- [ ] **5.26** (R5) — Align cross-correlation code with text (exploratory vs. reported).
- [ ] **5.27** (R5) — Reframe FBG–OptiTrack comparison as post-registration consistency check.
- [ ] **5.30** (R5) — Separate FBG delay estimation from delay correction in `process_data.m`.
- [ ] **5.34** (R5) — Reconcile 2 s (text) vs. 3 s (code) pre-motion period. Owner: Tongjia.
- [ ] **5.35** (R5) — Verify no NaN edge samples after 100 Hz resampling.
- [ ] **5.37** (R5) — Check Mark-10 noise calculation (raw pulley force vs. ÷2 tension convention).
- [ ] **5.41** (R5) — Bundle of small code/label cleanups (plot units, obsolete calls, column comments, dependency docs).

## Larger (4) — multi-step, meaningful new work

- [~] **5.1** (R5) — Related-work citations/paragraph. Five bibliography entries drafted; paragraph not written. Combine with 4.1.
- [ ] **5.2** (R5) — Figure 2 redesign (contrast, alignment, capitalization).
- [ ] **5.3** (R5) — Mounting stiffness / structural eigenfrequency bench test. Owner: Greg.
- [~] **5.19** (R5) — ATI `Fs` estimation bug (29–34× overestimate, effective cutoff 0.52 Hz instead of 15 Hz). Quantified impact confirmed; code fix drafted (bundled with 5.33); **full reprocessing of all `base_wrench.csv` + re-derivation of downstream numbers not yet run.** Owner: Andrea.

## Blocked — needs a decision or external input before it can be drafted (7)

- [!] **E2** (Editor) — Data Descriptors must not include results/conclusions/discussion/analyses. **Root blocker** for everything below — needs a framing decision (validation-style content is fine, a "Conclusions" section is not).
- [!] **5.8** (R5) — Add a conclusion/summary section. Blocked on E2.
- [!] **5.9 / 5.10 / 1.3 / 4.7** (R5/R1/R4) — GVS baseline: release model code + parameters, run forward-dynamics comparison, write it up. **Highest-leverage single item — answers 4 reviewers at once.** Owner: Andrea. Blocked on E2 framing.
- [!] **5.25** (R5) — Soften temporal-synchronization claims. Blocked on confirming the acquisition PC's Python version (relevant to whether pre-3.13 Windows `time.time()` resolution explains the ATI timestamp burst pattern).

---

## Open questions needing a teammate, not tied to a single comment number

- [!] **FBGS "Angle" frame consistency** — `process_data.m` rotates the reconstructed 3-D shape into the robot body frame but never rotates `Angle`/`Curvature`. Curvature is frame-invariant (fine as-is), but per Moore & Rogge 2012, "Angle" may represent bend *direction*, which could need the same rotation. **Needs Spencer to confirm against the Shape CORE definition** before finalizing 5.40 or deciding on a code fix.
- [ ] **Acquisition-machine Python version** — needed for 5.19/5.25 (Windows `time.time()` precision history). Ask Tongjia/whoever ran acquisition.
- [x] **Duplicate LaTeX label** `\label{sec:postprocessing}` — checked in the latest `main.tex`, now appears only once. Worth a final grep before submission for any other duplicate labels.

## Owner summary

- **Andrea (self):** GVS baseline, ATI reprocessing (5.19+5.33), mocap doc fix, Figure 2, editor items, most Trivial/Easy items; 4.3/5.6/5.7/5.11/5.13/5.15/5.29/5.31/5.33/5.36/5.39/5.40 drafted (5.31/5.36/5.40 response letter + a couple manuscript spots still need the fixes listed above).
- **Tongjia (actuator control):** 5.18, 5.23, 5.24, 5.32, 5.34; optionally 5.21.
- **Chengnan (actuator design):** 5.12 (rationale), 4.1/5.1 (related work), light input on 2.1.
- **Spencer (FBGS + OptiTrack):** 5.22, 4.2, 5.17 (sign-off), 5.27, 5.28; FBGS Angle frame-consistency question.
- **Greg (lab technician):** 5.3 (stiffness bench test), 4.5 (videos), physical half of 5.12.
