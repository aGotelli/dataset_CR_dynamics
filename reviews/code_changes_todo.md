# Code changes to address — ranked by size (smallest → biggest)

Extracted from `authors_response.tex` (all `\TODO{...}` items tied to code, plus
explicit "we are updating / need to verify code" statements). Paper-only edits
(wording, tables, figure regeneration from existing scripts) are excluded —
this list is for the dedicated code session.

Size estimate = files touched + rough LOC changed. Not a hard science, just
enough to triage what to tackle first.

---

## Confirmed (already drafted in the response letter)

### 1. Sync `runAllDataCollection.bat` — Comment 5.21
**Size: tiny — 1 file, ~2 lines**
- Add the missing call to the Resense/HEX12 acquisition executable (already
  exists locally on the acquisition machine, just not synced to the repo).
- Remove the obsolete `Gyro` process line (`sparkfun_ism330dhcx.exe`), unused
  since early prototyping.
- Applies to both copies in the repo
  (`data_collection/dataCollectionPack/figshare/code/data_acquisition/` and
  `data_collection/dataCollectionPack/`) — check whether both should be kept
  in sync going forward or whether one should just be a copy of the other.

### 2. Clean up exploratory cross-correlation block — Comment 5.26
**Size: tiny — 1 file, ~15–20 lines**
- `check_temporal_sync.m`: remove or clearly comment the Motor→Tendon
  (`lag_MC`/`r_MC`) and Motor→ATI (`lag_MA`/`r_MA`) blocks, and the two
  unsaved diagnostic `figure()` calls ("Angles and Tensions", "Angles and
  Torque"), so the code doesn't compute results that are excluded from
  `sync_results.txt` and never used.

### 3. Fix the `"circle" or "Lissajous"` boolean-literal bug — Comment 5.18
**Size: small — 1 file, 2 lines changed, but needs impact verification**
- `read4MotorCircle.py` line 57 (`angle_tol`/`vel_tol`) — verified harmless
  against recorded data; safe to fix.
- `read4MotorCircle.py` line 299 (`limit_speed` during phase transitions) —
  fix is the same one-line change, but assess impact on recorded trajectories
  first (not yet checked, unlike line 57).
- Change both to `motion_pattern in ("circle", "Lissajous")`.

### 4. Verify/apply ATI timestamp centering fix — Comment 5.33
**Size: small — 1 file, a few lines**
- `readATIFT.py` (or wherever the shift is applied in `process_data.m`):
  confirm the response's claimed `-2.5 ms` timestamp shift (half the 5-sample
  averaging window) is actually implemented, not just described.
  Response currently has `\TODO{MAKE SURE CODE IS UPDATED}` — this is a
  correctness check, not just a cleanup.

### 5. Decide + implement `wand_pose.csv` — Comment 5.20 (part b)
**Size: small — 1 file, ~5–10 lines (if writing the generator)**
- `pose_wand` already exists in memory in `compare_ft_sensors.m`, just never
  written to disk. Either (a) add a `writetable`/`writematrix` call for it,
  or (b) drop the `wand_pose.csv` claim from the manuscript's file listing.
  Decide, then implement whichever side isn't already done.

### 6. Amplitude selection for quasi-static recordings — Comment 5.24
**Size: medium — 1 file, ~10–20 lines**
- `read4MotorCircle.py`: replace the hardcoded single `desired_degree = 180`
  with a list of the four values actually used (100, 150, 180, 200°) and an
  explicit selection mechanism (CLI argument recommended, to match the
  pattern already used for motor IDs/duration/filename — an uncommented
  in-script index is the fallback if CLI plumbing is too invasive).
- Decide the mechanism, then update the response's
  `\TODO{confirm final mechanism...}` accordingly once implemented.

### 7. Bring `compare_ft_sensors.m` into line with `process_data.m` — Comment 5.20 (part a)
**Size: medium — 1 file, ~20–30 lines**
- Change `cutoffHz` from 30 to 15 to match the canonical pipeline.
- Decide + implement: either redirect its outputs to a non-colliding path
  (e.g. `processed/validation/`) or stop writing `base_wrench.csv` /
  `contact_wrench.csv` entirely if those are only needed for the script's own
  plots — it should stop producing a second, inconsistent version of files
  `process_data.m` already writes for the same recordings.

### 8. Contact-subset tip + mid-disk RMSE analysis — Comment 4.2
**Size: largest — likely a new analysis script (or a substantial addition to
`process_data.m`) plus reprocessing multiple contact-subset recordings**
- Confirm the existing `RMSEs.txt` values are reproducible with the current
  `process_data.m` (their timestamps currently predate the latest script
  version — needs a fresh run to confirm, not just re-reading old output).
- Add mid-backbone-disk RMSE (OptiTrack vs. FBG) alongside the existing tip
  RMSE, for both the contact subset and the non-contact dynamic trajectories
  — confirm the exact arc-length/disk index to use.
- Feeds the new Table (separate from Table `tab:rmse_tip`) and the Technical
  Validation paragraph referenced in the "Changes made to the article" for
  Comment 4.2. This is the one item here that's an analysis task as much as
  a code-editing task — start this one first if the RMSE numbers are needed
  soon, regardless of its size ranking.

---

## Not yet triaged — comments with no drafted response yet

These weren't analyzed in this session, but their text describes what sound
like code-level issues. Listed here so they aren't lost; each needs the same
"verify against actual code/data" pass the items above already got before
anything is assumed about scope.

- **5.19** — `process_data.m`'s `Fs = 1/median(diff(t))` may be wrong for the
  burst-like ATI timestamps. If real, this is potentially the **biggest**
  item on the whole list: it could mean the sampling-frequency estimate (and
  therefore the filter) used for the *entire* released processed dataset,
  not just one script, needs to be redone on a regular time grid, followed
  by full reprocessing and re-release of affected files. Check this first
  among the untriaged items — it gates how big the rest of the code session
  needs to be.
- **5.22** — hard-coded parameters/paths in `process_data.m`; no top-level
  README/dependency file; and a possible mismatch between the manuscript's
  described OptiTrack calibration procedure (offsets fixed from
  `straight_config`) and the code's actual behavior (recalculated from the
  first 3s of every recording, `process_data.m` lines 141–195). The
  calibration-procedure part needs a verify-first pass like 5.34 got;
  the reproducibility-packaging part (README, dependencies) is separate,
  smaller, non-code-logic work.
- **5.28** — `disk_k_is_valid` validity flags loaded in `data_optitrack.m`
  but apparently unused and not retained in processed output.
- **5.30** — FBG delay estimation and correction currently mixed in
  `process_data.m` (timestamps corrected before the sync check runs);
  should probably be reordered so lag is estimated on uncorrected signals.
- **5.32** — four motors queried sequentially but stored under one shared
  timestamp; either timestamp individually or quantify total readout
  latency.
- **5.35** — 100 Hz resampling grid is based on motor recording duration
  rather than the common overlap of all streams; check for NaN/undefined
  edge samples.
- **5.37** — `analyze_reference_noise.m` may compute Mark-10 noise from raw
  pulley reaction force without the ÷2 conversion to tendon tension that
  normal processing applies.
- **5.41** — grab-bag: incorrect torque units in plot labels, obsolete
  function calls, inconsistent FBG column-layout comments, incomplete
  MATLAB/Python dependency documentation. Probably several small,
  independent fixes rather than one task.
