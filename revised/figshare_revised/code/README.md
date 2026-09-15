# Tendon-Driven Continuum Robot Dataset — Code

Data-acquisition and post-processing code for the tendon-driven continuum
robot dataset described in the accompanying Scientific Data paper
(dataset DOI: https://doi.org/10.6084/m9.figshare.31990188).

## Repository layout

```
code/
├── data_acquisition/     Python (+ one C++ client) scripts used to record
│                         each sensor stream during data collection
├── postprocessing/       MATLAB pipeline that turns raw per-sensor CSVs
│   ├── process_data.m    entry point (see "Post-processing" below)
│   ├── outils/           core pipeline functions (alignment, filtering,
│   │                     synchronization) called by process_data.m
│   └── tests/            standalone reviewer-response/validation scripts
│                         (see "Development / reviewer-response scripts")
└── LICENSE                MIT
```

This code expects to sit next to the released `data/` folder, in exactly the
layout distributed on figshare:

```
figshare repository/
├── code/
│   ├── data_acquisition/
│   └── postprocessing/
└── data/
    ├── quasi_static/               subset 1 — static bending sweeps
    ├── dynamic_motion/             subset 2 — dynamic trajectories
    ├── contact_motion/             subset 3 — contact experiments
    ├── references/
    │   ├── straight_config/        robot verified straight (laser-checked)
    │   └── released_config/        robot hanging, tendons released
    └── postprocess_calibration/    auto-generated on first pipeline run
```

Because the pipeline only ever uses paths relative to this layout (see
"Running the post-processing pipeline" below), it runs unmodified on any
machine once the repository and data are downloaded and placed side by
side — no path needs to be edited except the one line selecting which
experiment folder to process.

## Dependencies

See [`DEPENDENCIES.md`](DEPENDENCIES.md) for the full software (Python +
MATLAB) and hardware list.

## Workflow: raw data to processed data

### 1. Data acquisition (optional — only needed to recollect data)

`data_acquisition/runAllDataCollection.bat` starts one process per sensor in
parallel, each writing its own raw CSV into a shared output folder:

| Script | Sensor | Output |
|---|---|---|
| `readFBGS.exe` | FBG interrogator | `dataFBGS.csv` |
| `readATIFT.py` | ATI mini40 base F/T sensor | `dataATIFT.csv` |
| `optitrackPython.py` | OptiTrack motion capture | `dataOptiTrack.csv` |
| `readMark10.py` (×4) | Mark-10 tendon-tension gauges | `dataMark10_{+x,-x,+y,-y}.csv` |
| `read4MotorCircle.py` | Cybergear actuators | `dataMotor.csv` |
| `read_resense_ft.py` | Resense HEX12 contact wand (contact subset only) | `dataResenseFT.csv` |

Run the `.bat` file from `data_acquisition/`, after editing `duration` and
`output_dir` at the top of the file and confirming the COM ports listed for
each Mark-10 gauge match your setup. Each sensor script can also be run
standalone (they all take `duration` and an output path as arguments; see
`--help` for options).

### 2. Post-processing

The only file you need to run is `postprocessing/process_data.m`. Every
editable setting — which experiment folder to process, the filter cutoff
frequency, the resampling rate, and the plotting switches — is grouped in
the `%% ====== PATHS / SETTINGS ======` block at the top of the file. Set
`folder` to the experiment you want to process (e.g.
`fullfile(data_root, "quasi_static/", "static_bend_y_n200/")`) and run the
script from MATLAB with `postprocessing/` as the working directory.

On first run, the pipeline automatically computes two calibration files
that are shared by every recording in the dataset (not recomputed per
recording — see Section "Coordinate frame alignment" in the paper):

- `data/postprocess_calibration/mocap_correction.csv` — per-disk static
  OptiTrack misalignment offset, computed once from
  `data/references/straight_config/`
- `data/postprocess_calibration/measured_sensors_delay_ms.txt` — FBG
  pipeline delay, computed once by `outils/compute_sensors_delay.m` from
  the four fast `dynamic_motion` trajectories

Both are cached after the first run and simply reloaded on every
subsequent one. This means `data/references/straight_config/` must be
present locally before processing any recording for the first time, even
if `straight_config` itself is not the experiment you are currently
interested in.

### 3. Output

Processed files are written to `<experiment_folder>/processed/`: one CSV
per sensor stream, filtered and resampled onto a common time base, plus a
sanity-check `figures/` subfolder and RMSE technical-validation numbers.
See the paper's Data Records section for the full list of output filenames
and column definitions.

## Development / reviewer-response scripts

`postprocessing/tests/` contains standalone analysis scripts written to
answer specific reviewer questions during peer review. They are not part
of the core acquisition/post-processing pipeline and are not required to
reproduce the released processed files. Files prefixed `TO_DELETE_` are
scratch scripts kept for now but not intended for release.

## License

MIT — see [`LICENSE`](LICENSE).
