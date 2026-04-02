# Reference Sensor Noise Analysis Script

## Quick Start

```matlab
matlab -batch analyze_reference_noise
```

Or interactively in MATLAB:
```matlab
>> cd /sessions/keen-sleepy-curie/mnt/dataset_CR_dynamics/data_collection/postprocessing/
>> analyze_reference_noise
```

## What It Does

Analyzes static reference sensor data to characterize noise in 4 sensor systems:

1. **Mark10 Tension Gauges** (4 independent gauges)
   - Computes mean and std of tension (N)
   - Reports per-gauge and overall statistics

2. **ATI-FT Force/Torque Sensor** (6 channels)
   - Noise in Fx, Fy, Fz (forces in N)
   - Noise in Tx, Ty, Tz (torques in Nm)

3. **OptiTrack Mocap System** (5 disks)
   - Position noise per disk (x, y, z in mm)
   - Orientation noise per disk (in degrees)
   - Computed via quaternion angle deviations

4. **FBGS Curvature Sensor** (26 gratings + shape)
   - Curvature noise per grating (standard metric)
   - Angle noise per grating (in degrees)
   - Shape noise (502 arc-length points in mm)
   - Special focus on tip position noise

## Output

**Console:**
- Detailed statistics for each sensor
- Ready-to-use LaTeX table for papers

**File:** `analyze_reference_noise_results.mat`
- MATLAB structures with all computed statistics
- Load with: `load('analyze_reference_noise_results.mat')`

## Input Data Location

```
/sessions/keen-sleepy-curie/mnt/dataset_CR_dynamics/data_collection/
  dataCollectionPack/data/references/released_config/
  ├── dataATIFT.csv
  ├── dataMark10_+x.csv
  ├── dataMark10_+y.csv
  ├── dataMark10_-x.csv
  ├── dataMark10_-y.csv
  ├── dataOptiTrack.csv
  └── dataFBGS.csv (30 MB)
```

## Key Features

- **Robust:** Handles NaN values, large files (30 MB FBGS)
- **Accurate:** Proper quaternion processing for orientation noise
- **Complete:** Per-sensor and overall statistics
- **Publication-ready:** Direct LaTeX table output
- **Well-documented:** 450+ lines with clear section headers

## Implementation Details

### Mark10 Gauges
- Reads 4 CSV files
- Removes NaN values
- Computes mean, std, and sample count

### ATI-FT Sensor
- 6 independent channels
- Robust NaN handling
- Channel-specific and combined noise metrics

### OptiTrack Mocap
- 5 disks with position (x, y, z) and orientation (quaternion)
- Position: reports std of each coordinate
- Orientation: converts quaternion variations to angle deviations
  - Computes mean quaternion as reference
  - Computes relative quaternion for each frame
  - Converts to angle via: angle = 2 * acos(qw)
  - Reports std of angle deviations

### FBGS Sensor
- **Curvature:** std per grating, average across 26 gratings
- **Angle:** std per grating in degrees, average across 26 gratings
- **Shape:** 
  - 1506 columns = 3 coordinates × 502 points
  - 1mm arc-length resolution
  - Reports position std per point and average
  - Special analysis of tip position (point 502)

## Processing Time

Typical runtime: 5-10 minutes
- Mostly reading and processing the 30 MB FBGS file
- Shape analysis: ~2-3 minutes (502 points with std calculations)

## Output Structure

```matlab
% After running, results are saved in:
% analyze_reference_noise_results.mat

mark10_stats.gauge_+X.mean/std/n_samples
mark10_stats.gauge_+Y.mean/std/n_samples
mark10_stats.gauge_-X.mean/std/n_samples
mark10_stats.gauge_-Y.mean/std/n_samples
mark10_stats.overall.mean/std/n_samples

ati_stats.Fx/Fy/Fz/Tx/Ty/Tz.mean/std/n_samples

opti_stats.disk_0 through disk_4:
  .pos_std_x/y/z (meters)
  .pos_std_mean (meters)
  .ori_std_rad/deg (radians/degrees)
  .n_frames_pos/ori

fbgs_stats.curvature_std_per_grating (26 values)
fbgs_stats.curvature_std_mean/std
fbgs_stats.angle_std_per_grating (26 values in degrees)
fbgs_stats.angle_std_mean/std (degrees)
fbgs_stats.shape_pos_std_per_point (502 values in meters)
fbgs_stats.shape_pos_std_mean/std (meters)
fbgs_stats.tip_std_x/y/z (meters)
fbgs_stats.tip_std_mean (meters)
```

---
Generated: 2026-03-31
Script location: `/sessions/keen-sleepy-curie/mnt/dataset_CR_dynamics/data_collection/postprocessing/analyze_reference_noise.m`
