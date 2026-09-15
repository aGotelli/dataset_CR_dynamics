# Dependencies

## Post-processing (`postprocessing/`)

- **MATLAB R2025b** (developed and tested on this version)
- **Signal Processing Toolbox** — the core pipeline calls `butter`,
  `filtfilt`, and `interp1` (filtering/resampling in `process_data.m`) and
  `xcorr` (temporal-synchronization check in `outils/check_temporal_sync.m`,
  used by `outils/compute_sensors_delay.m`). No other toolbox is required
  to run `process_data.m`.
- **Statistics and Machine Learning Toolbox** — only needed for
  `postprocessing/tests/analyze_reference_noise.m` (calls `normpdf`), a
  standalone reviewer-response script, not for `process_data.m` itself.

## Data acquisition (`data_acquisition/`)

Python 3.x. Per-script package requirements:

| Script | Python packages | Notes |
|---|---|---|
| `optitrackPython.py` | — | Imports `NatNetClient`, `DataDescriptions`, `MoCapData` — see note below. |
| `readATIFT.py` | `nidaqmx`, `numpy` | Requires the NI-DAQmx driver installed (National Instruments) |
| `readMark10.py` | `pyserial`, `numpy` | |
| `read_resense_ft.py` | `pyserial`, `numpy`, `matplotlib` | |
| `read4MotorCircle.py` | `numpy`, `python-can`, `matplotlib` | Uses `sensors/cybergear/pcan_cybergear.py` (included). Requires a PEAK-System PCAN-USB adapter and its driver. |

**FBG interrogator client**: `readFBGS.exe` (pre-compiled Windows
executable) is included and is what `runAllDataCollection.bat` actually
runs. Its C++ source (`sensors/FBGS/read_FBGS.cpp`, `StreamClient.cpp/h`)
is included for users who need to rebuild it; doing so requires the FBGS
Shape Sensing StreamClient SDK (proprietary, from FBGS Technologies) and a
C++ compiler.

**OptiTrack NatNet client**: `optitrackPython.py` imports three modules (`NatNetClient`, `DataDescriptions`, `MoCapData`) that are OptiTrack's proprietary NatNet SDK sample files, and can be downloaded (matching your Motive version) from the OptiTrack developer site (https://optitrack.com/support/downloads/developer-tools.html).
To run the `optitrackPython.py` script, copy `NatNetClient.py`, `DataDescriptions.py`, and `MoCapData.py` from the SDK's Python samples into `data_acquisition/` alongside `optitrackPython.py`.

## Hardware

Full specifications, accuracy and sampling rates for every sensor are given in the paper's sensor-summary table (Table 2). 
Devices used:

- OptiTrack multi-camera motion-capture system, with Motive
- ATI mini40 six-axis force/torque sensor, read via a National Instruments DAQ device
- 4× Mark-10 digital force gauges (serial/USB), one per tendon
- FBGS interrogator (Shape Sensing)
- 4× Cybergear actuators, driven over CAN via a PEAK-System PCAN-USB adapter
- Resense HEX12 force/torque sensor + evaluation board (serial) — used only for the contact subset, mounted on the instrumented contact wand
