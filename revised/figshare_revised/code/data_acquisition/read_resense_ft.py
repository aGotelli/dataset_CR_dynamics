import os
import serial
from sys import exit
import struct
import time
import argparse

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def plot_resense_ft_data(csv_path: str, output_dir: str | None = None) -> list[str]:
    """Plot Resense FT data from a CSV file and save the plot(s) to disk.

    This creates two separate plots:
    - force components (Fx, Fy, Fz)
    - torque components (Tx, Ty, Tz)

    Args:
        csv_path: Path to the CSV file containing the data.
        output_dir: Directory to save the generated plot(s). If None, uses the CSV file directory.

    Returns:
        A list of paths to the saved plot images.
    """

    if output_dir is None:
        output_dir = os.path.dirname(os.path.abspath(csv_path))

    os.makedirs(output_dir, exist_ok=True)

    data = np.genfromtxt(csv_path, delimiter=",", skip_header=1)
    if data.size == 0:
        raise ValueError(f"No data found in '{csv_path}'")

    data = data.reshape(-1, 7)
    timestamps = data[:, 0]

    # Force and torque groups with consistent y-axis scaling
    forces = {"Fx": data[:, 1], "Fy": data[:, 2], "Fz": data[:, 3]}
    torques = {"Tx": data[:, 4], "Ty": data[:, 5], "Tz": data[:, 6]}

    def _save_plot(series: dict[str, np.ndarray], title: str, suffix: str) -> str:
        fig, ax = plt.subplots(figsize=(10, 6))
        for label, values in series.items():
            ax.plot(timestamps - timestamps[0], values, label=label)

        ax.set_xlabel("Time (s)")
        ax.set_ylabel(title)
        ax.set_title(f"Resense FT {title}")
        ax.legend()
        ax.grid(True)

        # Use consistent y-limits across components for each plot
        all_values = np.concatenate(list(series.values()))
        pad = (all_values.max() - all_values.min()) * 0.05
        if pad == 0:
            pad = 1.0
        ax.set_ylim(all_values.min() - pad, all_values.max() + pad)

        plot_path = os.path.join(output_dir, os.path.splitext(os.path.basename(csv_path))[0] + suffix)
        fig.tight_layout()
        fig.savefig(plot_path, dpi=150)
        plt.close(fig)
        return plot_path

    force_plot = _save_plot(forces, "Force", "_force.png")
    torque_plot = _save_plot(torques, "Torque", "_torque.png")

    return [force_plot, torque_plot]


if __name__ == "__main__":


    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="Read samples from Vicon")
    parser.add_argument('duration', type=int,
                        help="Duration to run the data collection (in seconds)")
    parser.add_argument('filename', type=str, help="Filename to save the data")
    parser.add_argument('--start-time', type=float, default=None,
                        help="Shared start timestamp (seconds)")
    args = parser.parse_args()

    duration = args.duration
    filename = args.filename

    file = open(args.filename, 'w')

    header = "timestamp (s), Fx, Fy, Fz, Tx, Ty, Tz\n"
    file.write(header)




    
    # Enable matrix calculation on electronics 
    ser = serial.Serial('COM8', 12000000) # Specify COM port as seen in device manager or FT-Explorer


    start = time.time()

    print("starting")
    elapsed = time.time() - start
    while elapsed < duration:

        serial_line = ser.read(28)  # Read one dataset from COM Port
        current_time = time.time()
        [CH2, CH1, CH4, CH3, CH6, CH5, temp] = struct.unpack('fffffff', serial_line[0:28])  # Unpack data

        file.write(f"{current_time},{CH2},{CH1},{CH4},{CH3},{CH6},{CH5}\n")

        elapsed = current_time - start

    file.close()

    # Generate and save a plot from the collected data
    plot_path = plot_resense_ft_data(filename)
    print(f"Plot saved to: {plot_path}")

    # print("\n\Elapsed : ")
    # print(elapsed)
    # print("\n\nFrequency : ")
    # print(N_samples/elapsed)



    # start = time.time()
    # for x in range(1,sample*rec_time):
        
        
    #     serial_line = ser.read(28)  # Read one dataset from COM Port 
    #     [CH2, CH1, CH4, CH3, CH6, CH5, temp] = struct.unpack('fffffff', serial_line[0:28]) # Unpack data
    #     data.append([datetime.datetime.now(), CH2, CH1, CH4, CH3, CH6, CH5, temp]); # add data to pandas datafame
    
    # end = time.time()


    # print(end - start)

    # df = pd.DataFrame(data)
    # values= df.to_numpy() # convert dataframe to numpy array

    # plt.plot(values[:,[1]],label='Fx')
    # plt.plot(values[:,[2]],label='Fy')
    # plt.plot(values[:,[3]],label='Fz')
    # plt.plot(values[:,[4]],label='Mx')
    # plt.plot(values[:,[5]],label='My')
    # plt.plot(values[:,[6]],label='Mz')


    # plt.legend()
    # #df.to_csv(name)
    # ser.close()


