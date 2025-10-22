import csv
import math
import os
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


import can  # type: ignore
import numpy as np  # type: ignore
import serial  # type: ignore
from sensors.cybergear.pcan_cybergear import CANMotorController

class Motor:
    """Minimal helper around the Cybergear CAN motor."""

    def __init__(self, motor_cfg, bus):
        self.motor_id = motor_cfg["id"]
        self.max_speed = motor_cfg.get("max_speed", 6)
        self.motor = CANMotorController(bus, motor_id=self.motor_id, main_can_id=254)
        self.motor.set_run_mode(self.motor.RunModes.POSITION_MODE)
        self.motor.enable()

        angle_rad, angle_deg = self.get_current_angle()
        self.last_command_rad = angle_rad if angle_rad is not None else 0.0
        print(
            f"✅ Motor {self.motor_id} ready at "
            f"{self.last_command_rad:.4f} rad ({angle_deg:.2f} deg if available)"
        )

    def get_current_angle(self, max_retries=5, retry_delay=0.05):
        """Return tuple (rad, deg); (None, None) on failure."""
        for _ in range(max_retries):
            status = self.motor.get_motor_status()
            if status and len(status) > 1:
                angle_rad = status[1]
                angle_deg = math.degrees(angle_rad)
                return angle_rad, angle_deg
            time.sleep(retry_delay)
        print(f"⚠️  Motor {self.motor_id}: could not read angle")
        return None, None

    def command(self, target_angle_rad):
        """Send position command in radians."""
        self.last_command_rad = target_angle_rad
        self.motor.set_motor_position_control(
            limit_spd=self.max_speed, loc_ref=target_angle_rad
        )

    def disable(self):
        try:
            self.motor.disable()
        except Exception as exc:
            print(f"⚠️  Motor {self.motor_id} disable warning: {exc}")


class Mark10Sensor:
    """Serial interface to Mark-10 force gauge."""

    def __init__(self, com_port, sampling_rate, timeout=0.1):
        self.com_port = com_port
        self.sampling_rate = sampling_rate
        self.timeout = timeout
        self.serial_connection = None

        try:
            self.serial_connection = serial.Serial(self.com_port, baudrate=115200, timeout=self.timeout)
            timing = self.measure_timing(50)
            if timing:
                print(
                    f"✅ Mark-10 {self.com_port}: {timing['data_rate']:.1f} Hz (target {sampling_rate} Hz)"
                )
        except Exception as exc:
            print(f"❌ Mark-10 {self.com_port} connection failed: {exc}")
            self.serial_connection = None
            raise

    def measure_timing(self, num_samples=200):
        """Returns:
                timing_stats: Dictionary with timing statistics"""
        if not self.serial_connection or not self.serial_connection.is_open:
            print(f"⚠️  Mark-10 {self.com_port}: serial connection not open")
            return None

        timing_data = np.zeros(num_samples)
        for idx in range(num_samples):
            start = time.perf_counter()
            self.serial_connection.write("?\r".encode())
            self.serial_connection.readline() #old was response = self.serial_connection.readline().decode().strip()
            end = time.perf_counter()
            timing_data[idx] = (end - start) * 1000.0 # Convert to ms

        stats = {
            "avg_time": float(np.mean(timing_data)),
            "min_time": float(np.min(timing_data)),
            "max_time": float(np.max(timing_data)),
            "std_time": float(np.std(timing_data)),
            "data_rate": 1000.0 / float(np.mean(timing_data)),
            'num_samples': num_samples,
        }
        return stats

    def get_tension(self):
        if not self.serial_connection or not self.serial_connection.is_open:
            return float("nan")

        self.serial_connection.write("?\r".encode())
        response = self.serial_connection.readline().decode("utf-8", errors="ignore").strip()
        if not response:
            return float("nan")

        force_str = (response.replace("N", "").replace("lbF", "").replace("lb", "").strip())
        try:
            return float(force_str)
        except ValueError:
            return float("nan")

    def close(self):
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
            print(f"ℹ️  Mark-10 {self.com_port} connection closed")


def pretension_single_motor(motor,mark10_sensor,target_tension,direction=1,max_angle_change=2.0,step_size=0.01,wait_time=0.2,tolerance=0.1):
    # """Pretension a single motor using positive target tension-negative M10 readings are handled."""
    """ Pretension a single motor. Tension of M10 is read as negative"""
    angle_rad, _ = motor.get_current_angle()
    current_angle = angle_rad if angle_rad is not None else motor.last_command_rad

    max_steps = int(max_angle_change / step_size)
    tension = mark10_sensor.get_tension()

    for step in range(max_steps):
        if math.isnan(tension):
            print(f"⚠️  Motor {motor.motor_id}: invalid tension reading, retrying…")
            time.sleep(wait_time)
            tension = mark10_sensor.get_tension()
            continue

        error = target_tension - tension
        print(
            f"Motor {motor.motor_id} pretension step {step + 1}: "
            f"angle={current_angle:.4f} rad, tension={tension:.3f} N, error={error:.3f}"
        )
        if abs(error) <= tolerance:
            return True, current_angle, tension
        step_direction = direction if error > 0 else -direction
        current_angle += step_direction * step_size
        motor.command(current_angle)
        time.sleep(wait_time)
        tension = mark10_sensor.get_tension()
       
        # # # Older version handling absolute values 
        # # abs_tension = abs(tension)
        # # abs_target = abs(target_tension)# useless but be sure positive tensions are compared with positive tensions
        # # error = abs_target - abs_tension

        # # print(
        # #     f"Motor {motor.motor_id} pretension step {step + 1}: "
        # #     f"angle={current_angle:.4f} rad, tension={tension:.3f} N "
        # #     f"(abs {abs_tension:.3f} vs target {abs_target:.3f}, error {error:.3f})"
        # # )

        # # if abs(error) <= tolerance:
        # #     return True, current_angle, tension

        # # if abs_tension < abs_target:
        # #     current_angle += direction * step_size
        # # else:
        # #     current_angle -= direction * step_size

        # # motor.command(current_angle)
        # # time.sleep(wait_time)
        # # tension = mark10_sensor.get_tension()

    print(
        f"⚠️  Motor {motor.motor_id}: pretension max steps reached "
        f"(final tension {tension:.3f} N, abs {abs(tension):.3f} N)"
    )
    return False, current_angle, tension


def run_pretensioning_thread(motor1,motor2,sensor1,sensor2,target_tension,direction1=1,direction2=-1):
    """Run pretension on both motors in parallel."""

    results = {"motor1": None, "motor2": None}

    def pretension_motor(motor, sensor, direction, key):
        try:
            success, angle, tension = pretension_single_motor(
                motor, sensor, target_tension, direction=direction
            )
            results[key] = (success, angle, tension)
        except Exception as exc:
            print(f"❌ Pretension motor {motor.motor_id} failed: {exc}")
            results[key] = (False, None, None)

    thread1 = threading.Thread(
        target=pretension_motor, args=(motor1, sensor1, direction1, "motor1")
    )
    thread2 = threading.Thread(
        target=pretension_motor, args=(motor2, sensor2, direction2, "motor2")
    )

    thread1.start()
    thread2.start()
    thread1.join()
    thread2.join()

    print(
        f"ℹ️  Pretension results -> motor1: {results['motor1']}, motor2: {results['motor2']}"
    )
    return results


class MotorControl:
    """Simple coordinator used by the acquisition pipeline."""

    def __init__(self, motor1_cfg, motor2_cfg, mark10_ports, mark10_rate, motors_frequency, experiment_dir, target_tension=None):
        os.makedirs(experiment_dir, exist_ok=True)

        self.experiment_dir = experiment_dir
        self.mark10_rate = mark10_rate
        self.motors_frequency = motors_frequency
        self.target_tension = target_tension
        self.motor1_base_angle = None
        self.motor2_base_angle = None
        self.pretension_results = None

        self.motor1_cfg = self._normalize_motor_cfg(motor1_cfg, mark10_ports, 0, 1)
        self.motor2_cfg = self._normalize_motor_cfg(motor2_cfg, mark10_ports, 1, -1)

        self.bus = can.interface.Bus(interface="candle", channel=0, bitrate=1_000_000)
        self.motor1 = Motor(self.motor1_cfg, self.bus)
        self.motor2 = Motor(self.motor2_cfg, self.bus)

        self.sensor1 = Mark10Sensor(self.motor1_cfg["mark10_port"], mark10_rate)
        self.sensor2 = Mark10Sensor(self.motor2_cfg["mark10_port"], mark10_rate)

        self.log_path = os.path.join(self.experiment_dir, "motors_mark10.csv")
        print(f"ℹ️  Motor+Mark10 data will be logged to {self.log_path}")


    def _normalize_motor_cfg(self, cfg, mark10_ports, index, default_direction):
        if isinstance(cfg, dict):
            data = dict(cfg)
        else:
            data = {}
            if len(cfg) > 0:
                data["id"] = cfg[0]
            if len(cfg) > 1:
                data["mark10_port"] = cfg[1]
            if len(cfg) > 2 and callable(cfg[2]):
                data["trajectory"] = cfg[2]
        if "id" not in data:
            raise ValueError("Motor configuration missing motor ID")
        if "trajectory" not in data:
            raise ValueError(f"Motor {data['id']} configuration missing trajectory function")

        ports = list(mark10_ports or [])
        if "mark10_port" not in data:
            if len(ports) > index:
                data["mark10_port"] = ports[index]
            else:
                raise ValueError(f"Motor {data['id']} configuration missing Mark-10 port")

        data["direction"] = data.get("direction", default_direction)
        return data

    def pretension(self):
        if self.target_tension is None:
            print("ℹ️  Pretension skipped (no target tension provided)")
            return True

        print("ℹ️  Starting pretension routine…")
        results = run_pretensioning_thread(
            self.motor1,
            self.motor2,
            self.sensor1,
            self.sensor2,
            target_tension=self.target_tension,
            direction1=self.motor1_cfg["direction"],
            direction2=self.motor2_cfg["direction"],
        )
        self.pretension_results = results

        motor1_info = results.get("motor1")
        motor2_info = results.get("motor2")

        ok1 = motor1_info[0] if motor1_info else False
        ok2 = motor2_info[0] if motor2_info else False

        if ok1:
            self.motor1_base_angle = motor1_info[1]
        if ok2:
            self.motor2_base_angle = motor2_info[1]

        if ok1 and ok2:
            print(
                f"✅ Pretension completed: "
                f"motor1 angle {self.motor1_base_angle:.4f} rad, "
                f"motor2 angle {self.motor2_base_angle:.4f} rad"
            )
        else:
            print("❌ Pretension failed")

        return ok1 and ok2

    def get_base_angles(self):
        """Return the angles (rad) memorized at the end of pretension."""
        return self.motor1_base_angle, self.motor2_base_angle

    def configure_axis_trajectories(self, total_duration, plan=None):
        """
        Create trajectories that move one motor at a time (X then Y by default).

        Args:
            total_duration: overall acquisition duration (seconds).
            plan: optional dict with keys:
                - order: iterable like ("x","y") specifying movement sequence.
                - amplitude_deg: float or dict {"x": deg, "y": deg}.
                - segment_duration: optional duration for each axis segment (sec).
        """
        order = plan.get("order") if plan else None
        if order is None:
            order = ("x", "y")
        if isinstance(order, str):
            order = tuple(order)
        order = tuple(order)
        if not order:
            order = ("x", "y")

        amp_param = plan.get("amplitude_deg") if plan else 5.0
        if isinstance(amp_param, dict):
            amp_x_deg = float(amp_param.get("x", 5.0))
            amp_y_deg = float(amp_param.get("y", 5.0))
        else:
            amp_x_deg = amp_y_deg = float(amp_param)

        segment_duration = plan.get("segment_duration") if plan else None
        if segment_duration:
            segment_duration = float(segment_duration)
            total_required = segment_duration * len(order)
            if total_required > total_duration:
                raise ValueError(
                    f"Axis plan requires {total_required}s but total duration is {total_duration}s"
                )
        else:
            segment_duration = total_duration / len(order) if order else total_duration

        base1 = self.motor1_base_angle
        base2 = self.motor2_base_angle
        if base1 is None or base2 is None:
            base1 = self.motor1.get_current_angle()[0] or self.motor1.last_command_rad
            base2 = self.motor2.get_current_angle()[0] or self.motor2.last_command_rad

        amp_x_rad = math.radians(amp_x_deg)
        amp_y_rad = math.radians(amp_y_deg)

        def segment_wave(local_t, seg_dur, amplitude_rad):
            if seg_dur <= 0:
                return 0.0
            progress = max(0.0, min(local_t / seg_dur, 1.0))
            return amplitude_rad * math.sin(math.pi * progress)

        def motor1_traj(t):
            cur = 0.0
            for axis in order:
                seg_end = cur + segment_duration
                if t < seg_end:
                    local_t = t - cur
                    if axis.lower() == "x":
                        return base1 + segment_wave(local_t, segment_duration, amp_x_rad)
                    return base1
                cur = seg_end
            return base1

        def motor2_traj(t):
            cur = 0.0
            for axis in order:
                seg_end = cur + segment_duration
                if t < seg_end:
                    local_t = t - cur
                    if axis.lower() == "y":
                        return base2 + segment_wave(local_t, segment_duration, amp_y_rad)
                    return base2
                cur = seg_end
            return base2

        self.motor1_cfg["trajectory"] = motor1_traj
        self.motor2_cfg["trajectory"] = motor2_traj
        print(
            f"ℹ️  Axis trajectories configured with order {order}, "
            f"segment {segment_duration:.2f}s, amplitudes X={amp_x_deg}°, Y={amp_y_deg}°"
        )

    def run_trajectory_acquisition(self, duration, sample_rate=None, skip_pretension=False):
        """Generate commands, read feedback/forces, and log to CSV."""
        if not skip_pretension:
            if not self.pretension():
                print("❌ Skipping trajectory execution due to failed pretension.")
                return

        sr = sample_rate or self.motors_frequency or 50.0
        
        traj1 = self.motor1_cfg.get("trajectory")
        traj2 = self.motor2_cfg.get("trajectory")
        if traj1 is None:
            base1 = self.motor1_base_angle
            if base1 is None:
                base1 = self.motor1.get_current_angle()[0] or self.motor1.last_command_rad
            traj1 = lambda t, base=base1: base
            self.motor1_cfg["trajectory"] = traj1
        if traj2 is None:
            base2 = self.motor2_base_angle
            if base2 is None:
                base2 = self.motor2.get_current_angle()[0] or self.motor2.last_command_rad
            traj2 = lambda t, base=base2: base
            self.motor2_cfg["trajectory"] = traj2

        header = [
            "timestamp",
            "elapsed_s",
            "motor1_cmd_rad",
            "motor1_cmd_deg",
            "motor1_feedback_rad",
            "motor1_feedback_deg",
            "mark10_motor1_N",
            "motor2_cmd_rad",
            "motor2_cmd_deg",
            "motor2_feedback_rad",
            "motor2_feedback_deg",
            "mark10_motor2_N",
        ]

        start_time = time.time()
        period = 1.0 / sr

        with open(self.log_path, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(header)

            iteration = 0
            rows = []
            while (time.time() - start_time) < duration:
                now = time.time()
                elapsed = now - start_time

                cmd1 = float(self.motor1_cfg["trajectory"](elapsed))
                cmd2 = float(self.motor2_cfg["trajectory"](elapsed))
                self.motor1.command(cmd1)
                self.motor2.command(cmd2)

                feedback1 = self.motor1.get_current_angle(max_retries=3, retry_delay=0.01)
                feedback2 = self.motor2.get_current_angle(max_retries=3, retry_delay=0.01)

                tension1 = self.sensor1.get_tension()
                tension2 = self.sensor2.get_tension()

                rows.append(
                    [
                        datetime.now().isoformat(),
                        round(elapsed, 6),
                        cmd1,
                        math.degrees(cmd1),
                        feedback1[0] if feedback1[0] is not None else "",
                        feedback1[1] if feedback1[1] is not None else "",
                        tension1,
                        cmd2,
                        math.degrees(cmd2),
                        feedback2[0] if feedback2[0] is not None else "",
                        feedback2[1] if feedback2[1] is not None else "",
                        tension2,
                    ]
                )

                iteration += 1
                next_time = start_time + iteration * period
                sleep_time = next_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

            writer.writerows(rows)

        print("✅ Motor trajectory acquisition completed")

    def stop(self):
        self.stop_requested = True

    def cleanup(self):
        """Release hardware and close the CAN bus."""
        self.stop_requested = True
        try:
            self.sensor1.close()
        except Exception as exc:
            print(f"⚠️  Mark-10 cleanup warning: {exc}")

        try:
            self.sensor2.close()
        except Exception as exc:
            print(f"⚠️  Mark-10 cleanup warning: {exc}")

        try:
            self.motor1.disable()
        except Exception as exc:
            print(f"⚠️  Motor 1 cleanup warning: {exc}")

        try:
            self.motor2.disable()
        except Exception as exc:
            print(f"⚠️  Motor 2 cleanup warning: {exc}")

        try:
            self.bus.shutdown()
        except Exception as exc:
            print(f"⚠️  CAN bus shutdown warning: {exc}")
