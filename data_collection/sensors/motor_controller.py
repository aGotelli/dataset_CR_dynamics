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


def run_pretension_motors(motor1,motor2,sensor1,sensor2,target_tension,direction1=1,direction2=-1):
    step_size=0.01
    tolerance=0.1
    not_ok_m1=True
    not_ok_m2=True
    count = 0
    while not_ok_m1 or not_ok_m2:
        count+=1
        if count>200:
            print(f"WARNING _ TBTESTED!!! Current tension m1: {abs(sensor1.get_tension())}; m2: {abs(sensor2.get_tension())}")
            while True:
                resp = input("Do you still want to actuate the motors? (y/n): ").strip().lower()
                if resp in ("y", "yes"):
                    return True, True
                if resp in ("n", "no"):
                    return False, False
                
        not_ok_m1=True
        not_ok_m2=True
        tension1 = abs(sensor1.get_tension())
        rd1,_=motor1.get_current_angle()
        rd1_to_act=rd1
        if abs(target_tension-tension1) <= tolerance:
            not_ok_m1=False # m1 ok
        if tension1 < target_tension:
            rd1_to_act += direction1 * step_size
        else:
            rd1_to_act -= direction1 * step_size
        motor1.command(rd1_to_act)

        tension2 = abs(sensor2.get_tension())
        rd2,_=motor2.get_current_angle()
        rd2_to_act=rd2
        if abs(target_tension-tension2) <= tolerance:
            not_ok_m2=False # m2 ok
        if tension2 < target_tension:
            rd2_to_act += direction2 * step_size
        else:
            rd2_to_act -= direction2 * step_size
        motor2.command(rd2_to_act)
        print(f"M1: current: {rd1}; actuated {rd1_to_act}; next reading {motor1.get_current_angle()} - M1 tension {sensor1.get_tension()} - pretensioned: {not(not_ok_m1)}")
        print(f"M2: current: {rd2}; actuated {rd2_to_act}; next reading {motor2.get_current_angle()} - M2 tension {sensor2.get_tension()} - pretensioned: {not(not_ok_m2)}")
        time.sleep(0.2)


    print(f"Pretensinoing m1: {not(not_ok_m1)} - tension {sensor1.get_tension()}; m2: {not(not_ok_m2)} - tension {sensor2.get_tension()}")
    return not(not_ok_m1), not(not_ok_m2)  # Tupla di due valori



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
            if len(cfg) > 2 and callable(cfg[2],dict):
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
    
    def _build_relative_trajectory(self):
        """Build trajectories for motors taking into account pretension angle."""
        
        # Costruisci traiettoria per motore 1 (asse X) - già con angolo base incluso
        self.motor1_cfg["trajectory"] = self._create_trajectory_function(self.motor1_cfg["trajectory"], "x", self.motor1_base_angle)
        
        # Costruisci traiettoria per motore 2 (asse Y) - già con angolo base incluso
        self.motor2_cfg["trajectory"] = self._create_trajectory_function(self.motor2_cfg["trajectory"], "y", self.motor2_base_angle)
        
        print("✅ Trajectories built successfully")

    def _create_trajectory_function(self, cfg, axis, base_angle):
        """Crea una funzione di traiettoria assoluta per un singolo motore."""
        traj_type = cfg.get("type")
        if not traj_type:
            raise ValueError("Trajectory config missing 'type'")

        traj_type = traj_type.lower()
        freq = float(cfg.get("frequency_hz", 0.5))

        if traj_type == "ramp":
            duration = float(cfg.get("duration", 5.0))
            max_rad = math.radians(cfg.get("max_deg", 5.0))
            target_axis = cfg.get("axis", "x").lower() # default x

            if target_axis == axis:
                return lambda t: base_angle + min(max(t / duration, 0.0), 1.0) * max_rad
            else:
                return lambda t: base_angle

        if traj_type == "sine":
            amp_rad = math.radians(cfg.get("amplitude_deg", 5.0))
            delay = float(cfg.get("start_delay", 0.0))
            print(f"Delay {delay}")
            def traj(t, base_angle=base_angle):
                if t < delay:
                    return base_angle
                return base_angle + amp_rad * math.sin(2 * math.pi * freq * (t - delay))
            return traj

            # return lambda t: base_angle + amp_rad * math.sin(2 * math.pi * freq * (t-delay))

        if traj_type == "cosine":
            amp_rad = math.radians(cfg.get("amplitude_deg", 5.0))
            return lambda t: base_angle + amp_rad * math.cos(2 * math.pi * freq * t)

        if traj_type == "circle":
            radius_rad = math.radians(cfg.get("radius_deg", 5.0))
            phase_deg = cfg.get("phase_deg", 0.0)
            phase_rad = math.radians(phase_deg)
            if axis == "x":
                return lambda t: base_angle + radius_rad * math.cos(2 * math.pi * freq * t + phase_rad)
            if axis == "y":
                return lambda t: base_angle + radius_rad * math.sin(2 * math.pi * freq * t + phase_rad)
            return lambda t: base_angle

        raise ValueError(f"Unsupported trajectory type: {traj_type}")

    def pretension(self, flag):
        successm1 = successm2 = False
        print(f"current agles m1: {self.motor1.get_current_angle()[0]} - {self.motor1_base_angle}, m2: {self.motor2.get_current_angle()[0]} - {self.motor2_base_angle} ")
        if self.target_tension is None:
            print("ℹ️  Pretension skipped (no target tension provided)")
            return True, True
        if flag:
            print("ℹ️  Starting pretension routine…")
            successm1 = successm2 = False
            try:
                successm1,successm2 = run_pretension_motors(
                self.motor1,
                self.motor2,
                self.sensor1,
                self.sensor2,
                target_tension=self.target_tension,
                direction1=self.motor1_cfg["direction"],
                direction2=self.motor2_cfg["direction"],
                )
                if successm1 and successm2:
                    angle1 = self.motor1.get_current_angle()[0]
                    angle2 = self.motor2.get_current_angle()[0]
                    self.motor1_base_angle = angle1
                    self.motor2_base_angle = angle2
            except Exception as e:
                print(f"pretension Error: {e}")
                self.cleanup()
            return successm1,successm2
        else:
            angle1 = self.motor1.get_current_angle()[0]
            angle2 = self.motor2.get_current_angle()[0]
            self.motor1_base_angle = angle1
            self.motor2_base_angle = angle2
            print(f"current agles m1: {self.motor1.get_current_angle()[0]} - {self.motor1_base_angle}, m2: {self.motor2.get_current_angle()[0]} - {self.motor2_base_angle} ")
            return True, True

    def readVal(self, duration):
        count=0
        start_time = time.time()
        while (time.time() - start_time) < duration:
            count+=1
            cm1=self.motor1.get_current_angle(max_retries=3, retry_delay=0.02)
            val=cm1[0] if cm1[0] is not None else ""
            # cmd2=self.motor2.get_current_angle

            # s1=self.sensor1.get_tension()
            # s2=self.sensor2.get_tension()
            print (f"COUNT: {count} -- m1: {val}")#, tension1 {s1}, m2: {cmd2}, tension2 {s2}")

    def run_trajectory_acquisition(self, duration):
        """Generate commands, read feedback/forces, and log to CSV."""
        sr = self.motors_frequency
        
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
            self.motor1.command(math.radians(-100))
            while (time.time() - start_time) < duration:
                now = time.time()
                elapsed = now - start_time

                cmd1 = float(self.motor1_cfg["trajectory"](elapsed))
                cmd2 = float(self.motor2_cfg["trajectory"](elapsed))
                
                # DEBUG: Print commands instead of sending to motors
                # print(f"ITER: {iteration} -- t={elapsed:.3f}s | Motor1: {cmd1:.4f} rad ({math.degrees(cmd1):.2f}°) | Motor2: {cmd2:.4f} rad ({math.degrees(cmd2):.2f}°)")
                a=time.time()
                # print(a)
                # # # # # # # self.motor1.command(cmd1)
                # # # # # # # # print(time.time()-a)
                # # # # # # # self.motor2.command(cmd2)

                try:
                    feedback1 = self.motor1.get_current_angle(max_retries=3, retry_delay=0.01)
                    # feedback2 = self.motor2.get_current_angle(max_retries=3, retry_delay=0.01)
                except Exception as e:
                    feedback1 = [None,None]
                    feedback1 = [None, None]

                tension1 = self.sensor1.get_tension()
                # tension2 = self.sensor2.get_tension()

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
                        # feedback2[0] if feedback2[0] is not None else "",
                        # feedback2[1] if feedback2[1] is not None else "",
                        # tension2,
                    ]
                )

                iteration += 1
                # next_time = start_time + iteration * period
                # sleep_time = next_time - time.time()
                # if sleep_time > 0:
                #     time.sleep(sleep_time)

            writer.writerows(rows)

        print("✅ Motor trajectory acquisition completed")


    def cleanup(self):
        """Release hardware and close the CAN bus."""
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
