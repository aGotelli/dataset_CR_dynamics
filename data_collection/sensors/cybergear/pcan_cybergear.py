import struct
import can
import logging
import enum
import math


class CANMotorController:
    PARAM_TABLE = {
        "motorOverTemp": {"feature_code": 0x200D, "type": "int16"},
        "overTempTime": {"feature_code": 0x200E, "type": "int32"},
        "limit_torque": {"feature_code": 0x2007, "type": "float"},
        "cur_kp": {"feature_code": 0x2012, "type": "float"},
        "cur_ki": {"feature_code": 0x2013, "type": "float"},
        "spd_kp": {"feature_code": 0x2014, "type": "float"},
        "spd_ki": {"feature_code": 0x2015, "type": "float"},
        "loc_kp": {"feature_code": 0x2016, "type": "float"},
        "spd_filt_gain": {"feature_code": 0x2017, "type": "float"},
        "limit_spd": {"feature_code": 0x2018, "type": "float"},
        "limit_cur": {"feature_code": 0x2019, "type": "float"},
    }

    PARAMETERS = {
        "run_mode": {"index": 0x7005, "format": "u8"},
        "iq_ref": {"index": 0x7006, "format": "f"},
        "spd_ref": {"index": 0x700A, "format": "f"},
        "limit_torque": {"index": 0x700B, "format": "f"},
        "cur_kp": {"index": 0x7010, "format": "f"},
        "cur_ki": {"index": 0x7011, "format": "f"},
        "cur_filt_gain": {"index": 0x7014, "format": "f"},
        "loc_ref": {"index": 0x7016, "format": "f"},
        "limit_spd": {"index": 0x7017, "format": "f"},
        "limit_cur": {"index": 0x7018, "format": "f"},
    }
    TWO_BYTES_BITS = 16

    def __init__(self, bus, motor_id=127, main_can_id=254):
        """
        Initialize CAN motor controller.

        Args:
            bus: CAN bus object
            motor_id: Motor's CAN ID
            main_can_id: Main CAN ID
        """
        self.bus = bus
        self.MOTOR_ID = motor_id
        self.MAIN_CAN_ID = main_can_id
        self.P_MIN = -12.5
        self.P_MAX = 12.5
        self.V_MIN = -30.0
        self.V_MAX = 30.0
        self.T_MIN = -12.0
        self.T_MAX = 12.0
        self.KP_MIN, self.KP_MAX = 0.0, 500.0  # 0.0 ~ 500.0
        self.KD_MIN, self.KD_MAX = 0.0, 5.0  # 0.0 ~ 5.0

    # Communication types
    class CmdModes:
        GET_DEVICE_ID = 0
        MOTOR_CONTROL = 1
        MOTOR_FEEDBACK = 2
        MOTOR_ENABLE = 3
        MOTOR_STOP = 4
        SET_MECHANICAL_ZERO = 6
        SET_MOTOR_CAN_ID = 7
        PARAM_TABLE_WRITE = 8
        SINGLE_PARAM_READ = 17
        SINGLE_PARAM_WRITE = 18
        FAULT_FEEDBACK = 21
    
    # Control modes
    class RunModes(enum.Enum):
        CONTROL_MODE = 0  # Motion control mode
        POSITION_MODE = 1  # Position mode
        SPEED_MODE = 2    # Speed mode
        CURRENT_MODE = 3  # Current mode

    def _float_to_uint(self, x, x_min, x_max, bits):
        """
        Convert float to unsigned integer.

        Args:
            x: Input float
            x_min: Minimum acceptable float value
            x_max: Maximum acceptable float value
            bits: Number of bits for output unsigned integer

        Returns:
            Converted unsigned integer
        """
        span = x_max - x_min
        offset = x_min
        x = max(min(x, x_max), x_min)  # Clamp x to the range [x_min, x_max]
        return int(((x - offset) * ((1 << bits) - 1)) / span)

    def _uint_to_float(self, x, x_min, x_max, bits):
        """
        Convert unsigned integer to float.

        Args:
            x: Input unsigned integer
            x_min: Minimum acceptable float value
            x_max: Maximum acceptable float value
            bits: Number of bits for input unsigned integer

        Returns:
            Converted float value
        """
        span = (1 << bits) - 1
        offset = x_max - x_min
        x = max(min(x, span), 0)  # Clamp x to the range [0, span]
        return offset * x / span + x_min

    def _linear_mapping(self, value, value_min, value_max, target_min=0, target_max=65535):
        """
        Perform linear mapping of input value.

        Args:
            value: Input value
            value_min: Minimum input value
            value_max: Maximum input value
            target_min: Minimum target value
            target_max: Maximum target value

        Returns:
            Mapped value
        """
        return int(
            (value - value_min) / (value_max - value_min) * (target_max - target_min)
            + target_min
        )

    def format_data(self, data=[], format="f f", type="decode"):
        """
        Encode or decode data.

        Args:
            data: Input data list
            format: Data format
            type: "encode" or "decode"

        Returns:
            Encoded or decoded data
        """
        format_list = format.split()
        rdata = []
        if type == "decode":
            p = 0
            for f in format_list:
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                ba = bytearray()
                if len(s_f) == 2:
                    for i in range(s_f[0]):
                        ba.append(data[p])
                        p = p + 1
                    rdata.append(struct.unpack(s_f[1], ba)[0])
                else:
                    logging.info("unknown format in format_data(): " + f)
                    return []
            return rdata
        elif type == "encode" and len(format_list) == len(data):
            for i in range(len(format_list)):
                f = format_list[i]
                s_f = []
                if f == "f":
                    s_f = [4, "f"]
                elif f == "u16":
                    s_f = [2, "H"]
                elif f == "s16":
                    s_f = [2, "h"]
                elif f == "u32":
                    s_f = [4, "I"]
                elif f == "s32":
                    s_f = [4, "i"]
                elif f == "u8":
                    s_f = [1, "B"]
                elif f == "s8":
                    s_f = [1, "b"]
                if f != "f":
                    data[i] = int(data[i])
                if len(s_f) == 2:
                    bs = struct.pack(s_f[1], data[i])
                    for j in range(s_f[0]):
                        rdata.append(bs[j])
                else:
                    logging.info("unkown format in format_data(): " + f)
                    return []
            if len(rdata) < 4:
                for i in range(4 - len(rdata)):
                    rdata.append(0x00)
            return rdata

    def pack_to_8bytes(self, target_angle, target_velocity, Kp, Kd):
        """
        Pack control parameters into 8 bytes of data.

        Args:
            target_angle: Target angle
            target_velocity: Target velocity
            Kp: Proportional gain
            Kd: Derivative gain

        Returns:
            8 bytes of data
        """
        # 对输入变量进行线性映射
        target_angle_mapped = self._linear_mapping(
            target_angle, self.P_MIN, self.P_MAX)
        target_velocity_mapped = self._linear_mapping(
            target_velocity, self.V_MIN, self.V_MAX
        )
        Kp_mapped = self._linear_mapping(Kp, self.KP_MIN, self.KP_MAX)
        Kd_mapped = self._linear_mapping(Kd, self.KD_MIN, self.KD_MAX)

        # 使用Python的struct库进行打包
        # 使用H表示无符号短整数(2字节), 共需要8字节
        data1_bytes = struct.pack(
            "HHHH", target_angle_mapped, target_velocity_mapped, Kp_mapped, Kd_mapped
        )
        data1 = [b for b in data1_bytes]
        return data1

    def send_receive_can_message(self, cmd_mode, data2, data1, timeout=200):
        """
        Send CAN message and receive response.

        Args:
            cmd_mode: Command mode
            data2: Data area 2
            data1: Data bytes to send
            timeout: Message timeout in ms (default: 200ms)

        Returns:
            Tuple containing received message data and arbitration ID
        """
        # Calculate the arbitration ID
        arbitration_id = (cmd_mode << 24) | (data2 << 8) | self.MOTOR_ID
        message = can.Message(
            arbitration_id=arbitration_id, data=data1, is_extended_id=True
        )

        # Send the CAN message
        try:
            self.bus.send(message)
        except:
            logging.info("Failed to send the message.")
            return None, None

        # Output details of the sent message
        logging.debug(
            f"Sent message with ID {hex(arbitration_id)}, data: {data1}")

        # Receive a CAN message with a 1-second timeout
        # 1-second timeout for receiving
        received_msg = self.bus.recv(timeout=1)
        if received_msg:
            return received_msg.data, received_msg.arbitration_id
        else:
            return None, None

    def parse_received_msg(self, data, arbitration_id):
        """
        Parse received CAN message.

        Args:
            data: Received data
            arbitration_id: Message arbitration ID

        Returns:
            Tuple containing motor CAN ID, position (rad), velocity (rad/s), torque (Nm), temperature (°C)
        """
        if data is not None:
            logging.debug(f"Received message with ID {hex(arbitration_id)}")
            print(f"Received message with ID {hex(arbitration_id & 0xFF)}") # debug comm issue
            # 解析电机CAN ID
            motor_can_id = (arbitration_id >> 8) & 0xFF

            pos = self._uint_to_float(
                (data[0] << 8) + data[1], self.P_MIN, self.P_MAX, self.TWO_BYTES_BITS
            )
            vel = self._uint_to_float(
                (data[2] << 8) + data[3], self.V_MIN, self.V_MAX, self.TWO_BYTES_BITS
            )
            torque = self._uint_to_float(
                (data[4] << 8) + data[5], self.T_MIN, self.T_MAX, self.TWO_BYTES_BITS
            )
            # 解析温度数据
            temperature_raw = (data[6] << 8) + data[7]
            temperature_celsius = temperature_raw / 10.0
            
            logging.info(
                f"Motor CAN ID: {motor_can_id}, pos: {pos:.2f} rad, vel: {vel:.2f} rad/s, "
                f"torque: {torque:.2f} Nm, temperature: {temperature_celsius:.1f} °C"
            )
            
            return motor_can_id, pos, vel, torque, temperature_celsius
        else:
            logging.info("No message received within the timeout period.")
            return None, None, None, None, None

    def clear_can_rx(self, timeout=10):
        """
        Clear all existing messages from receive buffer.

        Args:
            timeout: Time to wait for clear operation in ms
        """
        timeout_seconds = timeout / 1000.0  # Convert to seconds
        while True:
            received_msg = self.bus.recv(timeout=timeout_seconds)
            if received_msg is None:
                break
            logging.info(
                f"Cleared message with ID {hex(received_msg.arbitration_id)}")

    def _write_single_param(self, index, value, format="u32"):
        """
        Write single parameter.

        Args:
            index: Parameter index
            value: Value to set
            format: Data format

        Returns:
            Parsed received message
        """
        encoded_data = self.format_data(
            data=[value], format=format, type="encode")
        data1 = [b for b in struct.pack("<I", index)] + encoded_data

        self.clear_can_rx()  # 空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SINGLE_PARAM_WRITE,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def write_single_param(self, param_name, value):
        """
        Write single parameter by name.

        Args:
            param_name: Parameter name
            value: Value to set

        Returns:
            Write operation result
        """
        param_info = self.PARAMETERS.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return

        index = param_info["index"]
        format = param_info["format"]

        return self._write_single_param(index=index, value=value, format=format)

    def write_param_table(self, param_name, value):
        """
        Write parameter table.

        Args:
            param_name: Parameter name
            value: Value to set

        Returns:
            Received message data and arbitration ID
        """
        # Get the parameter info from PARAM_TABLE
        param_info = self.PARAM_TABLE.get(param_name)
        if param_info is None:
            logging.info(f"Unknown parameter name: {param_name}")
            return None, None, None

        feature_code = param_info["feature_code"]
        param_type = param_info["type"]

        # Type code mapping
        type_code_mapping = {
            "float": 0x06,
            "int16": 0x03,
            "int32": 0x04,
        }

        type_code = type_code_mapping.get(param_type)
        if type_code is None:
            logging.info(f"Unknown parameter type: {param_type}")
            return None, None, None

        # Encode the value based on the type
        format_mapping = {
            "float": "f",
            "int16": "s16",
            "int32": "s32",
        }

        format = format_mapping.get(param_type)
        encoded_value = self.format_data(
            data=[value], format=format, type="encode")

        # Construct data1
        data1 = [b for b in struct.pack("<H", feature_code)]
        data1.extend([type_code, 0x00])
        data1.extend(encoded_value)

        # Clear the CAN receive buffer
        self.clear_can_rx()

        # Send the CAN message
        cmd_mode = self.CmdModes.PARAM_TABLE_WRITE
        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=cmd_mode,
            data2=self.MAIN_CAN_ID,
            data1=data1,
        )

        return received_msg_data, received_msg_arbitration_id

    def set_0_pos(self):
        """
        Set motor mechanical zero point.

        Returns:
            Parsed received message
        """
        self.clear_can_rx()  # 清空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.SET_MECHANICAL_ZERO,
            data2=self.MAIN_CAN_ID,
            data1=[1],
        )

        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def enable(self):
        """
        Enable motor operation.

        Returns:
            Parsed received message
        """
        self.clear_can_rx(0)  # 清空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_ENABLE, data2=self.MAIN_CAN_ID, data1=[]
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def disable(self):
        """
        Stop motor operation.

        Returns:
            Parsed received message
        """
        self.clear_can_rx(0)  # 空CAN接收缓存, 避免读到老数据

        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=self.CmdModes.MOTOR_STOP,
            data2=self.MAIN_CAN_ID,
            data1=[0, 0, 0, 0, 0, 0, 0, 0],
        )
        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)

    def set_run_mode(self, mode):
        """
        Set operation mode.

        Args:
            mode: Operation mode (must be an instance of RunModes enum)

        Returns:
            Write operation result
        """
        if not isinstance(mode, self.RunModes):
            raise ValueError(
                f"Invalid mode: {mode}. Must be an instance of RunModes enum.")
        return self.write_single_param("run_mode", value=mode.value)

    def set_motor_position_control(self, limit_spd, loc_ref):
        """
        Set motor position control parameters in position mode.

        Args:
            limit_spd: Maximum motor speed
            loc_ref: Target motor position
        """
        # 设置电机最大速度
        self.write_single_param(param_name="limit_spd", value=limit_spd)
        # 设置电机目标位置
        self.write_single_param(param_name="loc_ref", value=loc_ref)

    def send_motor_control_command(self, torque, target_angle, target_velocity, Kp, Kd):
        """
        Send motor control command in motion control mode.

        Args:
            torque: Torque
            target_angle: Target angle
            target_velocity: Target velocity
            Kp: Proportional gain
            Kd: Derivative gain

        Returns:
            Parsed received message
        """

        # 生成29位的仲裁ID的组成部分
        cmd_mode = self.CmdModes.MOTOR_CONTROL
        torque_mapped = self._linear_mapping(
            torque, -12.0, 12.0, target_min=0, target_max=65535)
        data2 = torque_mapped

        # 将实际值映射到消息值
        target_angle_mapped = self._linear_mapping(
            target_angle, -4 * math.pi, 4 * math.pi)
        target_velocity_mapped = self._linear_mapping(
            target_velocity, -30.0, 30.0)
        Kp_mapped = self._linear_mapping(Kp, 0.0, 500.0)
        Kd_mapped = self._linear_mapping(Kd, 0.0, 5.0)

        # 创建8字节的数据区
        data1_bytes = struct.pack(
            "HHHH", target_angle_mapped, target_velocity_mapped, Kp_mapped, Kd_mapped
        )
        data1 = [b for b in data1_bytes]

        # 使用send_receive_can_message方法发送消息并接收响应
        received_msg_data, received_msg_arbitration_id = self.send_receive_can_message(
            cmd_mode=cmd_mode,
            data2=data2,
            data1=data1
        )

        return self.parse_received_msg(received_msg_data, received_msg_arbitration_id)
