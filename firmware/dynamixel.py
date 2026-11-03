"""Dynamixel Protocol 2.0 driver for CircuitPython."""

import busio
import digitalio
import time

HEADER = b"\xff\xff\xfd\x00"

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REBOOT = 0x08

# Model numbers (control table address 0).
MODEL_NAMES = {
    1080: "XC430-W240",
    1210: "XC330-T181",
    1220: "XC330-T288",
}

# Models where address 126 is Present Current (1 mA) instead of
# Present Load (0.1 %). All other addresses used here are shared.
CURRENT_MODELS = (1210, 1220)
INST_SYNC_WRITE = 0x83

# CRC lookup table for Protocol 2.0
_CRC_TABLE = [
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202,
]


def _crc16(data):
    crc = 0
    for b in data:
        i = ((crc >> 8) ^ b) & 0xFF
        crc = ((crc << 8) ^ _CRC_TABLE[i]) & 0xFFFF
    return crc


class Dynamixel:
    def __init__(self, uart, dir_pin=None, timeout=0.02):
        """dir_pin: DigitalInOut toggled around each write, or None if the
        UART drives the direction pin itself (busio.UART rs485_dir).
        """
        self.uart = uart
        self.dir = dir_pin
        if self.dir is not None:
            self.dir.switch_to_output(value=False)  # RX mode by default
        self.timeout = timeout
        self._models = {}          # motor_id -> model number
        self._current_limits = {}  # motor_id -> Current Limit (mA)

    def _build_packet(self, motor_id, instruction, params=b""):
        length = len(params) + 3  # instruction + params + 2 CRC bytes
        pkt = bytearray(HEADER)
        pkt.append(motor_id)
        pkt.append(length & 0xFF)
        pkt.append((length >> 8) & 0xFF)
        pkt.append(instruction)
        pkt.extend(params)
        crc = _crc16(pkt)
        pkt.append(crc & 0xFF)
        pkt.append((crc >> 8) & 0xFF)
        return pkt

    def _send(self, packet):
        self.uart.reset_input_buffer()
        if self.dir is None:
            self.uart.write(packet)  # UART driver toggles rs485_dir around this
            return
        self.dir.value = True  # TX mode
        self.uart.write(packet)
        # wait for bytes to finish transmitting
        time.sleep(len(packet) * 10 / self.uart.baudrate)
        self.dir.value = False  # RX mode

    def _recv(self):
        start = time.monotonic()
        buf = bytearray()
        while time.monotonic() - start < self.timeout:
            chunk = self.uart.read(64)
            if chunk:
                buf.extend(chunk)
                # find the header in the buffer
                idx = 0
                while idx <= len(buf) - 4:
                    if buf[idx:idx+4] == HEADER:
                        break
                    idx += 1
                else:
                    continue
                # strip noise bytes before header
                if idx > 0:
                    buf = buf[idx:]
                # check if we have a complete packet
                if len(buf) >= 7:
                    pkt_len = buf[5] | (buf[6] << 8)
                    if len(buf) >= 7 + pkt_len:
                        return buf[:7 + pkt_len]
        return buf if buf else None

    def _transact(self, motor_id, instruction, params=b"", retries=3):
        for attempt in range(retries):
            pkt = self._build_packet(motor_id, instruction, params)
            self._send(pkt)
            if motor_id == 0xFE:  # broadcast, no response
                return None
            resp = self._recv()
            if resp is not None and len(resp) >= 11:
                error = resp[8]
                if error:
                    raise RuntimeError(f"Motor {motor_id} error: 0x{error:02X}")
                return resp
        raise RuntimeError(f"No valid response from motor {motor_id} after {retries} attempts")

    def ping(self, motor_id):
        resp = self._transact(motor_id, INST_PING)
        model = resp[9] | (resp[10] << 8)
        fw = resp[11]
        self._models[motor_id] = model
        return {"model": model, "name": MODEL_NAMES.get(model, "unknown"),
                "firmware": fw}

    def get_model(self, motor_id):
        """Model number, cached after the first ping."""
        if motor_id not in self._models:
            self.ping(motor_id)
        return self._models[motor_id]

    def reboot(self, motor_id):
        self._transact(motor_id, INST_REBOOT)

    def read(self, motor_id, address, length):
        params = bytearray([
            address & 0xFF, (address >> 8) & 0xFF,
            length & 0xFF, (length >> 8) & 0xFF,
        ])
        resp = self._transact(motor_id, INST_READ, params)
        return resp[9:9 + length]

    def write(self, motor_id, address, data):
        params = bytearray([address & 0xFF, (address >> 8) & 0xFF])
        params.extend(data)
        self._transact(motor_id, INST_WRITE, params)

    def sync_write(self, address, length, id_data_pairs):
        """Write same-length data to multiple motors in one packet.

        id_data_pairs: list of (motor_id, bytes) tuples.
        No response is returned (broadcast).
        """
        params = bytearray([
            address & 0xFF, (address >> 8) & 0xFF,
            length & 0xFF, (length >> 8) & 0xFF,
        ])
        for mid, data in id_data_pairs:
            params.append(mid)
            params.extend(data)
        pkt = self._build_packet(0xFE, INST_SYNC_WRITE, params)
        self._send(pkt)

    def sync_move(self, moves):
        """Move multiple motors simultaneously.

        moves: list of (motor_id, velocity, position) tuples.
        Sync writes Profile Velocity (addr 112, 4 bytes) and
        Goal Position (addr 116, 4 bytes) in one packet.
        """
        pairs = []
        for mid, vel, pos in moves:
            data = bytearray([
                vel & 0xFF, (vel >> 8) & 0xFF,
                (vel >> 16) & 0xFF, (vel >> 24) & 0xFF,
                pos & 0xFF, (pos >> 8) & 0xFF,
                (pos >> 16) & 0xFF, (pos >> 24) & 0xFF,
            ])
            pairs.append((mid, data))
        self.sync_write(112, 8, pairs)

    # -- convenience methods --

    def write_u8(self, motor_id, address, value):
        self.write(motor_id, address, bytes([value & 0xFF]))

    def write_u16(self, motor_id, address, value):
        self.write(motor_id, address, bytes([value & 0xFF, (value >> 8) & 0xFF]))

    def write_u32(self, motor_id, address, value):
        self.write(motor_id, address, bytes([
            value & 0xFF, (value >> 8) & 0xFF,
            (value >> 16) & 0xFF, (value >> 24) & 0xFF,
        ]))

    def read_u8(self, motor_id, address):
        return self.read(motor_id, address, 1)[0]

    def read_u16(self, motor_id, address):
        d = self.read(motor_id, address, 2)
        return d[0] | (d[1] << 8)

    def read_u32(self, motor_id, address):
        d = self.read(motor_id, address, 4)
        return d[0] | (d[1] << 8) | (d[2] << 16) | (d[3] << 24)

    def read_s16(self, motor_id, address):
        v = self.read_u16(motor_id, address)
        return v if v < 0x8000 else v - 0x10000

    def read_s32(self, motor_id, address):
        v = self.read_u32(motor_id, address)
        return v if v < 0x80000000 else v - 0x100000000

    # -- XC430 / XC330 control table shortcuts --

    def set_torque(self, motor_id, enable):
        self.write_u8(motor_id, 64, 1 if enable else 0)

    def set_led(self, motor_id, on):
        self.write_u8(motor_id, 65, 1 if on else 0)

    def set_goal_position(self, motor_id, position):
        """Position in range 0-4095 (0-360 degrees)."""
        self.write_u32(motor_id, 116, position)

    def get_position(self, motor_id):
        return self.read_s32(motor_id, 132)

    def set_profile_velocity(self, motor_id, velocity):
        """Velocity in units of 0.229 RPM. 0 = max speed."""
        self.write_u32(motor_id, 112, velocity)

    def get_profile_velocity(self, motor_id):
        """Returns profile velocity in units of 0.229 RPM."""
        return self.read_u32(motor_id, 112)

    def set_profile_acceleration(self, motor_id, accel):
        """Acceleration in units of 214.577 rev/min^2. 0 = max."""
        self.write_u32(motor_id, 108, accel)

    def set_operating_mode(self, motor_id, mode):
        """Set operating mode. Disable torque first!
        1=velocity, 3=position, 4=extended position, 5=current-based position,
        16=PWM
        """
        self.write_u8(motor_id, 11, mode)

    def get_load(self, motor_id):
        """Returns present load as a percentage (-100.0 to 100.0).

        XC430: Present Load, percentage of max torque.
        XC330: Present Current as a percentage of Current Limit.
        """
        if self.get_model(motor_id) in CURRENT_MODELS:
            return self.get_current(motor_id) * 100.0 / self.get_current_limit(motor_id)
        return self.read_s16(motor_id, 126) / 10.0

    def get_current(self, motor_id):
        """Returns present current in mA (XC330 only)."""
        if self.get_model(motor_id) not in CURRENT_MODELS:
            raise RuntimeError(f"Motor {motor_id} has no current sensing")
        return self.read_s16(motor_id, 126)

    def get_current_limit(self, motor_id):
        """Returns Current Limit in mA (XC330 only), cached after first read."""
        if motor_id not in self._current_limits:
            self._current_limits[motor_id] = self.read_u16(motor_id, 38)
        return self._current_limits[motor_id]

    def get_velocity(self, motor_id):
        """Returns present velocity in units of 0.229 RPM."""
        return self.read_s32(motor_id, 128)

    def get_moving(self, motor_id):
        return bool(self.read_u8(motor_id, 122))

    def get_temperature(self, motor_id):
        """Returns temperature in degrees C."""
        return self.read_u8(motor_id, 146)

    def get_voltage(self, motor_id):
        """Returns voltage in units of 0.1V."""
        return self.read_u16(motor_id, 144)
