# MOOG Control Class
# Author: Hudson Burke
from collections import deque
import serial
import threading
import time

# taskkill /F /IM python.exe


class MOOG():

    DOF_MAX = 32767
    DOF_NEUTRAL = 16383
    DOF_HEAVE_NEUTRAL = 29000
    LENGTH_NEUTRAL = 1024
    MACHINE_ID_LOW = 0x29
    MACHINE_ID_HIGH = 0x00

    def __init__(self, port='COM3', baudrate=57600, frequency=60, timeout=0.002, frame_timeout=0.012):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,                          # Serial port transfer rate (bits/s)
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=timeout,
            write_timeout=timeout,
        )
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        self._frame_timeout = frame_timeout

        # TODO: Enumerate these
        self.command_types_hex = {
            'ESTOP':                0xe6,
            'DISABLE':              0xdc,
            'PARK':                 0xd2,
            'LOW LIMIT ENABLE':     0xc8,
            'LOW LIMIT DISABLE':    0xbe,
            'ENGAGE':               0xb4,
            'START':                0xaf,
            'LENGTH MODE':          0xac,
            'DOF MODE':             0xaa,
            'RESET':                0xa0,
            'INHIBIT':              0x96,
            'RESERVED':             0x8c,
            'NEW POSITION':         0x82,
        }

        # Initial command to begin communication with the platform
        self._command = b"\xff\x82\x43\x00\x04\x00\x04\x00\x04\x00\x04\x00\x04\x00\x04\x29\x00"
        self._prev_command = self._command
        self._rx_buffer = bytearray()
        self._command_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None

        # Timing diagnostics for the 60 Hz transmit loop.
        self._last_send_time = None
        self._send_intervals = deque(maxlen=600)
        self._send_count = 0
        self._deadline_miss_count = 0
        self._transmit_error_count = 0

        # Status diagnostics for the read loop.
        self._status_frame_count = 0
        self._status_short_read_count = 0
        self._status_bad_sync_count = 0
        self._status_bad_id_count = 0
        self._status_bad_state_count = 0
        self._status_resync_count = 0
        self._status_error_count = 0
        self._last_status_time = None

        self.state = 'POWERUP'
        self.states = {
            0b0000: 'POWERUP',  # Also POWER_UP in manual
            0b0001: 'IDLE',
            0b0010: 'STANDBY',
            0b0011: 'ENGAGED',
            0b0100: '(NOT USED)',
            0b1000: 'PARKING',
            0b1001: 'FAULT2',
            0b1010: 'FAULT3',
            0b1011: 'DISABLED',
            0b1100: 'INHIBITED',
        }

        self.text_output = ""
        self.mode = 0
        self.current_actuator_commands = [1024, 1024, 1024, 1024, 1024, 1024]

        self._frequency = frequency
        self._period = 1 / self._frequency
        self._initialized = False

        self._thread = threading.Thread(target=self.communication_loop, daemon=True)
        self._thread.start()
        time.sleep(2)
        print("Ready to initialize")

    def communication_loop(self):  # publish the latest command at self._frequency Hz
        next_tick = time.monotonic()

        while not self._stop_event.is_set():
            with self._command_lock:
                command = self._command

            send_started = time.monotonic()
            if self._last_send_time is not None:
                self._send_intervals.append(send_started - self._last_send_time)
            self._last_send_time = send_started
            self._send_count += 1

            try:
                self.communicate(command)
            except (serial.SerialException, OSError) as exc:
                self._transmit_error_count += 1
                self._status_error_count += 1
                self.text_output = f"Serial communication error: {exc}"

            next_tick += self._period
            now = time.monotonic()
            sleep_time = next_tick - now

            if sleep_time > 0:
                self._stop_event.wait(sleep_time)
                continue

            missed_periods = max(1, int((-sleep_time) // self._period) + 1)
            self._deadline_miss_count += missed_periods
            next_tick += missed_periods * self._period

    # min = 15 max = 120
    def override_frequency(self, new_freq):
        if new_freq <= 0:
            raise ValueError('Frequency must be positive')
        self._frequency = new_freq
        self._period = 1 / self._frequency

    def communicate(self, command=None):
        if command is None:
            with self._command_lock:
                command = self._command

        self.ser.write(command)  # Write current command to platform base
        deadline = time.monotonic() + self._frame_timeout

        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            bytes_to_read = waiting if waiting > 0 else 1
            response_bytes = self.ser.read(bytes_to_read)

            if response_bytes:
                self._rx_buffer.extend(response_bytes)
                frame = self._extract_status_frame()
                if frame is not None:
                    if self._parse_status_frame(frame):
                        self._status_frame_count += 1
                        return True

        self._status_short_read_count += 1
        self.text_output = f'Timed out waiting for full status frame; buffered {len(self._rx_buffer)} bytes'
        return False

    def _extract_status_frame(self):
        frame_length = 20

        while len(self._rx_buffer) >= frame_length:
            sync_index = self._rx_buffer.find(b'\xff')
            if sync_index < 0:
                self._status_bad_sync_count += 1
                self._status_resync_count += len(self._rx_buffer)
                self._rx_buffer.clear()
                return None

            if sync_index > 0:
                self._status_bad_sync_count += 1
                self._status_resync_count += sync_index
                del self._rx_buffer[:sync_index]

            if len(self._rx_buffer) < frame_length:
                return None

            candidate = bytes(self._rx_buffer[:frame_length])
            if candidate[18] == self.MACHINE_ID_LOW and candidate[19] == self.MACHINE_ID_HIGH:
                del self._rx_buffer[:frame_length]
                return candidate

            self._status_bad_id_count += 1
            self._status_resync_count += 1
            del self._rx_buffer[0]

        return None

    def _parse_status_frame(self, response_bytes):
        if response_bytes[0] != 0xff:
            self._status_bad_sync_count += 1
            self.text_output = f'Invalid frame sync byte: 0x{response_bytes[0]:02x}'
            return False

        if response_bytes[18] != self.MACHINE_ID_LOW or response_bytes[19] != self.MACHINE_ID_HIGH:
            self._status_bad_id_count += 1
            self.text_output = (
                'Invalid motion base ID in status frame: '
                f'0x{response_bytes[18]:02x} 0x{response_bytes[19]:02x}'
            )
            return False

        machine_state_int = response_bytes[17]
        state_code = machine_state_int & 0b1111
        if state_code not in self.states:
            self._status_bad_state_count += 1
            self.text_output = f'Invalid machine state code in status frame: {state_code}'
            return False

        self.response = [f'{byte:02x}' for byte in response_bytes]

        # parse through 20 bytes read from serial port
        self.frame_sync = self.response[0]
        self.fault_data_1 = self.response[1]
        self.fault_data_2 = self.response[2]
        self.discrete_io_info = self.response[3]
        self.checksum = self.response[4]
        self.actuator_a_low_feedback = self.response[5]
        self.actuator_a_high_feedback = self.response[6]
        self.actuator_b_low_feedback = self.response[7]
        self.actuator_b_high_feedback = self.response[8]
        self.actuator_c_low_feedback = self.response[9]
        self.actuator_c_high_feedback = self.response[10]
        self.actuator_d_low_feedback = self.response[11]
        self.actuator_d_high_feedback = self.response[12]
        self.actuator_e_low_feedback = self.response[13]
        self.actuator_e_high_feedback = self.response[14]
        self.actuator_f_low_feedback = self.response[15]
        self.actuator_f_high_feedback = self.response[16]
        self.machine_state_info = self.response[17]
        self.motion_base_id_low = self.response[18]
        self.motion_base_id_high = self.response[19]

        self.state = self.states[state_code]
        self.mode = (machine_state_int >> 4) & 1
        self._last_status_time = time.monotonic()
        return True

    # command_type string corresponding to hex in dict ; commands list of ints
    def build_frame(self, command_type: str, commands=None):
        if command_type not in self.command_types_hex:
            raise KeyError(f'Unknown command type: {command_type}')

        if commands is None:
            commands = [0, 0, 0, 0, 0, 0]

        if len(commands) != 6:
            raise ValueError('MOOG frames require exactly 6 command values')

        command_prefix_hex = 0xff  # frame sync
        command_machine_id_low = self.MACHINE_ID_LOW
        command_machine_id_high = self.MACHINE_ID_HIGH

        command_type_hex = self.command_types_hex[command_type]

        # Split commands into low and high bytes in hex (high bytes must have msb of 0)
        commands_hex = []
        for command in commands:
            command = int(command) & 0x7FFF
            commands_hex.append(command & 0xff)  # low byte
            commands_hex.append(command >> 8)    # high byte

        # Add bytes 1 and 3-16, limit to 8 bits, then zero MSB
        checksum = command_type_hex + sum(commands_hex) + command_machine_id_low + command_machine_id_high
        checksum &= 0b01111111

        # Final command to send in hex
        frame = bytes([
            command_prefix_hex,
            command_type_hex,
            checksum,
            *commands_hex,
            command_machine_id_low,
            command_machine_id_high,
        ])
        return frame

    def wait_until(self, predicate, timeout, poll_interval=None, on_poll=None):
        deadline = time.monotonic() + timeout
        poll_interval = self._period if poll_interval is None else poll_interval

        while time.monotonic() < deadline and not self._stop_event.is_set():
            if predicate():
                return True
            if on_poll is not None:
                on_poll()
            time.sleep(poll_interval)

        return predicate()

    def initialize_platform(self):
        if self.state in {'FAULT2', 'FAULT3', 'INHIBITED'}:
            raise RuntimeError('MOOG must be reset before initialization')

        if not self.wait_until(lambda: self.mode == 1, timeout=10.0, on_poll=self.dof_mode):
            raise TimeoutError(f'Timed out entering DOF mode. Current state: {self.state}')

        # Continue to send
        self.command_dof()
        time.sleep(1)

        if not self.wait_until(lambda: self.state == 'STANDBY', timeout=10.0, on_poll=self.engage):
            raise TimeoutError(f'Timed out entering STANDBY. Current state: {self.state}')

        if not self.wait_until(lambda: self.state == 'ENGAGED', timeout=30.0, on_poll=self.command_dof):
            raise TimeoutError(f'Timed out entering ENGAGED. Current state: {self.state}')

        print('Engaged')
        self._initialized = True

    def e_stop(self):
        if self.state != 'IDLE':
            self.command('ESTOP')

    def disable(self):
        self.command('DISABLE')
        self.text_output = 'Base Disabled. Please remove & re-apply power to reset.'

    def park(self, timeout=5.0):
        if self.state in {'PARKING', 'IDLE'}:
            return True

        if self.state not in {'ENGAGED', 'STANDBY'}:
            self.text_output = 'PARK valid only in ENGAGED, STANDBY states'
            print(self.text_output)
            return False

        return self.wait_until(
            lambda: self.state in {'PARKING', 'IDLE'},
            timeout=timeout,
            on_poll=lambda: self.command('PARK'),
        )

    def close(self, park=True, timeout=5.0):
        if self._stop_event.is_set():
            return

        if park and self.state in {'ENGAGED', 'STANDBY'}:
            self.park(timeout=timeout)
            self.wait_until(lambda: self.state == 'IDLE', timeout=timeout)

        self._initialized = False
        self._stop_event.set()

        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)

        if self.ser.is_open:
            self.ser.close()

    def get_timing_stats(self):
        intervals = list(self._send_intervals)
        if not intervals:
            return {
                'target_frequency_hz': self._frequency,
                'samples': 0,
                'send_count': self._send_count,
                'deadline_miss_count': self._deadline_miss_count,
                'transmit_error_count': self._transmit_error_count,
            }

        average_period = sum(intervals) / len(intervals)
        return {
            'target_frequency_hz': self._frequency,
            'samples': len(intervals),
            'average_period_s': average_period,
            'average_frequency_hz': 1 / average_period if average_period > 0 else float('inf'),
            'min_period_s': min(intervals),
            'max_period_s': max(intervals),
            'send_count': self._send_count,
            'deadline_miss_count': self._deadline_miss_count,
            'transmit_error_count': self._transmit_error_count,
        }

    def get_status_stats(self):
        return {
            'status_frame_count': self._status_frame_count,
            'status_short_read_count': self._status_short_read_count,
            'status_bad_sync_count': self._status_bad_sync_count,
            'status_bad_id_count': self._status_bad_id_count,
            'status_bad_state_count': self._status_bad_state_count,
            'status_resync_count': self._status_resync_count,
            'status_error_count': self._status_error_count,
            'last_status_age_s': self.get_status_age(),
            'rx_buffer_len': len(self._rx_buffer),
            'state': self.state,
            'mode': self.mode,
        }

    def get_status_age(self):
        if self._last_status_time is None:
            return None
        return time.monotonic() - self._last_status_time

    def low_limit_enable(self):
        self.command('LOW LIMIT ENABLE')

    def low_limit_disable(self):
        self.command('LOW LIMIT DISABLE')

    def engage(self):
        if self.state == 'IDLE':
            if self.mode:
                self.text_output = 'Engaging in DOF MODE...'
                commands = [16383, 16383, 29000, 16383, 16383, 16383]
            else:
                self.text_output = 'Engaging in Length Mode...'
                commands = [1024, 1024, 1024, 1024, 1024, 1024]
            self.command('ENGAGE', commands)
        else:
            self.text_output = 'ENGAGE valid only in the IDLE state'

    def is_engaged(self):
        return self.state == 'ENGAGED'

    # Same as ENGAGE, except the user may define the starting position of the base.
    def start(self, starting_position=None):  # TODO: differentiate between length and DOF mode and add limits
        if self.state == 'IDLE':
            self.command('START', starting_position)
        else:
            self.text_output = 'START valid only in the IDLE state'

    def length_mode(self):
        if self.state == 'IDLE' or self.state == 'POWERUP':
            self.command('LENGTH MODE', [1024, 1024, 1024, 1024, 1024, 1024])
        else:
            self.text_output = 'LENGTH MODE valid only in IDLE, POWERUP states'

    def dof_mode(self):
        if self.state == 'IDLE' or self.state == 'POWERUP':
            self.command('DOF MODE', [16383, 16383, 29000, 16383, 16383, 16383])
        else:
            self.text_output = 'LENGTH MODE valid only in IDLE, POWERUP states'

    def reset(self):
        if self.state == 'FAULT2' or self.state == 'FAULT3' or self.state == 'INHIBITED':
            self.command('RESET')

    def inhibit(self):
        if self.state == 'POWERUP' or self.state == 'IDLE':
            self.command('INHIBIT')
            self.text_output = 'Base will ignore all other commands until next RESET'
        else:
            self.text_output = 'INHIBIT valid only in IDLE, POWERUP states'

    def command_dof(self, roll=16383, pitch=16383, heave=29000, surge=16383, yaw=16383, lateral=16383, buffer=False):
        self.command('NEW POSITION', [roll, pitch, heave, surge, yaw, lateral], buffer=buffer)

    def command_dof_degrees(self, roll=0, pitch=0, heave=29000, surge=16383, yaw=0, lateral=16383, buffer=False):
        roll = max(min(roll, 29), -29)
        pitch = max(min(pitch, 33), -33)
        yaw = max(min(yaw, 29), -29)

        roll = max(int(32767 / 58 * (roll + 29)), 0)
        pitch = max(int(32767 / 66 * (pitch + 33)), 0)
        yaw = max(int(32767 / 58 * (yaw + 29)), 0)

        self.command_dof(roll, pitch, heave, surge, yaw, lateral, buffer)

    def command_length(self, a=1024, b=1024, c=1024, d=1024, e=1024, f=1024):
        if not self.mode:
            self.command('NEW POSITION', [a, b, c, d, e, f])
        else:
            self.text_output = 'Command rejected: Base currently in DOF Mode'

    def set_command(self, new_command):
        with self._command_lock:
            self._prev_command = self._command
            self._command = bytes(new_command)

    def command(self, command_type, commands=None, buffer=False):
        if commands is None:
            self.text_output = 'No actuator values provided. Will use default for current mode...'
            if self.mode:  # checks for DOF mode
                commands = [16383, 16383, 29000, 16383, 16383, 16383]
            else:
                commands = [1024, 1024, 1024, 1024, 1024, 1024]

        if buffer:
            self.text_output = 'Command buffering is disabled; latest telemetry command replaces the pending command.'

        frame = self.build_frame(command_type, commands)

        if len(commands) == 6:
            self.current_actuator_commands = list(commands)

        # Deliberately do not queue commands: latest telemetry should win so the
        # base tracks the most recent Assetto state at the next 60 Hz send tick.
        with self._command_lock:
            self._prev_command = self._command
            self._command = frame
