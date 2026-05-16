from PyAccSharedMemory import accSharedMemory
from moog_class import MOOG
import time
import numpy as np

# Tunable Parameters
roll_window_size = 40  # proposed 50 / old 16
pitch_window_size = 40  # proposed 50 / old 18
yaw_window_size = 15  # proposed 30 / old 4

yaw_vel_threshold = 0.5
# 4 and 10 completely got rid of it. Maybe we still want some?
gear_dampening_scale_factor = 7
gear_dampening_window_size = 20

roll_scale_factor = 1.0
pitch_scale_factor = 1.0
yaw_scale_factor = 1.0

roll_degree_excursion = 25
pitch_degree_excursion = 28
yaw_degree_excursion = 29

roll_actuator_max = 32767
pitch_actuator_max = 32767
yaw_actuator_max = 32767

roll_actuator_min = 0
pitch_actuator_min = 0
yaw_actuator_min = 0

x_accel_limit = 1  # Gs (corresponds to roll)
z_accel_limit = 1  # Gs (corresponds to pitch)
g_force_is_in_g = True


def signed_angle_diff(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def main():
    moog = None
    asm = None

    try:
        moog = MOOG()
        time.sleep(2)
        
        print('Resetting...')
        if not moog.wait_until(lambda: moog.state == 'IDLE', timeout=30.0, on_poll=moog.reset):
            raise TimeoutError(f'Timed out waiting for MOOG to become IDLE. Current state: {moog.state}')

        moog.initialize_platform()
        
        asm = accSharedMemory()

        roll_avg = np.zeros(roll_window_size)
        pitch_avg = np.zeros(pitch_window_size)
        yaw_avg = np.zeros(yaw_window_size)

        index = 0
        gear_dampening_index = 0
        initialized = False
        frequency = 120  # Hz
        period = 1 / frequency
        previous_gear = 0
        disengaged_since = None
        disengaged_timeout = 0.5

        while True:
            start_time = time.monotonic()
            sm = asm.read_shared_memory()

            if sm is None:
                elapsed_time = time.monotonic() - start_time
                sleep_time = period - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)
                continue

            if moog.is_engaged():
                disengaged_since = None
            else:
                if disengaged_since is None:
                    disengaged_since = time.monotonic()
                elif time.monotonic() - disengaged_since >= disengaged_timeout:
                    print('MOOG not engaged. Exiting Assetto program')
                    print(f'MOOG state: {moog.state}')
                    print(f'MOOG status stats: {moog.get_status_stats()}')
                    break

            roll = sm.Physics.roll
            pitch = sm.Physics.pitch
            heading = sm.Physics.heading
            vel_x = sm.Physics.velocity.x
            vel_z = sm.Physics.velocity.z

            if abs(vel_x) < yaw_vel_threshold and abs(vel_z) < yaw_vel_threshold:
                vel_angle = heading
            else:
                vel_angle = -np.arctan2(vel_x, vel_z)

            yaw = signed_angle_diff(vel_angle, heading)

            x_accel = sm.Physics.g_force.x
            z_accel = sm.Physics.g_force.z

            # Gear shift dampening
            gear = sm.Physics.gear
            if gear != previous_gear:
                gear_dampening_index = gear_dampening_window_size
            if gear_dampening_index > 0:
                z_accel /= gear_dampening_scale_factor
                gear_dampening_index -= 1
            previous_gear = gear

            # Calculate angle from acceleration
            if g_force_is_in_g:
                x_accel_normalized = x_accel
                z_accel_normalized = z_accel
            else:
                x_accel_normalized = x_accel / 9.81
                z_accel_normalized = z_accel / 9.81

            x_angle = np.arcsin(np.clip(x_accel_normalized, -x_accel_limit, x_accel_limit))
            z_angle = np.arcsin(np.clip(z_accel_normalized, -z_accel_limit, z_accel_limit))

            roll = roll - x_angle
            pitch = -pitch - z_angle

            # Convert to degrees and scale
            roll = roll_scale_factor * np.degrees(roll)
            pitch = pitch_scale_factor * np.degrees(pitch)
            yaw = yaw_scale_factor * np.degrees(yaw)

            # Limit degrees to max/min values from manual
            roll = np.clip(roll, -roll_degree_excursion, roll_degree_excursion)
            pitch = np.clip(pitch, -pitch_degree_excursion, pitch_degree_excursion)
            yaw = np.clip(yaw, -yaw_degree_excursion, yaw_degree_excursion)

            # Map degrees to 0-32767 range for MOOG
            roll = int(np.clip(32767 / 58 * (roll + 29), roll_actuator_min, roll_actuator_max))
            pitch = int(np.clip(32767 / 66 * (pitch + 33), pitch_actuator_min, pitch_actuator_max))
            yaw = int(np.clip(32767 / 58 * (yaw + 29), yaw_actuator_min, yaw_actuator_max))

            # Moving average filter
            if not initialized:
                roll_avg = np.full(roll_window_size, roll)
                pitch_avg = np.full(pitch_window_size, pitch)
                yaw_avg = np.full(yaw_window_size, yaw)
                initialized = True

            roll_avg[index % roll_window_size] = roll
            pitch_avg[index % pitch_window_size] = pitch
            yaw_avg[index % yaw_window_size] = yaw
            index += 1

            final_roll = int(np.mean(roll_avg))
            final_pitch = int(np.mean(pitch_avg))
            final_yaw = int(np.mean(yaw_avg))

            # Send frame
            moog.command_dof(roll=final_roll, pitch=final_pitch, yaw=final_yaw)

            # Calculate elapsed time and sleep for the remaining time
            elapsed_time = time.monotonic() - start_time
            sleep_time = period - elapsed_time
            if sleep_time > 0:
                time.sleep(sleep_time)

    finally:
        if asm is not None:
            asm.close()
        if moog is not None:
            moog.close()


if __name__ == "__main__":
    main()
