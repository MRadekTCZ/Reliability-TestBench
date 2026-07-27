import sys
import time
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

START_VOLTAGE_V = 23.0
END_VOLTAGE_V = 123.0
RAMP_RATE_V_PER_S = 20.0

CURRENT_LIMIT_A = 11.0
POWER_LIMIT_W = 1500.0

VOLTAGE_STEP_V = 1.0


def send_command(ser, command, read_response=False):
    print(f">> {command}")

    ser.write((command + "\n").encode("ascii"))
    ser.flush()
    time.sleep(0.02)

    if read_response:
        response = ser.readline().decode(errors="replace").strip()
        print(f"<< {response}")
        return response

    return None


def emergency_off():
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
            ser.write(b"OUTP OFF\n")
            ser.flush()
            time.sleep(0.1)

        print("Emergency OUTP OFF sent.")

    except Exception as error:
        print(f"Could not send OUTP OFF: {error}")


def main():
    output_enabled = False

    try:
        with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
            ser.reset_input_buffer()

            send_command(ser, "*IDN?", True)
            send_command(ser, "SYST:LOCK ON")

            owner = send_command(ser, "SYST:LOCK:OWNER?", True)
            if "REMOTE" not in owner.upper():
                raise RuntimeError(f"Remote control not active: {owner}")

            # Safe configuration with output disabled
            send_command(ser, "OUTP OFF")
            send_command(ser, f"VOLT {START_VOLTAGE_V:.1f}")
            send_command(ser, f"CURR {CURRENT_LIMIT_A:.1f}")
            send_command(ser, f"POW {POWER_LIMIT_W:.1f}")

            # Verify settings
            send_command(ser, "VOLT?", True)
            send_command(ser, "CURR?", True)
            send_command(ser, "POW?", True)

            error = send_command(ser, "SYST:ERR?", True)
            if error and not error.startswith("0,"):
                raise RuntimeError(f"Power supply error: {error}")

            print(
                f"\nRamp: {START_VOLTAGE_V:.1f} V -> "
                f"{END_VOLTAGE_V:.1f} V at "
                f"{RAMP_RATE_V_PER_S:.1f} V/s"
            )

            send_command(ser, "OUTP ON")
            output_enabled = True

            start_time = time.monotonic()
            voltage = START_VOLTAGE_V

            while voltage < END_VOLTAGE_V:
                voltage = min(
                    voltage + VOLTAGE_STEP_V,
                    END_VOLTAGE_V
                )

                target_time = (
                    start_time
                    + (voltage - START_VOLTAGE_V)
                    / RAMP_RATE_V_PER_S
                )

                delay = target_time - time.monotonic()
                if delay > 0:
                    time.sleep(delay)

                send_command(ser, f"VOLT {voltage:.1f}")
                print(f"Setpoint: {voltage:.1f} V")

            print("\nRamp completed.")

            send_command(ser, "VOLT?", True)
            send_command(ser, "CURR?", True)
            send_command(ser, "POW?", True)
            send_command(ser, "OUTP?", True)
            send_command(ser, "MEAS:ARR?", True)
            send_command(ser, "SYST:ERR?", True)

            print("\nOutput remains ON at 123 V.")
            print("Current limit: 11 A")
            print("Power limit: 1500 W")

    except KeyboardInterrupt:
        print("\nInterrupted. Switching output OFF immediately.")
        if output_enabled:
            emergency_off()
        sys.exit(1)

    except Exception as error:
        print(f"\nERROR: {error}")
        if output_enabled:
            emergency_off()
        sys.exit(1)


if __name__ == "__main__":
    main()
