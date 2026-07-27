import re
import sys
import time
import serial

PORT = "/dev/ttyACM0"
BAUDRATE = 115200

END_VOLTAGE_V = 23.0
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


def parse_number(response):
    match = re.search(r"[-+]?\d+(?:\.\d+)?", response)

    if not match:
        raise ValueError(
            f"Cannot extract number from response: {response!r}"
        )

    return float(match.group())


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
    try:
        with serial.Serial(PORT, BAUDRATE, timeout=2) as ser:
            ser.reset_input_buffer()

            send_command(ser, "SYST:LOCK ON")

            owner = send_command(ser, "SYST:LOCK:OWNER?", True)
            if "REMOTE" not in owner.upper():
                raise RuntimeError(f"Remote control not active: {owner}")

            # Ensure expected limits are active
            send_command(ser, f"CURR {CURRENT_LIMIT_A:.1f}")
            send_command(ser, f"POW {POWER_LIMIT_W:.1f}")

            send_command(ser, "CURR?", True)
            send_command(ser, "POW?", True)

            output_state = send_command(ser, "OUTP?", True)

            if "ON" not in output_state.upper():
                print("\nOutput is already OFF.")
                send_command(ser, "SYST:LOCK OFF")
                return

            voltage_response = send_command(ser, "VOLT?", True)
            current_voltage = parse_number(voltage_response)

            print(
                f"\nCurrent voltage setpoint: "
                f"{current_voltage:.2f} V"
            )

            if current_voltage > END_VOLTAGE_V:
                print(
                    f"Ramp down to {END_VOLTAGE_V:.1f} V "
                    f"at {RAMP_RATE_V_PER_S:.1f} V/s\n"
                )

                start_voltage = current_voltage
                start_time = time.monotonic()
                voltage = current_voltage

                while voltage > END_VOLTAGE_V:
                    voltage = max(
                        voltage - VOLTAGE_STEP_V,
                        END_VOLTAGE_V
                    )

                    target_time = (
                        start_time
                        + (start_voltage - voltage)
                        / RAMP_RATE_V_PER_S
                    )

                    delay = target_time - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)

                    send_command(ser, f"VOLT {voltage:.1f}")
                    print(f"Setpoint: {voltage:.1f} V")

            elif current_voltage < END_VOLTAGE_V:
                print(
                    f"Voltage is below {END_VOLTAGE_V:.1f} V. "
                    "No upward ramp will be performed."
                )

            else:
                print("Voltage is already 23 V.")

            send_command(ser, "VOLT?", True)

            print("\nSwitching output OFF.")
            send_command(ser, "OUTP OFF")
            time.sleep(0.2)

            send_command(ser, "OUTP?", True)
            send_command(ser, "MEAS:ARR?", True)
            send_command(ser, "SYST:ERR?", True)

            send_command(ser, "SYST:LOCK OFF")

            print("\nSafe shutdown completed.")
            print("Output is OFF.")

    except KeyboardInterrupt:
        print("\nInterrupted. Switching output OFF immediately.")
        emergency_off()
        sys.exit(1)

    except Exception as error:
        print(f"\nERROR: {error}")
        print("Switching output OFF immediately.")
        emergency_off()
        sys.exit(1)


if __name__ == "__main__":
    main()
