import socket
import serial
import struct
import time
import subprocess
from pathlib import Path
from enum import Enum

from gpiozero import DigitalOutputDevice


# ===============================
# Network settings
# ===============================

LOCAL_IP = "192.168.10.1"
MCAST_GRP = "239.0.0.1"
PORT = 50002
BUFFER_SIZE = 4096
EXPECTED_SENDER_IP = "192.168.10.101"   # set to None to accept any sender

MEASURE_DATA_ID = 3001

# Full acquisition header is 18 bytes total.
# We only care about:
#   data_id       UINT16
#   segment_id    UINT16
#   size_bytes    UINT16
#   data_type     UINT8
#   module_source UINT8
#   channel       UINT8
#   measure_unit  UINT8
# then 6 more bytes/fields we ignore here
HEADER_FORMAT = "<HHHBBBBHBBBBBB"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


# ===============================
# Recording settings
# ===============================

START_TEMP_C = 50.0          # Channel starts recording when its own temperature exceeds this value
RESET_TEMP_C = 35.0          # After channel cycle, channel must fall below this value before it can be ready again
RECORD_DURATION_S = 800.0    # Recording duration per channel

SCRIPT_DIR = Path(__file__).resolve().parent
LAST_DRIVE_CYCLE_FILE = SCRIPT_DIR / "last_drive_cycle.txt"
TJ_DATA_RECORD_FILE = SCRIPT_DIR / "Tj_data_record.txt"


# ===============================
# UART settings
# ===============================

UART_PORT = "/dev/ttyAMA2"
UART_BAUDRATE = 115200

UART_HEADER = b"mrb"
UART_FOOTER_BYTE = ord("B")

UART_FRAME_LEN = 15
UART_PAYLOAD_LEN = 11

UART_TX_PERIOD_S = 0.01   # 100 Hz = every 10 ms


# ===============================
# GPIO safety settings
# ===============================
# gpiozero uses BCM GPIO numbering.
#
# GPIO19 = safety contactor
#          ON/HIGH  -> contactor closed, system allowed
#          OFF/LOW  -> contactor open, safety trip
#
# GPIO21 = OK output
# GPIO20 = warning latch, set when temperature was >= 160 C
# GPIO26 = fault latch, set when temperature > 172 C or value 65535 appears

GPIO_CONTACTOR = 22
GPIO_CONTACTOR_2 = 23
GPIO_FAULT = 26
GPIO_WARNING = 20
GPIO_OK = 21

TEMP_OK_LIMIT_C = 165.0
TEMP_WARNING_C = 170.0
TEMP_FAULT_C = 177.0
UNDER_TEMP_FAULT_C = 140.0
NO_DATA_VALUE = 65535.0

# Start in safe-open state until both channels have valid data.
gpio22_contactor = DigitalOutputDevice(GPIO_CONTACTOR, initial_value=False)
gpio23_contactor = DigitalOutputDevice(GPIO_CONTACTOR_2, initial_value=False)
gpio26_fault = DigitalOutputDevice(GPIO_FAULT, active_high=False, initial_value=False)
gpio20_warning = DigitalOutputDevice(GPIO_WARNING, active_high=False, initial_value=False)
gpio21_ok = DigitalOutputDevice(GPIO_OK, active_high=False, initial_value=False)


# ===============================
# Runtime variables
# ===============================

last_sample = {
    1: None,
    2: None,
}

last_packet_values = {
    1: None,
    2: None,
}


class ChannelState(Enum):
    IDLE = 0                 # This channel waits for its own temperature > START_TEMP_C
    RECORDING = 1            # This channel records its own 800 s cycle
    WAIT_FOR_COOLDOWN = 2    # This channel waits until its own temperature < RESET_TEMP_C
    WAIT_FOR_PAIR_SAVE = 3   # This channel finished and waits for the other channel to finish too


channel_state = {
    1: ChannelState.IDLE,
    2: ChannelState.IDLE,
}

channel_cycle_index = {
    1: 0,
    2: 0,
}

channel_start_time = {
    1: None,
    2: None,
}

channel_start_wall_time = {
    1: None,
    2: None,
}

# Samples currently being recorded for each channel.
# Each entry: (wall_time_s, elapsed_s, channel, temperature_C)
channel_current_samples = {
    1: [],
    2: [],
}

# Completed but not yet paired cycles.
# channel_completed_cycles[1] is filled after CH1 finishes its new cycle.
# channel_completed_cycles[2] is filled after CH2 finishes its new cycle.
# When both are filled, files are saved and this dict is cleared.
channel_completed_cycles = {
    1: None,
    2: None,
}

paired_cycle_index = 0

uart_ser = None
uart_tx_counter = 0
uart_tx_frames = 0
uart_last_tx_time = 0.0

# Current-cycle UART extrema.
# These reset when a new channel cycle starts, and are cleared after paired save.
uart_cycle_min = {
    1: None,
    2: None,
}

uart_cycle_max = {
    1: None,
    2: None,
}

warning_latched = False
fault_latched = False



def is_no_data_value(value):
    """
    Sensor no-data value is treated exactly like very high temperature.
    """
    return value >= NO_DATA_VALUE


def apply_gpio_outputs():
    """
    Apply latched GPIO safety outputs.

    LED logic:
        - warning latch -> warning LED ON
        - fault latch   -> red LED ON, green LED OFF

    GPIO22/GPIO23 logic:
        - normal/no fault: LOW/GND
        - fault shutdown sequence: GPIO22 HIGH after 1 s, GPIO23 HIGH after 2 s
          This delayed sequence is handled inside run_power_supply_safe_off_once().
    """
    if warning_latched:
        gpio20_warning.on()
    else:
        gpio20_warning.off()

    if fault_latched:
        gpio26_fault.on()
        gpio21_ok.off()
        return

    gpio26_fault.off()

    # Normal state for external fault outputs is LOW/GND.
    gpio22_contactor.off()
    gpio23_contactor.off()

    ch1 = last_sample.get(1)
    ch2 = last_sample.get(2)

    both_known = (
        ch1 is not None and
        ch2 is not None and
        not is_no_data_value(ch1) and
        not is_no_data_value(ch2)
    )

    both_below_ok = (
        both_known and
        ch1 < TEMP_OK_LIMIT_C and
        ch2 < TEMP_OK_LIMIT_C
    )

    if both_below_ok:
        gpio21_ok.on()
    else:
        gpio21_ok.off()




def run_power_supply_safe_off_once(reason):
    """
    Run FAULT shutdown sequence only once:

        1) Start power_supply_safe_off.py
        2) Wait 1 second
        3) Set GPIO22 HIGH / pull-up
        4) Wait 1 second
        5) Set GPIO23 HIGH / pull-up

    Normal/no-fault state for GPIO22 and GPIO23 is LOW/GND.
    LEDs keep their own previous logic.
    """
    global power_supply_safe_off_done

    # Safety latch variable. Create it if old patch did not define it globally.
    if "power_supply_safe_off_done" not in globals():
        power_supply_safe_off_done = False

    if power_supply_safe_off_done:
        return

    power_supply_safe_off_done = True

    # Safe-off script path. Create fallback if old patch did not define it globally.
    if "POWER_SUPPLY_SAFE_OFF_SCRIPT" not in globals():
        POWER_SUPPLY_SAFE_OFF_SCRIPT = "/home/user10/power_supply_safe_off.py"

    print("")
    print("FAULT detected -> running delayed shutdown sequence")
    print(f"Reason: {reason}")
    print(f"Step 1: start /usr/bin/python3 {POWER_SUPPLY_SAFE_OFF_SCRIPT}")
    print("Step 2: after 1 s -> GPIO22 HIGH")
    print("Step 3: after next 1 s -> GPIO23 HIGH")
    print("")

    try:
        subprocess.Popen(
            ["/usr/bin/python3", POWER_SUPPLY_SAFE_OFF_SCRIPT],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        print("power_supply_safe_off.py started")
    except Exception as e:
        print(f"ERROR while starting power_supply_safe_off.py: {e}")

    time.sleep(1.0)

    try:
        gpio22_contactor.on()
        print("GPIO22 HIGH / pull-up")
    except Exception as e:
        print(f"ERROR while switching GPIO22 HIGH: {e}")

    time.sleep(1.0)

    try:
        gpio23_contactor.on()
        print("GPIO23 HIGH / pull-up")
    except Exception as e:
        print(f"ERROR while switching GPIO23 HIGH: {e}")

    print("FAULT shutdown sequence finished")
    print("")



def update_gpio_safety(channel, value):
    """
    Update latched safety logic from every received temperature sample.

    Rules:
        - If any channel ever reaches >= 160 C:
              GPIO20 warning latches ON until program/RPi restart.

        - If any channel ever exceeds 172 C:
              GPIO26 fault latches ON until program/RPi restart.
              GPIO19 contactor opens.

        - If value 65535 appears:
              same as very high temperature fault.
    """
    global warning_latched
    global fault_latched

    if value is None:
        apply_gpio_outputs()
        return

    if is_no_data_value(value):
        warning_latched = True
        fault_latched = True
        run_power_supply_safe_off_once(f"CH{channel} value={value}")
        apply_gpio_outputs()
        return

    if value > TEMP_WARNING_C:
        warning_latched = True

    if value > TEMP_FAULT_C:
        fault_latched = True
        run_power_supply_safe_off_once(f"CH{channel} value={value}")

    apply_gpio_outputs()


def init_uart():
    """
    Open UART used to send safety/status frame to external device.
    If UART cannot be opened, logger continues working without UART.
    """
    global uart_ser
    global uart_last_tx_time

    try:
        uart_ser = serial.Serial(
            port=UART_PORT,
            baudrate=UART_BAUDRATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.001,
            write_timeout=0.001,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False
        )

        uart_last_tx_time = time.perf_counter()

        print("")
        print(f"UART TX enabled on {UART_PORT}, {UART_BAUDRATE} baud, 8N1")
        print("UART frame: m r b + 11 payload bytes + B")
        print("UART payload[8]  = current-cycle Tj_max")
        print("UART payload[9]  = current-cycle Tj_min")
        print("")

    except Exception as e:
        uart_ser = None
        print("")
        print(f"WARNING: UART disabled. Could not open {UART_PORT}: {e}")
        print("")


def clamp_u8(value):
    """
    Convert value to unsigned 8-bit integer for UART payload.
    """
    if value is None:
        return 0

    try:
        value = int(round(float(value)))
    except Exception:
        return 0

    if value < 0:
        return 0

    if value > 255:
        return 255

    return value


def update_uart_cycle_extrema(channel, value):
    """
    Update min/max temperature for current cycle of selected channel.
    These values are sent over UART before the 800 s cycle is finished.
    """
    if value is None:
        return

    if uart_cycle_min[channel] is None or value < uart_cycle_min[channel]:
        uart_cycle_min[channel] = value

    if uart_cycle_max[channel] is None or value > uart_cycle_max[channel]:
        uart_cycle_max[channel] = value


def reset_uart_channel_extrema(channel, first_value=None):
    """
    Reset UART extrema when a new cycle starts for selected channel.
    """
    uart_cycle_min[channel] = first_value
    uart_cycle_max[channel] = first_value


def clear_uart_all_extrema():
    """
    Clear UART extrema after paired cycle has been saved.
    """
    uart_cycle_min[1] = None
    uart_cycle_min[2] = None
    uart_cycle_max[1] = None
    uart_cycle_max[2] = None


def get_uart_pair_tj_min_max():
    """
    Return current pair-level Tj_min and Tj_max.
    It uses available current-cycle extrema from CH1 and CH2.
    """
    mins = [v for v in (uart_cycle_min[1], uart_cycle_min[2]) if v is not None]
    maxs = [v for v in (uart_cycle_max[1], uart_cycle_max[2]) if v is not None]

    tj_min = min(mins) if mins else 0
    tj_max = max(maxs) if maxs else 0

    return tj_min, tj_max


def make_uart_tx_frame():
    """
    Create 15-byte UART frame:
        [0]  'm'
        [1]  'r'
        [2]  'b'
        [3]  counter
        [4]  10
        [5]  20
        [6]  30
        [7]  40
        [8]  50
        [9]  60
        [10] Tj_max_CH1 live
        [11] Tj_max_CH2 live
        [12] Tj_min_CH1 live
        [13] Tj_min_CH2 live
        [14] 'B'
    """

    payload = bytes([
        uart_tx_counter & 0xFF,
        10,
        20,
        30,
        40,
        50,
        60,
        clamp_u8(uart_cycle_max[1]),
        clamp_u8(uart_cycle_max[2]),
        clamp_u8(uart_cycle_min[1]),
        clamp_u8(uart_cycle_min[2])
    ])

    frame = UART_HEADER + payload + bytes([UART_FOOTER_BYTE])

    if len(frame) != UART_FRAME_LEN:
        raise RuntimeError(f"Wrong UART TX frame length: {len(frame)}")

    return frame


def uart_send_periodic(now):
    """
    Send UART frame every UART_TX_PERIOD_S.
    """
    global uart_tx_counter
    global uart_tx_frames
    global uart_last_tx_time

    if uart_ser is None:
        return

    if now - uart_last_tx_time < UART_TX_PERIOD_S:
        return

    try:
        frame = make_uart_tx_frame()
        uart_ser.write(frame)
        uart_ser.flush()

        uart_tx_frames += 1
        uart_tx_counter = (uart_tx_counter + 1) & 0xFF

        uart_last_tx_time += UART_TX_PERIOD_S

        if now - uart_last_tx_time > UART_TX_PERIOD_S:
            uart_last_tx_time = now

    except Exception as e:
        print(f"UART TX error: {e}")


def parse_packet(data: bytes):
    """
    Parse one UDP packet.

    Returns:
        None if packet is not relevant or invalid.
        (channel, values) if packet contains valid float values for CH1 or CH2.
    """
    if len(data) < HEADER_SIZE:
        return None

    header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])

    data_id = header[0]
    size_bytes = header[2]
    channel = header[5]

    if data_id != MEASURE_DATA_ID:
        return None

    if channel not in (1, 2):
        return None

    payload = data[HEADER_SIZE:HEADER_SIZE + size_bytes]

    if len(payload) < size_bytes:
        return None

    if size_bytes % 4 != 0:
        return None

    float_count = size_bytes // 4
    values = struct.unpack("<" + "f" * float_count, payload)

    if not values:
        return None

    last_packet_values[channel] = values
    last_sample[channel] = values[-1]

    return channel, values


def ensure_summary_file_header():
    """
    Create Tj_data_record.txt with header if it does not exist yet.
    One row is written only after both CH1 and CH2 have completed a new cycle.
    """
    if TJ_DATA_RECORD_FILE.exists():
        return

    with TJ_DATA_RECORD_FILE.open("w", encoding="utf-8") as f:
        f.write(
            "paired_cycle_index\t"
            "ch1_cycle_index\t"
            "ch1_start_wall_time_s\t"
            "ch1_end_wall_time_s\t"
            "ch1_duration_s\t"
            "ch1_samples\t"
            "ch1_avg_C\t"
            "ch1_min_C\t"
            "ch1_max_C\t"
            "ch2_cycle_index\t"
            "ch2_start_wall_time_s\t"
            "ch2_end_wall_time_s\t"
            "ch2_duration_s\t"
            "ch2_samples\t"
            "ch2_avg_C\t"
            "ch2_min_C\t"
            "ch2_max_C\t"
            "all_samples\t"
            "all_avg_C\t"
            "all_min_C\t"
            "all_max_C\n"
        )


def stats(values):
    """
    Return (count, avg, min, max). If empty, values are returned as None.
    """
    if not values:
        return 0, None, None, None

    return len(values), sum(values) / len(values), min(values), max(values)


def fmt(value):
    """
    Format float for text file. Use 'nan' if value is missing.
    """
    if value is None:
        return "nan"

    return f"{value:.6f}"


def start_channel_cycle(channel, now, trigger_value):
    """
    Start a new independent cycle for selected channel.
    """
    channel_cycle_index[channel] += 1
    channel_start_time[channel] = now
    channel_start_wall_time[channel] = time.time()
    channel_current_samples[channel] = []
    channel_state[channel] = ChannelState.RECORDING
    reset_uart_channel_extrema(channel, trigger_value)

    print("")
    print(f"CH{channel} cycle started.")
    print(f"CH{channel} cycle index: {channel_cycle_index[channel]}")
    print(f"Trigger: CH{channel} = {trigger_value:.6f} C")
    print(f"Recording duration: {RECORD_DURATION_S:.1f} s")
    print("")


def finish_channel_cycle(channel, now):
    """
    Finish independent cycle for selected channel and store it in memory.
    Files are not updated here. Files are updated only when both channels
    have completed a new cycle.
    """
    start_time = channel_start_time[channel]
    start_wall_time = channel_start_wall_time[channel]
    end_wall_time = time.time()

    duration = now - start_time if start_time is not None else 0.0

    samples = channel_current_samples[channel]

    values = [sample[3] for sample in samples]
    count, avg_value, min_value, max_value = stats(values)

    channel_completed_cycles[channel] = {
        "channel": channel,
        "cycle_index": channel_cycle_index[channel],
        "start_wall_time_s": start_wall_time,
        "end_wall_time_s": end_wall_time,
        "duration_s": duration,
        "samples": samples,
        "count": count,
        "avg": avg_value,
        "min": min_value,
        "max": max_value,
    }

    channel_current_samples[channel] = []
    channel_start_time[channel] = None
    channel_start_wall_time[channel] = None
    channel_state[channel] = ChannelState.WAIT_FOR_COOLDOWN

    print("")
    print(f"CH{channel} cycle finished.")
    print(
        f"CH{channel}: n={count}, avg={fmt(avg_value)}, "
        f"min={fmt(min_value)}, max={fmt(max_value)}"
    )
    print(f"CH{channel} waits for cooldown below {RESET_TEMP_C:.1f} C.")
    print("")


def write_last_drive_cycle_pair(ch1_cycle, ch2_cycle):
    """
    Overwrite last_drive_cycle.txt only when both channels have a new completed cycle.
    The elapsed_s axis is synchronized from each channel's own recording start:
        CH1 elapsed_s starts at 0
        CH2 elapsed_s starts at 0
    even if CH1 and CH2 were recorded at different real times.
    """
    with LAST_DRIVE_CYCLE_FILE.open("w", encoding="utf-8") as f:
        f.write(
            "paired_cycle_index\t"
            "channel\t"
            "channel_cycle_index\t"
            "elapsed_s\t"
            "temperature_C\t"
            "wall_time_s\n"
        )

        for cycle in (ch1_cycle, ch2_cycle):
            channel = cycle["channel"]
            cycle_index = cycle["cycle_index"]

            for wall_time_s, elapsed_s, sample_channel, value in cycle["samples"]:
                f.write(
                    f"{paired_cycle_index}\t"
                    f"{channel}\t"
                    f"{cycle_index}\t"
                    f"{elapsed_s:.6f}\t"
                    f"{value:.6f}\t"
                    f"{wall_time_s:.6f}\n"
                )


def append_summary_pair(ch1_cycle, ch2_cycle):
    """
    Append one row to Tj_data_record.txt only when both channels have a new completed cycle.
    """
    ensure_summary_file_header()

    ch1_values = [sample[3] for sample in ch1_cycle["samples"]]
    ch2_values = [sample[3] for sample in ch2_cycle["samples"]]
    all_values = ch1_values + ch2_values

    all_count, all_avg, all_min, all_max = stats(all_values)

    with TJ_DATA_RECORD_FILE.open("a", encoding="utf-8") as f:
        f.write(
            f"{paired_cycle_index}\t"
            f"{ch1_cycle['cycle_index']}\t"
            f"{ch1_cycle['start_wall_time_s']:.6f}\t"
            f"{ch1_cycle['end_wall_time_s']:.6f}\t"
            f"{ch1_cycle['duration_s']:.6f}\t"
            f"{ch1_cycle['count']}\t"
            f"{fmt(ch1_cycle['avg'])}\t"
            f"{fmt(ch1_cycle['min'])}\t"
            f"{fmt(ch1_cycle['max'])}\t"
            f"{ch2_cycle['cycle_index']}\t"
            f"{ch2_cycle['start_wall_time_s']:.6f}\t"
            f"{ch2_cycle['end_wall_time_s']:.6f}\t"
            f"{ch2_cycle['duration_s']:.6f}\t"
            f"{ch2_cycle['count']}\t"
            f"{fmt(ch2_cycle['avg'])}\t"
            f"{fmt(ch2_cycle['min'])}\t"
            f"{fmt(ch2_cycle['max'])}\t"
            f"{all_count}\t"
            f"{fmt(all_avg)}\t"
            f"{fmt(all_min)}\t"
            f"{fmt(all_max)}\n"
        )


def update_channel_after_pair_saved(channel):
    """
    After both channels have been saved as one paired result,
    each channel may return to IDLE only if it is already below RESET_TEMP_C.
    Otherwise it must wait for cooldown.
    """
    value = last_sample[channel]

    if value is not None and value < RESET_TEMP_C:
        channel_state[channel] = ChannelState.IDLE
    else:
        channel_state[channel] = ChannelState.WAIT_FOR_COOLDOWN


def check_under_temperature_fault_after_pair():
    """
    End-of-cycle under-temperature FAULT.

    If CH1 and CH2 both completed a cycle, but neither channel reached
    UNDER_TEMP_FAULT_C, then trigger the same FAULT shutdown sequence.
    """
    global warning_latched
    global fault_latched

    ch1_cycle = channel_completed_cycles.get(1)
    ch2_cycle = channel_completed_cycles.get(2)

    if ch1_cycle is None or ch2_cycle is None:
        return

    ch1_max = ch1_cycle.get("max")
    ch2_max = ch2_cycle.get("max")

    if ch1_max is None or ch2_max is None:
        warning_latched = True
        fault_latched = True
        apply_gpio_outputs()
        run_power_supply_safe_off_once("under-temperature check: missing max value")
        return

    if is_no_data_value(ch1_max) or is_no_data_value(ch2_max):
        warning_latched = True
        fault_latched = True
        apply_gpio_outputs()
        run_power_supply_safe_off_once(
            f"under-temperature check: invalid max value, CH1_max={ch1_max}, CH2_max={ch2_max}"
        )
        return

    print(
        f"Under-temp check after pair: "
        f"CH1_max={ch1_max:.3f}, CH2_max={ch2_max:.3f}, "
        f"limit={UNDER_TEMP_FAULT_C:.3f}"
    )

    if (ch1_max < UNDER_TEMP_FAULT_C) and (ch2_max < UNDER_TEMP_FAULT_C):
        warning_latched = True
        fault_latched = True
        apply_gpio_outputs()
        run_power_supply_safe_off_once(
            f"under-temperature after full cycle: CH1_max={ch1_max:.3f}, CH2_max={ch2_max:.3f}, limit={UNDER_TEMP_FAULT_C:.3f}"
        )




def save_pair_if_ready():
    # End-of-cycle under-temperature check.
    # Runs only when both completed channel cycles are available.
    if (channel_completed_cycles.get(1) is not None) and (channel_completed_cycles.get(2) is not None):
        check_under_temperature_fault_after_pair()

    """
    Save files only when both CH1 and CH2 have completed new cycles.
    Then clear completed-cycle buffers.
    """
    global paired_cycle_index

    ch1_cycle = channel_completed_cycles[1]
    ch2_cycle = channel_completed_cycles[2]

    if ch1_cycle is None or ch2_cycle is None:
        return

    paired_cycle_index += 1

    write_last_drive_cycle_pair(ch1_cycle, ch2_cycle)
    append_summary_pair(ch1_cycle, ch2_cycle)

    print("")
    print("Both CH1 and CH2 have completed new cycles.")
    print(f"Paired cycle index: {paired_cycle_index}")
    print(f"Updated raw pair file: {LAST_DRIVE_CYCLE_FILE}")
    print(f"Updated summary file: {TJ_DATA_RECORD_FILE}")
    print(
        f"CH1: n={ch1_cycle['count']}, avg={fmt(ch1_cycle['avg'])}, "
        f"min={fmt(ch1_cycle['min'])}, max={fmt(ch1_cycle['max'])}"
    )
    print(
        f"CH2: n={ch2_cycle['count']}, avg={fmt(ch2_cycle['avg'])}, "
        f"min={fmt(ch2_cycle['min'])}, max={fmt(ch2_cycle['max'])}"
    )
    print("")

    channel_completed_cycles[1] = None
    channel_completed_cycles[2] = None

    update_channel_after_pair_saved(1)
    update_channel_after_pair_saved(2)


def process_sample(channel, value, now):
    """
    Independent channel state machine.

    Important UART behavior:
        - When a channel cycle starts, UART min/max for this channel are reset.
        - During RECORDING, every received sample updates UART min/max.
        - UART therefore sends live Tj_max/Tj_min during the current cycle.
    """
    update_gpio_safety(channel, value)

    # Do not use 65535/no-data samples for cycle detection or recording.
    # They only trigger safety fault.
    if is_no_data_value(value):
        return

    state = channel_state[channel]

    if state == ChannelState.IDLE:
        if value > START_TEMP_C:
            start_channel_cycle(channel, now, value)
            state = channel_state[channel]
        else:
            return

    if state == ChannelState.RECORDING:
        elapsed = now - channel_start_time[channel]
        wall_time_s = time.time()

        channel_current_samples[channel].append(
            (wall_time_s, elapsed, channel, value)
        )

        # This is the important live UART update.
        # It must happen for every sample during the active cycle.
        update_uart_cycle_extrema(channel, value)

        if elapsed >= RECORD_DURATION_S:
            finish_channel_cycle(channel, now)
            save_pair_if_ready()

    elif state == ChannelState.WAIT_FOR_COOLDOWN:
        if value < RESET_TEMP_C:
            if channel_completed_cycles[channel] is None:
                channel_state[channel] = ChannelState.IDLE
                print("")
                print(f"CH{channel} cooldown completed. CH{channel} is ready for next cycle.")
                print("")
            else:
                channel_state[channel] = ChannelState.WAIT_FOR_PAIR_SAVE
                print("")
                print(
                    f"CH{channel} cooldown completed, but CH{channel} waits for the other "
                    f"channel cycle to finish before a new cycle can start."
                )
                print("")

    elif state == ChannelState.WAIT_FOR_PAIR_SAVE:
        # This channel already has a completed cycle waiting in memory.
        # It cannot start another cycle until both channels are saved together.
        pass


def channel_state_text(channel):
    state = channel_state[channel]

    if state == ChannelState.IDLE:
        return f"CH{channel}=IDLE"

    if state == ChannelState.RECORDING:
        elapsed = time.monotonic() - channel_start_time[channel]
        remaining = max(0.0, RECORD_DURATION_S - elapsed)
        return f"CH{channel}=RECORDING {elapsed:.1f}s/{RECORD_DURATION_S:.1f}s, remaining={remaining:.1f}s"

    if state == ChannelState.WAIT_FOR_COOLDOWN:
        return f"CH{channel}=WAIT_FOR_COOLDOWN < {RESET_TEMP_C:.1f}C"

    if state == ChannelState.WAIT_FOR_PAIR_SAVE:
        return f"CH{channel}=WAIT_FOR_PAIR_SAVE"

    return f"CH{channel}=UNKNOWN"


def main():
    ensure_summary_file_header()
    apply_gpio_outputs()
    init_uart()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))

    mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton(LOCAL_IP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    sock.settimeout(0.05)

    print(f"Listening on multicast {MCAST_GRP}:{PORT} via {LOCAL_IP}")
    print(f"Expected sender IP: {EXPECTED_SENDER_IP}")
    print("Independent CH1 and CH2 recording enabled.")
    print("Files are updated only after both channels complete new cycles.")
    print(f"Start threshold: {START_TEMP_C:.1f} C")
    print(f"Reset threshold: {RESET_TEMP_C:.1f} C")
    print(f"Record duration per channel: {RECORD_DURATION_S:.1f} s")
    print(f"Raw paired cycle file: {LAST_DRIVE_CYCLE_FILE}")
    print(f"Summary file: {TJ_DATA_RECORD_FILE}")
    print("")

    last_print = time.monotonic()

    while True:
        now = time.monotonic()

        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            src_ip, _ = addr

            if EXPECTED_SENDER_IP and src_ip != EXPECTED_SENDER_IP:
                continue

            parsed = parse_packet(data)

            if parsed is not None:
                channel, values = parsed
                now = time.monotonic()

                for value in values:
                    process_sample(channel, value, now)

        except socket.timeout:
            pass

        now = time.monotonic()

        uart_send_periodic(time.perf_counter())

        if now - last_print >= 0.2:
            ch1 = last_sample[1]
            ch2 = last_sample[2]

            ch1_text = f"{ch1:.6f}" if ch1 is not None else "no data"
            ch2_text = f"{ch2:.6f}" if ch2 is not None else "no data"

            print(
                f"CH1: {ch1_text} | CH2: {ch2_text} | "
                f"{channel_state_text(1)} | {channel_state_text(2)}"
            )
            last_print = now


if __name__ == "__main__":
    main()
