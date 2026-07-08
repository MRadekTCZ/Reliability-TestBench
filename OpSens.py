import socket
import struct
import time
from pathlib import Path
from enum import Enum


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

START_TEMP_C = 40.0          # Start recording when CH1 or CH2 exceeds this value
RESET_TEMP_C = 35.0          # After cycle, wait until both channels are below this value
RECORD_DURATION_S = 800.0    # Drive cycle recording duration

SCRIPT_DIR = Path(__file__).resolve().parent
LAST_DRIVE_CYCLE_FILE = SCRIPT_DIR / "last_drive_cycle.txt"
TJ_DATA_RECORD_FILE = SCRIPT_DIR / "Tj_data_record.txt"


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


class RecorderState(Enum):
    IDLE = 0                 # Ready to start when any channel exceeds START_TEMP_C
    RECORDING = 1            # Currently recording 800 s cycle
    WAIT_FOR_COOLDOWN = 2    # Waiting until both channels fall below RESET_TEMP_C


state = RecorderState.IDLE

cycle_index = 0
cycle_start_time = None
cycle_start_wall_time = None

# Samples collected during current cycle:
# Each entry: (wall_time_s, elapsed_s, channel, value)
cycle_samples = []


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


def both_channels_below_reset():
    """
    Return True only if both channels have valid samples and both are below RESET_TEMP_C.
    """
    ch1 = last_sample[1]
    ch2 = last_sample[2]

    if ch1 is None or ch2 is None:
        return False

    return ch1 < RESET_TEMP_C and ch2 < RESET_TEMP_C


def open_new_last_drive_cycle_file():
    """
    Clear previous last_drive_cycle.txt and write header for the new cycle.
    """
    with LAST_DRIVE_CYCLE_FILE.open("w", encoding="utf-8") as f:
        f.write("wall_time_s\telapsed_s\tchannel\ttemperature_C\n")


def append_sample_to_last_drive_cycle(wall_time_s, elapsed_s, channel, value):
    """
    Append one raw sample to last_drive_cycle.txt.
    """
    with LAST_DRIVE_CYCLE_FILE.open("a", encoding="utf-8") as f:
        f.write(f"{wall_time_s:.6f}\t{elapsed_s:.6f}\t{channel}\t{value:.6f}\n")


def ensure_summary_file_header():
    """
    Create Tj_data_record.txt with header if it does not exist yet.
    """
    if TJ_DATA_RECORD_FILE.exists():
        return

    with TJ_DATA_RECORD_FILE.open("w", encoding="utf-8") as f:
        f.write(
            "cycle_index\t"
            "start_wall_time_s\t"
            "end_wall_time_s\t"
            "duration_s\t"
            "ch1_samples\t"
            "ch1_avg_C\t"
            "ch1_min_C\t"
            "ch1_max_C\t"
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


def finish_cycle(now):
    """
    Calculate statistics for the completed cycle and append one summary line
    to Tj_data_record.txt.
    """
    global state, cycle_start_time, cycle_start_wall_time, cycle_samples

    ensure_summary_file_header()

    ch1_values = [sample[3] for sample in cycle_samples if sample[2] == 1]
    ch2_values = [sample[3] for sample in cycle_samples if sample[2] == 2]
    all_values = [sample[3] for sample in cycle_samples]

    ch1_count, ch1_avg, ch1_min, ch1_max = stats(ch1_values)
    ch2_count, ch2_avg, ch2_min, ch2_max = stats(ch2_values)
    all_count, all_avg, all_min, all_max = stats(all_values)

    duration = now - cycle_start_time if cycle_start_time is not None else 0.0
    end_wall_time = time.time()

    with TJ_DATA_RECORD_FILE.open("a", encoding="utf-8") as f:
        f.write(
            f"{cycle_index}\t"
            f"{cycle_start_wall_time:.6f}\t"
            f"{end_wall_time:.6f}\t"
            f"{duration:.6f}\t"
            f"{ch1_count}\t"
            f"{fmt(ch1_avg)}\t"
            f"{fmt(ch1_min)}\t"
            f"{fmt(ch1_max)}\t"
            f"{ch2_count}\t"
            f"{fmt(ch2_avg)}\t"
            f"{fmt(ch2_min)}\t"
            f"{fmt(ch2_max)}\t"
            f"{all_count}\t"
            f"{fmt(all_avg)}\t"
            f"{fmt(all_min)}\t"
            f"{fmt(all_max)}\n"
        )

    print("")
    print("Drive cycle finished.")
    print(f"Cycle index: {cycle_index}")
    print(f"Saved raw data: {LAST_DRIVE_CYCLE_FILE}")
    print(f"Updated summary: {TJ_DATA_RECORD_FILE}")
    print(
        f"CH1: n={ch1_count}, avg={fmt(ch1_avg)}, min={fmt(ch1_min)}, max={fmt(ch1_max)} | "
        f"CH2: n={ch2_count}, avg={fmt(ch2_avg)}, min={fmt(ch2_min)}, max={fmt(ch2_max)}"
    )
    print(f"Waiting until both channels fall below {RESET_TEMP_C:.1f} C before next cycle.")
    print("")

    cycle_samples = []
    cycle_start_time = None
    cycle_start_wall_time = None
    state = RecorderState.WAIT_FOR_COOLDOWN


def start_cycle(now, trigger_channel, trigger_value):
    """
    Start a new recording cycle. Previous last_drive_cycle.txt is overwritten.
    """
    global state, cycle_index, cycle_start_time, cycle_start_wall_time, cycle_samples

    cycle_index += 1
    cycle_start_time = now
    cycle_start_wall_time = time.time()
    cycle_samples = []

    open_new_last_drive_cycle_file()

    state = RecorderState.RECORDING

    print("")
    print("Drive cycle started.")
    print(f"Cycle index: {cycle_index}")
    print(f"Trigger: CH{trigger_channel} = {trigger_value:.6f} C")
    print(f"Recording duration: {RECORD_DURATION_S:.1f} s")
    print(f"Raw data file: {LAST_DRIVE_CYCLE_FILE}")
    print("")


def process_sample(channel, value, now):
    """
    Main recording state machine.

    IDLE:
        Wait for CH1 or CH2 > START_TEMP_C.
    RECORDING:
        Save all samples from both channels for RECORD_DURATION_S.
    WAIT_FOR_COOLDOWN:
        Do not start another cycle until CH1 and CH2 are both below RESET_TEMP_C.
    """
    global state

    if state == RecorderState.IDLE:
        if value > START_TEMP_C:
            start_cycle(now, channel, value)

    if state == RecorderState.RECORDING:
        elapsed = now - cycle_start_time

        wall_time_s = time.time()
        cycle_samples.append((wall_time_s, elapsed, channel, value))
        append_sample_to_last_drive_cycle(wall_time_s, elapsed, channel, value)

        if elapsed >= RECORD_DURATION_S:
            finish_cycle(now)

    elif state == RecorderState.WAIT_FOR_COOLDOWN:
        if both_channels_below_reset():
            print("")
            print(
                f"Cooldown completed. Both channels are below {RESET_TEMP_C:.1f} C. "
                f"Ready for next cycle."
            )
            print("")
            state = RecorderState.IDLE


def state_text():
    if state == RecorderState.IDLE:
        return f"IDLE, waiting for CH1 or CH2 > {START_TEMP_C:.1f} C"

    if state == RecorderState.RECORDING:
        elapsed = time.monotonic() - cycle_start_time
        remaining = max(0.0, RECORD_DURATION_S - elapsed)
        return f"RECORDING, elapsed={elapsed:.1f} s, remaining={remaining:.1f} s"

    if state == RecorderState.WAIT_FOR_COOLDOWN:
        return f"WAIT_FOR_COOLDOWN, waiting for both channels < {RESET_TEMP_C:.1f} C"

    return "UNKNOWN"


def main():
    ensure_summary_file_header()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))

    mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton(LOCAL_IP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    sock.settimeout(0.05)

    print(f"Listening on multicast {MCAST_GRP}:{PORT} via {LOCAL_IP}")
    print(f"Expected sender IP: {EXPECTED_SENDER_IP}")
    print("Printing latest sample from CH1 and CH2 every 200 ms")
    print(f"Start threshold: {START_TEMP_C:.1f} C")
    print(f"Reset threshold: {RESET_TEMP_C:.1f} C")
    print(f"Record duration: {RECORD_DURATION_S:.1f} s")
    print(f"Raw cycle file: {LAST_DRIVE_CYCLE_FILE}")
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

        if now - last_print >= 0.2:
            ch1 = last_sample[1]
            ch2 = last_sample[2]

            ch1_text = f"{ch1:.6f}" if ch1 is not None else "no data"
            ch2_text = f"{ch2:.6f}" if ch2 is not None else "no data"

            print(f"CH1: {ch1_text} | CH2: {ch2_text} | {state_text()}")
            last_print = now


if __name__ == "__main__":
    main()