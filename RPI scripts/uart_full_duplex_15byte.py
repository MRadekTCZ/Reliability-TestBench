import serial
import time

PORT = "/dev/ttyAMA2"
BAUDRATE = 115200

HEADER = b"mrb"
FOOTER_BYTE = ord("B")

FRAME_LEN = 15
PAYLOAD_LEN = 11

TX_PERIOD = 0.01   # 100 Hz = every 10 ms

ser = serial.Serial(
    port=PORT,
    baudrate=BAUDRATE,
    bytesize=8,
    parity="N",
    stopbits=1,
    timeout=0.001,
    write_timeout=0.001,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False
)

print(f"UART full duplex 15-byte test on {PORT}, {BAUDRATE} baud, 8N1")
print("TX frame: m r b + 11 payload bytes + B")
print("RX frame: m r b + 11 payload bytes + B")
print("Frame length = 15 bytes")
print("Press Ctrl+C to stop\n")

tx_counter = 0
tx_frames = 0

rx_state = 0
rx_frames = 0
rx_errors = 0
rx_payload = bytearray()


def make_tx_frame(counter):
    """
    Frame format, 15 bytes:

    [0]  'm'
    [1]  'r'
    [2]  'b'
    [3]  counter
    [4]  dummy value 1
    [5]  dummy value 2
    [6]  dummy value 3
    [7]  dummy value 4
    [8]  dummy value 5
    [9]  dummy value 6
    [10] dummy value 7
    [11] dummy value 8
    [12] dummy value 9
    [13] dummy value 10
    [14] 'B'
    """

    payload = bytes([
        counter & 0xFF,
        10,
        20,
        30,
        40,
        50,
        60,
        70,
        80,
        90,
        100
    ])

    return HEADER + payload + bytes([FOOTER_BYTE])


def send_frame():
    global tx_counter, tx_frames

    frame = make_tx_frame(tx_counter)

    if len(frame) != FRAME_LEN:
        raise RuntimeError(f"Wrong TX frame length: {len(frame)}")

    ser.write(frame)
    ser.flush()

    tx_frames += 1
    tx_counter = (tx_counter + 1) & 0xFF


def process_received_byte(b):
    global rx_state, rx_frames, rx_errors, rx_payload

    # State 0: wait for 'm'
    if rx_state == 0:
        if b == ord("m"):
            rx_state = 1

    # State 1: wait for 'r'
    elif rx_state == 1:
        if b == ord("r"):
            rx_state = 2
        elif b == ord("m"):
            rx_state = 1
        else:
            rx_state = 0
            rx_errors += 1

    # State 2: wait for 'b'
    elif rx_state == 2:
        if b == ord("b"):
            rx_payload = bytearray()
            rx_state = 3
        elif b == ord("m"):
            rx_state = 1
        else:
            rx_state = 0
            rx_errors += 1

    # State 3: read 11 payload bytes
    elif rx_state == 3:
        rx_payload.append(b)

        if len(rx_payload) >= PAYLOAD_LEN:
            rx_state = 4

    # State 4: wait for final 'B'
    elif rx_state == 4:
        if b == FOOTER_BYTE:
            rx_frames += 1

            rx_counter       = rx_payload[0]
            current_meas_im  = rx_payload[1]
            t_ntc            = rx_payload[2]
            tj_ntc_based     = rx_payload[3]
            tj_est           = rx_payload[4]
            power_t_est      = rx_payload[5]
            tj_no_atc        = rx_payload[6]
            fsw_khz          = rx_payload[7]
            tj_max           = rx_payload[8]
            tj_start         = rx_payload[9]
            tj_ref           = rx_payload[10]

            print(
                f"RXcnt={rx_counter:3d} | "
                f"Im={current_meas_im:3d} | "
                f"T_NTC={t_ntc:3d} | "
                f"Tj_NTC={tj_ntc_based:3d} | "
                f"Tj_est={tj_est:3d} | "
                f"P={power_t_est:3d} | "
                f"Tj_NoATC={tj_no_atc:3d} | "
                f"fsw={fsw_khz:3d} | "
                f"Tj_max={tj_max:3d} | "
                f"Tj_start={tj_start:3d} | "
                f"Tj_ref={tj_ref:3d} | "
                f"RXframes={rx_frames} | "
                f"TXframes={tx_frames} | "
                f"ERR={rx_errors}"
            )
        else:
            rx_errors += 1

        rx_state = 0

    else:
        rx_state = 0


try:
    last_tx_time = time.perf_counter()

    while True:
        now = time.perf_counter()

        # TX at 100 Hz
        if now - last_tx_time >= TX_PERIOD:
            send_frame()
            last_tx_time += TX_PERIOD

            # Avoid burst if Linux scheduling was delayed
            if now - last_tx_time > TX_PERIOD:
                last_tx_time = now

        # RX: read everything currently waiting
        waiting = ser.in_waiting

        if waiting > 0:
            data = ser.read(waiting)

            for byte in data:
                process_received_byte(byte)
        else:
            time.sleep(0.0005)

except KeyboardInterrupt:
    print("\nStopped")
    print(f"TX frames: {tx_frames}")
    print(f"RX frames: {rx_frames}")
    print(f"RX errors: {rx_errors}")

finally:
    ser.close()
