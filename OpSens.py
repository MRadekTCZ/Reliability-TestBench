import socket
import struct
import time

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

# Last sample seen for each channel
last_sample = {
    1: None,
    2: None,
}

# Optional: keep last whole packet for each channel
last_packet_values = {
    1: None,
    2: None,
}


def parse_packet(data: bytes):
    if len(data) < HEADER_SIZE:
        return

    header = struct.unpack(HEADER_FORMAT, data[:HEADER_SIZE])

    data_id = header[0]
    size_bytes = header[2]
    channel = header[5]

    if data_id != MEASURE_DATA_ID:
        return

    if channel not in (1, 2):
        return

    payload = data[HEADER_SIZE:HEADER_SIZE + size_bytes]

    if len(payload) < size_bytes:
        return

    if size_bytes % 4 != 0:
        return

    float_count = size_bytes // 4
    values = struct.unpack("<" + "f" * float_count, payload)

    if not values:
        return

    # Save whole packet if you want access to it later
    last_packet_values[channel] = values

    # Save only the last sample from this packet
    last_sample[channel] = values[-1]


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", PORT))

    mreq = socket.inet_aton(MCAST_GRP) + socket.inet_aton(LOCAL_IP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    sock.settimeout(0.05)

    print(f"Listening on multicast {MCAST_GRP}:{PORT} via {LOCAL_IP}")
    print("Printing latest sample from CH1 and CH2 every 200 ms")

    last_print = time.time()

    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
            src_ip, _ = addr

            if EXPECTED_SENDER_IP and src_ip != EXPECTED_SENDER_IP:
                continue

            parse_packet(data)

        except socket.timeout:
            pass

        now = time.time()
        if now - last_print >= 0.2:
            ch1 = last_sample[1]
            ch2 = last_sample[2]

            ch1_text = f"{ch1:.6f}" if ch1 is not None else "no data"
            ch2_text = f"{ch2:.6f}" if ch2 is not None else "no data"

            print(f"CH1: {ch1_text} | CH2: {ch2_text}")
            last_print = now


if __name__ == "__main__":
    main()