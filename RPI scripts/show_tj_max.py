#!/usr/bin/env python3

from pathlib import Path

FILE = Path("/home/user10/ATCstudy/OpticSensorUDP/source/Tj_data_record.txt")
N_LAST = 10

if not FILE.exists():
    print(f"File not found: {FILE}")
    raise SystemExit(1)

lines = FILE.read_text(encoding="utf-8").splitlines()
lines = [line.strip() for line in lines if line.strip()]

if len(lines) <= 1:
    print("No completed cycles yet.")
    raise SystemExit(0)

data_lines = lines[1:]
last_lines = data_lines[-N_LAST:]

print("")
print(f"Last {len(last_lines)} completed paired cycles:")
print("")
print(
    f"{'Pair':>6} | "
    f"{'CH1 min':>9} | {'CH1 max':>9} | {'CH1 dT':>9} | "
    f"{'CH2 min':>9} | {'CH2 max':>9} | {'CH2 dT':>9} | "
    f"{'C':>9} | {'Higher CH':>9}"
)
print("-" * 97)

global_max = None
global_max_ch = None
global_max_pair = None

for line in last_lines:
    cols = line.split()

    try:
        pair_idx = int(cols[0])

        ch1_min = float(cols[7])
        ch1_max = float(cols[8])
        ch1_dt = ch1_max - ch1_min

        ch2_min = float(cols[15])
        ch2_max = float(cols[16])
        ch2_dt = ch2_max - ch2_min

    except (IndexError, ValueError):
        print(f"Skipped malformed line: {line}")
        continue

    if ch1_max >= ch2_max:
        higher = ch1_max
        higher_ch = "CH1"
    else:
        higher = ch2_max
        higher_ch = "CH2"

    if global_max is None or higher > global_max:
        global_max = higher
        global_max_ch = higher_ch
        global_max_pair = pair_idx

    print(
        f"{pair_idx:6d} | "
        f"{ch1_min:9.3f} | {ch1_max:9.3f} | {ch1_dt:9.3f} | "
        f"{ch2_min:9.3f} | {ch2_max:9.3f} | {ch2_dt:9.3f} | "
        f"{higher:9.3f} | {higher_ch:>9}"
    )

print("-" * 97)

if global_max is not None:
    print(f"Highest in shown cycles: {global_max:.3f} C, {global_max_ch}, pair {global_max_pair}")

print("")
