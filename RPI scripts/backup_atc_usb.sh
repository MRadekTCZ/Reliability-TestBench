#!/bin/bash

set -e

DEV="$1"
MOUNT_POINT="/mnt/usb"
DEST_DIR="$MOUNT_POINT/ATCstudy"
SRC_DIR="/home/user10/ATCstudy/OpticSensorUDP/source"
LOG_FILE="/tmp/atc_usb_backup.log"

echo "========================================" >> "$LOG_FILE"
echo "$(date): USB backup started for $DEV" >> "$LOG_FILE"

sleep 3

mkdir -p "$MOUNT_POINT"

if mountpoint -q "$MOUNT_POINT"; then
    echo "$(date): $MOUNT_POINT already mounted" >> "$LOG_FILE"
else
    mount "$DEV" "$MOUNT_POINT"
    echo "$(date): mounted $DEV to $MOUNT_POINT" >> "$LOG_FILE"
fi

mkdir -p "$DEST_DIR"

cp "$SRC_DIR/last_drive_cycle.txt" "$DEST_DIR/"
cp "$SRC_DIR/Tj_data_record.txt" "$DEST_DIR/"

sync

echo "$(date): copied files to $DEST_DIR" >> "$LOG_FILE"
ls -lh "$DEST_DIR" >> "$LOG_FILE"

umount "$MOUNT_POINT"

echo "$(date): unmounted $MOUNT_POINT" >> "$LOG_FILE"
echo "$(date): USB backup finished" >> "$LOG_FILE"
