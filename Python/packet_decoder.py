import serial
import struct
import time

# Serial port settings
SERIAL_PORT = "COM6"  # Change if needed
BAUD_RATE = 115200
PACKET_SIZE = 19  # 1 Start + 1 Type + 4 Timestamp + 12 Data + 1 Checksum

# Packet types
TYPE_ACCEL = 0x01
TYPE_GYRO  = 0x02

# Open serial port
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# Store last timestamp for time offset calculation
last_timestamp_accel = None
last_timestamp_gyro = None
gyro_hz = 0
accel_hz = 0

def calculate_checksum(packet):
    """ Calculate XOR checksum (excluding the last byte, which is the received checksum). """
    checksum = 0
    for byte in packet[:-1]:  # Exclude received checksum
        checksum ^= byte & 0xFF
    return checksum

def parse_packet(packet):
    global last_timestamp_accel, last_timestamp_gyro
    global gyro_hz, accel_hz

    system_time = time.time()

    # Unpack packet (Little-endian: 1B start, 1B type, 4B timestamp, 3x4B floats, 1B checksum)
    start, pkt_type, timestamp, x, y, z, received_checksum = struct.unpack("<BBIfffB", packet)

    # Validate start byte
    if start != 0xAA:
        print("[ERROR] Invalid start byte: 0x{:02X}".format(start))
        return
    
    # Verify checksum
    computed_checksum = calculate_checksum(packet)
    if computed_checksum != received_checksum:
        print(f"[ERROR] Checksum mismatch! Received: 0x{received_checksum:02X}, Expected: 0x{computed_checksum:02X}")
        return

    # Determine which system_time to use
    if pkt_type == TYPE_ACCEL:
        time_offset = (system_time - last_timestamp_accel) * 1000 if last_timestamp_accel is not None else 0
        last_timestamp_accel = system_time  # Update only for Accelerometer
        sensor_type = "Accelerometer"
    elif pkt_type == TYPE_GYRO:
        time_offset = (system_time - last_timestamp_gyro) * 1000 if last_timestamp_gyro is not None else 0
        last_timestamp_gyro = system_time  # Update only for Gyroscope
        sensor_type = "Gyroscope----"
    else:
        print("[ERROR] Unknown packet type:", pkt_type)
        return

    print(f"[{sensor_type}] IMU Time: {timestamp*0.000025*1000:.2f} ms | System Time: {system_time:.6f} s | Offset: {time_offset:.6f} ms | X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")

while True:
    try:
        # Read a full packet
        packet = ser.read(PACKET_SIZE)

        if len(packet) == PACKET_SIZE:
            parse_packet(packet)  # Process the received packet
        else:
            print("[WARNING] Incomplete packet received.")

    except KeyboardInterrupt:
        print("\n[INFO] Exiting program.")
        ser.close()
        break
