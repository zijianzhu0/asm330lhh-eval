import struct
import time
import serial
import curses
import threading
import queue
from collections import deque

class SerialPacketParser:
    PACKET_FORMAT = "<BBIfffB"
    PACKET_SIZE = struct.calcsize(PACKET_FORMAT)

    def __init__(self, port, baudrate=115200):
        self.serial_port = serial.Serial(port, baudrate, timeout=0)  # Non-blocking mode
        self.last_timestamps = {"gyro": None, "accel": None}  # Store last arrival timestamps
        self.packet_intervals = {"gyro": deque(maxlen=100), "accel": deque(maxlen=100)}
        self.packet_queue = queue.Queue()  # Thread-safe queue for packets
        self.recording = False
        self.recorded_data = []
        self.running = True

    def compute_xor_checksum(self, data):
        """Compute XOR checksum over the data bytes."""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def read_serial_data(self):
        """Runs in a separate thread to continuously read serial data at 100+ Hz."""
        while self.running:
            if self.serial_port.in_waiting >= self.PACKET_SIZE:
                packet = self.serial_port.read(self.PACKET_SIZE)
                arrival_time = time.time()  # Use arrival time for offset calculation
                parsed_data = self.process_packet(packet, arrival_time)
                if parsed_data:
                    self.packet_queue.put(parsed_data)  # Store in queue for UI thread

    def process_packet(self, packet, arrival_time):
        """Parse and validate the packet."""
        if len(packet) != self.PACKET_SIZE:
            return None

        start, pkt_type, timestamp, x, y, z, received_checksum = struct.unpack(self.PACKET_FORMAT, packet)
        
        expected_checksum = self.compute_xor_checksum(packet[:-1])
        if expected_checksum != received_checksum:
            return None  # Drop packet if checksum is invalid

        pkt_key = "gyro" if pkt_type == 1 else "accel" if pkt_type == 2 else None
        if not pkt_key:
            return None

        # Calculate offset (time difference from last packet of same type)
        last_arrival = self.last_timestamps[pkt_key]
        offset = (arrival_time - last_arrival) if last_arrival else 0
        self.last_timestamps[pkt_key] = arrival_time

        # Calculate frequency based on arrival time differences
        if last_arrival:
            self.packet_intervals[pkt_key].append(offset)

        avg_interval = sum(self.packet_intervals[pkt_key]) / len(self.packet_intervals[pkt_key]) if self.packet_intervals[pkt_key] else float('inf')
        frequency = 1 / avg_interval if avg_interval > 0 else 0

        # Store recorded data with offsets
        if self.recording:
            self.recorded_data.append((arrival_time, pkt_type, offset, x, y, z))

        return {"pkt_type": pkt_type, "x": x, "y": y, "z": z, "frequency": frequency, "offset": offset}

    def toggle_recording(self):
        """Toggle recording mode."""
        self.recording = not self.recording
        if not self.recording:
            self.save_recorded_data()

    def save_recorded_data(self):
        """Save recorded data to a file with offset information."""
        with open("packet_log.csv", "w") as f:
            f.write("arrival_time,pkt_type,offset,x,y,z\n")
            for entry in self.recorded_data:
                f.write(",".join(map(str, entry)) + "\n")
        self.recorded_data = []

def terminal_ui(stdscr, parser):
    """Curses-based UI, refreshes at 10 Hz while serial runs at 100+ Hz."""
    stdscr.nodelay(True)  # Non-blocking input
    stdscr.timeout(100)  # 10 Hz UI refresh rate

    gyro_data = {"x": 0, "y": 0, "z": 0, "frequency": 0, "offset": 0}
    accel_data = {"x": 0, "y": 0, "z": 0, "frequency": 0, "offset": 0}

    while parser.running:
        try:
            while not parser.packet_queue.empty():
                data = parser.packet_queue.get_nowait()
                if data["pkt_type"] == 1:
                    gyro_data = data
                elif data["pkt_type"] == 2:
                    accel_data = data
        except queue.Empty:
            pass  # No new data

        # UI Update
        stdscr.clear()

        # Gyro Data
        stdscr.addstr(1, 2, "Gyroscope Data:", curses.A_BOLD)
        stdscr.addstr(2, 4, f"X: {gyro_data['x']:.2f}")
        stdscr.addstr(3, 4, f"Y: {gyro_data['y']:.2f}")
        stdscr.addstr(4, 4, f"Z: {gyro_data['z']:.2f}")
        stdscr.addstr(5, 4, f"Frequency: {gyro_data['frequency']:.2f} Hz")
        stdscr.addstr(6, 4, f"Offset: {gyro_data['offset']:.6f} sec")

        # Accelerometer Data
        stdscr.addstr(8, 2, "Accelerometer Data:", curses.A_BOLD)
        stdscr.addstr(9, 4, f"X: {accel_data['x']:.2f}")
        stdscr.addstr(10, 4, f"Y: {accel_data['y']:.2f}")
        stdscr.addstr(11, 4, f"Z: {accel_data['z']:.2f}")
        stdscr.addstr(12, 4, f"Frequency: {accel_data['frequency']:.2f} Hz")
        stdscr.addstr(13, 4, f"Offset: {accel_data['offset']:.6f} sec")

        # Recording Status
        stdscr.addstr(15, 2, f"Recording: {'ON' if parser.recording else 'OFF'}", curses.A_BOLD)

        # Instructions
        stdscr.addstr(17, 2, "Press 'r' to toggle recording")
        stdscr.addstr(18, 2, "Press 'q' to quit")

        # Handle keypress
        key = stdscr.getch()
        if key == ord('r'):
            parser.toggle_recording()
        elif key == ord('q'):
            parser.running = False
            break

        stdscr.refresh()

if __name__ == "__main__":
    parser = SerialPacketParser(port="COM6")  # Change COM3 to your port

    # Start the serial reading thread
    serial_thread = threading.Thread(target=parser.read_serial_data, daemon=True)
    serial_thread.start()

    # Start the curses UI
    curses.wrapper(terminal_ui, parser)

    # Ensure the serial thread stops after exiting UI
    parser.running = False
    serial_thread.join()
