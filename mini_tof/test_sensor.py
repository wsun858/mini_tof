import time
import sys
import os

# --- IMPORTS AS REQUESTED ---
try:
    from mini_tof.readers.tmf882x_reader import TMF882XReader
    from mini_tof.readers.vl53l8ch_reader import VL53L8CHReader, VL53L8CHReaderNoAggregation
    from mini_tof_interfaces.msg import ToFFrame
except ImportError as e:
    print("CRITICAL ERROR: Could not import mini_tof modules.")
    print(f"Details: {e}")
    print("\nSUGGESTION: Ensure you have sourced your ROS workspace!")
    print("  Run: source install/setup.bash")
    sys.exit(1)

# --- CONFIGURATION ---
PORT = "/dev/ttyUSB0"  # Change if needed (e.g., /dev/ttyUSB0)
NUM_ZONES = 16         # VL53L8CH supports 16 or 64

def print_grid(distances):
    """Helper to print a clean 4x4 grid of the data."""
    # Clear console (ANSI escape code) to make the output look like a static video feed
    print("\033[2J\033[H") 
    print(f"--- Sensor Data Stream ({time.strftime('%H:%M:%S')}) ---")
    print(f"Driver: VL53L8CHReaderNoAggregation | Port: {PORT}")
    print("-" * 37)
    
    # Loop through 4 rows
    for row in range(4):
        line = "| "
        for col in range(4):
            idx = row * 4 + col
            try:
                val = distances[idx]
                # Color code: Green if close (<500mm), default otherwise
                if val < 500:
                    val_str = f"\033[92m{val:6.1f}\033[0m" # Green
                else:
                    val_str = f"{val:6.1f}"
                line += f"{val_str} | "
            except (IndexError, TypeError):
                line += "  N/A  | "
        print(line)
        print("-" * 37)

def main():
    print(f"Initializing VL53L8CHReaderNoAggregation on {PORT}...")
    
    try:
        # Instantiate the reader class directly from your library
        reader = VL53L8CHReaderNoAggregation(PORT, NUM_ZONES)
    except Exception as e:
        print(f"\nFailed to open serial port or initialize reader: {e}")
        return

    print("Connection successful. Starting data loop...")
    time.sleep(1)

    try:
        while True:
            # Call the library function directly
            # returns: (frame_data, frame_ambient_light, distance_mm, range_sigma_mm)
            measurement = reader.get_measurement()
            
            if measurement is None:
                continue

            # Unpack the data we care about
            # distance_mm is the list of computed distances provided by the MCU
            _, _, distance_mm, _ = measurement

            # Check if we actually got distance data (it might be None on partial reads)
            if distance_mm:
                print_grid(distance_mm)
            
            # Small sleep is implicit in serial reading, but we can yield slightly
            # to allow Ctrl+C to catch easily
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nRuntime error: {e}")

if __name__ == "__main__":
    main()