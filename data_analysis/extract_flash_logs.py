import serial
import os
import argparse
import sys
import threading

def read_serial(ser, output_dir):
    current_filepath = None
    active_file = None
    
    while True:
        try:
            # Read line and normalize line endings
            line = ser.readline().decode('utf-8', errors='replace').replace('\r', '')
            
            if not line:
                continue

            if line.startswith("START_FILE:"):
                filename = line.split(":", 1)[1].strip()
                current_filepath = os.path.join(output_dir, filename)
                
                # Close any previously dangling file just in case
                if active_file:
                    active_file.close()
                    
                # Open directly in write mode to stream the data to disk
                active_file = open(current_filepath, 'w')
                print(f"\nReceiving {filename} (Streaming data directly to disk)...")
                continue
            
            if line.startswith("END_FILE"):
                if active_file:
                    active_file.close()
                    active_file = None
                    print(f"Saved -> {current_filepath}")
                    current_filepath = None
                continue
            
            # If we are inside a file block, write directly to the file stream
            if active_file is not None:
                active_file.write(line)
            else:
                # Normal serial prints from the ESP32 (debug logs, etc.)
                print(line, end='', flush=True)
                
        except Exception as e:
            print(f"\nSerial read exception: {e}")
            if active_file:
                active_file.close()
            break

def extract_logs_from_serial(port, baudrate, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"Listening on {port} at {baudrate} baud...")
    print(f"Saving extracted files to '{output_dir}/'")
    print("You can now type commands and press Enter.")
    print("Press Ctrl+C to stop.\n")

    try:
        ser = serial.Serial(port, baudrate, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # Start the background thread for reading
    reader_thread = threading.Thread(target=read_serial, args=(ser, output_dir), daemon=True)
    reader_thread.start()

    # Use the main thread to capture user input and send it to the ESP32
    try:
        while True:
            user_input = sys.stdin.readline()
            if user_input:
                ser.write(user_input.encode('utf-8'))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract JSONL logs from ESP32 serial dump")
    parser.add_argument("--port", "-p", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--out", "-o", default="flash_logs", help="Output directory (default: flash_logs)")
    
    args = parser.parse_args()
    extract_logs_from_serial(args.port, args.baud, args.out)