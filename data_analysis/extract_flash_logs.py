import serial
import os
import argparse
import sys
import threading

def read_serial(ser, output_dir):
    current_file = None
    file_content = []
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='replace').replace('\r', '')
            
            if not line:
                continue

            if line.startswith("START_FILE:"):
                filename = line.split(":", 1)[1].strip()
                current_file = os.path.join(output_dir, filename)
                file_content = []
                print(f"\nReceiving {filename}...")
                continue
            
            if line.startswith("END_FILE"):
                if current_file:
                    with open(current_file, 'w') as f:
                        f.writelines(file_content)
                    print(f"Saved -> {current_file}")
                    current_file = None
                continue
            
            # If we are inside a file block, capture the data
            if current_file is not None:
                file_content.append(line)
            else:
                print(line, end='', flush=True)
        except Exception as e:
            break

def extract_logs_from_serial(port, baudrate, output_dir):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    print(f"Listening on {port} at {baudrate} baud...")
    print(f"Saving extracted JSON files to '{output_dir}/'")
    print("You can now type commands (e.g., '12' or '11') and press Enter.")
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
    parser = argparse.ArgumentParser(description="Extract JSON logs from ESP32 serial dump")
    parser.add_argument("--port", "-p", required=True, help="Serial port (e.g., /dev/ttyUSB0 or COM3)")
    parser.add_argument("--baud", "-b", type=int, default=115200, help="Baud rate (default: 115200)")
    parser.add_argument("--out", "-o", default="flash_logs", help="Output directory (default: flash_logs)")
    
    args = parser.parse_args()
    extract_logs_from_serial(args.port, args.baud, args.out)
