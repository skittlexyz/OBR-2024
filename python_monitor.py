import serial
import serial.tools.list_ports
import time
import json
from pygments import highlight, lexers, formatters

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def select_com_port():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found!")
        return None

    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i + 1}: {port}")

    while True:
        try:
            choice = int(input("Select the COM port by number: ")) - 1
            if 0 <= choice < len(ports):
                return ports[choice]
            else:
                print("Invalid selection. Try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def get_baud_rate():
    while True:
        try:
            baud_rate = int(input("Enter baud rate (e.g., 9600, 115200): "))
            return baud_rate
        except ValueError:
            print("Invalid input. Please enter a valid number.")

def main():
    com_port = select_com_port()
    if not com_port:
        return

    baud_rate = get_baud_rate()

    try:
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the connection to be established
        print(f"Connected to {com_port} at {baud_rate} baud rate.")
        print("Reading data from the serial port. Press 'Ctrl+C' to quit.")

        while True:
            if ser.in_waiting > 0:
                try:
                    data = ser.readline().decode().strip()
                    if data:
                        print(highlight(json.dumps(json.loads(data), indent=4), lexers.JsonLexer(), formatters.TerminalFormatter()))
                except:
                    data = ser.readline().decode('utf-8', 'ignore').strip()
                    if data:
                        print(data)

    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
