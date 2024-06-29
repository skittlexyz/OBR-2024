import serial
import serial.tools.list_ports

def list_serial_ports():
    ports = serial.tools.list_ports.comports()
    return [port.device for port in ports]

def select_serial_port(ports):
    print("Available serial ports:")
    for i, port in enumerate(ports):
        print(f"{i}: {port}")
    
    while True:
        try:
            choice = int(input("Select a serial port by number: "))
            if 0 <= choice < len(ports):
                return ports[choice]
            else:
                print("Invalid choice. Try again.")
        except ValueError:
            print("Invalid input. Enter a number.")

def select_baud_rate():
    baud_rates = [9600, 14400, 19200, 38400, 57600, 115200]
    print("Available baud rates:")
    for i, rate in enumerate(baud_rates):
        print(f"{i}: {rate}")
    
    while True:
        try:
            choice = int(input("Select a baud rate by number: "))
            if 0 <= choice < len(baud_rates):
                return baud_rates[choice]
            else:
                print("Invalid choice. Try again.")
        except ValueError:
            print("Invalid input. Enter a number.")

def main():
    ports = list_serial_ports()
    if not ports:
        print("No serial ports found.")
        return
    
    selected_port = select_serial_port(ports)
    selected_baud_rate = select_baud_rate()

    try:
        ser = serial.Serial(selected_port, selected_baud_rate, timeout=1)
        print(f"Connected to {selected_port} at {selected_baud_rate} baud rate")

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').rstrip()
                print(f"@> {line}")

    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed")

if __name__ == "__main__":
    main()
