import serial
import serial.tools.list_ports
from pynput import keyboard
import time

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

def on_press(key, ser):
    try:
        key_char = key.char.upper()
        if key_char == 'W':
            ser.write('F'.encode())
            print("Sent: F")
        elif key_char == 'A':
            ser.write('L'.encode())
            print("Sent: L")
        elif key_char == 'S':
            ser.write('B'.encode())
            print("Sent: B")
        elif key_char == 'D':
            ser.write('R'.encode())
            print("Sent: R")
    except AttributeError:
        pass  # Special keys (e.g., shift) are ignored

def on_release(key, ser):
    try:
        ser.write('S'.encode())
        print("Sent: S")
    except AttributeError:
        pass  # Special keys (e.g., shift) are ignored
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def main():
    com_port = select_com_port()
    if not com_port:
        return
    
    baud_rate = get_baud_rate()

    try:
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the connection to be established
        print(f"Connected to {com_port}")
        print("Press W, A, S, or D to send commands to the Arduino.")
        print("Press 'ESC' to quit.")

        with keyboard.Listener(
            on_press=lambda key: on_press(key, ser),
            on_release=lambda key: on_release(key, ser)) as listener:
            listener.join()

    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()
