import sys
import serial
import serial.tools.list_ports
import threading

BAUD_RATE = 115200

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if not ports: return None

    # Pro Check: FTDI hardware ID
    for port in ports:
        if port.vid == 0x0403 and port.pid == 0x6001: return port.device

    # OS Fallback
    if sys.platform.startswith('linux'):
        for port in ports:
            if 'USB' in port.device: return port.device
    elif sys.platform.startswith('win'):
        for port in ports:
            if 'COM' in port.device: return port.device
                
    return ports[0].device

def listen_to_stm32(ser):
    """Background thread that constantly reads incoming UART packets."""
    while True:
        try:
            # Wait until at least 2 bytes are available
            if ser.in_waiting >= 2:
                # Read the first byte
                byte1 = int.from_bytes(ser.read(1), 'little')
                
                # Check if it's our Control Byte (MSB == 1)
                if byte1 & 0x80:
                    byte2 = int.from_bytes(ser.read(1), 'little')
                    
                    is_neg = (byte1 & 0x40)
                    val = byte2 & 0x7F
                    if is_neg:
                        val = -val
                    
                    # \r returns the cursor to the start of the line so we can print cleanly 
                    # without messing up the user's current input prompt.
                    print(f"\r[STM32 Counter Update]: {val}          \nCmd >> ", end="", flush=True)
        except Exception as e:
            # Thread exits silently if the port closes
            break

def main():
    print("Scanning for STM32 connection...")
    auto_port = find_serial_port()
    
    if not auto_port:
        print("Error: No serial/USB devices found. Check your physical connection.")
        return

    try:
        ser = serial.Serial(auto_port, BAUD_RATE)
        print(f"-> Success! Auto-connected to STM32 on {auto_port}.\n")
        
        # Start the background listener thread!
        listener_thread = threading.Thread(target=listen_to_stm32, args=(ser,), daemon=True)
        listener_thread.start()

        print("2-Way Protocol Active (Motor Tx + Counter Rx).")
        print("Format: [Percent][Motor]. Example: -100A, 80B, 0A")
        print("Type 'exit' to quit.\n")

        while True:
            user_input = input("Cmd >> ").strip()
            if user_input.lower() == 'exit': break
            
            if len(user_input) < 2:
                print("Invalid format. Need value and motor (e.g., 50A).")
                continue
            
            motor_char = user_input[-1].upper()
            val_str = user_input[:-1]
            
            if motor_char not in ['A', 'B']:
                print("Error: Command must end with 'A' or 'B'.")
                continue
                
            try:
                val = int(val_str)
                percent = abs(val)
                
                if percent > 100:
                    print("Error: Percentage must be between 0 and 100.")
                    continue
                
                byte_1 = 0x80 
                if motor_char == 'B': byte_1 |= 0x40  
                if val < 0: byte_1 |= 0x20  
                
                byte_2 = percent 
                ser.write(bytes([byte_1, byte_2]))
                
                dir_str = "Reverse" if val < 0 else "Forward"
                if percent == 0: dir_str = "OFF"
                
                print(f"-> Motor {motor_char} | {percent}% | {dir_str}")
                    
            except ValueError:
                print("Invalid number format. Example: -100A")

    except serial.SerialException as e:
        print(f"Connection Error on {auto_port}: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port closed.")

if __name__ == '__main__':
    main()