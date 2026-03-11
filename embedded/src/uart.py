import serial

PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE)
        print("Connected. 2-Byte Dual-Motor Protocol Active.")
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
                
                # --- Construct Byte 1: Control ---
                # Start with the MSB set to 1 (0x80)
                byte_1 = 0x80 
                
                if motor_char == 'B':
                    byte_1 |= 0x40  # Set Bit 6 for Motor B
                
                if val < 0:
                    byte_1 |= 0x20  # Set Bit 5 for Reverse
                
                # --- Construct Byte 2: Payload ---
                # Since percent is 0-100, MSB is naturally 0
                byte_2 = percent 
                
                # Transmit both bytes
                ser.write(bytes([byte_1, byte_2]))
                
                dir_str = "Reverse" if val < 0 else "Forward"
                if percent == 0: dir_str = "OFF"
                khz = percent * 2
                
                print(f"-> Motor {motor_char} | {percent}% ({khz} kHz) | {dir_str} | (Sent: 0x{byte_1:02X} 0x{byte_2:02X})")
                    
            except ValueError:
                print("Invalid number format. Example: -100A")

    except serial.SerialException as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == '__main__':
    main()