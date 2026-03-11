import serial

PORT = '/dev/ttyUSB1' 
BAUD_RATE = 115200

def main():
    try:
        ser = serial.Serial(PORT, BAUD_RATE)
        print(f"Connected to STM32 on {PORT}.")
        print("Commands: Enter a frequency between 100 and 200 (e.g., 150).")
        print("Add a '-' sign (e.g., -150) to reverse direction.")
        print("Enter '0' to stop the motor.")
        print("Type 'exit' to quit.\n")

        while True:
            user_input = input("Freq (kHz) >> ").strip()
            
            if user_input.lower() == 'exit':
                break
            
            try:
                val = int(user_input)
                mag = abs(val)
                
                if mag == 0:
                    # 127 (0x7F) is our special out-of-bounds trigger for OFF
                    encoded_byte = 127 
                    ser.write(bytes([encoded_byte]))
                    print("-> Sent Packed Byte: 0x7F | Decodes to: OFF (0% Duty Cycle)")
                    
                elif 100 <= mag <= 200:
                    # Strip the base 100 offset
                    encoded_byte = mag - 100
                    
                    # Apply direction bit mask
                    if val < 0:
                        encoded_byte |= 0x80
                    
                    ser.write(bytes([encoded_byte]))
                    
                    dir_str = "Reverse" if val < 0 else "Forward"
                    print(f"-> Sent Packed Byte: 0x{encoded_byte:02X} | Decodes to: {mag} kHz, {dir_str}")
                else:
                    print("Error: Frequency magnitude must be 0 (OFF), or strictly between 100 and 200.")
                    
            except ValueError:
                print("Invalid input. Please enter a valid number (e.g., 150, -150, or 0).")

    except serial.SerialException as e:
        print(f"Connection Error: {e}")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Port closed.")

if __name__ == '__main__':
    main()