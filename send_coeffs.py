import serial
import csv

# File paths
input_csv_file = "my_path.csv"
output_csv_file = "my_path_echo.csv"

# Serial port configuration
serial_port = "COM5"
baud_rate = 115200

def send_and_receive_coefficients(input_file, output_file, port, baud):
    with serial.Serial(port, baud, timeout=2) as ser:
        # Read coefficients from the CSV file
        with open(input_file, 'r') as infile:
            reader = csv.reader(infile)
            for row in reader:
                csv_data = ",".join(row) + "\n"  # Convert row to CSV string with newline
                
            ser.write(csv_data.encode('utf-8'))  # Send data to STM32

        # Wait for acknowledgment
        try:
            ack = ser.readline().decode().strip()  # Read acknowledgment
            print(f"STM32 Acknowledgment: {ack}")
        except Exception as e:
            print(f"Failed to receive acknowledgment: {e}")

        # Wait for echoed coefficients
        try:
            echoed_data = ser.read_until().decode('utf-8')  # Read echoed coefficients
            print(f"Echoed Coefficients: {echoed_data}")

            # Save echoed coefficients to a new CSV file
            with open(output_file, 'w', newline='') as outfile:
                writer = csv.writer(outfile)
                writer.writerow(echoed_data.split(","))  # Write to file
            print(f"Echoed coefficients saved to {output_file}.")
        except Exception as e:
            print(f"Failed to read echoed coefficients: {e}")

if __name__ == "__main__":
    send_and_receive_coefficients(input_csv_file, output_csv_file, serial_port, baud_rate)

