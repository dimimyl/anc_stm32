import serial
import os

def save_csv(file_name, data):
    # Ensure the file ends with .csv
    if not file_name.endswith('.csv'):
        file_name += '.csv'
    
    # Save the data to the specified file
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
    file_path = os.path.join(desktop_path, file_name)

    with open(file_path, 'a') as file:
        file.write(data)
    
    print(f"Data saved to {file_path}")

def main():
    # Configure the serial port (update 'COMX' to match your UART port)
    ser = serial.Serial('COM5', baudrate=115200, timeout=100)  # Replace 'COMX' with your port
    print("Waiting for data...")

    # Read data from UART
    received_data = ser.read_until().decode('utf-8')  # Read until newline
    print("Data received")

    # Ask user for file name
    file_name = input("Enter the desired CSV file name: ")

    # Save the data to a CSV file
    save_csv(file_name, received_data)

if __name__ == "__main__":
    main()