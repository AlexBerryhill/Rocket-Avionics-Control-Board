import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def parse_telemetry(line):
    try:
        # Parse the telemetry data
        parts = line.split(", ")
        message = parts[0].split(": ")[1]
        latitude = float(parts[1].split(": ")[1])
        longitude = float(parts[2].split(": ")[1])
        pitch = float(parts[3].split(": ")[1])
        roll = float(parts[4].split(": ")[1])
        yaw = float(parts[5].split(": ")[1])
        return message, latitude, longitude, pitch, roll, yaw
    except (IndexError, ValueError):
        return None

def read_telemetry(port, baud_rate):
    try:
        # Open the serial port
        with serial.Serial(port, baud_rate, timeout=1) as ser:
            print(f"Listening for telemetry on {port} at {baud_rate} baud...")

            # Initialize the plot
            plt.ion()
            fig = plt.figure(figsize=(10, 5))

            # Subplot for latitude and longitude
            ax1 = fig.add_subplot(121)
            ax1.set_title("Latitude vs Longitude")
            ax1.set_xlabel("Longitude")
            ax1.set_ylabel("Latitude")
            latitudes, longitudes = [], []

            # Subplot for 3D rotation
            ax2 = fig.add_subplot(122, projection='3d')
            ax2.set_title("3D Orientation")
            ax2.set_xlim([-1, 1])
            ax2.set_ylim([-1, 1])
            ax2.set_zlim([-1, 1])

            while True:
                # Read a line from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line:
                    telemetry = parse_telemetry(line)
                    if telemetry:
                        message, latitude, longitude, pitch, roll, yaw = telemetry
                        print(f"Message: {message}, Lat: {latitude}, Long: {longitude}, Pitch: {pitch}, Roll: {roll}, Yaw: {yaw}")

                        # Update latitude and longitude plot
                        latitudes.append(latitude)
                        longitudes.append(longitude)
                        ax1.plot(longitudes, latitudes, 'b.-')
                        ax1.relim()
                        ax1.autoscale_view()

                        # Update 3D orientation plot
                        ax2.cla()
                        ax2.set_title("3D Orientation")
                        ax2.set_xlim([-1, 1])
                        ax2.set_ylim([-1, 1])
                        ax2.set_zlim([-1, 1])

                        # Calculate orientation vectors
                        pitch_rad = np.radians(pitch)
                        roll_rad = np.radians(roll)
                        yaw_rad = np.radians(yaw)

                        # Rotation matrix
                        R = np.array([
                            [np.cos(yaw_rad) * np.cos(pitch_rad),
                             np.cos(yaw_rad) * np.sin(pitch_rad) * np.sin(roll_rad) - np.sin(yaw_rad) * np.cos(roll_rad),
                             np.cos(yaw_rad) * np.sin(pitch_rad) * np.cos(roll_rad) + np.sin(yaw_rad) * np.sin(roll_rad)],
                            [np.sin(yaw_rad) * np.cos(pitch_rad),
                             np.sin(yaw_rad) * np.sin(pitch_rad) * np.sin(roll_rad) + np.cos(yaw_rad) * np.cos(roll_rad),
                             np.sin(yaw_rad) * np.sin(pitch_rad) * np.cos(roll_rad) - np.cos(yaw_rad) * np.sin(roll_rad)],
                            [-np.sin(pitch_rad),
                             np.cos(pitch_rad) * np.sin(roll_rad),
                             np.cos(pitch_rad) * np.cos(roll_rad)]
                        ])

                        # Plot orientation
                        origin = np.array([0, 0, 0])
                        x_axis = R @ np.array([1, 0, 0])
                        y_axis = R @ np.array([0, 1, 0])
                        z_axis = R @ np.array([0, 0, 1])

                        ax2.quiver(*origin, *x_axis, color='r', label='X-axis')
                        ax2.quiver(*origin, *y_axis, color='g', label='Y-axis')
                        ax2.quiver(*origin, *z_axis, color='b', label='Z-axis')
                        ax2.legend()

                        plt.pause(0.01)
    except serial.SerialException as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        print("\nExiting...")
        plt.close()

if __name__ == "__main__":
    # Define the serial port and baud rate
    SERIAL_PORT = "COM16"
    BAUD_RATE = 115200

    # Start reading telemetry
    read_telemetry(SERIAL_PORT, BAUD_RATE)