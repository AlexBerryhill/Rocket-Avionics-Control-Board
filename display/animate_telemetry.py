import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import mpl_toolkits.mplot3d as mp3d

class SensorDataVisualizer:
    def __init__(self, bmp_file, mpu_file):
        """
        Initialize the visualizer with BMP and MPU log files
        
        Args:
            bmp_file (str): Path to BMP180 CSV log file
            mpu_file (str): Path to MPU-6050 CSV log file
        """
        # Read CSV files
        self.bmp_data = pd.read_csv(bmp_file)
        self.mpu_data = pd.read_csv(mpu_file)
        
        # Normalize timestamps to start from 0
        self.bmp_data['Timestamp'] = self.bmp_data['Timestamp'] - self.bmp_data['Timestamp'].min()
        self.mpu_data['Timestamp'] = self.mpu_data['Timestamp'] - self.mpu_data['Timestamp'].min()
        
        # Create figure with subplots (now 4 subplots)
        self.fig = plt.figure(figsize=(15, 15))
        self.fig.suptitle('Sensor Data Visualization', fontsize=16)
        
        # Create grid of subplots
        gs = self.fig.add_gridspec(2, 2)
        
        # Orientation angles plot
        self.ax1 = self.fig.add_subplot(gs[0, 0])
        self.line1, = self.ax1.plot([], [], label='Roll', color='blue')
        self.line2, = self.ax1.plot([], [], label='Pitch', color='red')
        
        self.ax1.set_title('Orientation Angles')
        self.ax1.set_xlabel('Time (ms)')
        self.ax1.set_ylabel('Angle (degrees)')
        self.ax1.legend()
        self.ax1.grid(True)
        
        # Temperature plot
        self.ax2 = self.fig.add_subplot(gs[0, 1])
        self.line3, = self.ax2.plot([], [], label='Temperature', color='green')
        
        self.ax2.set_title('Temperature')
        self.ax2.set_xlabel('Time (ms)')
        self.ax2.set_ylabel('Temperature (°C)')
        self.ax2.legend()
        self.ax2.grid(True)
        
        # Altitude plot
        self.ax3 = self.fig.add_subplot(gs[1, 0])
        self.line4, = self.ax3.plot([], [], label='Altitude', color='purple')
        
        self.ax3.set_title('Altitude')
        self.ax3.set_xlabel('Time (ms)')
        self.ax3.set_ylabel('Altitude (m)')
        self.ax3.legend()
        self.ax3.grid(True)
        
        # 3D rotation visualization
        self.ax4 = self.fig.add_subplot(gs[1, 1], projection='3d')
        
        # Create a cuboid to represent the object
        self.cube_vertices = np.array([
            [1, 1, 1],
            [1, 1, -1],
            [1, -1, 1],
            [1, -1, -1],
            [-1, 1, 1],
            [-1, 1, -1],
            [-1, -1, 1],
            [-1, -1, -1]
        ])
        
        # Edges of the cuboid
        self.cube_edges = [
            [0, 1], [0, 2], [0, 4], 
            [1, 3], [1, 5], 
            [2, 3], [2, 6], 
            [3, 7], 
            [4, 5], [4, 6], 
            [5, 7], 
            [6, 7]
        ]
        
        # Rotation line
        self.rotation_line, = self.ax4.plot([], [], [], color='red', linewidth=2)
        
        self.ax4.set_title('3D Object Rotation')
        self.ax4.set_xlabel('X')
        self.ax4.set_ylabel('Y')
        self.ax4.set_zlabel('Z')
        
        # Set axis limits for 3D plot
        self.ax4.set_xlim(-2, 2)
        self.ax4.set_ylim(-2, 2)
        self.ax4.set_zlim(-2, 2)
        
        # Set initial x and y limits for other plots
        max_time = max(self.bmp_data['Timestamp'].max(), self.mpu_data['Timestamp'].max())
        
        self.ax1.set_xlim(0, max_time)
        self.ax1.set_ylim(
            min(self.mpu_data[['Roll', 'Pitch']].min()) - 5, 
            max(self.mpu_data[['Roll', 'Pitch']].max()) + 5
        )
        
        self.ax2.set_xlim(0, max_time)
        self.ax2.set_ylim(
            self.mpu_data['Temperature'].min() - 1, 
            self.mpu_data['Temperature'].max() + 1
        )
        
        self.ax3.set_xlim(0, max_time)
        self.ax3.set_ylim(
            self.bmp_data['Altitude'].min() - 1, 
            self.bmp_data['Altitude'].max() + 1
        )
        
        # Animation parameters
        self.max_points = 100  # Number of points to show at a time
    
    def rotate_point(self, point, roll, pitch, yaw):
        """
        Rotate a 3D point given roll, pitch, and yaw angles
        
        Args:
            point (np.array): 3D point coordinates
            roll (float): Roll angle in degrees
            pitch (float): Pitch angle in degrees
            yaw (float): Yaw angle in degrees
        
        Returns:
            np.array: Rotated point coordinates
        """
        # Convert degrees to radians
        roll_rad = np.deg2rad(roll)
        pitch_rad = np.deg2rad(pitch)
        yaw_rad = np.deg2rad(yaw)
        
        # Rotation matrices
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(roll_rad), -np.sin(roll_rad)],
            [0, np.sin(roll_rad), np.cos(roll_rad)]
        ])
        
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        Rz = np.array([
            [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
            [np.sin(yaw_rad), np.cos(yaw_rad), 0],
            [0, 0, 1]
        ])
        
        # Combined rotation matrix
        R = Rz @ Ry @ Rx
        
        # Rotate point
        return R @ point
    
    def animate(self, frame):
        """
        Animation update function
        
        Args:
            frame (int): Current animation frame
        """
        # Filter data up to current frame
        mpu_subset = self.mpu_data[self.mpu_data['Timestamp'] <= frame]
        bmp_subset = self.bmp_data[self.bmp_data['Timestamp'] <= frame]
        
        # Take only last max_points
        mpu_subset = mpu_subset.tail(self.max_points)
        bmp_subset = bmp_subset.tail(self.max_points)
        
        # Update line data for graphs
        self.line1.set_data(mpu_subset['Timestamp'], mpu_subset['Roll'])
        self.line2.set_data(mpu_subset['Timestamp'], mpu_subset['Pitch'])
        self.line3.set_data(mpu_subset['Timestamp'], mpu_subset['Temperature'])
        self.line4.set_data(bmp_subset['Timestamp'], bmp_subset['Altitude'])
        
        # Get the latest rotation data
        if not mpu_subset.empty:
            latest_roll = mpu_subset['Roll'].iloc[-1]
            latest_pitch = mpu_subset['Pitch'].iloc[-1]
            latest_yaw = 0  # Assuming no yaw data, you can modify if available
            
            # Rotate cube vertices
            rotated_vertices = np.array([
                self.rotate_point(vertex, latest_roll, latest_pitch, latest_yaw) 
                for vertex in self.cube_vertices
            ])
            
            # Plot cube edges
            self.ax4.clear()
            self.ax4.set_title('3D Object Rotation')
            self.ax4.set_xlabel('X')
            self.ax4.set_ylabel('Y')
            self.ax4.set_zlabel('Z')
            self.ax4.set_xlim(-2, 2)
            self.ax4.set_ylim(-2, 2)
            self.ax4.set_zlim(-2, 2)
            
            for edge in self.cube_edges:
                v1, v2 = rotated_vertices[edge[0]], rotated_vertices[edge[1]]
                self.ax4.plot3D([v1[0], v2[0]], [v1[1], v2[1]], [v1[2], v2[2]], color='blue')
            
            # Plot rotation axis line
            self.ax4.plot3D([0, rotated_vertices[0][0]], 
                            [0, rotated_vertices[0][1]], 
                            [0, rotated_vertices[0][2]], 
                            color='red', linewidth=2)
        
        return self.line1, self.line2, self.line3, self.line4
    
    def show(self, interval=50):
        """
        Start the animation
        
        Args:
            interval (int, optional): Milliseconds between frames. Defaults to 50.
        """
        max_time = max(self.bmp_data['Timestamp'].max(), self.mpu_data['Timestamp'].max())
        
        # Create animation
        ani = animation.FuncAnimation(
            self.fig, 
            self.animate, 
            frames=np.linspace(0, max_time, 200),
            interval=interval, 
            blit=False,  # Changed to False to allow 3D plot clearing
            repeat=False
        )
        
        plt.tight_layout()
        plt.show()

if __name__ == "__main__":
    bmp_file = 'display/logs/bmp_log.csv'
    mpu_file = 'display/logs/mpu_log.csv'
    
    visualizer = SensorDataVisualizer(bmp_file, mpu_file)
    visualizer.show()