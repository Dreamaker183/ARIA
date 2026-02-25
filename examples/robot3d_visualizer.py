#!/usr/bin/env python3
"""
3D Robot and Room Visualizer for Pioneer 3DX
Shows robot position, triangle pattern, obstacles, and room layout
"""

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import time
import threading
import queue
import json
import os
import sys

class Robot3DVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Robot data
        self.robot_positions = []
        self.robot_headings = []
        self.obstacle_data = []
        self.triangle_vertices = []
        self.room_bounds = [-3000, 3000, -3000, 3000]  # Room size in mm
        
        # Visualization settings
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Height (mm)')
        self.ax.set_title('Pioneer 3DX Robot - 3D Navigation Visualization')
        
        # Set room boundaries
        self.setup_room()
        
        # Data queue for real-time updates
        self.data_queue = queue.Queue()
        self.running = True
        
    def setup_room(self):
        """Setup room walls and obstacles"""
        # Room floor
        x_room = [self.room_bounds[0], self.room_bounds[1], self.room_bounds[1], self.room_bounds[0], self.room_bounds[0]]
        y_room = [self.room_bounds[2], self.room_bounds[2], self.room_bounds[3], self.room_bounds[3], self.room_bounds[2]]
        z_room = [0, 0, 0, 0, 0]
        self.ax.plot(x_room, y_room, z_room, 'k-', linewidth=2, label='Room Floor')
        
        # Room walls (vertical lines)
        wall_height = 2000  # 2m walls
        for i in range(len(x_room)-1):
            self.ax.plot([x_room[i], x_room[i]], [y_room[i], y_room[i]], [0, wall_height], 'k-', linewidth=2)
        
        # Add some obstacles (red cylinders)
        self.add_obstacle(1000, 1000, 500, 300)  # Obstacle 1
        self.add_obstacle(-800, 1200, 400, 250)  # Obstacle 2
        self.add_obstacle(500, -1000, 600, 400)  # Obstacle 3
        self.add_obstacle(-1200, -800, 350, 200) # Obstacle 4
        
    def add_obstacle(self, x, y, radius, height):
        """Add a cylindrical obstacle"""
        # Create cylinder
        theta = np.linspace(0, 2*np.pi, 20)
        z = np.linspace(0, height, 10)
        theta_grid, z_grid = np.meshgrid(theta, z)
        
        x_cyl = x + radius * np.cos(theta_grid)
        y_cyl = y + radius * np.sin(theta_grid)
        
        self.ax.plot_surface(x_cyl, y_cyl, z_grid, color='red', alpha=0.7, label='Obstacle')
        
        # Add top and bottom circles
        theta_circle = np.linspace(0, 2*np.pi, 50)
        x_circle = x + radius * np.cos(theta_circle)
        y_circle = y + radius * np.sin(theta_circle)
        
        # Bottom circle
        self.ax.plot(x_circle, y_circle, np.zeros_like(x_circle), 'r-', linewidth=2)
        # Top circle
        self.ax.plot(x_circle, y_circle, np.full_like(x_circle, height), 'r-', linewidth=2)
    
    def update_robot_position(self, x, y, heading, battery_voltage, sonar_distance):
        """Update robot position and data"""
        self.robot_positions.append([x, y, 50])  # Robot at 50mm height
        self.robot_headings.append(heading)
        
        # Keep only last 100 positions for trail
        if len(self.robot_positions) > 100:
            self.robot_positions.pop(0)
            self.robot_headings.pop(0)
        
        # Update visualization
        self.draw_robot()
        self.draw_triangle_pattern()
        self.update_info_display(x, y, heading, battery_voltage, sonar_distance)
        
    def draw_robot(self):
        """Draw robot as a 3D model"""
        if not self.robot_positions:
            return
            
        # Get current position
        pos = self.robot_positions[-1]
        heading = self.robot_headings[-1]
        
        # Robot body (rectangular)
        robot_length = 400  # mm
        robot_width = 300   # mm
        robot_height = 200  # mm
        
        # Calculate robot corners
        cos_h = np.cos(np.radians(heading))
        sin_h = np.sin(np.radians(heading))
        
        # Robot corners in local coordinates
        corners = np.array([
            [-robot_length/2, -robot_width/2, 0],
            [robot_length/2, -robot_width/2, 0],
            [robot_length/2, robot_width/2, 0],
            [-robot_length/2, robot_width/2, 0],
            [-robot_length/2, -robot_width/2, robot_height],
            [robot_length/2, -robot_width/2, robot_height],
            [robot_length/2, robot_width/2, robot_height],
            [-robot_length/2, robot_width/2, robot_height]
        ])
        
        # Rotate and translate corners
        rotated_corners = []
        for corner in corners:
            # Rotate
            x_rot = corner[0] * cos_h - corner[1] * sin_h
            y_rot = corner[0] * sin_h + corner[1] * cos_h
            # Translate
            rotated_corners.append([pos[0] + x_rot, pos[1] + y_rot, pos[2] + corner[2]])
        
        rotated_corners = np.array(rotated_corners)
        
        # Draw robot body (blue)
        self.draw_rectangular_prism(rotated_corners, 'blue', alpha=0.8)
        
        # Draw heading arrow (green)
        arrow_length = 200
        arrow_x = pos[0] + arrow_length * cos_h
        arrow_y = pos[1] + arrow_length * sin_h
        self.ax.plot([pos[0], arrow_x], [pos[1], arrow_y], [pos[2] + robot_height/2, pos[2] + robot_height/2], 
                    'g-', linewidth=3, label='Robot Heading')
        
        # Draw sonar range (yellow cone)
        if len(self.robot_positions) > 0:
            self.draw_sonar_cone(pos, heading, 1000)  # 1m range
    
    def draw_rectangular_prism(self, corners, color, alpha=0.7):
        """Draw a rectangular prism given 8 corners"""
        # Define faces (6 faces of a cube)
        faces = [
            [0, 1, 2, 3],  # bottom
            [4, 5, 6, 7],  # top
            [0, 1, 5, 4],  # front
            [2, 3, 7, 6],  # back
            [0, 3, 7, 4],  # left
            [1, 2, 6, 5]   # right
        ]
        
        for face in faces:
            face_corners = corners[face]
            # Create triangular faces
            for i in range(len(face_corners) - 2):
                triangle = np.array([face_corners[0], face_corners[i+1], face_corners[i+2]])
                self.ax.plot_trisurf(triangle[:, 0], triangle[:, 1], triangle[:, 2], 
                                   color=color, alpha=alpha)
    
    def draw_sonar_cone(self, pos, heading, range_mm):
        """Draw sonar detection cone"""
        cone_angle = 45  # degrees
        cone_height = 100
        
        # Create cone points
        theta = np.linspace(np.radians(heading - cone_angle/2), 
                           np.radians(heading + cone_angle/2), 20)
        
        for i in range(len(theta)-1):
            x1 = pos[0] + range_mm * np.cos(theta[i])
            y1 = pos[1] + range_mm * np.sin(theta[i])
            x2 = pos[0] + range_mm * np.cos(theta[i+1])
            y2 = pos[1] + range_mm * np.sin(theta[i+1])
            
            # Draw cone surface
            self.ax.plot([pos[0], x1, x2], [pos[1], y1, y2], 
                        [pos[2] + cone_height, pos[2], pos[2]], 
                        'y-', alpha=0.3, linewidth=1)
    
    def draw_triangle_pattern(self):
        """Draw the triangle pattern the robot is following"""
        if len(self.robot_positions) < 3:
            return
            
        # Get triangle vertices from robot positions
        if len(self.robot_positions) >= 3:
            # Find triangle vertices (every 3 significant position changes)
            vertices = []
            step = max(1, len(self.robot_positions)//3)
            for i in range(0, len(self.robot_positions), step):
                if i < len(self.robot_positions):
                    vertices.append(self.robot_positions[i])
            
            # Ensure we have at least 3 unique vertices
            if len(vertices) >= 3:
                # Remove duplicate vertices
                unique_vertices = []
                for v in vertices:
                    if not unique_vertices or any(abs(v[0] - uv[0]) > 50 or abs(v[1] - uv[1]) > 50 for uv in unique_vertices):
                        unique_vertices.append(v)
                
                if len(unique_vertices) >= 3:
                    # Draw triangle outline
                    triangle_x = [v[0] for v in unique_vertices] + [unique_vertices[0][0]]
                    triangle_y = [v[1] for v in unique_vertices] + [unique_vertices[0][1]]
                    triangle_z = [v[2] for v in unique_vertices] + [unique_vertices[0][2]]
                    
                    self.ax.plot(triangle_x, triangle_y, triangle_z, 'g--', linewidth=2, alpha=0.7, label='Triangle Pattern')
    
    def update_info_display(self, x, y, heading, battery, sonar):
        """Update information display"""
        info_text = f'Robot Position: ({x:.0f}, {y:.0f})\nHeading: {heading:.1f}°\nBattery: {battery:.1f}V\nSonar: {sonar:.0f}mm'
        
        # Clear previous text
        for text in self.ax.texts:
            if 'Robot Position:' in text.get_text():
                text.remove()
        
        # Add new text
        self.ax.text2D(0.02, 0.98, info_text, transform=self.ax.transAxes, 
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    def start_visualization(self):
        """Start the visualization"""
        plt.ion()  # Interactive mode
        plt.show()
        
        # Set equal aspect ratio
        self.ax.set_xlim(self.room_bounds[0], self.room_bounds[1])
        self.ax.set_ylim(self.room_bounds[2], self.room_bounds[3])
        self.ax.set_zlim(0, 2500)
        
        # Add legend
        self.ax.legend()
        
        print("3D Visualization started! Robot will appear as blue rectangular body with green heading arrow.")
        print("Red cylinders represent obstacles. Yellow cone shows sonar range.")
        print("Green dashed line shows triangle pattern being followed.")
    
    def update_from_file(self, log_file_path):
        """Update visualization from log file"""
        try:
            if os.path.exists(log_file_path):
                with open(log_file_path, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        # Get last line
                        last_line = lines[-1].strip()
                        if last_line and not last_line.startswith('#'):
                            parts = last_line.split()
                            if len(parts) >= 6:
                                x = float(parts[1])
                                y = float(parts[2])
                                heading = float(parts[3])
                                battery = float(parts[4])
                                sonar = float(parts[5])
                                
                                self.update_robot_position(x, y, heading, battery, sonar)
                                plt.draw()
                                plt.pause(0.1)
        except Exception as e:
            print(f"Error reading log file: {e}")

def main():
    """Main function"""
    print("=== Pioneer 3DX Robot 3D Visualizer ===")
    print("This visualizer shows:")
    print("- Blue robot body with green heading arrow")
    print("- Red cylindrical obstacles")
    print("- Yellow sonar detection cone")
    print("- Green dashed triangle pattern")
    print("- Real-time position and sensor data")
    print()
    
    visualizer = Robot3DVisualizer()
    visualizer.start_visualization()
    
    # Check for log file argument
    log_file = "examples/telemetry.log"
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    
    print(f"Monitoring log file: {log_file}")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            visualizer.update_from_file(log_file)
            time.sleep(0.1)  # Update every 100ms
    except KeyboardInterrupt:
        print("\nVisualization stopped.")
        plt.close()

if __name__ == "__main__":
    main()
