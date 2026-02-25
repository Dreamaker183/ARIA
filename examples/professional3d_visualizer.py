#!/usr/bin/env python3
"""
Professional 3D Visualizer for Multi-Robot Triangle Formation
Real-time visualization with ultrasonic sensor fusion, occupancy mapping,
formation control, and professional robotics visualization layers.

Features:
- Robot meshes and coordinate frames
- Ultrasonic point cloud visualization
- Occupancy grid with inflated obstacles
- Triangle formation vertices and paths
- Real-time trajectory tracking
- Professional HUD with system status
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
import math
from collections import deque

class Professional3DVisualizer:
    def __init__(self):
        self.fig = plt.figure(figsize=(16, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Robot data
        self.robot1_data = {
            'position': deque(maxlen=100),
            'heading': deque(maxlen=100),
            'trajectory': deque(maxlen=200),
            'sonar_points': deque(maxlen=500),
            'target': None,
            'connected': False
        }
        
        self.robot2_data = {
            'position': deque(maxlen=100),
            'heading': deque(maxlen=100),
            'trajectory': deque(maxlen=200),
            'sonar_points': deque(maxlen=500),
            'target': None,
            'connected': False
        }
        
        # Formation data
        self.triangle_vertices = []
        self.triangle_centroid = None
        self.staging_area = None
        self.occupancy_grid = None
        
        # System state
        self.current_phase = "INITIALIZING"
        self.cycle_running = False
        self.formation_stable = False
        self.safety_violations = []
        
        # Visualization settings
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Height (mm)')
        self.ax.set_title('Professional Multi-Robot Triangle Formation System')
        
        # Set room boundaries (10m x 10m x 3m)
        self.room_bounds = [-5000, 5000, -5000, 5000, 0, 3000]
        self.setup_room()
        
        # Data queue for real-time updates
        self.data_queue = queue.Queue()
        self.running = True
        
        # Professional visualization layers
        self.setup_visualization_layers()
        
    def setup_room(self):
        """Setup professional room environment with obstacles"""
        # Room floor
        x_room = [self.room_bounds[0], self.room_bounds[1], self.room_bounds[1], self.room_bounds[0], self.room_bounds[0]]
        y_room = [self.room_bounds[2], self.room_bounds[2], self.room_bounds[3], self.room_bounds[3], self.room_bounds[2]]
        z_room = [0, 0, 0, 0, 0]
        self.ax.plot(x_room, y_room, z_room, 'k-', linewidth=3, label='Room Floor', alpha=0.8)
        
        # Room walls (vertical lines)
        wall_height = 2500  # 2.5m walls
        for i in range(len(x_room)-1):
            self.ax.plot([x_room[i], x_room[i]], [y_room[i], y_room[i]], [0, wall_height], 'k-', linewidth=3, alpha=0.8)
        
        # Add professional obstacles (red cylinders with safety margins)
        self.add_obstacle(2000, 1500, 400, 600, "Obstacle 1")  # Large obstacle
        self.add_obstacle(-1800, 2000, 300, 500, "Obstacle 2")  # Medium obstacle
        self.add_obstacle(1000, -2000, 500, 700, "Obstacle 3")  # Large obstacle
        self.add_obstacle(-2500, -1500, 350, 550, "Obstacle 4") # Medium obstacle
        self.add_obstacle(0, 0, 200, 400, "Central Obstacle")   # Small central obstacle
        
    def add_obstacle(self, x, y, radius, height, name):
        """Add a professional cylindrical obstacle with safety margins"""
        # Create cylinder with safety inflation
        safety_radius = radius + 150  # 150mm safety margin
        
        theta = np.linspace(0, 2*np.pi, 30)
        z = np.linspace(0, height, 15)
        theta_grid, z_grid = np.meshgrid(theta, z)
        
        # Main obstacle (dark red)
        x_cyl = x + radius * np.cos(theta_grid)
        y_cyl = y + radius * np.sin(theta_grid)
        self.ax.plot_surface(x_cyl, y_cyl, z_grid, color='darkred', alpha=0.8, label=name)
        
        # Safety margin (light red)
        x_safety = x + safety_radius * np.cos(theta_grid)
        y_safety = y + safety_radius * np.sin(theta_grid)
        self.ax.plot_surface(x_safety, y_safety, z_grid, color='red', alpha=0.3)
        
        # Add top and bottom circles
        theta_circle = np.linspace(0, 2*np.pi, 50)
        x_circle = x + safety_radius * np.cos(theta_circle)
        y_circle = y + safety_radius * np.sin(theta_circle)
        
        # Bottom circle (safety margin)
        self.ax.plot(x_circle, y_circle, np.zeros_like(x_circle), 'r-', linewidth=2, alpha=0.6)
        # Top circle (safety margin)
        self.ax.plot(x_circle, y_circle, np.full_like(x_circle, height), 'r-', linewidth=2, alpha=0.6)
        
    def setup_visualization_layers(self):
        """Setup professional visualization layers"""
        # Initialize empty plots for dynamic updates
        self.robot1_plot, = self.ax.plot([], [], [], 'bo-', linewidth=2, label='Robot 1', markersize=8)
        self.robot2_plot, = self.ax.plot([], [], [], 'go-', linewidth=2, label='Robot 2', markersize=8)
        self.robot1_trajectory, = self.ax.plot([], [], [], 'b-', linewidth=1, alpha=0.6, label='Robot 1 Path')
        self.robot2_trajectory, = self.ax.plot([], [], [], 'g-', linewidth=1, alpha=0.6, label='Robot 2 Path')
        self.triangle_plot, = self.ax.plot([], [], [], 'm--', linewidth=3, alpha=0.8, label='Triangle Formation')
        self.sonar_cloud, = self.ax.plot([], [], [], 'y.', markersize=2, alpha=0.7, label='Ultrasonic Points')
        self.occupancy_plot = None
        
    def update_robot_data(self, robot_id, x, y, theta, sonar_data, target=None, connected=True):
        """Update robot data for visualization"""
        robot_data = self.robot1_data if robot_id == 1 else self.robot2_data
        
        robot_data['position'].append([x, y, 50])  # Robot at 50mm height
        robot_data['heading'].append(theta)
        robot_data['trajectory'].append([x, y, 50])
        robot_data['target'] = target
        robot_data['connected'] = connected
        
        # Process sonar data
        if sonar_data:
            for i, range_val in enumerate(sonar_data):
                if range_val < 5000.0:  # Valid reading
                    angle = (i * 22.5) - 180.0  # Sensor angles
                    sonar_x = x + range_val * math.cos((theta + angle) * math.pi / 180.0)
                    sonar_y = y + range_val * math.sin((theta + angle) * math.pi / 180.0)
                    robot_data['sonar_points'].append([sonar_x, sonar_y, 50])
        
        # Update visualization
        self.update_robot_visualization(robot_id)
        
    def update_robot_visualization(self, robot_id):
        """Update robot visualization elements"""
        robot_data = self.robot1_data if robot_id == 1 else self.robot2_data
        
        if not robot_data['position']:
            return
            
        # Get current position
        pos = robot_data['position'][-1]
        heading = robot_data['heading'][-1]
        
        # Draw robot body (rectangular)
        robot_length = 400  # mm
        robot_width = 300   # mm
        robot_height = 200  # mm
        
        # Calculate robot corners
        cos_h = math.cos(math.radians(heading))
        sin_h = math.sin(math.radians(heading))
        
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
            x_rot = corner[0] * cos_h - corner[1] * sin_h
            y_rot = corner[0] * sin_h + corner[1] * cos_h
            rotated_corners.append([pos[0] + x_rot, pos[1] + y_rot, pos[2] + corner[2]])
        
        rotated_corners = np.array(rotated_corners)
        
        # Draw robot body
        color = 'blue' if robot_id == 1 else 'green'
        self.draw_rectangular_prism(rotated_corners, color, alpha=0.8)
        
        # Draw heading arrow
        arrow_length = 300
        arrow_x = pos[0] + arrow_length * cos_h
        arrow_y = pos[1] + arrow_length * sin_h
        self.ax.plot([pos[0], arrow_x], [pos[1], arrow_y], [pos[2] + robot_height/2, pos[2] + robot_height/2], 
                    color=color, linewidth=4, alpha=0.9)
        
        # Draw sonar range cone
        self.draw_sonar_cone(pos, heading, 1000, color)
        
        # Update trajectory
        if len(robot_data['trajectory']) > 1:
            traj = np.array(robot_data['trajectory'])
            if robot_id == 1:
                self.robot1_trajectory.set_data_3d(traj[:, 0], traj[:, 1], traj[:, 2])
            else:
                self.robot2_trajectory.set_data_3d(traj[:, 0], traj[:, 1], traj[:, 2])
        
        # Draw target if available
        if robot_data['target']:
            target_pos = robot_data['target']
            self.ax.scatter([target_pos[0]], [target_pos[1]], [target_pos[2]], 
                          color=color, marker='*', s=200, alpha=0.8)
    
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
    
    def draw_sonar_cone(self, pos, heading, range_mm, color):
        """Draw sonar detection cone"""
        cone_angle = 45  # degrees
        cone_height = 100
        
        # Create cone points
        theta = np.linspace(math.radians(heading - cone_angle/2), 
                           math.radians(heading + cone_angle/2), 20)
        
        for i in range(len(theta)-1):
            x1 = pos[0] + range_mm * math.cos(theta[i])
            y1 = pos[1] + range_mm * math.sin(theta[i])
            x2 = pos[0] + range_mm * math.cos(theta[i+1])
            y2 = pos[1] + range_mm * math.sin(theta[i+1])
            
            # Draw cone surface
            self.ax.plot([pos[0], x1, x2], [pos[1], y1, y2], 
                        [pos[2] + cone_height, pos[2], pos[2]], 
                        color=color, alpha=0.3, linewidth=1)
    
    def update_triangle_formation(self, vertices, centroid):
        """Update triangle formation visualization"""
        self.triangle_vertices = vertices
        self.triangle_centroid = centroid
        
        if len(vertices) >= 3:
            # Draw triangle outline
            triangle_x = [v[0] for v in vertices] + [vertices[0][0]]
            triangle_y = [v[1] for v in vertices] + [vertices[0][1]]
            triangle_z = [v[2] for v in vertices] + [vertices[0][2]]
            
            self.triangle_plot.set_data_3d(triangle_x, triangle_y, triangle_z)
            
            # Draw centroid
            if centroid:
                self.ax.scatter([centroid[0]], [centroid[1]], [centroid[2]], 
                              color='purple', marker='o', s=100, alpha=0.8)
    
    def update_occupancy_grid(self, grid_data):
        """Update occupancy grid visualization"""
        self.occupancy_grid = grid_data
        
        # Clear previous occupancy plot
        if self.occupancy_plot:
            self.occupancy_plot.remove()
        
        # Create occupancy visualization
        if grid_data is not None:
            # Convert grid to 3D visualization
            occupied_points = []
            for y in range(len(grid_data)):
                for x in range(len(grid_data[y])):
                    if grid_data[y][x] > 0.5:  # Occupied cell
                        world_x = -5000 + x * 50  # Convert to world coordinates
                        world_y = -5000 + y * 50
                        occupied_points.append([world_x, world_y, 0])
            
            if occupied_points:
                occupied_points = np.array(occupied_points)
                self.occupancy_plot = self.ax.scatter(occupied_points[:, 0], occupied_points[:, 1], 
                                                    occupied_points[:, 2], c='red', alpha=0.3, s=1)
    
    def update_system_status(self, phase, cycle_running, formation_stable, violations=None):
        """Update system status display"""
        self.current_phase = phase
        self.cycle_running = cycle_running
        self.formation_stable = formation_stable
        if violations:
            self.safety_violations = violations
        
        # Update status HUD
        self.update_status_hud()
    
    def update_status_hud(self):
        """Update professional status HUD"""
        # Clear previous text
        for text in self.ax.texts:
            if 'System Status:' in text.get_text():
                text.remove()
        
        # Create status text
        status_text = f"""System Status: {self.current_phase}
Cycle Running: {'Yes' if self.cycle_running else 'No'}
Formation Stable: {'Yes' if self.formation_stable else 'No'}
Robot 1: {'Connected' if self.robot1_data['connected'] else 'Disconnected'}
Robot 2: {'Connected' if self.robot2_data['connected'] else 'Disconnected'}
Safety Violations: {len(self.safety_violations)}"""
        
        # Add status text
        self.ax.text2D(0.02, 0.98, status_text, transform=self.ax.transAxes, 
                      verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8),
                      fontsize=10, fontweight='bold')
    
    def start_visualization(self):
        """Start the professional 3D visualization"""
        plt.ion()  # Interactive mode
        plt.show()
        
        # Set equal aspect ratio
        self.ax.set_xlim(self.room_bounds[0], self.room_bounds[1])
        self.ax.set_ylim(self.room_bounds[2], self.room_bounds[3])
        self.ax.set_zlim(self.room_bounds[4], self.room_bounds[5])
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        print("Professional 3D Visualization started!")
        print("Features:")
        print("- Blue/Green robots with heading arrows")
        print("- Red obstacles with safety margins")
        print("- Yellow sonar detection cones")
        print("- Magenta triangle formation")
        print("- Real-time trajectory tracking")
        print("- Professional status HUD")
    
    def update_from_file(self, log_file_path):
        """Update visualization from log file"""
        try:
            if os.path.exists(log_file_path):
                with open(log_file_path, 'r') as f:
                    lines = f.readlines()
                    if lines:
                        # Process last line
                        last_line = lines[-1].strip()
                        if last_line and not last_line.startswith('#'):
                            self.process_log_line(last_line)
                            
        except Exception as e:
            print(f"Error reading log file: {e}")
    
    def process_log_line(self, line):
        """Process a single log line for visualization updates"""
        try:
            parts = line.split()
            if len(parts) >= 8:
                # Parse log data (format: time x y theta battery sonar phase cycle_running)
                x = float(parts[1])
                y = float(parts[2])
                theta = float(parts[3])
                battery = float(parts[4])
                sonar = float(parts[5])
                phase = parts[6]
                cycle_running = parts[7] == 'True'
                
                # Update robot data (assuming this is robot 1 for now)
                sonar_data = [sonar] * 16  # Simplified sonar data
                self.update_robot_data(1, x, y, theta, sonar_data, connected=True)
                
                # Update system status
                self.update_system_status(phase, cycle_running, False)
                
                # Redraw
                plt.draw()
                plt.pause(0.01)
                
        except Exception as e:
            print(f"Error processing log line: {e}")

def main():
    """Main function"""
    print("=== Professional Multi-Robot Triangle Formation 3D Visualizer ===")
    print("Advanced visualization with:")
    print("- Robot meshes and coordinate frames")
    print("- Ultrasonic point cloud visualization")
    print("- Occupancy grid with inflated obstacles")
    print("- Triangle formation vertices and paths")
    print("- Real-time trajectory tracking")
    print("- Professional status HUD")
    print()
    
    visualizer = Professional3DVisualizer()
    visualizer.start_visualization()
    
    # Check for log file argument
    log_file = "examples/formation_telemetry.log"
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    
    print(f"Monitoring log file: {log_file}")
    print("Press Ctrl+C to stop...")
    
    try:
        while True:
            visualizer.update_from_file(log_file)
            time.sleep(0.1)  # Update every 100ms
    except KeyboardInterrupt:
        print("\nProfessional visualization stopped.")
        plt.close()

if __name__ == "__main__":
    main()
