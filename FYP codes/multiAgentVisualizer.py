#!/usr/bin/env python3
"""
Live 3D Networked Robot Visualizer for Pioneer 3DX Swarm
Listens on UDP Port 50000 for RobotStateMsg structs to render the entire multi-agent system.
Displays Wi-Fi links, Triangle/Line/Circle formations, Sonar point clouds, and live positioning.
"""

import socket
import struct
import threading
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import defaultdict

# The C++ Struct equivalent we are catching over UDP:
# struct RobotStateMsg {
#     int robot_id;
#     int formation_type;
#     double x;
#     double y;
#     double theta;
#     double v;
#     double w;
#     double sonar[8];
# };
# Pack format matches: 2 integers, followed by 5+8 = 13 doubles = '2i 13d'
MSG_FORMAT = '2i13d'
MSG_SIZE = struct.calcsize(MSG_FORMAT)

class Swarm3DVisualizer:
    def __init__(self, port=50000):
        self.port = port
        self.running = True
        self.lock = threading.Lock()
        
        # State tracking per robot
        self.robots = {}
        self.last_seen = {}
        self.trajectories = defaultdict(list)
        
        # Formations
        self.SCATTER = 0
        self.TRIANGLE = 1
        self.CIRCLE = 2
        self.LINE = 3
        
        # Room Config
        self.room_bounds = [-5000, 5000, -5000, 5000, 0, 3000]
        
        # Initialize Matplotlib
        self.fig = plt.figure(figsize=(14, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Height (mm)')
        self.ax.set_title('Live Swarm Multi-Agent Visualizer')
        
        # Disable auto-scaling so the room stays fixed
        self.ax.set_xlim(self.room_bounds[0], self.room_bounds[1])
        self.ax.set_ylim(self.room_bounds[2], self.room_bounds[3])
        self.ax.set_zlim(self.room_bounds[4], self.room_bounds[5])
        
        # Sub-objects
        self.robot_plots = {}
        self.sonar_scatter = None
        self.wifi_lines = None
        self.formation_line = None
        
        self.setup_room()
        
    def setup_room(self):
        """Draw room boundaries"""
        x_r = [self.room_bounds[0], self.room_bounds[1], self.room_bounds[1], self.room_bounds[0], self.room_bounds[0]]
        y_r = [self.room_bounds[2], self.room_bounds[2], self.room_bounds[3], self.room_bounds[3], self.room_bounds[2]]
        z_r = [0, 0, 0, 0, 0]
        self.ax.plot(x_r, y_r, z_r, 'k-', linewidth=2, alpha=0.5, label='Room Floor')
        
        # Helper text HUD
        self.hud = self.ax.text2D(0.02, 0.98, "Initializing UDP UDP Listener...", 
                                  transform=self.ax.transAxes, verticalalignment='top', 
                                  bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
        
    def udp_listener(self):
        """Background thread catching UDP packets"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # MacOS specific reuseport
        try:
             sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
             pass

        sock.bind(('', self.port))
        print(f"📡 Listening for Swarm UDP Broadcasts on Port {self.port}...")
        
        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == MSG_SIZE:
                    unpacked = struct.unpack(MSG_FORMAT, data)
                    r_id = unpacked[0]
                    f_type = unpacked[1]
                    x, y, th, v, w = unpacked[2:7]
                    sonars = unpacked[7:]
                    
                    with self.lock:
                        self.robots[r_id] = {
                            'id': r_id, 'form': f_type,
                            'x': x, 'y': y, 'th': th,
                            'v': v, 'w': w, 'sonar': sonars
                        }
                        self.last_seen[r_id] = time.time()
                        
                        # Save trajectory points
                        self.trajectories[r_id].append([x, y])
                        if len(self.trajectories[r_id]) > 50:
                            self.trajectories[r_id].pop(0)
                        
            except Exception as e:
                print(f"UDP Error: {e}")
                time.sleep(0.1)

    def draw_robot_cube(self, rid, state):
        """Create and place 3D surfaces simulating the robots body"""
        x, y, th = state['x'], state['y'], state['th']
        
        # Dimensions matching Pioneer
        length, width, height = 400, 300, 200 
        
        # Center coordinates
        c_th = math.cos(math.radians(th))
        s_th = math.sin(math.radians(th))
        
        # Color dynamically based on ID (to tell them apart easily)
        colors = ['red', 'blue', 'green', 'purple', 'orange']
        clr = colors[(rid-1) % len(colors)]
        
        # If already drawn, we must clear old meshes
        if rid in self.robot_plots:
            for coll in self.robot_plots[rid]:
                try: coll.remove()
                except: pass
        
        self.robot_plots[rid] = []
        
        # Draw Arrow
        arrow_len = 350
        arr_x = x + arrow_len * c_th
        arr_y = y + arrow_len * s_th
        line, = self.ax.plot([x, arr_x], [y, arr_y], [height, height], color=clr, linewidth=4)
        self.robot_plots[rid].append(line)
        
        # Draw small cube dot representation
        dot = self.ax.scatter([x], [y], [height/2], color=clr, s=200, marker='s')
        self.robot_plots[rid].append(dot)

    def draw_sonar_cloud(self):
        """Render all valid sonar hits from all active robots"""
        # Clear old scatter
        if self.sonar_scatter is not None:
             try: self.sonar_scatter.remove()
             except: pass
             
        sx, sy, sz = [], [], []
        
        with self.lock:
             for r_id, state in self.robots.items():
                 # Ignore dead robots
                 if time.time() - self.last_seen[r_id] > 2.0:
                     continue
                 
                 x, y, th = state['x'], state['y'], state['th']
                 
                 # 8 Front Sonars are usually arranged from -90 to +90 degrees relative to front
                 # P3DX Array: 0=-90, 1=-50, 2=-30, 3=-10, 4=10, 5=30, 6=50, 7=90 (approx)
                 angles = [-90, -50, -30, -10, 10, 30, 50, 90]
                 
                 for i in range(8):
                     dist = state['sonar'][i]
                     if dist < 4900.0:  # Only plot if it bounced off something
                         abs_ang = math.radians(th + angles[i])
                         hit_x = x + dist * math.cos(abs_ang)
                         hit_y = y + dist * math.sin(abs_ang)
                         sx.append(hit_x)
                         sy.append(hit_y)
                         sz.append(50)  # low height for sonars
                         
        if len(sx) > 0:
             self.sonar_scatter = self.ax.scatter(sx, sy, sz, color='gold', marker='.', s=10, alpha=0.6)

    def draw_wifi_connections(self):
        """Draw lines connecting robots to show they are talking"""
        if self.wifi_lines is not None:
             for line in self.wifi_lines:
                 try: line.remove()
                 except: pass
        self.wifi_lines = []
        
        active_bots = []
        with self.lock:
             for r_id, state in self.robots.items():
                 if time.time() - self.last_seen[r_id] < 2.0:
                      active_bots.append((state['x'], state['y']))
                      
        if len(active_bots) > 1:
             for i in range(len(active_bots)):
                 for j in range(i+1, len(active_bots)):
                      # Draw line
                      p1 = active_bots[i]
                      p2 = active_bots[j]
                      line, = self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [200, 200], 
                                           color='cyan', linestyle=':', linewidth=1, alpha=0.4)
                      self.wifi_lines.append(line)

    def draw_formations(self):
        """Draw the ideal formation polygon connecting the robots if active"""
        if self.formation_line is not None:
             try: self.formation_line.remove()
             except: pass
             self.formation_line = None
             
        active = []
        fmt = self.SCATTER
        with self.lock:
             for r_id, state in self.robots.items():
                 if time.time() - self.last_seen[r_id] < 2.0:
                      active.append([state['x'], state['y'], 100])
                      fmt = state['form'] # Just grab the first one we see, they should be consensus
                      
        if len(active) >= 2 and fmt != self.SCATTER:
             pts = np.array(active)
             if fmt == self.TRIANGLE and len(pts) >= 3:
                 # Draw polygon enclosing them
                 pts = np.vstack((pts, pts[0])) # close loop
                 self.formation_line, = self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color='magenta', linewidth=2, linestyle='--')
             elif fmt == self.CIRCLE and len(pts) >= 3:
                 self.formation_line, = self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color='magenta', linewidth=2, linestyle='--')
             elif fmt == self.LINE:
                 self.formation_line, = self.ax.plot(pts[:,0], pts[:,1], pts[:,2], color='magenta', linewidth=2, linestyle='-')

    def render_loop(self):
        """Main Matplotlib Render Update Loop"""
        plt.ion()
        plt.show(block=False)
        
        form_dict = {0: "SCATTER", 1: "TRIANGLE", 2: "CIRCLE", 3: "LINE-UP"}
        
        while self.running:
            try:
                active_count = 0
                form_text = "N/A"
                
                with self.lock:
                    for r_id, state in list(self.robots.items()):
                        # Check watchdog natively on GUI
                        if time.time() - self.last_seen[r_id] > 2.0:
                            continue
                        
                        active_count += 1
                        form_text = form_dict.get(state['form'], "UNKNOWN")
                        self.draw_robot_cube(r_id, state)
                
                self.draw_wifi_connections()
                self.draw_formations()
                self.draw_sonar_cloud()
                
                # Update HUD
                hud_txt = f"=== Live Swarm Telemetry ===\n"
                hud_txt += f"Active Robots: {active_count}\n"
                hud_txt += f"Consensus Status: {form_text}\n"
                hud_txt += f"Network: UDP {self.port} O.K."
                self.hud.set_text(hud_txt)
                
                # Apply limits so camera doesn't bounce constantly
                self.ax.set_xlim(self.room_bounds[0], self.room_bounds[1])
                self.ax.set_ylim(self.room_bounds[2], self.room_bounds[3])
                self.ax.set_zlim(self.room_bounds[4], self.room_bounds[5])
                
                plt.pause(0.05) # ~20 FPS render target
                
            except Exception as e:
                print(f"Render Error: {e}")
                time.sleep(1)

    def stop(self):
        self.running = False
        plt.close()

if __name__ == "__main__":
    app = Swarm3DVisualizer()
    t = threading.Thread(target=app.udp_listener, daemon=True)
    t.start()
    
    try:
        app.render_loop()
    except KeyboardInterrupt:
        print("\nClosing Visualizer...")
        app.stop()
