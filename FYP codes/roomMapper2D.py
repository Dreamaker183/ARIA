#!/usr/bin/env python3
"""
Live 2D Room Mapper for Pioneer 3DX Swarm
Listens on UDP Port 50000 for RobotStateMsg structs.
Builds a persistent obstacle map from sonar readings.
Shows robots with directions, paths, and detected obstacles.
"""

import socket
import struct
import threading
import time
import math
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import defaultdict

# C++ struct: { int robot_id, int reserved, double x, y, theta, v, w, double sonar[8] }
MSG_FORMAT = '2i13d'
MSG_SIZE = struct.calcsize(MSG_FORMAT)

# P3DX sonar array angles (degrees relative to robot front)
# Front 8 sonars: approximately these angles
SONAR_ANGLES = [-90, -50, -30, -10, 10, 30, 50, 90]

# Colors for each robot
ROBOT_COLORS = {1: '#FF3B30', 2: '#007AFF', 3: '#34C759'}  # Red, Blue, Green
ROBOT_NAMES = {1: 'Robot 1', 2: 'Robot 2', 3: 'Robot 3'}


class RoomMapper2D:
    def __init__(self, port=50000):
        self.port = port
        self.running = True
        self.lock = threading.Lock()

        # Robot state
        self.robots = {}
        self.last_seen = {}
        self.trajectories = defaultdict(list)
        self.max_trail = 500  # Keep last N path points

        # Persistent obstacle map — points accumulate over time
        self.obstacle_points = []  # List of (x, y) tuples
        self.max_obstacles = 5000  # Cap to prevent memory issues

        # Setup matplotlib
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(1, 1, figsize=(12, 10))
        self.fig.canvas.manager.set_window_title('2D Room Mapper — Swarm Robot Controller')

        # Plot elements (initialized empty)
        self.obstacle_scatter = None
        self.robot_markers = {}
        self.robot_arrows = {}
        self.trail_lines = {}
        self.sonar_lines = {}
        self.status_text = None

        self._setup_plot()

    def _setup_plot(self):
        """Initial plot setup"""
        self.ax.set_facecolor('#1a1a2e')
        self.fig.set_facecolor('#0f0f1a')

        self.ax.set_xlabel('X (mm)', color='white', fontsize=12)
        self.ax.set_ylabel('Y (mm)', color='white', fontsize=12)
        self.ax.set_title('🗺️  Live 2D Room Map — Sonar Obstacle Detection', 
                          color='white', fontsize=14, fontweight='bold', pad=15)

        self.ax.set_xlim(-3000, 3000)
        self.ax.set_ylim(-3000, 3000)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.15, color='white', linestyle='--')

        # Origin marker
        self.ax.plot(0, 0, '+', color='white', markersize=15, alpha=0.3)

        # Status HUD
        self.status_text = self.ax.text(
            0.02, 0.98, 'Waiting for robot data...', 
            transform=self.ax.transAxes, fontsize=10,
            verticalalignment='top', color='white',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#16213e', alpha=0.9),
            family='monospace'
        )

        # Legend
        for rid, color in ROBOT_COLORS.items():
            self.ax.plot([], [], 'o', color=color, label=ROBOT_NAMES[rid], markersize=8)
        self.ax.plot([], [], 's', color='#FFD60A', label='Obstacles', markersize=6, alpha=0.7)
        self.ax.legend(loc='upper right', fontsize=9, framealpha=0.8, 
                       facecolor='#16213e', edgecolor='#444')

    def udp_listener(self):
        """Background thread: receive UDP telemetry"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        sock.settimeout(1.0)
        sock.bind(('', self.port))
        print(f"📡 Room Mapper listening on UDP port {self.port}...")

        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == MSG_SIZE:
                    unpacked = struct.unpack(MSG_FORMAT, data)
                    r_id = unpacked[0]
                    x, y, th = unpacked[2], unpacked[3], unpacked[4]
                    v, w = unpacked[5], unpacked[6]
                    sonars = list(unpacked[7:15])

                    with self.lock:
                        self.robots[r_id] = {
                            'x': x, 'y': y, 'th': th,
                            'v': v, 'w': w, 'sonar': sonars
                        }
                        self.last_seen[r_id] = time.time()

                        # Accumulate trajectory
                        self.trajectories[r_id].append((x, y))
                        if len(self.trajectories[r_id]) > self.max_trail:
                            self.trajectories[r_id] = self.trajectories[r_id][-self.max_trail:]

                        # Convert sonar hits to world coordinates and add to obstacle map
                        for i, dist in enumerate(sonars):
                            if 50 < dist < 4500:  # Valid sonar reading
                                angle_rad = math.radians(th + SONAR_ANGLES[i])
                                hit_x = x + dist * math.cos(angle_rad)
                                hit_y = y + dist * math.sin(angle_rad)
                                self.obstacle_points.append((hit_x, hit_y))

                        # Cap obstacle points
                        if len(self.obstacle_points) > self.max_obstacles:
                            self.obstacle_points = self.obstacle_points[-self.max_obstacles:]

            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"UDP Error: {e}")
                time.sleep(0.1)

        sock.close()

    def _update_frame(self, frame):
        """Called by FuncAnimation to update the plot"""
        artists = []

        with self.lock:
            now = time.time()
            active_robots = {rid: state for rid, state in self.robots.items()
                             if now - self.last_seen.get(rid, 0) < 3.0}

            # --- Draw obstacles ---
            if self.obstacle_scatter is not None:
                self.obstacle_scatter.remove()
                self.obstacle_scatter = None

            if self.obstacle_points:
                obs = np.array(self.obstacle_points)
                self.obstacle_scatter = self.ax.scatter(
                    obs[:, 0], obs[:, 1], 
                    c='#FFD60A', s=3, alpha=0.5, marker='s', zorder=2
                )
                artists.append(self.obstacle_scatter)

            # --- Clear old robot elements ---
            for key in list(self.robot_markers.keys()):
                try: self.robot_markers[key].remove()
                except: pass
            for key in list(self.robot_arrows.keys()):
                try: self.robot_arrows[key].remove()
                except: pass
            for key in list(self.trail_lines.keys()):
                try: self.trail_lines[key].remove()
                except: pass
            for key in list(self.sonar_lines.keys()):
                for line in self.sonar_lines[key]:
                    try: line.remove()
                    except: pass
            self.robot_markers.clear()
            self.robot_arrows.clear()
            self.trail_lines.clear()
            self.sonar_lines.clear()

            # --- Draw each active robot ---
            for rid, state in active_robots.items():
                x, y, th = state['x'], state['y'], state['th']
                color = ROBOT_COLORS.get(rid, '#FFFFFF')

                # Robot position marker
                marker = self.ax.plot(x, y, 'o', color=color, markersize=12,
                                      markeredgecolor='white', markeredgewidth=1.5, zorder=5)[0]
                self.robot_markers[rid] = marker

                # Direction arrow
                arrow_len = 300
                dx = arrow_len * math.cos(math.radians(th))
                dy = arrow_len * math.sin(math.radians(th))
                arrow = self.ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                                         arrowprops=dict(arrowstyle='->', color=color, lw=2.5),
                                         zorder=6)
                self.robot_arrows[rid] = arrow

                # Robot label
                self.ax.annotate(f'R{rid}', (x, y), textcoords="offset points",
                                 xytext=(10, 10), fontsize=8, color=color, fontweight='bold',
                                 zorder=7)

                # Path trail
                trail = self.trajectories.get(rid, [])
                if len(trail) > 1:
                    tx = [p[0] for p in trail]
                    ty = [p[1] for p in trail]
                    line = self.ax.plot(tx, ty, '-', color=color, linewidth=1, alpha=0.4, zorder=3)[0]
                    self.trail_lines[rid] = line

                # Live sonar beams
                self.sonar_lines[rid] = []
                for i, dist in enumerate(state['sonar']):
                    if dist > 0 and dist < 5000:
                        angle_rad = math.radians(th + SONAR_ANGLES[i])
                        end_x = x + dist * math.cos(angle_rad)
                        end_y = y + dist * math.sin(angle_rad)
                        beam_color = '#FF6B6B' if dist < 250 else '#4ECDC4'
                        beam_alpha = 0.6 if dist < 250 else 0.2
                        beam = self.ax.plot([x, end_x], [y, end_y], '-', 
                                            color=beam_color, linewidth=0.8, alpha=beam_alpha, zorder=4)[0]
                        self.sonar_lines[rid].append(beam)

            # --- Auto-scale to fit all data ---
            all_x, all_y = [], []
            for rid, state in active_robots.items():
                all_x.append(state['x'])
                all_y.append(state['y'])
            if self.obstacle_points:
                obs = np.array(self.obstacle_points)
                all_x.extend(obs[:, 0].tolist())
                all_y.extend(obs[:, 1].tolist())

            if all_x and all_y:
                margin = 1000
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                # Keep aspect ratio square
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range, 2000)
                cx = (x_min + x_max) / 2
                cy = (y_min + y_max) / 2
                self.ax.set_xlim(cx - max_range / 2, cx + max_range / 2)
                self.ax.set_ylim(cy - max_range / 2, cy + max_range / 2)

            # --- Update HUD ---
            hud = "══ ROOM MAPPER ══\n"
            hud += f"Robots: {len(active_robots)}/3\n"
            hud += f"Obstacles: {len(self.obstacle_points)}\n"
            for rid in sorted(active_robots.keys()):
                s = active_robots[rid]
                hud += f"R{rid}: ({s['x']:.0f}, {s['y']:.0f}) θ={s['th']:.0f}°\n"
            self.status_text.set_text(hud)

        return artists

    def run(self):
        """Start the visualizer"""
        # Start UDP listener
        listener = threading.Thread(target=self.udp_listener, daemon=True)
        listener.start()

        # Start animation
        self.anim = FuncAnimation(self.fig, self._update_frame, interval=100, blit=False, cache_frame_data=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            print("Room Mapper closed.")


if __name__ == "__main__":
    mapper = RoomMapper2D()
    mapper.run()
