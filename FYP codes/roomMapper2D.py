#!/usr/bin/env python3
"""
Live 2D Room Mapper for Pioneer 3DX Swarm
Listens on UDP Port 50000 for RobotStateMsg structs.
Sends keyboard commands to C++ on UDP Port 50001.
Builds a persistent obstacle map from sonar readings.
Shows robots with directions, paths, warnings, and detected obstacles.
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
from matplotlib.widgets import Button
from matplotlib.animation import FuncAnimation
from collections import defaultdict

# C++ struct: { int robot_id, int boids_active, double x,y,theta,v,w, double battery, double sonar[8] }
MSG_FORMAT = '2i14d'
MSG_SIZE = struct.calcsize(MSG_FORMAT)

# P3DX sonar array angles (degrees relative to robot front)
SONAR_ANGLES = [90, 50, 30, 10, -10, -30, -50, -90]

# ============================================================
# CONFIGURATION
# ============================================================

ROBOT_OFFSETS = {
    1: (0, 0),
    2: (600, 0),
    3: (-600, 0),
}

BROKEN_SONAR_ROBOTS = {1}

OBSTACLE_MAX_RANGE = 300
OBSTACLE_MIN_RANGE = 50
OBSTACLE_DEDUP_RADIUS = 30
MAX_OBSTACLE_POINTS = 8000

BATTERY_LOW_THRESHOLD = 11.5   # Volts — warn below this

# Network
TELEMETRY_PORT = 50000
COMMAND_PORT = 50001

# ============================================================

ROBOT_COLORS = {1: '#FF3B30', 2: '#007AFF', 3: '#34C759'}
ROBOT_NAMES = {1: 'Robot 1 (no sonar)', 2: 'Robot 2', 3: 'Robot 3'}


class RoomMapper2D:
    def __init__(self):
        self.running = True
        self.lock = threading.Lock()

        # Command socket (send keys to C++)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_addr = ('127.0.0.1', COMMAND_PORT)

        # Robot state
        self.robots = {}
        self.last_seen = {}
        self.trajectories = defaultdict(list)
        self.max_trail = 1000

        # Battery & status tracking
        self.battery = {}        # {rid: voltage}
        self.boids_active = False

        # Persistent obstacle map
        self.obstacle_points = np.empty((0, 2), dtype=np.float64)

        # Warning state
        self.warnings = []       # list of (message, color, expiry_time)

        # Last key pressed (for visual feedback)
        self.last_key_label = ""
        self.last_key_time = 0

        # Setup matplotlib
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('2D Room Mapper — Swarm Robot Controller')

        # Control guide panel (left, narrow)
        self.ax_guide = self.fig.add_axes([0.01, 0.02, 0.18, 0.96])
        self.ax_guide.set_facecolor('#16213e')
        self.ax_guide.set_xticks([])
        self.ax_guide.set_yticks([])
        for spine in self.ax_guide.spines.values():
            spine.set_color('#333')

        # Map panel (right, main)
        self.ax = self.fig.add_axes([0.22, 0.05, 0.76, 0.90])

        # Plot elements
        self.obstacle_scatter = None
        self.robot_markers = {}
        self.robot_arrows = {}
        self.robot_labels = {}
        self.trail_lines = {}
        self.sonar_lines = {}
        self.status_text = None
        self.key_feedback_text = None
        self.warning_texts = []

        self._setup_guide()
        self._setup_map()

        # Connect keyboard events
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)

    def _setup_guide(self):
        """Draw the control guide panel with boids button"""
        ax = self.ax_guide
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)

        # Title
        ax.text(0.5, 0.97, 'CONTROLS', ha='center', va='top',
                fontsize=13, fontweight='bold', color='#FFD60A', family='monospace')
        ax.axhline(y=0.955, color='#FFD60A', alpha=0.3, linewidth=1)

        y = 0.93
        gap = 0.028

        def section(title, color='#4ECDC4'):
            nonlocal y
            y -= gap * 0.5
            ax.text(0.5, y, title, ha='center', va='top',
                    fontsize=9, fontweight='bold', color=color, family='monospace')
            y -= gap

        def entry(keys, desc, color='white'):
            nonlocal y
            ax.text(0.08, y, keys, ha='left', va='top',
                    fontsize=8, fontweight='bold', color='#FFD60A', family='monospace')
            ax.text(0.08, y - gap * 0.7, desc, ha='left', va='top',
                    fontsize=7, color=color, family='monospace', alpha=0.8)
            y -= gap * 1.8

        section('Robot 1 Only', '#FF3B30')
        entry('Arrow Keys', 'Move Robot 1')

        section('Robot 2 & 3', '#007AFF')
        entry('T / G', 'Forward / Backward')
        entry('F / H', 'Turn Left / Right')

        section('ALL Robots', '#34C759')
        entry('W / S', 'Forward / Backward')
        entry('A / D', 'Turn Left / Right')

        section('Control')
        entry('SPACE / X', 'STOP ALL robots')
        entry('Q', 'Quit program')

        section('Record & Play')
        entry('R', 'Start/Stop recording')
        entry('1 / 2 / 3', 'Play slot 1, 2, 3')
        entry('N', 'Save & rename slot')

        section('Autonomous', '#FF6B6B')
        entry('E', 'Toggle Boids Explore')

        # Boids button
        self.boids_btn_ax = self.fig.add_axes([0.03, 0.03, 0.14, 0.04])
        self.boids_btn = Button(self.boids_btn_ax, 'Boids Explore',
                                color='#2d2d44', hovercolor='#34C759')
        self.boids_btn.label.set_fontsize(9)
        self.boids_btn.label.set_color('white')
        self.boids_btn.on_clicked(self._on_boids_click)

        # Footer
        ax.text(0.5, 0.10, 'Click map first\nthen press keys', ha='center', va='bottom',
                fontsize=7, color='white', alpha=0.5, family='monospace', style='italic')

    def _on_boids_click(self, event):
        """Toggle boids mode via button click"""
        try:
            self.cmd_sock.sendto(b'e', self.cmd_addr)
        except Exception as e:
            print(f"Send error: {e}")
        self.last_key_label = "E: Toggle Boids Explore"
        self.last_key_time = time.time()

    def _setup_map(self):
        """Setup the main map panel"""
        self.ax.set_facecolor('#1a1a2e')
        self.fig.set_facecolor('#0f0f1a')

        self.ax.set_xlabel('X (mm)', color='white', fontsize=11)
        self.ax.set_ylabel('Y (mm)', color='white', fontsize=11)
        self.ax.set_title('Live 2D Room Map — Sonar Obstacle Detection',
                          color='white', fontsize=13, fontweight='bold', pad=10)

        self.ax.set_xlim(-3000, 3000)
        self.ax.set_ylim(-3000, 3000)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.15, color='white', linestyle='--')

        self.ax.plot(0, 0, '+', color='white', markersize=15, alpha=0.3)

        # Status HUD (top-left of map)
        self.status_text = self.ax.text(
            0.01, 0.99, 'Waiting for robot data...',
            transform=self.ax.transAxes, fontsize=9,
            verticalalignment='top', color='white',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#16213e', alpha=0.9),
            family='monospace'
        )

        # Key feedback (bottom-center of map)
        self.key_feedback_text = self.ax.text(
            0.5, 0.02, '',
            transform=self.ax.transAxes, fontsize=11,
            ha='center', va='bottom', color='#FFD60A', fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='#16213e', alpha=0.9),
            family='monospace'
        )

        # Legend
        for rid, color in ROBOT_COLORS.items():
            self.ax.plot([], [], 'o', color=color, label=ROBOT_NAMES[rid], markersize=8)
        self.ax.plot([], [], 's', color='#FFD60A', label='Obstacles', markersize=6, alpha=0.7)
        self.ax.legend(loc='upper right', fontsize=8, framealpha=0.8,
                       facecolor='#16213e', edgecolor='#444')

    def _on_key_press(self, event):
        """Handle keyboard events from matplotlib window"""
        key = event.key
        cmd = None
        label = None

        key_map = {
            'up': ('UP', '↑ ALL Forward'),
            'down': ('DOWN', '↓ ALL Backward'),
            'left': ('LEFT', '← ALL Turn Left'),
            'right': ('RIGHT', '→ ALL Turn Right'),
            'w': ('w', 'W: ALL Forward'),
            's': ('s', 'S: ALL Backward'),
            'a': ('a', 'A: ALL Turn Left'),
            'd': ('d', 'D: ALL Turn Right'),
            't': ('t', 'T: R2&R3 Fwd/Bwd'),
            'g': ('g', 'G: R2&R3 Bwd/Fwd'),
            'f': ('f', 'F: R2&R3 Turn'),
            'h': ('h', 'H: R2&R3 Turn'),
            ' ': ('SPACE', 'STOP ALL'),
            'x': ('x', 'STOP ALL'),
            'e': ('e', 'E: Toggle Boids Explore'),
            'r': ('r', 'R: Record'),
            '1': ('1', 'Play Slot 1'),
            '2': ('2', 'Play Slot 2'),
            '3': ('3', 'Play Slot 3'),
            'n': ('n', 'N: Save Recording'),
            'q': ('q', 'Q: Quit'),
        }

        if key in key_map:
            cmd, label = key_map[key]
            try:
                self.cmd_sock.sendto(cmd.encode(), self.cmd_addr)
            except Exception as e:
                print(f"Send error: {e}")

            self.last_key_label = label
            self.last_key_time = time.time()

    def _add_obstacle_point(self, hx, hy):
        """Add obstacle point with deduplication"""
        if len(self.obstacle_points) > 0:
            dists = np.sqrt((self.obstacle_points[:, 0] - hx)**2 +
                            (self.obstacle_points[:, 1] - hy)**2)
            if np.min(dists) < OBSTACLE_DEDUP_RADIUS:
                return

        self.obstacle_points = np.vstack([self.obstacle_points, [hx, hy]])

        if len(self.obstacle_points) > MAX_OBSTACLE_POINTS:
            self.obstacle_points = self.obstacle_points[-MAX_OBSTACLE_POINTS:]

    def udp_listener(self):
        """Background thread: receive UDP telemetry"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except AttributeError:
            pass
        sock.settimeout(1.0)
        sock.bind(('', TELEMETRY_PORT))
        print(f"📡 Room Mapper listening on UDP port {TELEMETRY_PORT}...")

        while self.running:
            try:
                data, addr = sock.recvfrom(1024)
                if len(data) == MSG_SIZE:
                    unpacked = struct.unpack(MSG_FORMAT, data)
                    r_id = unpacked[0]
                    boids_flag = unpacked[1]
                    x_raw, y_raw, th = unpacked[2], unpacked[3], unpacked[4]
                    v, w = unpacked[5], unpacked[6]
                    batt = unpacked[7]
                    sonars = list(unpacked[8:16])

                    ox, oy = ROBOT_OFFSETS.get(r_id, (0, 0))
                    x = x_raw + ox
                    y = y_raw + oy

                    with self.lock:
                        self.robots[r_id] = {
                            'x': x, 'y': y, 'th': th,
                            'v': v, 'w': w, 'sonar': sonars
                        }
                        self.last_seen[r_id] = time.time()
                        self.battery[r_id] = batt
                        self.boids_active = (boids_flag == 1)

                        self.trajectories[r_id].append((x, y))
                        if len(self.trajectories[r_id]) > self.max_trail:
                            self.trajectories[r_id] = self.trajectories[r_id][-self.max_trail:]

                        if r_id not in BROKEN_SONAR_ROBOTS:
                            for i, dist in enumerate(sonars):
                                if OBSTACLE_MIN_RANGE < dist < OBSTACLE_MAX_RANGE:
                                    angle_rad = math.radians(th + SONAR_ANGLES[i])
                                    hit_x = x + dist * math.cos(angle_rad)
                                    hit_y = y + dist * math.sin(angle_rad)
                                    self._add_obstacle_point(hit_x, hit_y)

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

            # --- Build warnings list ---
            current_warnings = []

            # Connection loss warnings
            for rid in self.robots:
                if now - self.last_seen.get(rid, 0) > 3.0:
                    current_warnings.append((f"⚠ Robot {rid} CONNECTION LOST!", '#FF3B30'))

            # Low battery warnings
            for rid, batt in self.battery.items():
                if 0 < batt < BATTERY_LOW_THRESHOLD:
                    current_warnings.append((f"🔋 Robot {rid} LOW BATTERY: {batt:.1f}V", '#FFD60A'))

            # Boids mode indicator
            if self.boids_active:
                current_warnings.append(("🤖 BOIDS EXPLORE MODE — Self-driving", '#34C759'))

            # --- Draw obstacles ---
            if self.obstacle_scatter is not None:
                self.obstacle_scatter.remove()
                self.obstacle_scatter = None

            if len(self.obstacle_points) > 0:
                self.obstacle_scatter = self.ax.scatter(
                    self.obstacle_points[:, 0], self.obstacle_points[:, 1],
                    c='#FFD60A', s=6, alpha=0.7, marker='s', zorder=2
                )
                artists.append(self.obstacle_scatter)

            # --- Clear old robot elements ---
            for key in list(self.robot_markers.keys()):
                try: self.robot_markers[key].remove()
                except: pass
            for key in list(self.robot_arrows.keys()):
                try: self.robot_arrows[key].remove()
                except: pass
            for key in list(self.robot_labels.keys()):
                try: self.robot_labels[key].remove()
                except: pass
            for key in list(self.trail_lines.keys()):
                try: self.trail_lines[key].remove()
                except: pass
            for key in list(self.sonar_lines.keys()):
                for line in self.sonar_lines[key]:
                    try: line.remove()
                    except: pass
            for wt in self.warning_texts:
                try: wt.remove()
                except: pass
            self.robot_markers.clear()
            self.robot_arrows.clear()
            self.robot_labels.clear()
            self.trail_lines.clear()
            self.sonar_lines.clear()
            self.warning_texts.clear()

            # --- Draw warnings at top of map ---
            for i, (msg, color) in enumerate(current_warnings):
                wt = self.ax.text(
                    0.5, 0.98 - i * 0.04, msg,
                    transform=self.ax.transAxes, fontsize=10, ha='center', va='top',
                    color='white', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor=color, alpha=0.9),
                    zorder=20
                )
                self.warning_texts.append(wt)

            # --- Draw each active robot ---
            for rid, state in active_robots.items():
                x, y, th = state['x'], state['y'], state['th']
                color = ROBOT_COLORS.get(rid, '#FFFFFF')

                marker = self.ax.plot(x, y, 'o', color=color, markersize=12,
                                      markeredgecolor='white', markeredgewidth=1.5, zorder=5)[0]
                self.robot_markers[rid] = marker

                arrow_len = 300
                dx = arrow_len * math.cos(math.radians(th))
                dy = arrow_len * math.sin(math.radians(th))
                arrow = self.ax.annotate('', xy=(x + dx, y + dy), xytext=(x, y),
                                         arrowprops=dict(arrowstyle='->', color=color, lw=2.5),
                                         zorder=6)
                self.robot_arrows[rid] = arrow

                # Label with warnings
                sonar_tag = " ⚠" if rid in BROKEN_SONAR_ROBOTS else ""
                batt_v = self.battery.get(rid, 0)
                batt_tag = f" {batt_v:.1f}V" if batt_v > 0 else ""
                batt_color = '#FF3B30' if (0 < batt_v < BATTERY_LOW_THRESHOLD) else color
                label = self.ax.annotate(f'R{rid}{sonar_tag}{batt_tag}', (x, y),
                                         textcoords="offset points",
                                         xytext=(10, 10), fontsize=9, color=batt_color,
                                         fontweight='bold', zorder=7)
                self.robot_labels[rid] = label

                trail = self.trajectories.get(rid, [])
                if len(trail) > 1:
                    tx = [p[0] for p in trail]
                    ty = [p[1] for p in trail]
                    line = self.ax.plot(tx, ty, '-', color=color, linewidth=2.5, alpha=0.8, zorder=3)[0]
                    self.trail_lines[rid] = line

                self.sonar_lines[rid] = []
                if rid not in BROKEN_SONAR_ROBOTS:
                    for i, dist in enumerate(state['sonar']):
                        if dist > 0 and dist < 5000:
                            angle_rad = math.radians(th + SONAR_ANGLES[i])
                            end_x = x + dist * math.cos(angle_rad)
                            end_y = y + dist * math.sin(angle_rad)
                            beam_color = '#FF6B6B' if dist < 250 else '#4ECDC4'
                            beam_alpha = 0.6 if dist < 250 else 0.15
                            beam = self.ax.plot([x, end_x], [y, end_y], '-',
                                                color=beam_color, linewidth=0.8, alpha=beam_alpha, zorder=4)[0]
                            self.sonar_lines[rid].append(beam)

            # --- Auto-scale ---
            all_x, all_y = [], []
            for rid, state in active_robots.items():
                all_x.append(state['x'])
                all_y.append(state['y'])
            if len(self.obstacle_points) > 0:
                all_x.extend(self.obstacle_points[:, 0].tolist())
                all_y.extend(self.obstacle_points[:, 1].tolist())

            if all_x and all_y:
                margin = 1000
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range, 2000)
                cx = (x_min + x_max) / 2
                cy = (y_min + y_max) / 2
                self.ax.set_xlim(cx - max_range / 2, cx + max_range / 2)
                self.ax.set_ylim(cy - max_range / 2, cy + max_range / 2)

            # --- Update HUD ---
            mode_str = "BOIDS EXPLORE" if self.boids_active else "MANUAL"
            hud = f"══ ROOM MAPPER ══ [{mode_str}]\n"
            hud += f"Robots: {len(active_robots)}/3\n"
            hud += f"Obstacle pts: {len(self.obstacle_points)}\n"
            for rid in sorted(active_robots.keys()):
                s = active_robots[rid]
                tag = " ⚠no sonar" if rid in BROKEN_SONAR_ROBOTS else ""
                batt = self.battery.get(rid, 0)
                batt_str = f" {batt:.1f}V" if batt > 0 else ""
                hud += f"R{rid}: ({s['x']:.0f},{s['y']:.0f}) θ{s['th']:.0f}°{batt_str}{tag}\n"
            self.status_text.set_text(hud)

            # --- Update boids button color ---
            if self.boids_active:
                self.boids_btn_ax.set_facecolor('#34C759')
                self.boids_btn.label.set_text('⏹ Stop Explore')
            else:
                self.boids_btn_ax.set_facecolor('#2d2d44')
                self.boids_btn.label.set_text('🤖 Boids Explore')

            # --- Key feedback ---
            if now - self.last_key_time < 1.5 and self.last_key_label:
                self.key_feedback_text.set_text(f">> {self.last_key_label}")
                self.key_feedback_text.set_alpha(1.0)
            else:
                self.key_feedback_text.set_alpha(0.0)

        return artists

    def run(self):
        """Start the visualizer"""
        listener = threading.Thread(target=self.udp_listener, daemon=True)
        listener.start()

        self.anim = FuncAnimation(self.fig, self._update_frame, interval=100, blit=False, cache_frame_data=False)

        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            self.cmd_sock.close()
            print("Room Mapper closed.")


if __name__ == "__main__":
    mapper = RoomMapper2D()
    mapper.run()
