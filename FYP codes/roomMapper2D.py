#!/usr/bin/env python3
"""
Live 2D Room Mapper for Pioneer 3DX Swarm
Listens on UDP Port 50000 for RobotStateMsg structs.
Sends keyboard commands to C++ on UDP Port 50001.

-- Tesla/Apple Minimalist UI Edition --
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

OBSTACLE_MAX_RANGE = 2500
OBSTACLE_MIN_RANGE = 50
OBSTACLE_DEDUP_RADIUS = 30
MAX_OBSTACLE_POINTS = 8000
ROBOT_FILTER_RADIUS = 400     # Ignore sonar hits within this range of a known robot

BATTERY_LOW_THRESHOLD = 11.5

# Network
TELEMETRY_PORT = 50000
COMMAND_PORT = 50001

# ============================================================
# TESLA / APPLE MINIMALIST TOKENS
# ============================================================
# Deep graphite and sleek monochromatic styling typical of modern glass OS/EV dash
BG_COLOR         = '#000000' # True black for OLED feel
PANEL_BG         = '#121212' # Very dark gray panel
TEXT_MAIN        = '#F5F5F7' # Apple SF Pro white
TEXT_MUTED       = '#86868B' # Apple SF Pro gray
ACCENT_BLUE      = '#2997FF' # Apple Blue
ACCENT_GREEN     = '#34C759' # Apple Green
ACCENT_RED       = '#FF3B30' # Apple Red
ACCENT_YELLOW    = '#FFCC00' # Apple Yellow
OBSTACLE_COLOR   = '#E5E5EA' # Soft gray for environment

# Clean distinct colors for robots
ROBOT_COLORS = {
    1: '#FF3B30', # Red
    2: '#2997FF', # Blue
    3: '#34C759'  # Green
}
ROBOT_NAMES = {1: 'Unit 1 (Blind)', 2: 'Unit 2', 3: 'Unit 3'}


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
        self.max_trail = 2000

        # Battery & status tracking
        self.battery = {}        # {rid: voltage}
        self.boids_active = False

        # Waypoint state (Point-and-click navigation)
        self.wp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.wp_addr = ('127.0.0.1', 50002)
        self.waypoints = {}       # {rid: (x, y)}
        self.waypoint_markers = {} # drawing 'X'
        self.waypoint_lines = {}   # drawing dashed line
        self.active_drag_robot = None
        
        # Calibration state
        self.calibrated_robot = None
        self.calibration_time = 0
        self.calibration_halo = None

        # Persistent obstacle map: array of [X, Y, Confidence]
        self.obstacle_points = np.empty((0, 3), dtype=np.float64)

        # Warning state
        self.warnings = []       # list of (message, color, expiry_time)

        # Last key pressed (for visual feedback)
        self.last_key_label = ""
        self.last_key_time = 0

        # Setup matplotlib
        plt.style.use('dark_background')
        # Setting a generic sans-serif font that renders cleanly like SF Pro / Inter
        plt.rcParams['font.family'] = 'sans-serif'
        plt.rcParams['font.sans-serif'] = ['Helvetica Neue', 'Helvetica', 'Arial', 'sans-serif']
        
        self.fig = plt.figure(figsize=(16, 9), dpi=100)
        self.fig.canvas.manager.set_window_title('Swarm OS')
        self.fig.patch.set_facecolor(BG_COLOR)

        # Matplotlib toolbar removal
        try:
            self.fig.canvas.toolbar.pack_forget()
        except Exception:
            pass

        # Control panel (left)
        self.ax_guide = self.fig.add_axes([0.02, 0.03, 0.18, 0.94])
        self.ax_guide.set_facecolor(PANEL_BG)
        self.ax_guide.set_xticks([])
        self.ax_guide.set_yticks([])
        # Remove spines for that "floating glass" borderless look
        for spine in self.ax_guide.spines.values():
            spine.set_visible(False)

        # Map panel (center/right)
        self.ax = self.fig.add_axes([0.22, 0.03, 0.76, 0.94])

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

        # Connect keyboard and mouse events
        self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        self.fig.canvas.mpl_connect('button_press_event', self._on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self._on_mouse_release)

    def _setup_guide(self):
        """Draw the control guide panel with boids button"""
        ax = self.ax_guide
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 1)

        # Tesla-style sleek header
        ax.text(0.1, 0.95, 'Swarm OS', ha='left', va='top',
                fontsize=16, fontweight='bold', color=TEXT_MAIN)
        ax.text(0.1, 0.92, 'Control Center', ha='left', va='top',
                fontsize=9, color=TEXT_MUTED)

        y = 0.85
        gap = 0.035

        def section(title, color=TEXT_MAIN):
            nonlocal y
            y -= gap * 0.2
            ax.text(0.1, y, title, ha='left', va='top',
                    fontsize=9, fontweight='bold', color=color)
            y -= gap * 1.1

        def entry(keys, desc):
            nonlocal y
            ax.text(0.1, y, keys, ha='left', va='top',
                    fontsize=9, fontweight='bold', color=TEXT_MAIN)
            
            ax.text(0.1, y - gap * 0.6, desc, ha='left', va='top',
                    fontsize=8, color=TEXT_MUTED)
            y -= gap * 1.8

        section('Unit 1', ROBOT_COLORS[1])
        entry('Arrows', 'Manual Drive')

        section('Group Dynamics', ACCENT_BLUE)
        entry('W / A / S / D', 'Sync Translate/Rotate')
        entry('T & G / F & H', 'Split Translate/Rotate')

        section('System', TEXT_MAIN)
        entry('SPACE / X', 'Emergency Stop')
        entry('R / N', 'Record / Commit Segment')
        entry('1 / 2 / 3', 'Playback Segment')
        entry('Q', 'Shutdown Controller')

        # Modern Boids button styling (Apple pill style)
        section('Autopilot', ACCENT_GREEN)
        
        # Boids button
        self.boids_btn_ax = self.fig.add_axes([0.03, 0.08, 0.16, 0.05])
        self.boids_btn_ax.set_facecolor('#1C1C1E')
        self.boids_btn = Button(self.boids_btn_ax, 'Engage Explore',
                                color='#1C1C1E', hovercolor='#2C2C2E')
        self.boids_btn.label.set_fontsize(10)
        self.boids_btn.label.set_fontweight('bold')
        self.boids_btn.label.set_color(TEXT_MAIN)
        self.boids_btn.on_clicked(self._on_boids_click)
        
        # Home button
        self.home_btn_ax = self.fig.add_axes([0.03, 0.02, 0.16, 0.05])
        self.home_btn_ax.set_facecolor('#1C1C1E')
        self.home_btn = Button(self.home_btn_ax, 'Return Home',
                               color='#1C1C1E', hovercolor='#2C2C2E')
        self.home_btn.label.set_fontsize(10)
        self.home_btn.label.set_fontweight('bold')
        self.home_btn.label.set_color(TEXT_MAIN)
        self.home_btn.on_clicked(self._on_home_click)
        
        # Clean button border
        for spine in self.boids_btn_ax.spines.values():
            spine.set_color('#333333')
            spine.set_linewidth(1)
        for spine in self.home_btn_ax.spines.values():
            spine.set_color('#333333')
            spine.set_linewidth(1)
        ax.text(0.5, 0.02, 'Map must have focus for inputs', ha='center', va='bottom',
                fontsize=7, color=TEXT_MUTED)

    def _on_boids_click(self, event):
        """Toggle boids mode via button click"""
        try:
            self.cmd_sock.sendto(b'e', self.cmd_addr)
        except Exception:
            pass
        self.last_key_label = "Autopilot Toggle Requested"
        self.last_key_time = time.time()
        
    def _on_home_click(self, event):
        """Command all robots to return to their initial 0,0 relative origin via APF Waypoint"""
        try:
            for i in range(1, 4):
                # We want the robot to reach its logical (0,0) position. 
                # C++ expects coordinates without offsets, so (0,0) in spatial map means 
                # the target is exactly the inverse of their offset.
                ox, oy = ROBOT_OFFSETS.get(i, (0, 0))
                target_x, target_y = -ox, -oy
                
                # Send waypoint 0,0 relative to spatial map origin (subtract offsets)
                wp_msg = struct.pack('1i2d', i, target_x, target_y)
                self.wp_sock.sendto(wp_msg, self.wp_addr)
            
            with self.lock:
                self.last_key_label = "Return to Base Initiated"
                for i in range(1, 4):
                    self.waypoints[i] = (0, 0)
                self.boids_active = False # Manual waypoints override boids
                self.last_key_time = time.time()
        except Exception:
            pass

    def _setup_map(self):
        """Setup the main map panel"""
        self.ax.set_facecolor(PANEL_BG)
        for spine in self.ax.spines.values():
            spine.set_visible(False) # Borderless floating map

        self.ax.set_xlabel('Spatial X [mm]', color=TEXT_MUTED, fontsize=8)
        self.ax.set_ylabel('Spatial Y [mm]', color=TEXT_MUTED, fontsize=8)
        
        # Subtle grid
        self.ax.set_xlim(-3000, 3000)
        self.ax.set_ylim(-3000, 3000)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.08, color=TEXT_MAIN, linestyle='-', linewidth=0.5)
        self.ax.tick_params(colors=TEXT_MUTED, labelsize=7, length=0) # Hide tick marks, keep numbers

        # Origin
        self.ax.plot(0, 0, '+', color=TEXT_MUTED, markersize=15, alpha=0.2, lw=1)

        # Status HUD (clean text, no box, Apple Maps style glass overlay)
        self.status_text = self.ax.text(
            0.02, 0.98, 'System Initializing...',
            transform=self.ax.transAxes, fontsize=10,
            verticalalignment='top', color=TEXT_MAIN,
            bbox=dict(boxstyle='round,pad=0.5', facecolor='#1C1C1E', edgecolor='none', alpha=0.8)
        )

        # Key feedback (bottom-center)
        self.key_feedback_text = self.ax.text(
            0.5, 0.03, '',
            transform=self.ax.transAxes, fontsize=11,
            ha='center', va='bottom', color=TEXT_MAIN, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.6,rounding_size=0.3', facecolor=ACCENT_BLUE, edgecolor='none', alpha=0.9)
        )

    def _on_key_press(self, event):
        """Handle keyboard events from matplotlib window"""
        key = event.key
        cmd = None
        label = None

        key_map = {
            'up': ('UP', 'Manual Translate: Fwd'),
            'down': ('DOWN', 'Manual Translate: Rev'),
            'left': ('LEFT', 'Manual Rotate: CCW'),
            'right': ('RIGHT', 'Manual Rotate: CW'),
            'w': ('w', 'Swarm Link: Fwd'),
            's': ('s', 'Swarm Link: Rev'),
            'a': ('a', 'Swarm Link: CCW'),
            'd': ('d', 'Swarm Link: CW'),
            't': ('t', 'Split Link: Fwd/Rev'),
            'g': ('g', 'Split Link: Rev/Fwd'),
            'f': ('f', 'Split Link: CCW/CW'),
            'h': ('HOME', 'Return Home Base'),
            ' ': ('SPACE', 'Emergency Stop Interlock'),
            'x': ('x', 'Emergency Stop Interlock'),
            'e': ('e', 'Explore Autopilot Toggle'),
            'r': ('r', 'Telemetry Record Toggle'),
            '1': ('1', 'Execute Sequence 1'),
            '2': ('2', 'Execute Sequence 2'),
            '3': ('3', 'Execute Sequence 3'),
            'n': ('n', 'Commit Segment'),
            'q': ('q', 'System Power Down'),
        }

        if key in key_map:
            cmd, label = key_map[key]
            
            if cmd == 'SPACE' or cmd == 'x':
                with self.lock:
                    self.waypoints.clear()
            elif cmd == 'e':
                self._on_boids_click(None)
                return
            elif cmd == 'HOME':
                self._on_home_click(None)
                return
                
            try:
                self.cmd_sock.sendto(cmd.encode(), self.cmd_addr)
            except Exception:
                pass

            self.last_key_label = f"{label}"
            self.last_key_time = time.time()

    def _on_mouse_press(self, event):
        if event.inaxes != self.ax or event.button != 1: return
        click_x, click_y = event.xdata, event.ydata
        closest_rid = None
        min_dist = float('inf')
        
        with self.lock:
            for rid, state in self.robots.items():
                if time.time() - self.last_seen.get(rid, 0) < 3.0:
                    dist = math.hypot(state['x'] - click_x, state['y'] - click_y)
                    if dist < 450: # Click radius (a bit larger than robot chassis)
                        if dist < min_dist:
                            min_dist = dist
                            closest_rid = rid
                            
        self.active_drag_robot = closest_rid if closest_rid else 0
        self.last_key_label = f"Targeting Unit {closest_rid}..." if closest_rid else "Targeting Swarm..."
        self.last_key_time = time.time()

    def _on_mouse_release(self, event):
        if event.inaxes != self.ax or event.button != 1: 
            self.active_drag_robot = None
            return
            
        target_x, target_y = event.xdata, event.ydata
        source_rid = self.active_drag_robot
        self.active_drag_robot = None
        
        if source_rid is not None:
            # Check if dropped onto another robot for Calibration
            dropped_on_rid = None
            if source_rid != 0:
                with self.lock:
                    for rid, state in self.robots.items():
                        if rid != source_rid and time.time() - self.last_seen.get(rid, 0) < 3.0:
                            dist = math.hypot(state['x'] - target_x, state['y'] - target_y)
                            if dist < 450: # Dropped onto this robot
                                dropped_on_rid = rid
                                break
                                
            if dropped_on_rid is not None:
                # Trigger Auto-Calibration
                with self.lock:
                    source_state = self.robots[source_rid]
                    target_state = self.robots[dropped_on_rid]
                    
                    # Target robot's raw position (without offset)
                    raw_target_x = target_state['x'] - ROBOT_OFFSETS.get(dropped_on_rid, (0, 0))[0]
                    raw_target_y = target_state['y'] - ROBOT_OFFSETS.get(dropped_on_rid, (0, 0))[1]
                    
                    # Angle from Source to assumed Target position
                    angle_to_target = math.degrees(math.atan2(target_state['y'] - source_state['y'], 
                                                              target_state['x'] - source_state['x']))
                    
                    # Target physical dimensions: 38cm wide x 45cm long (380x450 mm)
                    # We approximate the center offset distance from an edge hit based on the angle
                    # If hitting front/back, offset is ~225mm. If hitting sides, offset is ~190mm.
                    
                    valid_centers_x = []
                    valid_centers_y = []
                    
                    for i, dist in enumerate(source_state['sonar']):
                        if OBSTACLE_MIN_RANGE < dist < OBSTACLE_MAX_RANGE:
                            beam_angle = source_state['th'] + SONAR_ANGLES[i]
                            # Normalize difference
                            diff = (beam_angle - angle_to_target + 180) % 360 - 180
                            
                            # If beam is pointing generally toward the target robot
                            if abs(diff) < 35:
                                beam_angle_rad = math.radians(beam_angle)
                                
                                # Where did the sonar hit?
                                hit_x = source_state['x'] + dist * math.cos(beam_angle_rad)
                                hit_y = source_state['y'] + dist * math.sin(beam_angle_rad)
                                
                                # What is the angle of THIS hit relative to the target robot's heading?
                                hit_angle_rel_target = (beam_angle - target_state['th'] + 180) % 360 - 180
                                
                                # Calculate radius from center to edge of a 380x450 rectangle at this angle
                                a = math.radians(hit_angle_rel_target)
                                w, h = 380.0, 450.0
                                # Radius of bounding box at angle 'a'
                                r_edge = min(abs(h / 2.0 / math.cos(a)) if math.cos(a) != 0 else float('inf'),
                                             abs(w / 2.0 / math.sin(a)) if math.sin(a) != 0 else float('inf'))
                                
                                # The true center is r_edge distance further along the beam
                                center_x = hit_x + r_edge * math.cos(beam_angle_rad)
                                center_y = hit_y + r_edge * math.sin(beam_angle_rad)
                                
                                valid_centers_x.append(center_x)
                                valid_centers_y.append(center_y)
                                
                    if len(valid_centers_x) > 0:
                        # Average all valid sonar beams that hit the target chassis
                        avg_target_x = sum(valid_centers_x) / len(valid_centers_x)
                        avg_target_y = sum(valid_centers_y) / len(valid_centers_y)
                        
                        # Calculate the new offset required so raw_target lands exactly on avg_target
                        new_offset_x = avg_target_x - raw_target_x
                        new_offset_y = avg_target_y - raw_target_y
                        
                        ROBOT_OFFSETS[dropped_on_rid] = (new_offset_x, new_offset_y)
                        
                        # Immediately snap the target robot to its new true position
                        self.robots[dropped_on_rid]['x'] = raw_target_x + new_offset_x
                        self.robots[dropped_on_rid]['y'] = raw_target_y + new_offset_y
                        
                        # Clear old stale trajectory trail because it jumped
                        self.trajectories[dropped_on_rid] = [(self.robots[dropped_on_rid]['x'], self.robots[dropped_on_rid]['y'])]
                        
                        # UI Feedback
                        self.calibrated_robot = dropped_on_rid
                        self.calibration_time = time.time()
                        self.last_key_label = f"Unit {dropped_on_rid} Calibrated ({len(valid_centers_x)} hits)"
                        self.last_key_time = time.time()
                    else:
                        self.last_key_label = f"Calibration Failed: Sonar missing"
                        self.last_key_time = time.time()
            else:
                # Normal Waypoint Logic
                try:
                    if source_rid == 0:
                        for i in range(1, 4):
                            ox, oy = ROBOT_OFFSETS.get(i, (0, 0))
                            wp_msg = struct.pack('1i2d', i, target_x - ox, target_y - oy)
                            self.wp_sock.sendto(wp_msg, self.wp_addr)
                    else:
                        ox, oy = ROBOT_OFFSETS.get(source_rid, (0, 0))
                        wp_msg = struct.pack('1i2d', source_rid, target_x - ox, target_y - oy)
                        self.wp_sock.sendto(wp_msg, self.wp_addr)
                    
                    with self.lock:
                        if source_rid == 0:
                            self.last_key_label = "Swarm Destination Set"
                            for i in range(1, 4):
                                self.waypoints[i] = (target_x, target_y)
                        else:
                            self.last_key_label = f"Unit {source_rid} Destination Set"
                            self.waypoints[source_rid] = (target_x, target_y)
                            
                        self.boids_active = False
                        self.last_key_time = time.time()
                except Exception:
                    pass

    def _add_obstacle_point(self, hx, hy):
        """Add obstacle point with deduplication"""
        if len(self.obstacle_points) > 0:
            dists = np.sqrt((self.obstacle_points[:, 0] - hx)**2 +
                            (self.obstacle_points[:, 1] - hy)**2)
            min_idx = np.argmin(dists)
            if dists[min_idx] < OBSTACLE_DEDUP_RADIUS:
                # Point already exists near here: Increase confidence to a max of 10
                self.obstacle_points[min_idx, 2] = min(10.0, self.obstacle_points[min_idx, 2] + 1)
                return

        # New point starts with Confidence = 1
        self.obstacle_points = np.vstack([self.obstacle_points, [hx, hy, 1.0]])

        if len(self.obstacle_points) > MAX_OBSTACLE_POINTS:
            self.obstacle_points = self.obstacle_points[-MAX_OBSTACLE_POINTS:]

    def _clear_obstacles_along_beam(self, rx, ry, clear_dist, angle_rad):
        """Erases existing obstacle points that fall inside a newly discovered clear path."""
        if len(self.obstacle_points) == 0:
            return
            
        # Beam vector
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        
        # Vector from robot to all obstacle points
        px = self.obstacle_points[:, 0] - rx
        py = self.obstacle_points[:, 1] - ry
        
        # Distance of points projected along the beam
        proj_dist = px * dx + py * dy
        
        # Only care about points that are in front of the robot and within the clear distance
        in_range_mask = (proj_dist > 0) & (proj_dist < clear_dist)
        
        if not np.any(in_range_mask):
            return
            
        # Perpendicular distance from the actual beam line
        perp_dist = np.abs(px * dy - py * dx)
        
        # We use a two-tiered clearance width depending on confidence
        confidences = self.obstacle_points[:, 2]
        
        # If confidence is low (< 3, Cyan), we erase with a wide beam (width ~ 150mm)
        # If confidence is high (>= 3, Orange), we erase only with a narrow beam (width ~ 30mm)
        # to protect confirmed static obstacles from nearby glances.
        max_clear_width = np.where(confidences < 3, 150.0, 30.0)
        
        # If the point is within range AND close to the beam line
        clear_mask = in_range_mask & (perp_dist < max_clear_width)
        
        if np.any(clear_mask):
            # Decrease confidence of cleared points
            self.obstacle_points[clear_mask, 2] -= 1
            
            # Keep only points that still have confidence > 0
            self.obstacle_points = self.obstacle_points[self.obstacle_points[:, 2] > 0]

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
        print(f"Server Online. Port {TELEMETRY_PORT}")

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
                                    
                                    # --- Raycast Clearance: Erase old dots in the physically clear space ---
                                    # This sonar beam went 'dist' far. Everything from robot to (dist - 200mm) is empty space.
                                    clear_dist = dist - 200
                                    if clear_dist > 0 and len(self.obstacle_points) > 0:
                                        self._clear_obstacles_along_beam(x, y, clear_dist, angle_rad)
                                    
                                    # Filter out hits that are near another known robot
                                    is_robot = False
                                    for other_rid, other_state in self.robots.items():
                                        if other_rid == r_id:
                                            continue
                                        if time.time() - self.last_seen.get(other_rid, 0) < 3.0:
                                            d = math.hypot(hit_x - other_state['x'], hit_y - other_state['y'])
                                            if d < ROBOT_FILTER_RADIUS:
                                                is_robot = True
                                                # Use this detection to refine relative offset
                                                # Measured distance between robot centers
                                                measured_dist = math.hypot(x - other_state['x'], y - other_state['y'])
                                                # Expected raw distance from odometry
                                                raw_x = x - ROBOT_OFFSETS.get(r_id, (0,0))[0]
                                                raw_y = y - ROBOT_OFFSETS.get(r_id, (0,0))[1]
                                                other_raw_x = other_state['x'] - ROBOT_OFFSETS.get(other_rid, (0,0))[0]
                                                other_raw_y = other_state['y'] - ROBOT_OFFSETS.get(other_rid, (0,0))[1]
                                                raw_dist = math.hypot(raw_x - other_raw_x, raw_y - other_raw_y)
                                                # Smooth correction: nudge offset toward measured sonar distance
                                                if raw_dist > 10:
                                                    correction = 0.05  # gentle filter
                                                    dx_off = other_state['x'] - x
                                                    real_dx = dist * math.cos(angle_rad)
                                                    # Only refine if detection seems consistent
                                                    if abs(dist - measured_dist) < 300:
                                                        cur_ox, cur_oy = ROBOT_OFFSETS.get(other_rid, (0, 0))
                                                        ROBOT_OFFSETS[other_rid] = (
                                                            cur_ox * (1-correction) + cur_ox * correction,
                                                            cur_oy * (1-correction) + cur_oy * correction,
                                                        )
                                                break
                                    if not is_robot:
                                        self._add_obstacle_point(hit_x, hit_y)
                                else:
                                    # Even if we didn't hit an obstacle (dist is max range ~5000), 
                                    # we still know the space up to OBSTACLE_MAX_RANGE is clear!
                                    if dist >= OBSTACLE_MAX_RANGE:
                                        angle_rad = math.radians(th + SONAR_ANGLES[i])
                                        self._clear_obstacles_along_beam(x, y, OBSTACLE_MAX_RANGE, angle_rad)

            except socket.timeout:
                pass
            except Exception:
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

            for rid in self.robots:
                if now - self.last_seen.get(rid, 0) > 3.0:
                    current_warnings.append((f"Network Lost: Unit {rid}", ACCENT_RED))

            for rid, batt in self.battery.items():
                if 0 < batt < BATTERY_LOW_THRESHOLD:
                    current_warnings.append((f"Low Battery: Unit {rid} ({batt:.1f}V)", ACCENT_YELLOW))

            if self.boids_active:
                current_warnings.append(("Autopilot Engaged", ACCENT_GREEN))

            # --- Draw obstacles ---
            if self.obstacle_scatter is not None:
                self.obstacle_scatter.remove()
                self.obstacle_scatter = None

            if len(self.obstacle_points) > 0:
                # Create an array of colors based on confidence
                # Confidence < 3 = unconfirmed (Cyan #00FFFF), Confidence >= 3 = static (Orange #FF9500)
                colors = np.where(self.obstacle_points[:, 2:3] < 3, '#00FFFF', '#FF9500')
                
                self.obstacle_scatter = self.ax.scatter(
                    self.obstacle_points[:, 0], self.obstacle_points[:, 1],
                    c=colors.flatten(),
                    s=12, alpha=0.8, edgecolors='none', zorder=1
                )
                artists.append(self.obstacle_scatter)

            # --- Clear old robot elements ---
            if self.calibration_halo is not None:
                try: self.calibration_halo.remove()
                except: pass
                self.calibration_halo = None
            for key in list(self.robot_markers.keys()):
                patches_list = self.robot_markers[key]
                if isinstance(patches_list, list):
                    for p in patches_list:
                        try: p.remove()
                        except: pass
                else:
                    try: patches_list.remove()
                    except: pass
            for dict_group in [self.robot_arrows, self.robot_labels, self.trail_lines]:
                for key in list(dict_group.keys()):
                    try: dict_group[key].remove()
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

            # --- Clear waypoints ---
            for key in list(self.waypoint_markers.keys()):
                try: self.waypoint_markers[key].remove()
                except: pass
            for key in list(self.waypoint_lines.keys()):
                try: self.waypoint_lines[key].remove()
                except: pass
            self.waypoint_markers.clear()
            self.waypoint_lines.clear()

            # --- Draw waypoints ---
            for rid, (wx, wy) in list(self.waypoints.items()):
                if rid in active_robots:
                    color = ROBOT_COLORS.get(rid, TEXT_MAIN)
                    rx, ry = active_robots[rid]['x'], active_robots[rid]['y']
                    
                    dist = math.hypot(wx - rx, wy - ry)
                    if dist > 150:
                        # Target 'X' crosshair
                        marker = self.ax.plot(wx, wy, 'x', color=color, markersize=14, markeredgewidth=2, alpha=0.9, zorder=5)[0]
                        self.waypoint_markers[rid] = marker
                        
                        # Dashed trajectory line
                        line = self.ax.plot([rx, wx], [ry, wy], '--', color=color, linewidth=1.5, alpha=0.5, zorder=3)[0]
                        self.waypoint_lines[rid] = line
                    else:
                        del self.waypoints[rid]

            # --- Draw warnings at top of map (Apple notification style) ---
            for i, (msg, color) in enumerate(current_warnings):
                wt = self.ax.text(
                    0.5, 0.98 - i * 0.05, msg,
                    transform=self.ax.transAxes, fontsize=11, ha='center', va='top',
                    color='#000000', fontweight='bold',
                    bbox=dict(boxstyle='round,pad=0.5,rounding_size=0.4', facecolor=color, edgecolor='none', alpha=0.95),
                    zorder=20
                )
                self.warning_texts.append(wt)

            # --- Draw each active robot ---
            for rid, state in active_robots.items():
                x, y, th = state['x'], state['y'], state['th']
                color = ROBOT_COLORS.get(rid, '#FFFFFF')
                # Pioneer 3-DX Top-Down Representation
                # Chassis (approx 44cm x 38cm, scaled a bit for visibility: 440x380)
                body_len = 500
                body_wid = 400
                wheel_len = 200
                wheel_wid = 60
                
                # We use an Affine2D transform to rotate and translate the shapes
                from matplotlib.transforms import Affine2D
                t = Affine2D().rotate_deg(th).translate(x, y) + self.ax.transData

                # 1. Main red chassis (rectangle with rounded front)
                # We'll use a FancyBboxPatch for the chassis
                chassis = patches.FancyBboxPatch(
                    (-body_len/2, -body_wid/2), body_len, body_wid,
                    boxstyle="round,pad=0,rounding_size=100",
                    facecolor='#C42A22', edgecolor='#801510', lw=1.5,
                    transform=t, zorder=6
                )
                
                # 2. Black top deck (slightly smaller, heavily rounded front)
                top_deck = patches.FancyBboxPatch(
                    (-body_len/2 + 20, -body_wid/2 + 20), body_len - 40, body_wid - 40,
                    boxstyle="round,pad=0,rounding_size=120",
                    facecolor='#1C1C1E', edgecolor='#30363D', lw=1,
                    transform=t, zorder=7
                )
                
                # 3. Left Wheel (rugged yellow/black)
                wheel_l = patches.Rectangle(
                    (-wheel_len/2, body_wid/2 - wheel_wid/2), wheel_len, wheel_wid,
                    facecolor='#1A1A1C', edgecolor='#D29922', lw=1,
                    transform=t, zorder=7.5
                )
                
                # 4. Right Wheel
                wheel_r = patches.Rectangle(
                    (-wheel_len/2, -body_wid/2 - wheel_wid/2), wheel_len, wheel_wid,
                    facecolor='#1A1A1C', edgecolor='#D29922', lw=1,
                    transform=t, zorder=7.5
                )

                # 5. Front Direction Indicator (subtle neon slit on the top deck)
                indicator = patches.Rectangle(
                    (body_len/2 - 80, -20), 40, 40,
                    facecolor=color, edgecolor='none', alpha=0.9,
                    transform=t, zorder=8
                )

                self.ax.add_patch(chassis)
                self.ax.add_patch(top_deck)
                self.ax.add_patch(wheel_l)
                self.ax.add_patch(wheel_r)
                self.ax.add_patch(indicator)
                
                self.robot_markers[rid] = [chassis, top_deck, wheel_l, wheel_r, indicator]
                
                # Calibration Halo
                if rid == self.calibrated_robot and time.time() - self.calibration_time < 3.0:
                    halo = patches.Circle((x, y), radius=350, facecolor='none', edgecolor='#FFFFAA', linewidth=4, alpha=0.9, zorder=5)
                    self.calibration_halo = self.ax.add_patch(halo)
                    # Add glow effect by drawing a semi-transparent slightly larger circle
                    glow = patches.Circle((x, y), radius=380, facecolor='none', edgecolor='#FFFFAA', linewidth=8, alpha=0.3, zorder=4)
                    self.ax.add_patch(glow)
                    self.robot_markers[rid].extend([halo, glow])

                # Apple Maps style clean elegant label
                sonar_tag = " (Blind)" if rid in BROKEN_SONAR_ROBOTS else ""
                batt_v = self.battery.get(rid, 0)
                batt_tag = f" {batt_v:.1f}V" if batt_v > 0 else ""
                batt_color = ACCENT_RED if (0 < batt_v < BATTERY_LOW_THRESHOLD) else TEXT_MUTED
                
                label_text = f"U{rid}{sonar_tag}\n{batt_tag}".strip()
                label = self.ax.annotate(label_text, (x, y),
                                         textcoords="offset points",
                                         xytext=(0, 18), fontsize=8, color=TEXT_MAIN,
                                         fontweight='normal', ha='center', zorder=8,
                                         bbox=dict(boxstyle='round,pad=0.3', facecolor='#1C1C1E', edgecolor='none', alpha=0.7))
                self.robot_labels[rid] = label

                # Smooth sweeping trail line
                trail = self.trajectories.get(rid, [])
                if len(trail) > 1:
                    tx = [p[0] for p in trail]
                    ty = [p[1] for p in trail]
                    line = self.ax.plot(tx, ty, '-', color=color, linewidth=2, alpha=0.3, zorder=3)[0]
                    self.trail_lines[rid] = line

                self.sonar_lines[rid] = []
                if rid not in BROKEN_SONAR_ROBOTS:
                    for i, dist in enumerate(state['sonar']):
                        if 0 < dist < 5000:
                            angle_rad = math.radians(th + SONAR_ANGLES[i])
                            end_x = x + dist * math.cos(angle_rad)
                            end_y = y + dist * math.sin(angle_rad)
                            beam_color = ACCENT_RED if dist < 250 else TEXT_MUTED
                            beam_alpha = 0.4 if dist < 250 else 0.05
                            beam = self.ax.plot([x, end_x], [y, end_y], '-',
                                                color=beam_color, linewidth=0.5, alpha=beam_alpha, zorder=4)[0]
                            self.sonar_lines[rid].append(beam)

            # --- Auto-scale smoothly (like pinch to zoom) ---
            all_x, all_y = [], []
            for rid, state in active_robots.items():
                all_x.append(state['x'])
                all_y.append(state['y'])
            if len(self.obstacle_points) > 0:
                all_x.extend(self.obstacle_points[:, 0].tolist())
                all_y.extend(self.obstacle_points[:, 1].tolist())

            if all_x and all_y:
                margin = 800
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range, 2500)
                cx = (x_min + x_max) / 2
                cy = (y_min + y_max) / 2
                self.ax.set_xlim(cx - max_range / 2, cx + max_range / 2)
                self.ax.set_ylim(cy - max_range / 2, cy + max_range / 2)

            # --- Update HUD (Tesla Top Bar Style) ---
            hud = f"System Link: {len(active_robots)}/3 Online    •    Mapped Points: {len(self.obstacle_points)}"
            self.status_text.set_text(hud)

            # --- Update boids button color ---
            if self.boids_active:
                self.boids_btn_ax.set_facecolor(ACCENT_GREEN)
                self.boids_btn.label.set_text('Autopilot Active')
                self.boids_btn.label.set_color('#000000')
            else:
                self.boids_btn_ax.set_facecolor('#1C1C1E')
                self.boids_btn.label.set_text('Engage Autopilot')
                self.boids_btn.label.set_color(TEXT_MAIN)

            # --- Key feedback fade ---
            if now - self.last_key_time < 0.6 and self.last_key_label:
                self.key_feedback_text.set_text(self.last_key_label)
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
            self.wp_sock.close()
            print("System Offline.")


if __name__ == "__main__":
    mapper = RoomMapper2D()
    mapper.run()
