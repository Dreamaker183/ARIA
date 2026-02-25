%% Multi-Agent UDP Live Plotter
% This script listens to the UDP broadcast from the multiAgentConsensus C++
% application and provides a live plot in MATLAB.

clear; clc; close all;

%% Configuration
UDP_PORT = 50000;
ROOM_SIZE = 5000; % mm

% Defined strictly to match the C++ struct packing
% struct RobotStateMsg { int id, int form, double x, y, th, v, w, sonar[8] }
% '2i13d' => 2 integers (4 bytes each) then 13 doubles (8 bytes each)
STRUCT_SIZE = (2 * 4) + (13 * 8); 

%% Initialize UDP
try
    u = udpport('LocalPort', UDP_PORT, 'EnablePortSharing', true);
    fprintf('📡 MATLAB Listening on UDP Port %d...\n', UDP_PORT);
catch
    error('Could not open UDP port. Ensure no other applications are blocking it.');
end

%% State Tracking
robots = containers.Map('KeyType', 'int32', 'ValueType', 'any');
colors = lines(10);

figure('Color', 'white', 'Name', 'FYP: Live Robot Telemetry');
hold on; grid on; axis equal;
xlim([-ROOM_SIZE ROOM_SIZE]); ylim([-ROOM_SIZE ROOM_SIZE]);
xlabel('X (mm)'); ylabel('Y (mm)');
title('Live Swarm Telemetry in MATLAB');

% HUD Text
htext = text(-ROOM_SIZE+200, ROOM_SIZE-400, 'Waiting for data...', 'FontSize', 12, 'FontWeight', 'bold');

%% Main Loop
disp('Press Ctrl+C in Command Window to stop.');
while true
    if u.NumBytesAvailable >= STRUCT_SIZE
        % Read the packet
        data = read(u, STRUCT_SIZE, 'uint8');
        
        % Unpack binary data
        % MATLAB read() returns uint8. We use typecast to extract values.
        % 1. IDs (First 8 bytes)
        ids = typecast(data(1:8), 'int32');
        robot_id = ids(1);
        formation_type = ids(2);
        
        % 2. Doubles (Remaining bytes)
        doubles = typecast(data(9:end), 'double');
        rx = doubles(1);
        ry = doubles(2);
        rth = doubles(3);
        rv = doubles(4);
        rw = doubles(5);
        sonars = doubles(6:13);
        
        % Update robot history
        if ~isKey(robots, robot_id)
            robots(robot_id) = struct('x', [], 'y', [], 'plot', [], 'trail', []);
        end
        
        rstate = robots(robot_id);
        rstate.x = [rstate.x, rx];
        rstate.y = [rstate.y, ry];
        
        % Keep trail short
        if length(rstate.x) > 50
            rstate.x(1) = [];
            rstate.y(1) = [];
        end
        
        % Update Visualization
        if ~isempty(rstate.plot)
            delete(rstate.plot);
            delete(rstate.trail);
        end
        
        clr = colors(mod(robot_id, 10)+1, :);
        rstate.trail = plot(rstate.x, rstate.y, 'Color', clr, 'LineWidth', 0.5, 'HandleVisibility', 'off');
        rstate.plot = plot(rx, ry, 'o', 'MarkerSize', 12, 'MarkerFaceColor', clr, 'DisplayName', sprintf('Robot %d', robot_id));
        
        robots(robot_id) = rstate;
        
        % Update HUD
        set(htext, 'String', sprintf('Active Robots: %d\nFormation: %d', robots.Count, formation_type));
        
        drawnow limitrate;
    end
    pause(0.01);
end
