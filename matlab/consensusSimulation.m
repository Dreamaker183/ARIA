%% Multi-Agent Consensus Simulation (Triangle, Circle, Line)
% This script simulates the decentralized consensus logic used in the ARIA C++ code.
% It demonstrates how robots converge to formations using only local neighbor data.

clear; clc; close all;

%% Simulation Parameters
numRobots = 5;
dt = 0.1;           % Time step
totalTime = 30;     % Seconds
steps = totalTime / dt;

% Gains (Matching C++ alphas and betas)
alpha = 0.5;        % Position gain
beta = 0.3;         % Velocity alignment gain
maxVel = 500;       % mm/s

% Formation Type: 1=Triangle, 2=Circle, 3=Line
activeFormation = 2; 

%% Initial States
% Random positions in a 5x5 meter area
x = (rand(1, numRobots) - 0.5) * 4000;
y = (rand(1, numRobots) - 0.5) * 4000;
theta = rand(1, numRobots) * 2 * pi;
vx = zeros(1, numRobots);
vy = zeros(1, numRobots);

% History for plotting
hx = zeros(steps, numRobots);
hy = zeros(steps, numRobots);

%% Simulation Loop
figure('Color', 'white', 'Position', [100 100 800 600]);
hold on; grid on; axis equal;
xlim([-5000 5000]); ylim([-5000 5000]);
xlabel('X (mm)'); ylabel('Y (mm)');
title('FYP: Multi-Agent Consensus Theoretical Simulation');

colors = lines(numRobots);

for t = 1:steps
    % 1. Calculate Consensus Vectors
    Ux = zeros(1, numRobots);
    Uy = zeros(1, numRobots);
    
    for i = 1:numRobots
        sumPos_x = 0;
        sumPos_y = 0;
        sumVel_x = 0;
        sumVel_y = 0;
        
        neighborCount = 0;
        
        for j = 1:numRobots
            if i == j, continue; end
            
            % Get theoretical offsets for formation
            [dx_ij, dy_ij] = getFormationOffset(activeFormation, i, j);
            
            % Position consensus
            sumPos_x = sumPos_x + (x(j) - x(i) - dx_ij);
            sumPos_y = sumPos_y + (y(j) - y(i) - dy_ij);
            
            % Velocity consensus
            sumVel_x = sumVel_x + (vx(j) - vx(i));
            sumVel_y = sumVel_y + (vy(j) - vy(i));
            
            neighborCount = neighborCount + 1;
        end
        
        if neighborCount > 0
            Ux(i) = alpha * (sumPos_x / neighborCount) + beta * (sumVel_x / neighborCount);
            Uy(i) = alpha * (sumPos_y / neighborCount) + beta * (sumVel_y / neighborCount);
        end
        
        % Speed limit
        speed = sqrt(Ux(i)^2 + Uy(i)^2);
        if speed > maxVel
            Ux(i) = (Ux(i) / speed) * maxVel;
            Uy(i) = (Uy(i) / speed) * maxVel;
        end
    end
    
    % 2. Update Dynamics (Nonholonomic simplified for simulation)
    % In reality, we map Ux/Uy to v/w. For this demo, we use simple kinematics.
    vx = Ux;
    vy = Uy;
    x = x + vx * dt;
    y = y + vy * dt;
    
    % Store history
    hx(t, :) = x;
    hy(t, :) = y;
    
    % 3. Real-time Plotting (every 5 steps)
    if mod(t, 5) == 0
        cla;
        % Draw target formation polygon hint
        if activeFormation ~= 0
             plot([x x(1)], [y y(1)], 'k--', 'LineWidth', 1, 'HandleVisibility', 'off');
        end
        
        for i = 1:numRobots
            % Path trail
            plot(hx(1:t, i), hy(1:t, i), 'Color', colors(i,:), 'LineWidth', 0.5);
            % Robot current pos
            plot(x(i), y(i), 'o', 'MarkerSize', 10, 'MarkerFaceColor', colors(i,:), 'DisplayName', ['Robot ' num2str(i)]);
        end
        legend('Location', 'northeastoutside');
        drawnow;
    end
end

fprintf('Simulation Complete.\n');

%% Helper: Formation Offset Logic
function [dx, dy] = getFormationOffset(type, i, j)
    % Relative offset from i to j based on formation
    % This matches the C++ logic
    
    [xi, yi] = getTargetPos(type, i);
    [xj, yj] = getTargetPos(type, j);
    
    dx = xj - xi;
    dy = yj - yi;
end

function [tx, ty] = getTargetPos(type, id)
    tx = 0; ty = 0;
    if type == 1 % Triangle
        if id == 1, tx = 0; ty = 0;
        elseif id == 2, tx = -1000; ty = -1000;
        elseif id == 3, tx = -1000; ty = 1000;
        else tx = -1500 * (id-1); ty = 0;
        end
    elseif type == 2 % Circle
        radius = 1500;
        angle = (2 * pi / 5) * (id - 1);
        tx = radius * cos(angle);
        ty = radius * sin(angle);
    elseif type == 3 % Line
        tx = 0;
        ty = -1000 * (id - 1);
    end
end
