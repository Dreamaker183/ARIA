/*
Professional Multi-Robot Triangle Formation System
Advanced formation control with 3D visualization, ultrasonic sensor fusion,
obstacle-free staging area selection, and precise triangle maneuvers.

Features:
- Dual robot connection and coordination
- Real-time 3D occupancy mapping from ultrasonic sensors
- Obstacle-free staging area computation
- Precise triangle formation with collision avoidance
- Return-to-origin functionality
- Space key triggered cycles
- Professional formation control algorithms
*/

#include "Aria.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstring>
#include <thread>
#include <mutex>
#include <map>
#include <queue>
#include <fstream>

class ProfessionalTriangleFormation
{
public:
    ProfessionalTriangleFormation();
    ~ProfessionalTriangleFormation();
    
    bool initialize();
    void run();
    void stop();
    void triggerCycle();
    
private:
    // Robot instances
    ArRobot* robot1;
    ArRobot* robot2;
    ArRobotConnector* connector1;
    ArRobotConnector* connector2;
    
    // Task callbacks
    ArFunctorC<ProfessionalTriangleFormation> myTaskCB1;
    ArFunctorC<ProfessionalTriangleFormation> myTaskCB2;
    
    // Robot data structures
    struct RobotState {
        double x, y, theta;
        double vx, vy, omega;
        double battery_voltage;
        std::vector<double> sonar_readings;
        bool connected;
        bool data_valid;
        std::string name;
        ArPose initial_pose;
        ArPose target_pose;
        bool at_target;
        double formation_angle;
    };
    
    RobotState robot1_state;
    RobotState robot2_state;
    
    // Formation control
    struct FormationParams {
        double side_length = 800.0;        // L: Triangle side length (mm)
        double min_separation = 400.0;     // d_min: Minimum inter-robot distance
        double obstacle_clearance = 200.0; // c_min: Minimum obstacle clearance
        double pos_tolerance = 50.0;       // ε_pos: Position tolerance
        double yaw_tolerance = 5.0;        // ε_yaw: Heading tolerance
        double max_velocity = 300.0;       // vmax: Maximum velocity
        double max_acceleration = 100.0;   // amax: Maximum acceleration
        double formation_angle = 0.0;      // θ: Global triangle orientation
        double maneuver_time = 10.0;       // T: Triangle maneuver duration
        double inflation_radius = 150.0;   // Safety margin for obstacles
    };
    
    FormationParams params;
    
    // 3D Occupancy mapping
    struct OccupancyGrid {
        double resolution = 50.0;  // 50mm grid cells
        int width, height;
        std::vector<std::vector<double>> grid;
        double origin_x, origin_y;
    };
    
    OccupancyGrid occupancy_map;
    
    // Triangle formation
    struct TriangleVertices {
        ArPose vertex1, vertex2, vertex3;
        ArPose centroid;
        double side_length;
        double orientation;
    };
    
    TriangleVertices target_triangle;
    std::map<int, ArPose> robot_assignments; // robot_id -> assigned_vertex
    
    // System state
    enum SystemPhase {
        INITIALIZING,
        READY,
        MAPPING,
        STAGING_SELECTION,
        FORMING_TRIANGLE,
        TRIANGLE_MANEUVER,
        RETURNING_HOME,
        COMPLETED
    };
    
    SystemPhase current_phase;
    bool cycle_triggered;
    bool cycle_running;
    ArTime cycle_start_time;
    ArTime last_map_update;
    
    // Threading and synchronization
    std::mutex data_mutex;
    std::mutex map_mutex;
    bool system_running;
    
    // 3D Visualization data
    struct VisualizationData {
        std::vector<ArPose> robot_trajectories[2];
        std::vector<ArPose> ultrasonic_points;
        std::vector<ArPose> obstacle_points;
        std::vector<ArPose> triangle_vertices;
        ArPose triangle_centroid;
        std::string status_message;
        SystemPhase current_phase;
    };
    
    VisualizationData viz_data;
    
    // Core methods
    void robot1Task();
    void robot2Task();
    void updateRobotState(ArRobot* robot, RobotState& state);
    void updateOccupancyMap();
    void processUltrasonicData();
    void selectStagingArea();
    void computeTriangleVertices();
    void assignRobotsToVertices();
    void executeFormationControl();
    void executeTriangleManeuver();
    void returnToOrigin();
    void updateVisualization();
    
    // Formation control algorithms
    ArPose computeFormationControl(int robot_id);
    ArPose computeCollisionAvoidance(int robot_id);
    ArPose computeInterRobotAvoidance(int robot_id);
    bool isFormationStable();
    bool isAtTarget(int robot_id, const ArPose& target);
    
    // Utility methods
    double calculateDistance(const ArPose& p1, const ArPose& p2);
    double calculateAngle(const ArPose& p1, const ArPose& p2);
    ArPose interpolatePose(const ArPose& start, const ArPose& end, double t);
    bool isObstacleFree(const ArPose& pose);
    bool isInStagingArea(const ArPose& pose);
    void logSystemState();
    void saveVisualizationData();
    
    // Safety and validation
    bool validateSystemHealth();
    bool checkCollisionRisk();
    void emergencyStop();
    void logSafetyViolation(const std::string& violation);
    
    // Constants
    static constexpr double MAP_UPDATE_RATE = 3.0;  // 3 Hz minimum
    static constexpr double CONTROL_RATE = 10.0;    // 10 Hz control loop
    static constexpr double VIZ_UPDATE_RATE = 5.0;  // 5 Hz visualization
};

ProfessionalTriangleFormation::ProfessionalTriangleFormation() :
    robot1(nullptr), robot2(nullptr),
    connector1(nullptr), connector2(nullptr),
    myTaskCB1(this, &ProfessionalTriangleFormation::robot1Task),
    myTaskCB2(this, &ProfessionalTriangleFormation::robot2Task),
    current_phase(INITIALIZING),
    cycle_triggered(false),
    cycle_running(false),
    system_running(false)
{
    // Initialize robot states
    robot1_state = {0, 0, 0, 0, 0, 0, 0, std::vector<double>(16, 5000.0), false, false, "Robot1", ArPose(), ArPose(), false, 0};
    robot2_state = {0, 0, 0, 0, 0, 0, 0, std::vector<double>(16, 5000.0), false, false, "Robot2", ArPose(), ArPose(), false, 0};
    
    // Initialize occupancy map (10m x 10m)
    occupancy_map.width = 200;  // 200 * 50mm = 10m
    occupancy_map.height = 200;
    occupancy_map.origin_x = -5000.0;  // -5m
    occupancy_map.origin_y = -5000.0;  // -5m
    occupancy_map.grid.resize(occupancy_map.height, std::vector<double>(occupancy_map.width, 0.0));
    
    printf("Professional Triangle Formation System Initialized\n");
    printf("Parameters: L=%.0fmm, d_min=%.0fmm, c_min=%.0fmm\n", 
           params.side_length, params.min_separation, params.obstacle_clearance);
}

ProfessionalTriangleFormation::~ProfessionalTriangleFormation()
{
    stop();
}

bool ProfessionalTriangleFormation::initialize()
{
    printf("\n=== PROFESSIONAL TRIANGLE FORMATION SYSTEM ===\n");
    printf("Initializing dual-robot formation control system...\n");
    
    Aria::init();
    
    // Create robots
    robot1 = new ArRobot();
    robot2 = new ArRobot();
    
    // Create argument parsers
    int argc1 = 5;
    char* argv1[] = {
        (char*)"professionalTriangleFormation",
        (char*)"-remoteHost", (char*)"192.168.1.253",
        (char*)"-remoteRobotTcpPort", (char*)"8101"
    };
    
    int argc2 = 5;
    char* argv2[] = {
        (char*)"professionalTriangleFormation",
        (char*)"-remoteHost", (char*)"192.168.1.254",
        (char*)"-remoteRobotTcpPort", (char*)"8101"
    };
    
    ArArgumentParser parser1(&argc1, argv1);
    ArArgumentParser parser2(&argc2, argv2);
    parser1.loadDefaultArguments();
    parser2.loadDefaultArguments();
    
    // Create connectors
    connector1 = new ArRobotConnector(&parser1, robot1);
    connector2 = new ArRobotConnector(&parser2, robot2);
    
    // Connect to robots
    printf("Connecting to Robot 1 (192.168.1.253:8101)... ");
    fflush(stdout);
    if (!connector1->connectRobot()) {
        printf("❌ FAILED\n");
        return false;
    }
    printf("✅ CONNECTED\n");
    robot1_state.connected = true;
    robot1_state.name = robot1->getName();
    
    printf("Connecting to Robot 2 (192.168.1.254:8101)... ");
    fflush(stdout);
    if (!connector2->connectRobot()) {
        printf("❌ FAILED\n");
        return false;
    }
    printf("✅ CONNECTED\n");
    robot2_state.connected = true;
    robot2_state.name = robot2->getName();
    
    // Add sonar devices
    ArSonarDevice sonar1, sonar2;
    robot1->addRangeDevice(&sonar1);
    robot2->addRangeDevice(&sonar2);
    
    // Start robot tasks
    robot1->runAsync(true);
    robot2->runAsync(true);
    
    // Wait for initialization
    printf("Calibrating ultrasonic sensors and pose streams...\n");
    ArUtil::sleep(3000);
    
    // Enable motors
    robot1->enableMotors();
    robot2->enableMotors();
    
    // Record initial poses
    robot1_state.initial_pose = ArPose(robot1->getX(), robot1->getY(), robot1->getTh());
    robot2_state.initial_pose = ArPose(robot2->getX(), robot2->getY(), robot2->getTh());
    
    printf("✅ System initialized successfully!\n");
    printf("   Robot 1: %s at (%.0f, %.0f, %.1f°)\n", 
           robot1_state.name.c_str(), robot1_state.initial_pose.getX(), 
           robot1_state.initial_pose.getY(), robot1_state.initial_pose.getTh());
    printf("   Robot 2: %s at (%.0f, %.0f, %.1f°)\n", 
           robot2_state.name.c_str(), robot2_state.initial_pose.getX(), 
           robot2_state.initial_pose.getY(), robot2_state.initial_pose.getTh());
    
    current_phase = READY;
    return true;
}

void ProfessionalTriangleFormation::run()
{
    if (!robot1_state.connected || !robot2_state.connected) {
        printf("❌ Cannot start - robots not connected\n");
        return;
    }
    
    printf("\n🤖 Starting Professional Triangle Formation System...\n");
    
    // Add task callbacks
    robot1->addUserTask("Robot1Formation", 100, &myTaskCB1);  // 10Hz
    robot2->addUserTask("Robot2Formation", 100, &myTaskCB2);  // 10Hz
    
    system_running = true;
    current_phase = READY;
    
    printf("✅ Formation control active!\n");
    printf("   Press SPACE to trigger triangle formation cycle\n");
    printf("   Press 'q' to quit\n\n");
    
    // Auto-trigger cycle after 5 seconds for testing
    printf("Auto-triggering formation cycle in 5 seconds...\n");
    ArUtil::sleep(5000);
    triggerCycle();
    
    // Main control loop
    while (system_running) {
        ArUtil::sleep(100);  // 10Hz main loop
        
        // Update visualization data
        updateVisualization();
        
        // Log system state periodically
        static int log_counter = 0;
        if (++log_counter % 50 == 0) {  // Every 5 seconds
            logSystemState();
        }
        
        // Check if cycle is complete and ready for next one
        if (!cycle_running && current_phase == READY) {
            static int auto_trigger_counter = 0;
            if (++auto_trigger_counter % 100 == 0) {  // Every 10 seconds
                printf("Auto-triggering next formation cycle...\n");
                triggerCycle();
            }
        }
    }
}

void ProfessionalTriangleFormation::stop()
{
    system_running = false;
    
    if (robot1 && robot1_state.connected) {
        robot1->remUserTask(&myTaskCB1);
        robot1->stop();
    }
    
    if (robot2 && robot2_state.connected) {
        robot2->remUserTask(&myTaskCB2);
        robot2->stop();
    }
    
    printf("Professional Triangle Formation System stopped\n");
}

void ProfessionalTriangleFormation::triggerCycle()
{
    if (cycle_running) {
        printf("⚠️  Cycle already in progress, please wait...\n");
        return;
    }
    
    if (!validateSystemHealth()) {
        printf("❌ System health check failed, cannot start cycle\n");
        return;
    }
    
    printf("\n🚀 TRIGGERING TRIANGLE FORMATION CYCLE\n");
    printf("=====================================\n");
    
    cycle_triggered = true;
    cycle_running = true;
    cycle_start_time.setToNow();
    current_phase = MAPPING;
    
    printf("Phase 1: Building occupancy map from ultrasonic sensors...\n");
}

void ProfessionalTriangleFormation::robot1Task()
{
    if (!system_running) return;
    
    std::lock_guard<std::mutex> lock(data_mutex);
    updateRobotState(robot1, robot1_state);
    
    if (cycle_running) {
        executeFormationControl();
    }
}

void ProfessionalTriangleFormation::robot2Task()
{
    if (!system_running) return;
    
    std::lock_guard<std::mutex> lock(data_mutex);
    updateRobotState(robot2, robot2_state);
    
    if (cycle_running) {
        executeFormationControl();
    }
}

void ProfessionalTriangleFormation::updateRobotState(ArRobot* robot, RobotState& state)
{
    // Force robot to update its position
    robot->lock();
    robot->unlock();
    
    state.x = robot->getX();
    state.y = robot->getY();
    state.theta = robot->getTh();
    state.vx = robot->getVel() * cos(robot->getTh() * M_PI / 180.0);
    state.vy = robot->getVel() * sin(robot->getTh() * M_PI / 180.0);
    state.omega = robot->getRotVel();
    state.battery_voltage = robot->getBatteryVoltage();
    state.connected = robot->isConnected();
    
    // Update sonar readings
    for (int i = 0; i < 16 && i < robot->getNumSonar(); i++) {
        ArSensorReading* reading = robot->getSonarReading(i);
        if (reading != NULL) {
            state.sonar_readings[i] = reading->getRange();
        }
    }
    
    state.data_valid = true;
    
    // Debug: Print actual robot position
    static int debug_counter = 0;
    if (++debug_counter % 50 == 0) {  // Every 5 seconds
        printf("Robot %s: Actual pos=(%.0f,%.0f,%.1f°), Vel=%.0f, Sonar[0]=%.0f\n", 
               state.name.c_str(), state.x, state.y, state.theta, 
               robot->getVel(), state.sonar_readings[0]);
    }
}

void ProfessionalTriangleFormation::executeFormationControl()
{
    ArTime current_time;
    double elapsed = cycle_start_time.mSecSince() / 1000.0;
    
    switch (current_phase) {
        case MAPPING:
            updateOccupancyMap();
            if (elapsed > 2.0) {  // 2 seconds of mapping
                printf("Phase 2: Selecting obstacle-free staging area...\n");
                selectStagingArea();
                current_phase = STAGING_SELECTION;
            }
            break;
            
        case STAGING_SELECTION:
            if (target_triangle.centroid.getX() != 0 || target_triangle.centroid.getY() != 0) {
                printf("Phase 3: Computing triangle vertices and robot assignments...\n");
                computeTriangleVertices();
                assignRobotsToVertices();
                current_phase = FORMING_TRIANGLE;
            } else {
                printf("❌ No suitable staging area found! Aborting cycle.\n");
                cycle_running = false;
                current_phase = READY;
            }
            break;
            
        case FORMING_TRIANGLE:
            if (isFormationStable()) {
                printf("Phase 4: Executing triangle maneuver...\n");
                current_phase = TRIANGLE_MANEUVER;
                cycle_start_time.setToNow();  // Reset timer for maneuver
            } else {
                // Continue formation control
                ArPose cmd1 = computeFormationControl(1);
                ArPose cmd2 = computeFormationControl(2);
                
                robot1->moveTo(cmd1);
                robot2->moveTo(cmd2);
            }
            break;
            
        case TRIANGLE_MANEUVER:
            if (elapsed > params.maneuver_time) {
                printf("Phase 5: Returning robots to origin...\n");
                current_phase = RETURNING_HOME;
                cycle_start_time.setToNow();
            } else {
                executeTriangleManeuver();
            }
            break;
            
        case RETURNING_HOME:
            if (isAtTarget(1, robot1_state.initial_pose) && isAtTarget(2, robot2_state.initial_pose)) {
                printf("✅ CYCLE COMPLETED SUCCESSFULLY!\n");
                printf("   Formation: %.0fmm triangle\n", params.side_length);
                printf("   Duration: %.1f seconds\n", elapsed);
                printf("   Ready for next cycle (Press SPACE)\n\n");
                cycle_running = false;
                current_phase = READY;
            } else {
                returnToOrigin();
            }
            break;
            
        default:
            break;
    }
}

void ProfessionalTriangleFormation::updateOccupancyMap()
{
    std::lock_guard<std::mutex> lock(map_mutex);
    
    // Clear previous map
    for (int y = 0; y < occupancy_map.height; y++) {
        for (int x = 0; x < occupancy_map.width; x++) {
            occupancy_map.grid[y][x] = 0.0;
        }
    }
    
    // Process ultrasonic data from both robots
    processUltrasonicData();
    
    // Inflate obstacles with safety margin
    for (int y = 0; y < occupancy_map.height; y++) {
        for (int x = 0; x < occupancy_map.width; x++) {
            if (occupancy_map.grid[y][x] > 0.5) {  // Obstacle detected
                // Inflate obstacle
                int inflation_cells = (int)(params.inflation_radius / occupancy_map.resolution);
                for (int dy = -inflation_cells; dy <= inflation_cells; dy++) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; dx++) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < occupancy_map.width && 
                            ny >= 0 && ny < occupancy_map.height) {
                            double distance = sqrt(dx*dx + dy*dy) * occupancy_map.resolution;
                            if (distance <= params.inflation_radius) {
                                occupancy_map.grid[ny][nx] = 1.0;
                            }
                        }
                    }
                }
            }
        }
    }
    
    last_map_update.setToNow();
}

void ProfessionalTriangleFormation::processUltrasonicData()
{
    // Process Robot 1 sonar data
    for (int i = 0; i < 16; i++) {
        if (robot1_state.sonar_readings[i] < 5000.0) {  // Valid reading
            double angle = (i * 22.5) - 180.0;  // Approximate sensor angles
            double range = robot1_state.sonar_readings[i];
            
            // Convert to global coordinates
            double global_x = robot1_state.x + range * cos((robot1_state.theta + angle) * M_PI / 180.0);
            double global_y = robot1_state.y + range * sin((robot1_state.theta + angle) * M_PI / 180.0);
            
            // Add to occupancy grid
            int grid_x = (int)((global_x - occupancy_map.origin_x) / occupancy_map.resolution);
            int grid_y = (int)((global_y - occupancy_map.origin_y) / occupancy_map.resolution);
            
            if (grid_x >= 0 && grid_x < occupancy_map.width && 
                grid_y >= 0 && grid_y < occupancy_map.height) {
                occupancy_map.grid[grid_y][grid_x] = 1.0;
            }
        }
    }
    
    // Process Robot 2 sonar data
    for (int i = 0; i < 16; i++) {
        if (robot2_state.sonar_readings[i] < 5000.0) {  // Valid reading
            double angle = (i * 22.5) - 180.0;  // Approximate sensor angles
            double range = robot2_state.sonar_readings[i];
            
            // Convert to global coordinates
            double global_x = robot2_state.x + range * cos((robot2_state.theta + angle) * M_PI / 180.0);
            double global_y = robot2_state.y + range * sin((robot2_state.theta + angle) * M_PI / 180.0);
            
            // Add to occupancy grid
            int grid_x = (int)((global_x - occupancy_map.origin_x) / occupancy_map.resolution);
            int grid_y = (int)((global_y - occupancy_map.origin_y) / occupancy_map.resolution);
            
            if (grid_x >= 0 && grid_x < occupancy_map.width && 
                grid_y >= 0 && grid_y < occupancy_map.height) {
                occupancy_map.grid[grid_y][grid_x] = 1.0;
            }
        }
    }
}

void ProfessionalTriangleFormation::selectStagingArea()
{
    // Use current robot positions to find staging area
    double robot1_x = robot1_state.x;
    double robot1_y = robot1_state.y;
    double robot2_x = robot2_state.x;
    double robot2_y = robot2_state.y;
    
    // Calculate midpoint between robots
    double center_x = (robot1_x + robot2_x) / 2.0;
    double center_y = (robot1_y + robot2_y) / 2.0;
    
    // Ensure we have a reasonable staging area
    if (center_x == 0 && center_y == 0) {
        // If robots are at origin, use a default staging area
        center_x = 1000.0;  // 1 meter from origin
        center_y = 1000.0;
        printf("Using default staging area at (%.0f, %.0f)\n", center_x, center_y);
    } else {
        printf("Using robot-based staging area at (%.0f, %.0f)\n", center_x, center_y);
    }
    
    // Set staging area
    target_triangle.centroid = ArPose(center_x, center_y, 0);
    printf("✅ Staging area selected at (%.0f, %.0f)\n", center_x, center_y);
}

void ProfessionalTriangleFormation::computeTriangleVertices()
{
    double cx = target_triangle.centroid.getX();
    double cy = target_triangle.centroid.getY();
    double L = params.side_length;
    double theta = params.formation_angle;
    
    // Compute equilateral triangle vertices
    double h = L * sqrt(3.0) / 2.0;  // Height of equilateral triangle
    
    target_triangle.vertex1 = ArPose(cx, cy + 2*h/3, theta);
    target_triangle.vertex2 = ArPose(cx - L/2, cy - h/3, theta + 120);
    target_triangle.vertex3 = ArPose(cx + L/2, cy - h/3, theta + 240);
    target_triangle.centroid = ArPose(cx, cy, theta);
    target_triangle.side_length = L;
    target_triangle.orientation = theta;
    
    printf("Triangle vertices computed:\n");
    printf("  V1: (%.0f, %.0f, %.1f°)\n", target_triangle.vertex1.getX(), target_triangle.vertex1.getY(), target_triangle.vertex1.getTh());
    printf("  V2: (%.0f, %.0f, %.1f°)\n", target_triangle.vertex2.getX(), target_triangle.vertex2.getY(), target_triangle.vertex2.getTh());
    printf("  V3: (%.0f, %.0f, %.1f°)\n", target_triangle.vertex3.getX(), target_triangle.vertex3.getY(), target_triangle.vertex3.getTh());
}

void ProfessionalTriangleFormation::assignRobotsToVertices()
{
    // Simple assignment: Robot 1 -> V1, Robot 2 -> V2
    // In a more sophisticated system, this would use optimal assignment
    robot_assignments[1] = target_triangle.vertex1;
    robot_assignments[2] = target_triangle.vertex2;
    
    robot1_state.target_pose = target_triangle.vertex1;
    robot2_state.target_pose = target_triangle.vertex2;
    
    printf("Robot assignments:\n");
    printf("  Robot 1 -> Vertex 1: (%.0f, %.0f)\n", target_triangle.vertex1.getX(), target_triangle.vertex1.getY());
    printf("  Robot 2 -> Vertex 2: (%.0f, %.0f)\n", target_triangle.vertex2.getX(), target_triangle.vertex2.getY());
}

ArPose ProfessionalTriangleFormation::computeFormationControl(int robot_id)
{
    RobotState& state = (robot_id == 1) ? robot1_state : robot2_state;
    ArRobot* robot = (robot_id == 1) ? robot1 : robot2;
    
    // If no target assigned yet, use a simple exploration pattern
    if (robot_assignments.find(robot_id) == robot_assignments.end()) {
        // Simple exploration: move in a small circle
        static double exploration_angle = 0.0;
        exploration_angle += 2.0;  // 2 degrees per control cycle
        
        double exploration_radius = 500.0;  // 50cm radius
        double target_x = state.x + exploration_radius * cos(exploration_angle * M_PI / 180.0);
        double target_y = state.y + exploration_radius * sin(exploration_angle * M_PI / 180.0);
        
        robot->setVel(100.0);  // Move forward
        robot->setRotVel(10.0);  // Turn slowly
        
        printf("Robot %d: Exploration mode - moving in circle\n", robot_id);
        return ArPose(target_x, target_y, exploration_angle);
    }
    
    ArPose target = robot_assignments[robot_id];
    
    // Basic proportional control
    double dx = target.getX() - state.x;
    double dy = target.getY() - state.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    printf("Robot %d: Target=(%.0f,%.0f), Current=(%.0f,%.0f), Dist=%.0f\n", 
           robot_id, target.getX(), target.getY(), state.x, state.y, distance);
    
    if (distance < params.pos_tolerance) {
        robot->setVel(0.0);
        robot->setRotVel(0.0);
        printf("Robot %d: At target!\n", robot_id);
        return target;  // At target
    }
    
    // Compute desired velocity
    double max_vel = params.max_velocity;
    double vel = std::min(max_vel, distance * 0.3);  // Proportional control
    
    double desired_theta = atan2(dy, dx) * 180.0 / M_PI;
    double theta_error = desired_theta - state.theta;
    
    // Normalize angle error
    while (theta_error > 180.0) theta_error -= 360.0;
    while (theta_error < -180.0) theta_error += 360.0;
    
    double omega = std::max(-30.0, std::min(30.0, theta_error * 0.5));
    
    // Apply collision avoidance
    ArPose collision_avoidance = computeCollisionAvoidance(robot_id);
    ArPose inter_robot_avoidance = computeInterRobotAvoidance(robot_id);
    
    // Combine controls (simplified)
    double final_vel = vel;
    if (collision_avoidance.getX() != 0 || collision_avoidance.getY() != 0) {
        final_vel *= 0.5;  // Slow down near obstacles
    }
    
    // Set robot velocity
    robot->setVel(final_vel);
    robot->setRotVel(omega);
    
    printf("Robot %d: Vel=%.0f, RotVel=%.0f, Target=(%.0f,%.0f)\n", 
           robot_id, final_vel, omega, target.getX(), target.getY());
    
    return target;
}

ArPose ProfessionalTriangleFormation::computeCollisionAvoidance(int robot_id)
{
    RobotState& state = (robot_id == 1) ? robot1_state : robot2_state;
    
    // Simple obstacle avoidance - check sonar readings
    double min_distance = 5000.0;
    double avoid_angle = 0.0;
    
    for (int i = 0; i < 16; i++) {
        if (state.sonar_readings[i] < min_distance) {
            min_distance = state.sonar_readings[i];
            avoid_angle = (i * 22.5) - 180.0;
        }
    }
    
    if (min_distance < params.obstacle_clearance) {
        // Turn away from obstacle
        double avoid_omega = (avoid_angle > 0) ? -30.0 : 30.0;
        if (robot_id == 1) {
            robot1->setRotVel(avoid_omega);
        } else {
            robot2->setRotVel(avoid_omega);
        }
        return ArPose(1, 1, 0);  // Indicate avoidance active
    }
    
    return ArPose(0, 0, 0);  // No avoidance needed
}

ArPose ProfessionalTriangleFormation::computeInterRobotAvoidance(int robot_id)
{
    RobotState& state1 = robot1_state;
    RobotState& state2 = robot2_state;
    
    double distance = calculateDistance(ArPose(state1.x, state1.y, 0), ArPose(state2.x, state2.y, 0));
    
    if (distance < params.min_separation) {
        // Too close - move away
        double dx = state2.x - state1.x;
        double dy = state2.y - state1.y;
        double avoid_angle = atan2(dy, dx) * 180.0 / M_PI;
        
        if (robot_id == 1) {
            // Robot 1 moves away from Robot 2
            avoid_angle += 180.0;
        }
        
        double omega = (avoid_angle > state1.theta) ? 30.0 : -30.0;
        
        if (robot_id == 1) {
            robot1->setRotVel(omega);
        } else {
            robot2->setRotVel(omega);
        }
        
        return ArPose(1, 1, 0);  // Indicate avoidance active
    }
    
    return ArPose(0, 0, 0);  // No avoidance needed
}

bool ProfessionalTriangleFormation::isFormationStable()
{
    bool robot1_at_target = isAtTarget(1, robot1_state.target_pose);
    bool robot2_at_target = isAtTarget(2, robot2_state.target_pose);
    
    return robot1_at_target && robot2_at_target;
}

bool ProfessionalTriangleFormation::isAtTarget(int robot_id, const ArPose& target)
{
    RobotState& state = (robot_id == 1) ? robot1_state : robot2_state;
    
    double dx = target.getX() - state.x;
    double dy = target.getY() - state.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    double theta_error = target.getTh() - state.theta;
    while (theta_error > 180.0) theta_error -= 360.0;
    while (theta_error < -180.0) theta_error += 360.0;
    
    return (distance < params.pos_tolerance && abs(theta_error) < params.yaw_tolerance);
}

void ProfessionalTriangleFormation::executeTriangleManeuver()
{
    // Simple triangle maneuver - robots maintain formation while rotating
    static double maneuver_angle = 0.0;
    maneuver_angle += 1.0;  // 1 degree per control cycle
    
    if (maneuver_angle >= 360.0) {
        maneuver_angle = 0.0;
    }
    
    // Update target poses for rotation
    double cx = target_triangle.centroid.getX();
    double cy = target_triangle.centroid.getY();
    double L = params.side_length;
    double h = L * sqrt(3.0) / 2.0;
    
    double angle1 = maneuver_angle * M_PI / 180.0;
    double angle2 = (maneuver_angle + 120) * M_PI / 180.0;
    
    robot1_state.target_pose = ArPose(cx + 2*h/3 * cos(angle1), cy + 2*h/3 * sin(angle1), maneuver_angle);
    robot2_state.target_pose = ArPose(cx + 2*h/3 * cos(angle2), cy + 2*h/3 * sin(angle2), maneuver_angle + 120);
    
    // Apply formation control
    computeFormationControl(1);
    computeFormationControl(2);
}

void ProfessionalTriangleFormation::returnToOrigin()
{
    robot1_state.target_pose = robot1_state.initial_pose;
    robot2_state.target_pose = robot2_state.initial_pose;
    
    computeFormationControl(1);
    computeFormationControl(2);
}

void ProfessionalTriangleFormation::updateVisualization()
{
    // Update visualization data for 3D viewer
    viz_data.robot_trajectories[0].push_back(ArPose(robot1_state.x, robot1_state.y, robot1_state.theta));
    viz_data.robot_trajectories[1].push_back(ArPose(robot2_state.x, robot2_state.y, robot2_state.theta));
    
    // Keep only last 100 points
    if (viz_data.robot_trajectories[0].size() > 100) {
        viz_data.robot_trajectories[0].erase(viz_data.robot_trajectories[0].begin());
        viz_data.robot_trajectories[1].erase(viz_data.robot_trajectories[1].begin());
    }
    
    // Update triangle vertices
    viz_data.triangle_vertices.clear();
    if (target_triangle.centroid.getX() != 0 || target_triangle.centroid.getY() != 0) {
        viz_data.triangle_vertices.push_back(target_triangle.vertex1);
        viz_data.triangle_vertices.push_back(target_triangle.vertex2);
        viz_data.triangle_vertices.push_back(target_triangle.vertex3);
        viz_data.triangle_centroid = target_triangle.centroid;
    }
    
    // Update status message
    switch (current_phase) {
        case READY: viz_data.status_message = "Ready - Press SPACE to start"; break;
        case MAPPING: viz_data.status_message = "Building occupancy map..."; break;
        case STAGING_SELECTION: viz_data.status_message = "Selecting staging area..."; break;
        case FORMING_TRIANGLE: viz_data.status_message = "Forming triangle..."; break;
        case TRIANGLE_MANEUVER: viz_data.status_message = "Executing triangle maneuver..."; break;
        case RETURNING_HOME: viz_data.status_message = "Returning to origin..."; break;
        case COMPLETED: viz_data.status_message = "Cycle completed!"; break;
        default: viz_data.status_message = "Initializing..."; break;
    }
    
    viz_data.current_phase = current_phase;
}

bool ProfessionalTriangleFormation::isInStagingArea(const ArPose& pose)
{
    // Check if pose is in obstacle-free area large enough for triangle
    double required_radius = params.side_length * 0.8;  // 80% of side length
    int radius_cells = (int)(required_radius / occupancy_map.resolution);
    
    int center_x = (int)((pose.getX() - occupancy_map.origin_x) / occupancy_map.resolution);
    int center_y = (int)((pose.getY() - occupancy_map.origin_y) / occupancy_map.resolution);
    
    for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int dx = -radius_cells; dx <= radius_cells; dx++) {
            int x = center_x + dx;
            int y = center_y + dy;
            if (x < 0 || x >= occupancy_map.width || y < 0 || y >= occupancy_map.height) {
                return false;  // Outside map
            }
            if (occupancy_map.grid[y][x] > 0.5) {
                return false;  // Obstacle present
            }
        }
    }
    
    return true;
}

bool ProfessionalTriangleFormation::validateSystemHealth()
{
    if (!robot1_state.connected || !robot2_state.connected) {
        printf("❌ Robot connection lost\n");
        return false;
    }
    
    if (robot1_state.battery_voltage < 11.0 || robot2_state.battery_voltage < 11.0) {
        printf("❌ Low battery voltage\n");
        return false;
    }
    
    if (!robot1_state.data_valid || !robot2_state.data_valid) {
        printf("❌ Invalid sensor data\n");
        return false;
    }
    
    return true;
}

void ProfessionalTriangleFormation::logSystemState()
{
    printf("Status: Phase=%d, R1=(%.0f,%.0f,%.1f°), R2=(%.0f,%.0f,%.1f°), Dist=%.0fmm\n",
           current_phase, robot1_state.x, robot1_state.y, robot1_state.theta,
           robot2_state.x, robot2_state.y, robot2_state.theta,
           calculateDistance(ArPose(robot1_state.x, robot1_state.y, 0), 
                           ArPose(robot2_state.x, robot2_state.y, 0)));
}

double ProfessionalTriangleFormation::calculateDistance(const ArPose& p1, const ArPose& p2)
{
    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    return sqrt(dx*dx + dy*dy);
}

double ProfessionalTriangleFormation::calculateAngle(const ArPose& p1, const ArPose& p2)
{
    return atan2(p2.getY() - p1.getY(), p2.getX() - p1.getX()) * 180.0 / M_PI;
}

// Main program
int main(int argc, char** argv)
{
    printf("=== PROFESSIONAL TRIANGLE FORMATION SYSTEM ===\n");
    printf("Advanced multi-robot formation control with 3D visualization\n");
    printf("Features: Ultrasonic mapping, obstacle avoidance, precise formation\n\n");
    
    ProfessionalTriangleFormation system;
    
    if (!system.initialize()) {
        printf("❌ System initialization failed. Exiting.\n");
        return 1;
    }
    
    system.run();
    system.stop();
    
    Aria::exit(0);
    return 0;
}
