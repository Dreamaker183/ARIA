/*
Professional Consensual Movements Algorithm for Pioneer 3DX Robot
Copyright (C) 2024

This program implements a sophisticated behavior-based navigation system
using consensual movements with sonar sensor fusion for obstacle avoidance,
goal seeking, and smooth navigation.

Robot: Pioneer 3DX (p3dx-sh) with 16 sonar sensors + ZLAN 7104 WiFi Module
Features:
- ZLAN 7104 WiFi module support with auto-discovery
- WiFi pairing and connection management
- SSH/TCP connection support  
- Manual start control (spacebar activation)
- Multi-behavior consensus algorithm
- Dynamic obstacle avoidance
- Smooth trajectory planning
- Sensor fusion and filtering
- Adaptive speed control
- Emergency stop behaviors
*/

#include "Aria.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstring>

class ConsensualMovements
{
public:
    ConsensualMovements(ArRobot* robot);
    ~ConsensualMovements();
    
    void run();
    void stop();
    void waitForStartCommand();
    void setMovementEnabled(bool enabled);
    
    // WiFi module support
    bool discoverZLAN7104();
    bool connectToZLAN7104(const std::string& ip_address, int port = 8101);
    void scanForRobots();
    std::string getZLANIP();
    
private:
    // Core robot interface
    ArRobot* myRobot;
    ArFunctorC<ConsensualMovements> myTaskCB;
    
    // Behavior weights and parameters
    struct BehaviorWeights {
        double obstacle_avoidance = 0.8;
        double goal_seeking = 0.6;
        double wall_following = 0.4;
        double exploration = 0.3;
        double smooth_motion = 0.5;
    };
    
    // Sensor data structure
    struct SensorData {
        std::vector<double> sonar_readings;
        std::vector<double> filtered_readings;
        double min_front_distance;
        double min_side_distance;
        bool obstacle_detected;
        double obstacle_angle;
    };
    
    // Movement command structure
    struct MovementCommand {
        double linear_velocity;
        double angular_velocity;
        double confidence;
        std::string behavior_name;
    };
    
    // Navigation state
    struct NavigationState {
        double current_x, current_y, current_theta;
        double goal_x, goal_y;
        double last_turn_time;
        bool exploring;
        bool wall_following_active;
        int stuck_counter;
        bool movement_enabled;
        bool system_ready;
    };
    
    BehaviorWeights myWeights;
    SensorData mySensorData;
    NavigationState myNavState;
    
    // WiFi connection management
    std::string myZLANIP;
    int myZLANPort;
    bool myWiFiConnected;
    
    // Core methods
    void consensualTask();
    void updateSensorData();
    void filterSensorReadings();
    
    // Behavior methods
    MovementCommand obstacleAvoidanceBehavior();
    MovementCommand goalSeekingBehavior();
    MovementCommand wallFollowingBehavior();
    MovementCommand explorationBehavior();
    MovementCommand smoothMotionBehavior();
    MovementCommand emergencyStopBehavior();
    
    // Consensus and execution
    MovementCommand consensusDecision(const std::vector<MovementCommand>& commands);
    void executeMovement(const MovementCommand& cmd);
    
    // Utility methods
    double normalizeAngle(double angle);
    double calculateDistance(double x1, double y1, double x2, double y2);
    double calculateAngleToGoal();
    bool isPathClear(double angle, double distance);
    void updateNavigationState();
    void setRandomGoal();
    
    // Safety and monitoring
    bool isSafeToMove();
    void logBehaviorDecision(const MovementCommand& cmd);
    
    // Constants
    static const double MAX_LINEAR_VEL;
    static const double MAX_ANGULAR_VEL;
    static const double SAFE_DISTANCE;
    static const double CRITICAL_DISTANCE;
    static const int SONAR_COUNT;
};

// Constants definition
const double ConsensualMovements::MAX_LINEAR_VEL = 400.0;  // mm/s
const double ConsensualMovements::MAX_ANGULAR_VEL = 60.0;  // deg/s
const double ConsensualMovements::SAFE_DISTANCE = 800.0;   // mm
const double ConsensualMovements::CRITICAL_DISTANCE = 400.0; // mm
const int ConsensualMovements::SONAR_COUNT = 16;

ConsensualMovements::ConsensualMovements(ArRobot* robot) :
    myRobot(robot),
    myTaskCB(this, &ConsensualMovements::consensualTask)
{
    // Initialize sensor data
    mySensorData.sonar_readings.resize(SONAR_COUNT, 5000.0);
    mySensorData.filtered_readings.resize(SONAR_COUNT, 5000.0);
    mySensorData.obstacle_detected = false;
    
    // Initialize navigation state
    myNavState.current_x = 0.0;
    myNavState.current_y = 0.0;
    myNavState.current_theta = 0.0;
    myNavState.exploring = true;
    myNavState.wall_following_active = false;
    myNavState.stuck_counter = 0;
    myNavState.last_turn_time = 0.0;
    myNavState.movement_enabled = false;  // Start with movement disabled
    myNavState.system_ready = false;
    
    // Initialize WiFi settings
    myZLANIP = "";
    myZLANPort = 5000;  // User's ZLAN 7104 port (discovered via port scan)
    myWiFiConnected = false;
    
    // Set initial random goal
    setRandomGoal();
    
    printf("ConsensualMovements: Initialized for Pioneer 3DX\n");
    printf("ConsensualMovements: Robot has %d sonar sensors\n", SONAR_COUNT);
}

ConsensualMovements::~ConsensualMovements()
{
    stop();
}

void ConsensualMovements::run()
{
    printf("ConsensualMovements: Starting consensual navigation system\n");
    myRobot->addUserTask("ConsensualMovements", 50, &myTaskCB);
    myRobot->enableMotors();
    myNavState.system_ready = true;
    
    printf("ConsensualMovements: System ready - Press SPACE to start movement\n");
}

void ConsensualMovements::waitForStartCommand()
{
    printf("\n🎮 CONSENSUAL MOVEMENTS READY!\n");
    printf("Robot: %s (Pioneer 3DX)\n", myRobot->getName());
    printf("Status: Connected and ready for autonomous navigation\n\n");
    
    printf("Press SPACE to start movement, 'q' to quit: ");
    fflush(stdout);
    
    int ch = getchar();
    if (ch == ' ' || ch == '\n') {
        myNavState.movement_enabled = true;
        printf("\n🚀 CONSENSUAL MOVEMENTS ACTIVE!\n");
        printf("Robot is now navigating autonomously!\n");
        printf("Press Ctrl+C to stop safely\n\n");
    } else if (ch == 'q' || ch == 'Q') {
        printf("\n👋 Exiting consensual movements...\n");
        return;
    }
}

void ConsensualMovements::setMovementEnabled(bool enabled)
{
    myNavState.movement_enabled = enabled;
    if (enabled) {
        ArLog::log(ArLog::Normal, "ConsensualMovements: Movement ENABLED");
    } else {
        ArLog::log(ArLog::Normal, "ConsensualMovements: Movement DISABLED");
        myRobot->stop();
    }
}

void ConsensualMovements::stop()
{
    ArLog::log(ArLog::Normal, "ConsensualMovements: Stopping navigation system");
    myRobot->remUserTask(&myTaskCB);
    myRobot->stop();
}

void ConsensualMovements::consensualTask()
{
    // Update all sensor data and robot state
    updateSensorData();
    updateNavigationState();
    
    // Generate behavior commands
    std::vector<MovementCommand> behaviors;
    
    // Always check emergency stop first
    MovementCommand emergency = emergencyStopBehavior();
    if (emergency.confidence > 0.9) {
        executeMovement(emergency);
        return;
    }
    
    // Generate all behavior suggestions
    behaviors.push_back(obstacleAvoidanceBehavior());
    behaviors.push_back(goalSeekingBehavior());
    behaviors.push_back(wallFollowingBehavior());
    behaviors.push_back(explorationBehavior());
    behaviors.push_back(smoothMotionBehavior());
    
    // Make consensual decision
    MovementCommand consensus = consensusDecision(behaviors);
    
    // Execute the agreed-upon movement
    executeMovement(consensus);
    
    // Log decision for monitoring
    logBehaviorDecision(consensus);
}

void ConsensualMovements::updateSensorData()
{
    // Get sonar readings
    for (int i = 0; i < SONAR_COUNT && i < myRobot->getNumSonar(); i++) {
        ArSensorReading* reading = myRobot->getSonarReading(i);
        if (reading != NULL) {
            mySensorData.sonar_readings[i] = reading->getRange();
        }
    }
    
    // Apply filtering
    filterSensorReadings();
    
    // Analyze sensor data
    mySensorData.min_front_distance = 5000.0;
    mySensorData.min_side_distance = 5000.0;
    
    // Front sensors (0-3, 12-15 for P3DX)
    for (int i : {0, 1, 2, 3, 12, 13, 14, 15}) {
        if (i < mySensorData.filtered_readings.size()) {
            mySensorData.min_front_distance = std::min(mySensorData.min_front_distance, 
                                                      mySensorData.filtered_readings[i]);
        }
    }
    
    // Side sensors (4-11)
    for (int i = 4; i < 12 && i < mySensorData.filtered_readings.size(); i++) {
        mySensorData.min_side_distance = std::min(mySensorData.min_side_distance, 
                                                 mySensorData.filtered_readings[i]);
    }
    
    // Detect obstacles
    mySensorData.obstacle_detected = (mySensorData.min_front_distance < SAFE_DISTANCE);
    
    // Calculate obstacle angle (weighted average)
    if (mySensorData.obstacle_detected) {
        double sum_angle = 0.0, sum_weight = 0.0;
        for (int i = 0; i < SONAR_COUNT && i < mySensorData.filtered_readings.size(); i++) {
            if (mySensorData.filtered_readings[i] < SAFE_DISTANCE) {
                double sensor_angle = (i * 22.5) - 180.0; // Approximate sensor angles
                double weight = 1.0 / (mySensorData.filtered_readings[i] + 1.0);
                sum_angle += sensor_angle * weight;
                sum_weight += weight;
            }
        }
        mySensorData.obstacle_angle = (sum_weight > 0) ? sum_angle / sum_weight : 0.0;
    }
}

void ConsensualMovements::filterSensorReadings()
{
    // Simple moving average filter
    static std::vector<std::vector<double>> history(SONAR_COUNT, std::vector<double>(3, 5000.0));
    
    for (int i = 0; i < SONAR_COUNT && i < mySensorData.sonar_readings.size(); i++) {
        // Shift history
        history[i][2] = history[i][1];
        history[i][1] = history[i][0];
        history[i][0] = mySensorData.sonar_readings[i];
        
        // Calculate filtered value
        mySensorData.filtered_readings[i] = (history[i][0] + history[i][1] + history[i][2]) / 3.0;
        
        // Reject outliers
        if (abs(mySensorData.sonar_readings[i] - mySensorData.filtered_readings[i]) > 1000.0) {
            mySensorData.filtered_readings[i] = history[i][1]; // Use previous value
        }
    }
}

ConsensualMovements::MovementCommand ConsensualMovements::obstacleAvoidanceBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "ObstacleAvoidance";
    cmd.confidence = 0.0;
    cmd.linear_velocity = 0.0;
    cmd.angular_velocity = 0.0;
    
    if (!mySensorData.obstacle_detected) {
        return cmd; // No obstacles, no input from this behavior
    }
    
    // High confidence when obstacles are close
    double distance_factor = std::max(0.0, (SAFE_DISTANCE - mySensorData.min_front_distance) / SAFE_DISTANCE);
    cmd.confidence = myWeights.obstacle_avoidance * distance_factor;
    
    // Determine avoidance direction
    if (mySensorData.min_front_distance < CRITICAL_DISTANCE) {
        // Emergency: stop and turn away
        cmd.linear_velocity = 0.0;
        cmd.angular_velocity = (mySensorData.obstacle_angle > 0) ? -MAX_ANGULAR_VEL : MAX_ANGULAR_VEL;
    } else {
        // Gradual avoidance
        cmd.linear_velocity = MAX_LINEAR_VEL * (1.0 - distance_factor);
        cmd.angular_velocity = -mySensorData.obstacle_angle * 0.5; // Turn away from obstacle
    }
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::goalSeekingBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "GoalSeeking";
    cmd.confidence = myWeights.goal_seeking;
    
    double angle_to_goal = calculateAngleToGoal();
    double distance_to_goal = calculateDistance(myNavState.current_x, myNavState.current_y,
                                               myNavState.goal_x, myNavState.goal_y);
    
    // Reduce confidence if path is blocked
    if (!isPathClear(angle_to_goal, std::min(distance_to_goal, 1000.0))) {
        cmd.confidence *= 0.3;
    }
    
    // Calculate movement towards goal
    cmd.angular_velocity = angle_to_goal * 0.8; // Proportional turning
    cmd.linear_velocity = MAX_LINEAR_VEL * std::max(0.2, (1.0 - abs(angle_to_goal) / 90.0));
    
    // Reached goal? Set new one
    if (distance_to_goal < 500.0) {
        setRandomGoal();
        ArLog::log(ArLog::Normal, "ConsensualMovements: Goal reached, setting new goal");
    }
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::wallFollowingBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "WallFollowing";
    cmd.confidence = 0.0;
    
    // Activate wall following if close to walls
    if (mySensorData.min_side_distance < SAFE_DISTANCE * 1.5) {
        myNavState.wall_following_active = true;
        cmd.confidence = myWeights.wall_following;
        
        // Follow wall at safe distance
        double wall_error = SAFE_DISTANCE - mySensorData.min_side_distance;
        cmd.linear_velocity = MAX_LINEAR_VEL * 0.7;
        cmd.angular_velocity = wall_error * 0.1; // Proportional control
    } else if (myNavState.wall_following_active && mySensorData.min_side_distance > SAFE_DISTANCE * 2.0) {
        myNavState.wall_following_active = false;
    }
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::explorationBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "Exploration";
    cmd.confidence = myWeights.exploration;
    
    // Add some randomness for exploration
    static double exploration_bias = 0.0;
    static int exploration_counter = 0;
    
    if (++exploration_counter > 100) { // Change bias every 5 seconds (50ms * 100)
        exploration_bias = (rand() % 60) - 30.0; // Random bias ±30 degrees
        exploration_counter = 0;
    }
    
    cmd.linear_velocity = MAX_LINEAR_VEL * 0.5;
    cmd.angular_velocity = exploration_bias * 0.3;
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::smoothMotionBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "SmoothMotion";
    cmd.confidence = myWeights.smooth_motion;
    
    // Get current velocities
    double current_vel = myRobot->getVel();
    double current_rot_vel = myRobot->getRotVel();
    
    // Smooth acceleration/deceleration
    cmd.linear_velocity = current_vel; // Maintain current velocity as baseline
    cmd.angular_velocity = current_rot_vel * 0.8; // Dampen rotation
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::emergencyStopBehavior()
{
    MovementCommand cmd;
    cmd.behavior_name = "EmergencyStop";
    cmd.confidence = 0.0;
    cmd.linear_velocity = 0.0;
    cmd.angular_velocity = 0.0;
    
    // Emergency conditions
    if (mySensorData.min_front_distance < CRITICAL_DISTANCE) {
        cmd.confidence = 1.0; // Override all other behaviors
        ArLog::log(ArLog::Terse, "ConsensualMovements: EMERGENCY STOP - Obstacle at %.0fmm", 
                   mySensorData.min_front_distance);
    }
    
    return cmd;
}

ConsensualMovements::MovementCommand ConsensualMovements::consensusDecision(const std::vector<MovementCommand>& commands)
{
    MovementCommand consensus;
    consensus.behavior_name = "Consensus";
    consensus.confidence = 0.0;
    consensus.linear_velocity = 0.0;
    consensus.angular_velocity = 0.0;
    
    double total_weight = 0.0;
    
    // Weighted average of all behaviors
    for (const auto& cmd : commands) {
        if (cmd.confidence > 0.0) {
            consensus.linear_velocity += cmd.linear_velocity * cmd.confidence;
            consensus.angular_velocity += cmd.angular_velocity * cmd.confidence;
            total_weight += cmd.confidence;
        }
    }
    
    if (total_weight > 0.0) {
        consensus.linear_velocity /= total_weight;
        consensus.angular_velocity /= total_weight;
        consensus.confidence = total_weight / commands.size();
    }
    
    // Apply safety limits
    consensus.linear_velocity = std::max(-MAX_LINEAR_VEL, 
                                        std::min(MAX_LINEAR_VEL, consensus.linear_velocity));
    consensus.angular_velocity = std::max(-MAX_ANGULAR_VEL, 
                                         std::min(MAX_ANGULAR_VEL, consensus.angular_velocity));
    
    return consensus;
}

void ConsensualMovements::executeMovement(const MovementCommand& cmd)
{
    // Only move if movement is enabled and it's safe
    if (myNavState.movement_enabled && isSafeToMove()) {
        myRobot->setVel(cmd.linear_velocity);
        myRobot->setRotVel(cmd.angular_velocity);
    } else {
        myRobot->stop();
    }
}

void ConsensualMovements::updateNavigationState()
{
    myNavState.current_x = myRobot->getX();
    myNavState.current_y = myRobot->getY();
    myNavState.current_theta = myRobot->getTh();
}

double ConsensualMovements::calculateAngleToGoal()
{
    double dx = myNavState.goal_x - myNavState.current_x;
    double dy = myNavState.goal_y - myNavState.current_y;
    double goal_angle = atan2(dy, dx) * 180.0 / M_PI;
    return normalizeAngle(goal_angle - myNavState.current_theta);
}

double ConsensualMovements::calculateDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

double ConsensualMovements::normalizeAngle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

bool ConsensualMovements::isPathClear(double angle, double distance)
{
    // Simple path checking - could be enhanced with ray casting
    return mySensorData.min_front_distance > distance;
}

void ConsensualMovements::setRandomGoal()
{
    // Set random goal within reasonable range
    double range = 3000.0; // 3 meters
    myNavState.goal_x = myNavState.current_x + (rand() % (int)(2*range)) - range;
    myNavState.goal_y = myNavState.current_y + (rand() % (int)(2*range)) - range;
    
    ArLog::log(ArLog::Normal, "ConsensualMovements: New goal set at (%.0f, %.0f)", 
               myNavState.goal_x, myNavState.goal_y);
}

bool ConsensualMovements::isSafeToMove()
{
    return mySensorData.min_front_distance > CRITICAL_DISTANCE;
}

void ConsensualMovements::logBehaviorDecision(const MovementCommand& cmd)
{
    static int log_counter = 0;
    if (++log_counter % 20 == 0) { // Log every second (50ms * 20)
        ArLog::log(ArLog::Normal, 
                   "ConsensualMovements: %s -> Vel:%.0f RotVel:%.1f Conf:%.2f MinDist:%.0f",
                   cmd.behavior_name.c_str(), cmd.linear_velocity, cmd.angular_velocity, 
                   cmd.confidence, mySensorData.min_front_distance);
    }
}

// WiFi Module Support Methods
bool ConsensualMovements::discoverZLAN7104()
{
    printf("🔍 Scanning for ZLAN 7104 WiFi modules...\n");
    
    // Common IP ranges for ZLAN modules
    std::vector<std::string> common_ips = {
        "192.168.1.152",   // User's specific ZLAN IP
        "192.168.1.100",   // Default ZLAN IP
        "192.168.0.100",   // Alternative default
        "192.168.4.1",     // AP mode default
        "10.0.0.100",      // Some configurations
        "172.16.0.100"     // Enterprise networks
    };
    
    for (const auto& ip : common_ips) {
        printf("   Testing %s:%d... ", ip.c_str(), myZLANPort);
        fflush(stdout);
        
        if (connectToZLAN7104(ip, myZLANPort)) {
            printf("✅ Found ZLAN 7104!\n");
            myZLANIP = ip;
            myWiFiConnected = true;
            return true;
        } else {
            printf("❌\n");
        }
    }
    
    printf("❌ No ZLAN 7104 modules found on common addresses\n");
    return false;
}

bool ConsensualMovements::connectToZLAN7104(const std::string& ip_address, int port)
{
    // Test TCP connection to the ZLAN module
    ArSocket testSocket;
    
    if (testSocket.connect(ip_address.c_str(), port, ArSocket::TCP, NULL)) {
        testSocket.close();
        return true;
    }
    return false;
}

void ConsensualMovements::scanForRobots()
{
    printf("\n=== ZLAN 7104 WiFi MODULE SCANNER ===\n");
    printf("Searching for Pioneer 3DX robots with WiFi modules...\n\n");
    
    // Try to discover ZLAN module automatically
    if (discoverZLAN7104()) {
        printf("✅ WiFi module discovered at: %s:%d\n", myZLANIP.c_str(), myZLANPort);
        return;
    }
    
    // Manual IP entry if auto-discovery fails
    printf("\n🔧 Manual Configuration Required\n");
    printf("Please enter the ZLAN 7104 IP address: ");
    
    char ip_input[50];
    if (fgets(ip_input, sizeof(ip_input), stdin)) {
        // Remove newline
        ip_input[strcspn(ip_input, "\n")] = 0;
        
        printf("Testing connection to %s:%d... ", ip_input, myZLANPort);
        fflush(stdout);
        
        if (connectToZLAN7104(ip_input, myZLANPort)) {
            printf("✅ Connected!\n");
            myZLANIP = ip_input;
            myWiFiConnected = true;
        } else {
            printf("❌ Connection failed\n");
            printf("Please check:\n");
            printf("  - ZLAN 7104 is powered and connected to WiFi\n");
            printf("  - IP address is correct\n");
            printf("  - Robot and computer are on same network\n");
        }
    }
}

std::string ConsensualMovements::getZLANIP()
{
    return myZLANIP;
}

// Main program
int main(int argc, char** argv)
{
    printf("=== CONSENSUAL MOVEMENTS NAVIGATION SYSTEM ===\n");
    printf("Professional autonomous navigation for Pioneer 3DX\n");
    printf("Supports ZLAN 7104 WiFi, SSH/TCP and serial connections\n\n");
    
    Aria::init();
    
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    
    ArRobot robot;
    ConsensualMovements navigator(&robot);  // Create navigator early for WiFi scanning
    
    // Check for WiFi pairing request
    bool use_wifi = false;
    bool wifi_scan = false;
    
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-wifi") == 0 || strcmp(argv[i], "--wifi") == 0) {
            use_wifi = true;
            wifi_scan = true;
        }
    }
    
    std::string wifi_ip = "";
    
    if (use_wifi || wifi_scan) {
        // WiFi pairing mode
        navigator.scanForRobots();
        wifi_ip = navigator.getZLANIP();
        
        if (wifi_ip.empty()) {
            printf("❌ WiFi pairing failed. Falling back to other connection methods.\n\n");
            use_wifi = false;
        } else {
            printf("✅ WiFi pairing successful! Using %s:5000\n\n", wifi_ip.c_str());
            use_wifi = true;
        }
    }
    
    // Set up connection parameters
    if (use_wifi && !wifi_ip.empty()) {
        // Override command line arguments for WiFi connection
        printf("🔗 Connecting via ZLAN 7104 WiFi module...\n");
        
        // Clear existing args and set WiFi parameters
        argc = 1;  // Reset to just program name
        char* wifi_args[] = {
            argv[0],
            (char*)"-remoteHost",
            (char*)wifi_ip.c_str(),
            (char*)"-remoteRobotTcpPort", 
            (char*)"5000"
        };
        argc = 5;
        argv = wifi_args;
        
        // Reinitialize parser with WiFi parameters
        ArArgumentParser wifi_parser(&argc, argv);
        wifi_parser.loadDefaultArguments();
        
        ArRobotConnector robotConnector(&wifi_parser, &robot);
        
        printf("Attempting WiFi connection to %s:5000...\n", wifi_ip.c_str());
        if (!robotConnector.connectRobot()) {
            ArLog::log(ArLog::Terse, "WiFi connection failed, exiting");
            printf("❌ WiFi connection failed! Check ZLAN 7104 configuration.\n");
            Aria::exit(1);
        }
    } else {
        // Standard connection methods
        printf("Connection options:\n");
        printf("  WiFi: -wifi (auto-discover ZLAN 7104)\n");
        printf("  Serial: -robotPort /dev/ttyUSB0\n");
        printf("  SSH/TCP: -remoteHost <robot_ip> -remoteRobotTcpPort 5000\n");
        printf("  Simulator: -remoteIsSim\n\n");
        
        ArRobotConnector robotConnector(&parser, &robot);
        
        printf("Attempting to connect to robot...\n");
        if (!robotConnector.connectRobot()) {
            ArLog::log(ArLog::Terse, "Could not connect to robot, exiting");
            printf("Connection failed! Check your connection parameters.\n");
            printf("💡 Try: ./consensualMovements -wifi  (for WiFi auto-discovery)\n");
            Aria::exit(1);
        }
    }
    
    printf("✅ Successfully connected to robot: %s\n", robot.getName());
    printf("   Type: %s, Subtype: %s\n", robot.getRobotType(), robot.getRobotSubType());
    printf("   Serial: %s\n\n", robot.getRobotName());
    
    // Add sonar device
    ArSonarDevice sonarDev;
    robot.addRangeDevice(&sonarDev);
    
    // Start robot task
    robot.runAsync(true);
    
    // Wait for robot to be ready
    printf("Initializing robot systems...\n");
    ArUtil::sleep(2000);  // Give robot time to initialize
    
    printf("✅ Navigation system initialized\n");
    printf("✅ %d sonar sensors active\n", robot.getNumSonar());
    printf("✅ Motors enabled\n\n");
    
    // Start consensual movements (navigator already created)
    navigator.run();
    
    // Wait for user to start movement
    printf("🎮 Robot connected successfully! Press SPACE to start autonomous navigation...\n");
    navigator.waitForStartCommand();
    
    // Main loop - monitor robot status
    printf("=== SYSTEM MONITORING ===\n");
    int status_counter = 0;
    
    while (robot.isConnected()) {
        ArUtil::sleep(1000);  // Check every second
        
        // Print status every 10 seconds
        if (++status_counter % 10 == 0) {
             int closest_sonar = robot.getClosestSonarRange(-90.0, 90.0);  // Front 180 degrees
            printf("Status: Pos(%.0f,%.0f) Heading:%.1f° Battery:%.1fV Sonar:%dmm\n",
                   robot.getX(), robot.getY(), robot.getTh(), 
                   robot.getBatteryVoltage(), closest_sonar);
        }
        
        // Check for low battery
        if (robot.getBatteryVoltage() < 11.0) {
            printf("⚠️  LOW BATTERY WARNING: %.1fV\n", robot.getBatteryVoltage());
        }
        
        // Check for obstacles
        int front_sonar = robot.getClosestSonarRange(-45.0, 45.0);  // Front 90 degrees
        if (front_sonar < 500) {
            if (status_counter % 5 == 0) {  // Don't spam
                printf("🚧 Obstacle detected at %dmm\n", front_sonar);
            }
        }
    }
    
    printf("\n❌ Robot disconnected!\n");
    navigator.stop();
    Aria::exit(0);
    return 0;
}
