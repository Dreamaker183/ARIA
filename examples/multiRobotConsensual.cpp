/*
Multi-Robot Consensual Movements System
Two Pioneer 3DX robots working together with consensual coordination
Robot 1: 192.168.1.253 (Primary)
Robot 2: 192.168.1.254 (Secondary)
*/

#include "Aria.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstring>
#include <thread>
#include <mutex>

class MultiRobotConsensual
{
public:
    MultiRobotConsensual();
    ~MultiRobotConsensual();
    
    bool connectRobots();
    void run();
    void stop();
    void waitForStartCommand();
    
private:
    // Robot instances
    ArRobot* robot1;  // Primary robot (192.168.1.253)
    ArRobot* robot2;  // Secondary robot (192.168.1.254)
    
    // Robot connectors
    ArRobotConnector* connector1;
    ArRobotConnector* connector2;
    
    // Task callbacks
    ArFunctorC<MultiRobotConsensual> myTaskCB1;
    ArFunctorC<MultiRobotConsensual> myTaskCB2;
    
    // Robot data structures
    struct RobotData {
        double x, y, theta;
        double battery_voltage;
        double min_sonar_distance;
        bool connected;
        bool movement_enabled;
        std::string name;
    };
    
public:
    RobotData robot1_data;
    RobotData robot2_data;
    
    // Public getter methods
    double calculateDistance(double x1, double y1, double x2, double y2);
    
    // Consensual coordination
    struct ConsensusData {
        double target_x, target_y;
        double formation_angle;
        double coordination_force;
        bool formation_active;
    };
    
    ConsensusData consensus;
    
    // Threading and synchronization
    std::mutex data_mutex;
    bool system_running;
    
    // Core methods
    void robot1Task();
    void robot2Task();
    void updateRobotData(ArRobot* robot, RobotData& data);
    void calculateConsensus();
    void executeConsensualMovement();
    
    // Formation behaviors
    void maintainFormation();
    void avoidCollision();
    void coordinateMovement();
    
    // Utility methods
    double calculateAngle(double x1, double y1, double x2, double y2);
    void setRobotVelocity(ArRobot* robot, double linear, double angular);
    bool isPathClear(ArRobot* robot, double angle, double distance);
    
    // Constants
    static const double FORMATION_DISTANCE;
    static const double SAFE_DISTANCE;
    static const double MAX_VELOCITY;
    static const double MAX_ANGULAR_VELOCITY;
};

const double MultiRobotConsensual::FORMATION_DISTANCE = 800.0;  // 80cm formation distance
const double MultiRobotConsensual::SAFE_DISTANCE = 600.0;       // 60cm safe distance
const double MultiRobotConsensual::MAX_VELOCITY = 300.0;        // 300mm/s max speed
const double MultiRobotConsensual::MAX_ANGULAR_VELOCITY = 45.0; // 45deg/s max rotation

MultiRobotConsensual::MultiRobotConsensual() :
    robot1(nullptr), robot2(nullptr),
    connector1(nullptr), connector2(nullptr),
    myTaskCB1(this, &MultiRobotConsensual::robot1Task),
    myTaskCB2(this, &MultiRobotConsensual::robot2Task),
    system_running(false)
{
    // Initialize robot data
    robot1_data = {0, 0, 0, 0, 5000, false, false, "Robot1"};
    robot2_data = {0, 0, 0, 0, 5000, false, false, "Robot2"};
    
    // Initialize consensus data
    consensus = {0, 0, 0, 0, false};
    
    printf("Multi-Robot Consensual Movements System Initialized\n");
    printf("Robot 1: 192.168.1.253 (Primary)\n");
    printf("Robot 2: 192.168.1.254 (Secondary)\n");
}

MultiRobotConsensual::~MultiRobotConsensual()
{
    stop();
}

bool MultiRobotConsensual::connectRobots()
{
    printf("\n=== CONNECTING TO MULTI-ROBOT SYSTEM ===\n");
    
    // Initialize ARIA
    Aria::init();
    
    // Create robots
    robot1 = new ArRobot();
    robot2 = new ArRobot();
    
    // Create argument parsers for each robot
    int argc1 = 5;
    char* argv1[] = {
        (char*)"multiRobotConsensual",
        (char*)"-remoteHost", (char*)"192.168.1.253",
        (char*)"-remoteRobotTcpPort", (char*)"8101"
    };
    
    int argc2 = 5;
    char* argv2[] = {
        (char*)"multiRobotConsensual",
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
    
    // Connect to Robot 1
    printf("Connecting to Robot 1 (192.168.1.253:8101)... ");
    fflush(stdout);
    if (!connector1->connectRobot()) {
        printf("❌ Failed\n");
        return false;
    }
    printf("✅ Connected\n");
    robot1_data.connected = true;
    robot1_data.name = robot1->getName();
    
    // Connect to Robot 2
    printf("Connecting to Robot 2 (192.168.1.254:8101)... ");
    fflush(stdout);
    if (!connector2->connectRobot()) {
        printf("❌ Failed\n");
        return false;
    }
    printf("✅ Connected\n");
    robot2_data.connected = true;
    robot2_data.name = robot2->getName();
    
    // Add sonar devices
    ArSonarDevice sonar1, sonar2;
    robot1->addRangeDevice(&sonar1);
    robot2->addRangeDevice(&sonar2);
    
    // Start robot tasks
    robot1->runAsync(true);
    robot2->runAsync(true);
    
    // Wait for initialization
    printf("Initializing robot systems...\n");
    ArUtil::sleep(3000);
    
    // Enable motors
    robot1->enableMotors();
    robot2->enableMotors();
    
    printf("✅ Multi-robot system ready!\n");
    printf("   Robot 1: %s (%s)\n", robot1_data.name.c_str(), robot1->getRobotSubType());
    printf("   Robot 2: %s (%s)\n", robot2_data.name.c_str(), robot2->getRobotSubType());
    
    return true;
}

void MultiRobotConsensual::run()
{
    if (!robot1_data.connected || !robot2_data.connected) {
        printf("❌ Cannot start - robots not connected\n");
        return;
    }
    
    printf("\n🤖 Starting Multi-Robot Consensual Movements...\n");
    
    // Add task callbacks
    robot1->addUserTask("Robot1Consensual", 50, &myTaskCB1);
    robot2->addUserTask("Robot2Consensual", 50, &myTaskCB2);
    
    system_running = true;
    
    // Set initial formation target
    consensus.target_x = 1000.0;
    consensus.target_y = 1000.0;
    consensus.formation_active = true;
    
    printf("✅ Consensual coordination active!\n");
    printf("   Formation distance: %.0fmm\n", FORMATION_DISTANCE);
    printf("   Safe distance: %.0fmm\n", SAFE_DISTANCE);
}

void MultiRobotConsensual::stop()
{
    system_running = false;
    
    if (robot1 && robot1_data.connected) {
        robot1->remUserTask(&myTaskCB1);
        robot1->stop();
    }
    
    if (robot2 && robot2_data.connected) {
        robot2->remUserTask(&myTaskCB2);
        robot2->stop();
    }
    
    printf("Multi-robot system stopped\n");
}

void MultiRobotConsensual::waitForStartCommand()
{
    printf("\n🎮 MULTI-ROBOT CONSENSUAL MOVEMENTS READY!\n");
    printf("Robot 1: %s (192.168.1.253)\n", robot1_data.name.c_str());
    printf("Robot 2: %s (192.168.1.254)\n", robot2_data.name.c_str());
    printf("Status: Both robots connected and ready for coordinated navigation\n\n");
    
    printf("Press SPACE to start consensual movements, 'q' to quit: ");
    fflush(stdout);
    
    int ch = getchar();
    if (ch == ' ' || ch == '\n') {
        robot1_data.movement_enabled = true;
        robot2_data.movement_enabled = true;
        printf("\n🚀 MULTI-ROBOT CONSENSUAL MOVEMENTS ACTIVE!\n");
        printf("Both robots are now coordinating movements!\n");
        printf("Press Ctrl+C to stop safely\n\n");
    } else if (ch == 'q' || ch == 'Q') {
        printf("\n👋 Exiting multi-robot system...\n");
        return;
    }
}

void MultiRobotConsensual::robot1Task()
{
    if (!system_running || !robot1_data.connected) return;
    
    std::lock_guard<std::mutex> lock(data_mutex);
    updateRobotData(robot1, robot1_data);
    calculateConsensus();
    executeConsensualMovement();
}

void MultiRobotConsensual::robot2Task()
{
    if (!system_running || !robot2_data.connected) return;
    
    std::lock_guard<std::mutex> lock(data_mutex);
    updateRobotData(robot2, robot2_data);
    calculateConsensus();
    executeConsensualMovement();
}

void MultiRobotConsensual::updateRobotData(ArRobot* robot, RobotData& data)
{
    data.x = robot->getX();
    data.y = robot->getY();
    data.theta = robot->getTh();
    data.battery_voltage = robot->getBatteryVoltage();
    data.min_sonar_distance = robot->getClosestSonarRange(-90.0, 90.0);
    data.connected = robot->isConnected();
}

void MultiRobotConsensual::calculateConsensus()
{
    if (!robot1_data.connected || !robot2_data.connected) return;
    
    // Calculate distance between robots
    double robot_distance = calculateDistance(robot1_data.x, robot1_data.y, 
                                            robot2_data.x, robot2_data.y);
    
    // Calculate center point
    double center_x = (robot1_data.x + robot2_data.x) / 2.0;
    double center_y = (robot1_data.y + robot2_data.y) / 2.0;
    
    // Calculate angle to target
    double angle_to_target = calculateAngle(center_x, center_y, 
                                          consensus.target_x, consensus.target_y);
    
    // Update consensus data
    consensus.formation_angle = angle_to_target;
    
    // Check if formation is maintained
    if (robot_distance > FORMATION_DISTANCE * 1.5) {
        consensus.coordination_force = 1.0; // High coordination needed
    } else if (robot_distance < FORMATION_DISTANCE * 0.5) {
        consensus.coordination_force = 0.5; // Moderate coordination
    } else {
        consensus.coordination_force = 0.2; // Low coordination
    }
}

void MultiRobotConsensual::executeConsensualMovement()
{
    if (!robot1_data.movement_enabled || !robot2_data.movement_enabled) return;
    
    // Calculate movement for each robot
    maintainFormation();
    avoidCollision();
    coordinateMovement();
}

void MultiRobotConsensual::maintainFormation()
{
    // Robot 1: Move towards target while maintaining formation
    double angle_to_target = calculateAngle(robot1_data.x, robot1_data.y, 
                                          consensus.target_x, consensus.target_y);
    double distance_to_target = calculateDistance(robot1_data.x, robot1_data.y, 
                                                consensus.target_x, consensus.target_y);
    
    // Basic movement towards target
    double linear_vel = std::min(MAX_VELOCITY, distance_to_target * 0.5);
    double angular_vel = angle_to_target * 0.3;
    
    // Apply formation constraints
    double robot_distance = calculateDistance(robot1_data.x, robot1_data.y, 
                                            robot2_data.x, robot2_data.y);
    
    if (robot_distance > FORMATION_DISTANCE * 1.2) {
        // Too far apart - slow down
        linear_vel *= 0.5;
    } else if (robot_distance < FORMATION_DISTANCE * 0.8) {
        // Too close - speed up
        linear_vel *= 1.2;
    }
    
    // Safety check
    if (robot1_data.min_sonar_distance < SAFE_DISTANCE) {
        linear_vel = 0;
        angular_vel = 45.0; // Turn away from obstacle
    }
    
    setRobotVelocity(robot1, linear_vel, angular_vel);
    
    // Robot 2: Follow Robot 1 with offset
    double angle_to_robot1 = calculateAngle(robot2_data.x, robot2_data.y, 
                                          robot1_data.x, robot1_data.y);
    double distance_to_robot1 = calculateDistance(robot2_data.x, robot2_data.y, 
                                                robot1_data.x, robot1_data.y);
    
    // Maintain formation distance
    double formation_error = distance_to_robot1 - FORMATION_DISTANCE;
    double linear_vel2 = std::min(MAX_VELOCITY, abs(formation_error) * 0.8);
    double angular_vel2 = angle_to_robot1 * 0.4;
    
    // Safety check for Robot 2
    if (robot2_data.min_sonar_distance < SAFE_DISTANCE) {
        linear_vel2 = 0;
        angular_vel2 = -45.0; // Turn away from obstacle
    }
    
    setRobotVelocity(robot2, linear_vel2, angular_vel2);
}

void MultiRobotConsensual::avoidCollision()
{
    // Check for inter-robot collision
    double robot_distance = calculateDistance(robot1_data.x, robot1_data.y, 
                                            robot2_data.x, robot2_data.y);
    
    if (robot_distance < SAFE_DISTANCE) {
        // Emergency separation
        double angle_away = calculateAngle(robot1_data.x, robot1_data.y, 
                                         robot2_data.x, robot2_data.y);
        
        // Robot 1 moves away
        setRobotVelocity(robot1, MAX_VELOCITY * 0.5, angle_away * 0.5);
        // Robot 2 moves in opposite direction
        setRobotVelocity(robot2, MAX_VELOCITY * 0.5, (angle_away + 180) * 0.5);
    }
}

void MultiRobotConsensual::coordinateMovement()
{
    // Update target periodically
    static int target_counter = 0;
    if (++target_counter > 200) { // Every 10 seconds
        // Set new random target
        consensus.target_x = (rand() % 2000) - 1000;
        consensus.target_y = (rand() % 2000) - 1000;
        target_counter = 0;
        
        printf("🎯 New target: (%.0f, %.0f)\n", consensus.target_x, consensus.target_y);
    }
}

double MultiRobotConsensual::calculateDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

double MultiRobotConsensual::calculateAngle(double x1, double y1, double x2, double y2)
{
    return atan2(y2-y1, x2-x1) * 180.0 / M_PI;
}

void MultiRobotConsensual::setRobotVelocity(ArRobot* robot, double linear, double angular)
{
    // Apply limits
    linear = std::max(-MAX_VELOCITY, std::min(MAX_VELOCITY, linear));
    angular = std::max(-MAX_ANGULAR_VELOCITY, std::min(MAX_ANGULAR_VELOCITY, angular));
    
    robot->setVel(linear);
    robot->setRotVel(angular);
}

bool MultiRobotConsensual::isPathClear(ArRobot* robot, double angle, double distance)
{
    // Simple path checking - could be enhanced
    return robot->getClosestSonarRange(angle-30, angle+30) > distance;
}

// Main program
int main(int argc, char** argv)
{
    printf("=== MULTI-ROBOT CONSENSUAL MOVEMENTS SYSTEM ===\n");
    printf("Two Pioneer 3DX robots working together\n");
    printf("Robot 1: 192.168.1.253 (Primary)\n");
    printf("Robot 2: 192.168.1.254 (Secondary)\n\n");
    
    MultiRobotConsensual system;
    
    if (!system.connectRobots()) {
        printf("❌ Failed to connect to robots. Exiting.\n");
        return 1;
    }
    
    system.run();
    system.waitForStartCommand();
    
    // Main monitoring loop
    printf("=== MULTI-ROBOT SYSTEM MONITORING ===\n");
    int status_counter = 0;
    
    while (system.robot1_data.connected || system.robot2_data.connected) {
        ArUtil::sleep(1000);
        
        if (++status_counter % 10 == 0) {
            printf("Robot 1: Pos(%.0f,%.0f) Heading:%.1f° Battery:%.1fV Sonar:%.0fmm\n",
                   system.robot1_data.x, system.robot1_data.y, system.robot1_data.theta,
                   system.robot1_data.battery_voltage, system.robot1_data.min_sonar_distance);
            
            printf("Robot 2: Pos(%.0f,%.0f) Heading:%.1f° Battery:%.1fV Sonar:%.0fmm\n",
                   system.robot2_data.x, system.robot2_data.y, system.robot2_data.theta,
                   system.robot2_data.battery_voltage, system.robot2_data.min_sonar_distance);
            
            double distance = system.calculateDistance(system.robot1_data.x, system.robot1_data.y,
                                                     system.robot2_data.x, system.robot2_data.y);
            printf("Formation distance: %.0fmm (target: %.0fmm)\n\n", distance, 800.0);
        }
    }
    
    printf("❌ Robot(s) disconnected!\n");
    system.stop();
    Aria::exit(0);
    return 0;
}
