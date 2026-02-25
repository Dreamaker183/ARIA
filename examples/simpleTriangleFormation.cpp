/*
Simple Triangle Formation System
A robust, simplified version of the professional triangle formation system
that focuses on core functionality without complex features.

Features:
- Dual robot connection and coordination
- Simple triangle formation
- Basic collision avoidance
- Return-to-origin functionality
- Space key triggered cycles
*/

#include "Aria.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <cstring>

class SimpleTriangleFormation
{
public:
    SimpleTriangleFormation();
    ~SimpleTriangleFormation();
    
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
    ArFunctorC<SimpleTriangleFormation> myTaskCB1;
    ArFunctorC<SimpleTriangleFormation> myTaskCB2;
    
    // Robot data
    struct RobotData {
        double x, y, theta;
        double battery_voltage;
        double min_sonar_distance;
        bool connected;
        bool at_target;
        ArPose initial_pose;
        ArPose target_pose;
        std::string name;
    };
    
    RobotData robot1_data;
    RobotData robot2_data;
    
    // Formation parameters
    double side_length = 800.0;        // Triangle side length (mm)
    double min_separation = 400.0;     // Minimum inter-robot distance
    double pos_tolerance = 100.0;      // Position tolerance (mm)
    double max_velocity = 200.0;       // Maximum velocity (mm/s)
    
    // System state
    enum Phase {
        READY,
        FORMING_TRIANGLE,
        TRIANGLE_MANEUVER,
        RETURNING_HOME,
        COMPLETED
    };
    
    Phase current_phase;
    bool cycle_running;
    bool cycle_triggered;
    bool system_running;
    ArTime cycle_start_time;
    
    // Triangle vertices
    ArPose triangle_vertex1, triangle_vertex2, triangle_vertex3;
    ArPose triangle_centroid;
    
    // Core methods
    void robot1Task();
    void robot2Task();
    void updateRobotData(ArRobot* robot, RobotData& data);
    void executeFormationControl();
    void computeTriangleVertices();
    void assignRobotsToVertices();
    void executeTriangleManeuver();
    void returnToOrigin();
    void moveRobotToTarget(int robot_id, const ArPose& target);
    bool isAtTarget(int robot_id, const ArPose& target);
    void setRobotVelocity(ArRobot* robot, double linear, double angular);
    double calculateDistance(const ArPose& p1, const ArPose& p2);
    void logSystemState();
};

SimpleTriangleFormation::SimpleTriangleFormation() :
    robot1(nullptr), robot2(nullptr),
    connector1(nullptr), connector2(nullptr),
    myTaskCB1(this, &SimpleTriangleFormation::robot1Task),
    myTaskCB2(this, &SimpleTriangleFormation::robot2Task),
    current_phase(READY),
    cycle_running(false),
    cycle_triggered(false),
    system_running(false)
{
    // Initialize robot data
    robot1_data = {0, 0, 0, 0, 5000, false, false, ArPose(), ArPose(), "Robot1"};
    robot2_data = {0, 0, 0, 0, 5000, false, false, ArPose(), ArPose(), "Robot2"};
    
    printf("Simple Triangle Formation System Initialized\n");
    printf("Parameters: Side=%.0fmm, MinSep=%.0fmm, MaxVel=%.0fmm/s\n", 
           side_length, min_separation, max_velocity);
}

SimpleTriangleFormation::~SimpleTriangleFormation()
{
    stop();
}

bool SimpleTriangleFormation::initialize()
{
    printf("\n=== SIMPLE TRIANGLE FORMATION SYSTEM ===\n");
    printf("Initializing dual-robot formation control...\n");
    
    Aria::init();
    
    // Create robots
    robot1 = new ArRobot();
    robot2 = new ArRobot();
    
    // Create argument parsers
    int argc1 = 5;
    char* argv1[] = {
        (char*)"simpleTriangleFormation",
        (char*)"-remoteHost", (char*)"192.168.1.253",
        (char*)"-remoteRobotTcpPort", (char*)"8101"
    };
    
    int argc2 = 5;
    char* argv2[] = {
        (char*)"simpleTriangleFormation",
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
    robot1_data.connected = true;
    robot1_data.name = robot1->getName();
    
    printf("Connecting to Robot 2 (192.168.1.254:8101)... ");
    fflush(stdout);
    if (!connector2->connectRobot()) {
        printf("❌ FAILED\n");
        return false;
    }
    printf("✅ CONNECTED\n");
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
    printf("Calibrating sensors...\n");
    ArUtil::sleep(2000);
    
    // Enable motors
    robot1->enableMotors();
    robot2->enableMotors();
    
    // Record initial poses
    robot1_data.initial_pose = ArPose(robot1->getX(), robot1->getY(), robot1->getTh());
    robot2_data.initial_pose = ArPose(robot2->getX(), robot2->getY(), robot2->getTh());
    
    printf("✅ System initialized successfully!\n");
    printf("   Robot 1: %s at (%.0f, %.0f, %.1f°)\n", 
           robot1_data.name.c_str(), robot1_data.initial_pose.getX(), 
           robot1_data.initial_pose.getY(), robot1_data.initial_pose.getTh());
    printf("   Robot 2: %s at (%.0f, %.0f, %.1f°)\n", 
           robot2_data.name.c_str(), robot2_data.initial_pose.getX(), 
           robot2_data.initial_pose.getY(), robot2_data.initial_pose.getTh());
    
    return true;
}

void SimpleTriangleFormation::run()
{
    if (!robot1_data.connected || !robot2_data.connected) {
        printf("❌ Cannot start - robots not connected\n");
        return;
    }
    
    printf("\n🤖 Starting Simple Triangle Formation System...\n");
    
    // Add task callbacks
    robot1->addUserTask("Robot1Formation", 100, &myTaskCB1);  // 10Hz
    robot2->addUserTask("Robot2Formation", 100, &myTaskCB2);  // 10Hz
    
    system_running = true;
    current_phase = READY;
    
    printf("✅ Formation control active!\n");
    printf("   Auto-triggering formation cycle in 3 seconds...\n\n");
    
    // Auto-trigger cycle
    ArUtil::sleep(3000);
    triggerCycle();
    
    // Main control loop
    while (system_running) {
        ArUtil::sleep(100);  // 10Hz main loop
        
        // Update visualization data
        updateRobotData(robot1, robot1_data);
        updateRobotData(robot2, robot2_data);
        
        // Log system state periodically
        static int log_counter = 0;
        if (++log_counter % 50 == 0) {  // Every 5 seconds
            logSystemState();
        }
        
        // Check if cycle is complete and ready for next one
        if (!cycle_running && current_phase == READY) {
            static int auto_trigger_counter = 0;
            if (++auto_trigger_counter % 200 == 0) {  // Every 20 seconds
                printf("Auto-triggering next formation cycle...\n");
                triggerCycle();
            }
        }
    }
}

void SimpleTriangleFormation::stop()
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
    
    printf("Simple Triangle Formation System stopped\n");
}

void SimpleTriangleFormation::triggerCycle()
{
    if (cycle_running) {
        printf("⚠️  Cycle already in progress, please wait...\n");
        return;
    }
    
    printf("\n🚀 TRIGGERING TRIANGLE FORMATION CYCLE\n");
    printf("=====================================\n");
    
    cycle_triggered = true;
    cycle_running = true;
    cycle_start_time.setToNow();
    current_phase = FORMING_TRIANGLE;
    
    printf("Phase 1: Computing triangle vertices and robot assignments...\n");
    computeTriangleVertices();
    assignRobotsToVertices();
}

void SimpleTriangleFormation::robot1Task()
{
    if (!system_running) return;
    
    updateRobotData(robot1, robot1_data);
    
    if (cycle_running) {
        executeFormationControl();
    }
}

void SimpleTriangleFormation::robot2Task()
{
    if (!system_running) return;
    
    updateRobotData(robot2, robot2_data);
    
    if (cycle_running) {
        executeFormationControl();
    }
}

void SimpleTriangleFormation::updateRobotData(ArRobot* robot, RobotData& data)
{
    // Check connection first
    if (!robot->isConnected()) {
        data.connected = false;
        printf("⚠️  Robot %s disconnected!\n", data.name.c_str());
        return;
    }
    
    data.connected = true;
    data.x = robot->getX();
    data.y = robot->getY();
    data.theta = robot->getTh();
    data.battery_voltage = robot->getBatteryVoltage();
    
    // Get minimum sonar distance with error handling
    data.min_sonar_distance = robot->getClosestSonarRange(-90.0, 90.0);
    if (data.min_sonar_distance < 0 || data.min_sonar_distance > 10000) {
        data.min_sonar_distance = 5000.0;  // Default safe distance
    }
    
    data.at_target = false;  // Will be updated in formation control
}

void SimpleTriangleFormation::executeFormationControl()
{
    ArTime current_time;
    double elapsed = cycle_start_time.mSecSince() / 1000.0;
    
    switch (current_phase) {
        case FORMING_TRIANGLE: {
            // Check if both robots are at their targets
            bool robot1_at_target = isAtTarget(1, robot1_data.target_pose);
            bool robot2_at_target = isAtTarget(2, robot2_data.target_pose);
            
            // Timeout after 30 seconds to prevent infinite formation attempts
            if (elapsed > 30.0) {
                printf("⚠️  Formation timeout - moving to next phase\n");
                current_phase = TRIANGLE_MANEUVER;
                cycle_start_time.setToNow();
                break;
            }
            
            if (robot1_at_target && robot2_at_target) {
                printf("✅ Phase 2: Both robots reached targets - executing triangle maneuver...\n");
                current_phase = TRIANGLE_MANEUVER;
                cycle_start_time.setToNow();  // Reset timer
            } else {
                // Continue formation control
                moveRobotToTarget(1, robot1_data.target_pose);
                moveRobotToTarget(2, robot2_data.target_pose);
                
                // Progress update every 10 seconds
                static int progress_counter = 0;
                if (++progress_counter % 100 == 0) {  // Every 10 seconds
                    printf("Formation Progress: R1=%s, R2=%s (%.1fs elapsed)\n", 
                           robot1_at_target ? "✅" : "🔄", 
                           robot2_at_target ? "✅" : "🔄", elapsed);
                }
            }
            break;
        }
            
        case TRIANGLE_MANEUVER: {
            if (elapsed > 15.0) {  // 15 seconds of maneuvering
                printf("✅ Phase 3: Triangle maneuver complete - returning robots to origin...\n");
                current_phase = RETURNING_HOME;
                cycle_start_time.setToNow();
            } else {
                executeTriangleManeuver();
            }
            break;
        }
            
        case RETURNING_HOME: {
            // Check if both robots are back at origin
            bool robot1_home = isAtTarget(1, robot1_data.initial_pose);
            bool robot2_home = isAtTarget(2, robot2_data.initial_pose);
            
            // Timeout after 30 seconds
            if (elapsed > 30.0) {
                printf("⚠️  Return timeout - cycle complete\n");
                cycle_running = false;
                current_phase = READY;
                break;
            }
            
            if (robot1_home && robot2_home) {
                printf("🎉 CYCLE COMPLETED SUCCESSFULLY!\n");
                printf("   Formation: %.0fmm triangle\n", side_length);
                printf("   Duration: %.1f seconds\n", elapsed);
                printf("   Both robots returned to origin\n");
                printf("   Ready for next cycle in 5 seconds...\n\n");
                cycle_running = false;
                current_phase = READY;
            } else {
                returnToOrigin();
                
                // Progress update every 10 seconds
                static int return_counter = 0;
                if (++return_counter % 100 == 0) {  // Every 10 seconds
                    printf("Return Progress: R1=%s, R2=%s (%.1fs elapsed)\n", 
                           robot1_home ? "✅" : "🔄", 
                           robot2_home ? "✅" : "🔄", elapsed);
                }
            }
            break;
        }
            
        default:
            break;
    }
}

void SimpleTriangleFormation::computeTriangleVertices()
{
    // Use current robot positions to compute triangle
    double robot1_x = robot1_data.x;
    double robot1_y = robot1_data.y;
    double robot2_x = robot2_data.x;
    double robot2_y = robot2_data.y;
    
    // Calculate midpoint between robots
    double center_x = (robot1_x + robot2_x) / 2.0;
    double center_y = (robot1_y + robot2_y) / 2.0;
    
    // If robots are at origin, use default position
    if (center_x == 0 && center_y == 0) {
        center_x = 1000.0;
        center_y = 1000.0;
    }
    
    // Make triangle smaller and more practical for the room
    double practical_side_length = std::min(side_length, 600.0);  // Max 60cm sides
    double h = practical_side_length * sqrt(3.0) / 2.0;  // Height of equilateral triangle
    
    // Compute equilateral triangle vertices
    triangle_vertex1 = ArPose(center_x, center_y + 2*h/3, 0);
    triangle_vertex2 = ArPose(center_x - practical_side_length/2, center_y - h/3, 0);
    triangle_vertex3 = ArPose(center_x + practical_side_length/2, center_y - h/3, 0);
    triangle_centroid = ArPose(center_x, center_y, 0);
    
    printf("Triangle vertices computed (%.0fmm sides):\n", practical_side_length);
    printf("  V1: (%.0f, %.0f)\n", triangle_vertex1.getX(), triangle_vertex1.getY());
    printf("  V2: (%.0f, %.0f)\n", triangle_vertex2.getX(), triangle_vertex2.getY());
    printf("  V3: (%.0f, %.0f)\n", triangle_vertex3.getX(), triangle_vertex3.getY());
    printf("  Center: (%.0f, %.0f)\n", triangle_centroid.getX(), triangle_centroid.getY());
    
    // Update side length for maneuver
    side_length = practical_side_length;
}

void SimpleTriangleFormation::assignRobotsToVertices()
{
    // Simple assignment: Robot 1 -> V1, Robot 2 -> V2
    robot1_data.target_pose = triangle_vertex1;
    robot2_data.target_pose = triangle_vertex2;
    
    printf("Robot assignments:\n");
    printf("  Robot 1 -> Vertex 1: (%.0f, %.0f)\n", triangle_vertex1.getX(), triangle_vertex1.getY());
    printf("  Robot 2 -> Vertex 2: (%.0f, %.0f)\n", triangle_vertex2.getX(), triangle_vertex2.getY());
}

void SimpleTriangleFormation::moveRobotToTarget(int robot_id, const ArPose& target)
{
    RobotData& data = (robot_id == 1) ? robot1_data : robot2_data;
    ArRobot* robot = (robot_id == 1) ? robot1 : robot2;
    
    // Calculate distance and angle to target
    double dx = target.getX() - data.x;
    double dy = target.getY() - data.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < pos_tolerance) {
        // At target
        setRobotVelocity(robot, 0.0, 0.0);
        data.at_target = true;
        printf("Robot %d: ✅ REACHED TARGET!\n", robot_id);
        return;
    }
    
    // Calculate desired heading
    double desired_theta = atan2(dy, dx) * 180.0 / M_PI;
    double theta_error = desired_theta - data.theta;
    
    // Normalize angle error
    while (theta_error > 180.0) theta_error -= 360.0;
    while (theta_error < -180.0) theta_error += 360.0;
    
    // Improved control logic
    double linear_vel = 0.0;
    double angular_vel = 0.0;
    
    // First, orient towards target if angle error is large
    if (abs(theta_error) > 15.0) {
        // Turn towards target
        angular_vel = (theta_error > 0) ? 20.0 : -20.0;
        linear_vel = 0.0;  // Don't move forward while turning
    } else {
        // Move forward towards target
        linear_vel = std::min(max_velocity, distance * 0.3);  // Slower, more controlled
        angular_vel = theta_error * 0.3;  // Fine-tune heading
    }
    
    // Apply collision avoidance
    if (data.min_sonar_distance < 800.0) {  // Obstacle within 80cm
        linear_vel *= 0.3;  // Slow down significantly
        angular_vel = (data.min_sonar_distance < 400.0) ? 30.0 : 15.0;  // Turn away
        printf("Robot %d: ⚠️  OBSTACLE DETECTED (%.0fmm) - avoiding\n", robot_id, data.min_sonar_distance);
    }
    
    // Limit velocities
    linear_vel = std::max(0.0, std::min(max_velocity, linear_vel));
    angular_vel = std::max(-30.0, std::min(30.0, angular_vel));
    
    // Set robot velocity
    setRobotVelocity(robot, linear_vel, angular_vel);
    
    // Debug output
    static int debug_counter = 0;
    if (++debug_counter % 30 == 0) {  // Every 3 seconds
        printf("Robot %d: Target=(%.0f,%.0f), Current=(%.0f,%.0f), Dist=%.0f, Vel=%.0f, RotVel=%.0f, AngleErr=%.1f°\n", 
               robot_id, target.getX(), target.getY(), data.x, data.y, distance, linear_vel, angular_vel, theta_error);
    }
}

void SimpleTriangleFormation::executeTriangleManeuver()
{
    // Simple triangle maneuver - robots maintain formation while rotating
    static double maneuver_angle = 0.0;
    maneuver_angle += 2.0;  // 2 degrees per control cycle
    
    if (maneuver_angle >= 360.0) {
        maneuver_angle = 0.0;
    }
    
    // Update target poses for rotation around centroid
    double cx = triangle_centroid.getX();
    double cy = triangle_centroid.getY();
    double h = side_length * sqrt(3.0) / 2.0;
    
    double angle1 = maneuver_angle * M_PI / 180.0;
    double angle2 = (maneuver_angle + 120) * M_PI / 180.0;
    
    robot1_data.target_pose = ArPose(cx + 2*h/3 * cos(angle1), cy + 2*h/3 * sin(angle1), maneuver_angle);
    robot2_data.target_pose = ArPose(cx + 2*h/3 * cos(angle2), cy + 2*h/3 * sin(angle2), maneuver_angle + 120);
    
    // Move robots to new targets
    moveRobotToTarget(1, robot1_data.target_pose);
    moveRobotToTarget(2, robot2_data.target_pose);
}

void SimpleTriangleFormation::returnToOrigin()
{
    robot1_data.target_pose = robot1_data.initial_pose;
    robot2_data.target_pose = robot2_data.initial_pose;
    
    moveRobotToTarget(1, robot1_data.target_pose);
    moveRobotToTarget(2, robot2_data.target_pose);
}

bool SimpleTriangleFormation::isAtTarget(int robot_id, const ArPose& target)
{
    RobotData& data = (robot_id == 1) ? robot1_data : robot2_data;
    
    double dx = target.getX() - data.x;
    double dy = target.getY() - data.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    return distance < pos_tolerance;
}

void SimpleTriangleFormation::setRobotVelocity(ArRobot* robot, double linear, double angular)
{
    robot->setVel(linear);
    robot->setRotVel(angular);
}

double SimpleTriangleFormation::calculateDistance(const ArPose& p1, const ArPose& p2)
{
    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    return sqrt(dx*dx + dy*dy);
}

void SimpleTriangleFormation::logSystemState()
{
    printf("Status: Phase=%d, R1=(%.0f,%.0f,%.1f°), R2=(%.0f,%.0f,%.1f°), Dist=%.0fmm\n",
           current_phase, robot1_data.x, robot1_data.y, robot1_data.theta,
           robot2_data.x, robot2_data.y, robot2_data.theta,
           calculateDistance(ArPose(robot1_data.x, robot1_data.y, 0), 
                           ArPose(robot2_data.x, robot2_data.y, 0)));
}

// Main program
int main(int argc, char** argv)
{
    printf("=== SIMPLE TRIANGLE FORMATION SYSTEM ===\n");
    printf("Dual-robot formation control with triangle patterns\n");
    printf("Features: Basic formation, collision avoidance, return-to-origin\n\n");
    
    SimpleTriangleFormation system;
    
    if (!system.initialize()) {
        printf("❌ System initialization failed. Exiting.\n");
        return 1;
    }
    
    system.run();
    system.stop();
    
    Aria::exit(0);
    return 0;
}
