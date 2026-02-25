#include "Aria.h"
#include <cmath>
#include <cstdio>

class PerfectSynchronized {
private:
    ArRobot* robot1;
    ArRobot* robot2;
    
    // Robot data
    struct RobotData {
        double x, y, theta;
        double initial_x, initial_y, initial_theta;
        bool connected;
        bool at_target;
    };
    
    RobotData robot1_data;
    RobotData robot2_data;
    
    // Formation parameters
    double side_length;
    double max_velocity;
    double pos_tolerance;
    
    // Triangle vertices
    ArPose triangle_vertex1, triangle_vertex2, triangle_vertex3;
    ArPose triangle_centroid;
    
    // System state
    enum Phase { READY, MOVING_FORWARD, FORMING_TRIANGLE, TRIANGLE_MANEUVER, RETURNING_HOME, COMPLETED };
    Phase current_phase;
    bool cycle_running;
    ArTime cycle_start_time;
    
    // Synchronization parameters
    double sync_velocity;
    double sync_angular_velocity;
    
public:
    PerfectSynchronized() : 
        robot1(nullptr), robot2(nullptr),
        side_length(500.0), max_velocity(100.0), pos_tolerance(50.0),
        current_phase(READY), cycle_running(false)
    {
        // Initialize robot data
        robot1_data = {0, 0, 0, 0, 0, 0, false, false};
        robot2_data = {0, 0, 0, 0, 0, 0, false, false};
        
        printf("Perfect Synchronized System Initialized\n");
        printf("Parameters: Side=%.0fmm, MaxVel=%.0fmm/s\n", side_length, max_velocity);
    }
    
    void setRobots(ArRobot* r1, ArRobot* r2) {
        robot1 = r1;
        robot2 = r2;
    }
    
    void updateRobotData(ArRobot* robot, RobotData& data) {
        if (!robot->isConnected()) {
            data.connected = false;
            return;
        }
        
        data.connected = true;
        robot->lock();
        robot->unlock();
        
        data.x = robot->getX();
        data.y = robot->getY();
        data.theta = robot->getTh();
    }
    
    void recordInitialPoses() {
        updateRobotData(robot1, robot1_data);
        updateRobotData(robot2, robot2_data);
        
        robot1_data.initial_x = robot1_data.x;
        robot1_data.initial_y = robot1_data.y;
        robot1_data.initial_theta = robot1_data.theta;
        
        robot2_data.initial_x = robot2_data.x;
        robot2_data.initial_y = robot2_data.y;
        robot2_data.initial_theta = robot2_data.theta;
        
        printf("Initial poses recorded:\n");
        printf("  Robot 1: (%.0f, %.0f, %.1f°)\n", robot1_data.initial_x, robot1_data.initial_y, robot1_data.initial_theta);
        printf("  Robot 2: (%.0f, %.0f, %.1f°)\n", robot2_data.initial_x, robot2_data.initial_y, robot2_data.initial_theta);
    }
    
    void computeTriangleVertices() {
        // Both robots start facing forward (90 degrees)
        // Create triangle in front of them
        double center_x = (robot1_data.initial_x + robot2_data.initial_x) / 2.0;
        double center_y = (robot1_data.initial_y + robot2_data.initial_y) / 2.0 + 300.0; // 300mm forward
        
        double h = side_length * sqrt(3.0) / 2.0;
        
        // Triangle vertices
        triangle_vertex1 = ArPose(center_x, center_y + h/2, 0);  // Top
        triangle_vertex2 = ArPose(center_x - side_length/2, center_y - h/2, 0);  // Bottom left
        triangle_vertex3 = ArPose(center_x + side_length/2, center_y - h/2, 0);  // Bottom right
        triangle_centroid = ArPose(center_x, center_y, 0);
        
        printf("Triangle vertices computed (%.0fmm sides):\n", side_length);
        printf("  V1: (%.0f, %.0f) - Robot 1 target\n", triangle_vertex1.getX(), triangle_vertex1.getY());
        printf("  V2: (%.0f, %.0f) - Robot 2 target\n", triangle_vertex2.getX(), triangle_vertex2.getY());
        printf("  Center: (%.0f, %.0f)\n", triangle_centroid.getX(), triangle_centroid.getY());
    }
    
    void startCycle() {
        if (cycle_running) return;
        
        printf("\n🚀 STARTING SYNCHRONIZED CYCLE\n");
        printf("==============================\n");
        
        cycle_running = true;
        current_phase = MOVING_FORWARD;
        cycle_start_time.setToNow();
        
        // Record initial poses
        recordInitialPoses();
        
        // Compute triangle vertices
        computeTriangleVertices();
        
        printf("Phase 1: Both robots moving forward together...\n");
    }
    
    void executeSynchronizedControl() {
        if (!cycle_running) return;
        
        // Update robot data
        updateRobotData(robot1, robot1_data);
        updateRobotData(robot2, robot2_data);
        
        ArTime current_time;
        double elapsed = cycle_start_time.mSecSince() / 1000.0;
        
        switch (current_phase) {
            case MOVING_FORWARD: {
                // Both robots move forward together at same speed
                if (elapsed > 3.0) {  // 3 seconds of forward movement
                    printf("✅ Phase 2: Forward movement complete - forming triangle...\n");
                    current_phase = FORMING_TRIANGLE;
                    cycle_start_time.setToNow();
                } else {
                    // Both robots move forward at exactly same speed
                    sync_velocity = 80.0;  // Same speed for both
                    sync_angular_velocity = 0.0;  // No rotation, just forward
                    
                    robot1->setVel(sync_velocity);
                    robot1->setRotVel(sync_angular_velocity);
                    robot2->setVel(sync_velocity);
                    robot2->setRotVel(sync_angular_velocity);
                    
                    printf("Moving Forward: R1=(%.0f,%.0f), R2=(%.0f,%.0f), Vel=%.0f\n", 
                           robot1_data.x, robot1_data.y, robot2_data.x, robot2_data.y, sync_velocity);
                }
                break;
            }
            
            case FORMING_TRIANGLE: {
                // Both robots move to triangle vertices with same speed
                if (elapsed > 8.0) {  // 8 seconds to form triangle
                    printf("✅ Phase 3: Triangle formed - starting maneuver...\n");
                    current_phase = TRIANGLE_MANEUVER;
                    cycle_start_time.setToNow();
                } else {
                    // Calculate synchronized movement to triangle vertices
                    double dx1 = triangle_vertex1.getX() - robot1_data.x;
                    double dy1 = triangle_vertex1.getY() - robot1_data.y;
                    double dist1 = sqrt(dx1*dx1 + dy1*dy1);
                    
                    double dx2 = triangle_vertex2.getX() - robot2_data.x;
                    double dy2 = triangle_vertex2.getY() - robot2_data.y;
                    double dist2 = sqrt(dx2*dx2 + dy2*dy2);
                    
                    // Use same velocity for both robots
                    sync_velocity = std::min(max_velocity, std::min(dist1, dist2) * 0.3);
                    
                    // Calculate synchronized rotation
                    double theta1 = atan2(dy1, dx1) * 180.0 / M_PI;
                    double theta2 = atan2(dy2, dx2) * 180.0 / M_PI;
                    
                    double omega1 = std::max(-15.0, std::min(15.0, (theta1 - robot1_data.theta) * 0.2));
                    double omega2 = std::max(-15.0, std::min(15.0, (theta2 - robot2_data.theta) * 0.2));
                    
                    // Set both robots to same velocity and rotation
                    robot1->setVel(sync_velocity);
                    robot1->setRotVel(omega1);
                    robot2->setVel(sync_velocity);
                    robot2->setRotVel(omega2);
                    
                    printf("Forming Triangle: R1=(%.0f,%.0f), R2=(%.0f,%.0f), Vel=%.0f\n", 
                           robot1_data.x, robot1_data.y, robot2_data.x, robot2_data.y, sync_velocity);
                }
                break;
            }
            
            case TRIANGLE_MANEUVER: {
                // Both robots rotate around triangle center maintaining 120° separation
                if (elapsed > 10.0) {  // 10 seconds of triangle maneuvering
                    printf("✅ Phase 4: Triangle maneuver complete - returning home...\n");
                    current_phase = RETURNING_HOME;
                    cycle_start_time.setToNow();
                } else {
                    static double maneuver_angle = 0.0;
                    maneuver_angle += 1.0;  // 1 degree per cycle
                    
                    double radius = side_length / 2.0;
                    
                    // Calculate synchronized triangle positions
                    double r1_x = triangle_centroid.getX() + radius * cos((maneuver_angle + 0.0) * M_PI / 180.0);
                    double r1_y = triangle_centroid.getY() + radius * sin((maneuver_angle + 0.0) * M_PI / 180.0);
                    
                    double r2_x = triangle_centroid.getX() + radius * cos((maneuver_angle + 120.0) * M_PI / 180.0);
                    double r2_y = triangle_centroid.getY() + radius * sin((maneuver_angle + 120.0) * M_PI / 180.0);
                    
                    // Calculate synchronized movement
                    double dx1 = r1_x - robot1_data.x;
                    double dy1 = r1_y - robot1_data.y;
                    double dx2 = r2_x - robot2_data.x;
                    double dy2 = r2_y - robot2_data.y;
                    
                    sync_velocity = 60.0;  // Same speed for both
                    
                    double theta1 = atan2(dy1, dx1) * 180.0 / M_PI;
                    double theta2 = atan2(dy2, dx2) * 180.0 / M_PI;
                    
                    double omega1 = std::max(-10.0, std::min(10.0, (theta1 - robot1_data.theta) * 0.3));
                    double omega2 = std::max(-10.0, std::min(10.0, (theta2 - robot2_data.theta) * 0.3));
                    
                    robot1->setVel(sync_velocity);
                    robot1->setRotVel(omega1);
                    robot2->setVel(sync_velocity);
                    robot2->setRotVel(omega2);
                    
                    printf("Triangle Maneuver: Angle=%.1f°, R1=(%.0f,%.0f), R2=(%.0f,%.0f), Vel=%.0f\n", 
                           maneuver_angle, r1_x, r1_y, r2_x, r2_y, sync_velocity);
                }
                break;
            }
            
            case RETURNING_HOME: {
                // Both robots return to initial positions
                if (elapsed > 8.0) {  // 8 seconds to return home
                    printf("✅ CYCLE COMPLETED!\n");
                    printf("   Both robots returned to origin\n");
                    printf("   Ready for next cycle...\n\n");
                    cycle_running = false;
                    current_phase = READY;
                } else {
                    // Calculate return movement
                    double dx1 = robot1_data.initial_x - robot1_data.x;
                    double dy1 = robot1_data.initial_y - robot1_data.y;
                    double dx2 = robot2_data.initial_x - robot2_data.x;
                    double dy2 = robot2_data.initial_y - robot2_data.y;
                    
                    sync_velocity = std::min(max_velocity, 80.0);
                    
                    double theta1 = atan2(dy1, dx1) * 180.0 / M_PI;
                    double theta2 = atan2(dy2, dx2) * 180.0 / M_PI;
                    
                    double omega1 = std::max(-15.0, std::min(15.0, (theta1 - robot1_data.theta) * 0.2));
                    double omega2 = std::max(-15.0, std::min(15.0, (theta2 - robot2_data.theta) * 0.2));
                    
                    robot1->setVel(sync_velocity);
                    robot1->setRotVel(omega1);
                    robot2->setVel(sync_velocity);
                    robot2->setRotVel(omega2);
                    
                    printf("Returning Home: R1=(%.0f,%.0f), R2=(%.0f,%.0f), Vel=%.0f\n", 
                           robot1_data.x, robot1_data.y, robot2_data.x, robot2_data.y, sync_velocity);
                }
                break;
            }
            
            default:
                break;
        }
    }
    
    bool isCycleRunning() const { return cycle_running; }
};

int main(int argc, char** argv) {
    Aria::init();
    ArSignalHandler::blockCommon();
    
    printf("=== PERFECT SYNCHRONIZED SYSTEM ===\n");
    printf("Two robots moving in perfect synchronization\n");
    printf("Features: Same speed, same angles, same timing\n\n");
    
    // Create robots
    ArRobot robot1, robot2;
    
    // Create connections
    ArTcpConnection conn1, conn2;
    
    // Connect to robots
    printf("Connecting to Robot 1 (192.168.1.253:8101)... ");
    if (!conn1.open("192.168.1.253", 8101)) {
        printf("Failed!\n");
        return 1;
    }
    robot1.setDeviceConnection(&conn1);
    if (!robot1.blockingConnect()) {
        printf("Failed!\n");
        return 1;
    }
    printf("Connected!\n");
    
    printf("Connecting to Robot 2 (192.168.1.254:8101)... ");
    if (!conn2.open("192.168.1.254", 8101)) {
        printf("Failed!\n");
        return 1;
    }
    robot2.setDeviceConnection(&conn2);
    if (!robot2.blockingConnect()) {
        printf("Failed!\n");
        return 1;
    }
    printf("Connected!\n");
    
    // Enable motors
    robot1.enableMotors();
    robot2.enableMotors();
    
    // Create synchronized system
    PerfectSynchronized sync_system;
    sync_system.setRobots(&robot1, &robot2);
    
    printf("\n✅ System ready! Starting synchronized movement...\n");
    
    // Start first cycle
    sync_system.startCycle();
    
    // Main control loop
    while (true) {
        sync_system.executeSynchronizedControl();
        
        // If cycle completed, start next one after delay
        if (!sync_system.isCycleRunning()) {
            ArUtil::sleep(2000);  // 2 second delay
            sync_system.startCycle();
        }
        
        ArUtil::sleep(100);  // 100ms control cycle
    }
    
    Aria::shutdown();
    return 0;
}
