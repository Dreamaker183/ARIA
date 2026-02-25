/*
Simple Consensual Movements for Pioneer 3DX Robot
Simplified version to avoid ARIA virtual function issues
*/

#include "Aria.h"
#include <iostream>
#include <cstdio>
#include <cmath>

int main(int argc, char** argv)
{
    printf("=== SIMPLE CONSENSUAL MOVEMENTS ===\n");
    printf("Pioneer 3DX Robot Control\n\n");
    
    Aria::init();
    
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    
    ArRobot robot;
    ArRobotConnector robotConnector(&parser, &robot);
    
    printf("Attempting to connect to robot...\n");
    if (!robotConnector.connectRobot()) {
        printf("❌ Could not connect to robot\n");
        Aria::exit(1);
    }
    
    printf("✅ Connected to robot: %s\n", robot.getName());
    printf("   Type: %s, Subtype: %s\n", robot.getRobotType(), robot.getRobotSubType());
    printf("   Serial: %s\n\n", robot.getRobotName());
    
    // Add sonar device
    ArSonarDevice sonarDev;
    robot.addRangeDevice(&sonarDev);
    
    // Start robot
    robot.runAsync(true);
    
    printf("Initializing robot systems...\n");
    ArUtil::sleep(2000);
    
    printf("✅ Robot ready!\n");
    printf("✅ %d sonar sensors active\n", robot.getNumSonar());
    
    // Enable motors explicitly
    robot.enableMotors();
    robot.comInt(ArCommands::ENABLE, 1);
    printf("✅ Motors enabled\n\n");
    
    printf("🎮 Press SPACE to start movement, 'q' to quit: ");
    fflush(stdout);
    
    int ch = getchar();
    if (ch == ' ' || ch == '\n') {
        printf("\n🚀 CONSENSUAL MOVEMENTS ACTIVE!\n");
        printf("Robot is now moving autonomously!\n");
        printf("Press Ctrl+C to stop safely\n\n");
        
        // Test movement first
        printf("Testing movement commands...\n");
        robot.setVel(100.0);
        robot.setRotVel(0.0);
        ArUtil::sleep(1000);
        robot.setVel(0.0);
        robot.setRotVel(0.0);
        printf("Movement test complete\n\n");
        
        // Triangle pattern movement
        int counter = 0;
        int triangle_side = 0;  // 0, 1, 2 for the three sides
        double start_x = robot.getX();
        double start_y = robot.getY();
        double start_heading = robot.getTh();
        double side_length = 1000.0;  // 1 meter sides
        double target_heading = start_heading;
        double distance_traveled = 0.0;
        
        printf("🎯 Starting Triangle Pattern Navigation\n");
        printf("   Side 1: Move forward %.0fmm\n", side_length);
        printf("   Side 2: Turn 120° and move forward %.0fmm\n", side_length);
        printf("   Side 3: Turn 120° and move forward %.0fmm\n", side_length);
        printf("   Then repeat...\n\n");
        
        while (robot.isConnected()) {
            ArUtil::sleep(100);  // 10Hz loop
            
            // Get sensor data
            double minDist = robot.getClosestSonarRange(-90.0, 90.0);
            double current_x = robot.getX();
            double current_y = robot.getY();
            double current_heading = robot.getTh();
            
            // Calculate distance traveled on current side
            if (triangle_side == 0) {
                distance_traveled = sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2));
            } else {
                // For sides 2 and 3, calculate from last turn point
                distance_traveled = sqrt(pow(current_x - start_x, 2) + pow(current_y - start_y, 2)) - (triangle_side * side_length);
            }
            
            // Check if we've completed the current side
            if (distance_traveled >= side_length) {
                triangle_side = (triangle_side + 1) % 3;
                target_heading = start_heading + (triangle_side * 120.0);
                start_x = current_x;
                start_y = current_y;
                distance_traveled = 0.0;
                printf("🔄 Completed side %d, starting side %d (target heading: %.1f°)\n", 
                       (triangle_side == 0) ? 3 : triangle_side, triangle_side + 1, target_heading);
            }
            
            // Obstacle avoidance takes priority
            if (minDist < 500.0) {
                // Close obstacle - turn right
                robot.setVel(0.0);
                robot.setRotVel(-45.0);
                printf("⚠️  Obstacle detected (%.0fmm) - turning right\n", minDist);
            } else if (minDist < 800.0) {
                // Obstacle ahead - turn left
                robot.setVel(100.0);
                robot.setRotVel(30.0);
                printf("⚠️  Obstacle ahead (%.0fmm) - turning left\n", minDist);
            } else {
                // Clear path - follow triangle pattern
                double heading_error = target_heading - current_heading;
                
                // Normalize heading error to -180 to +180
                while (heading_error > 180.0) heading_error -= 360.0;
                while (heading_error < -180.0) heading_error += 360.0;
                
                if (fabs(heading_error) > 10.0) {
                    // Turn towards target heading
                    if (heading_error > 0) {
                        robot.setVel(50.0);
                        robot.setRotVel(30.0);
                        printf("🔄 Side %d: Turning left to %.1f° (error: %.1f°)\n", 
                               triangle_side + 1, target_heading, heading_error);
                    } else {
                        robot.setVel(50.0);
                        robot.setRotVel(-30.0);
                        printf("🔄 Side %d: Turning right to %.1f° (error: %.1f°)\n", 
                               triangle_side + 1, target_heading, heading_error);
                    }
                } else {
                    // Move forward on current side
                    robot.setVel(200.0);
                    robot.setRotVel(0.0);
                    printf("➡️  Side %d: Moving forward (%.0f/%.0fmm, heading: %.1f°)\n", 
                           triangle_side + 1, distance_traveled, side_length, current_heading);
                }
            }
            
            // Status every 5 seconds
            if (++counter % 50 == 0) {
                printf("Status: Pos(%.0f,%.0f) Heading:%.1f° Battery:%.1fV Sonar:%.0fmm\n",
                       robot.getX(), robot.getY(), robot.getTh(), 
                       robot.getBatteryVoltage(), minDist);
            }
        }
    } else if (ch == 'q' || ch == 'Q') {
        printf("\n👋 Exiting...\n");
    }
    
    printf("\nStopping robot...\n");
    robot.stop();
    robot.disconnect();
    Aria::exit(0);
    return 0;
}
