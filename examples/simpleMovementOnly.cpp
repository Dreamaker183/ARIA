#include "Aria.h"
#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#include <algorithm>

/**
 * ============================================================================
 * MULTI-ROBOT CONSENSUS FORMATION CONTROL (SCALABLE ARCHITECTURE)
 * ============================================================================
 * Based on Graph Theoretic Consensus Protocols (e.g., MosesEbere/multi-robot-consensus)
 * * ARCHITECTURE:
 * 1. Multi-Agent System (MAS): Robots are treated as generic nodes in a graph.
 * 2. Topology: Interaction defined by an Adjacency Matrix.
 * 3. Dynamics: Feedback Linearization converts Unicycle -> Single Integrator.
 * 4. Safety: Artificial Potential Field (APF) using Sonar.
 * * CONFIGURATION:
 * - N Robots (Scalable)
 * - Movement: Circular Trajectory tracking
 */

// ============================================================================
// UTILITY STRUCTURES
// ============================================================================
struct Vector2D {
    double x;
    double y;
    
    Vector2D operator+(const Vector2D& other) const { return {x + other.x, y + other.y}; }
    Vector2D operator-(const Vector2D& other) const { return {x - other.x, y - other.y}; }
    Vector2D operator*(double scalar) const { return {x * scalar, y * scalar}; }
    double magnitude() const { return std::sqrt(x*x + y*y); }
};

// ============================================================================
// ROBOT AGENT CLASS
// ============================================================================
class RobotAgent {
public:
    int id;
    ArRobot* robot;
    ArTcpConnection* conn;
    Vector2D offset; // Desired formation offset from virtual center
    
    // Linearization constant (Look-ahead distance)
    const double L = 0.25; 

    RobotAgent(int id, std::string ip, Vector2D desired_offset) : id(id), offset(desired_offset) {
        robot = new ArRobot();
        conn = new ArTcpConnection();
        
        printf("[Agent %d] Connecting to %s...\n", id, ip.c_str());
        if (conn->open(ip.c_str(), 8101) != 0 || !robot->setDeviceConnection(conn) || !robot->blockingConnect()) {
            printf("[Agent %d] Connection Failed!\n", id);
        } else {
            printf("[Agent %d] Connected.\n", id);
            robot->enableMotors();
            robot->enableSonar(); // Enable Ultrasound
            robot->runAsync(true);
        }
    }

    ~RobotAgent() {
        robot->stop();
        robot->disconnect();
        delete robot;
        delete conn;
    }

    bool isConnected() { return robot->isConnected(); }

    // Returns the Feedback Linearized Position (Point Z)
    Vector2D getLinearizedPosition() {
        double th = robot->getTh() * M_PI / 180.0;
        double x = robot->getX() / 1000.0;
        double y = robot->getY() / 1000.0;
        
        // Z = [x + L cos(th), y + L sin(th)]
        return { x + L * std::cos(th), y + L * std::sin(th) };
    }
    
    double getHeading() { return robot->getTh() * M_PI / 180.0; }

    // Calculates Repulsive Force from Sonar Readings
    Vector2D getObstacleAvoidanceForce(double gain, double threshold) {
        Vector2D force = {0, 0};
        int numSonar = robot->getNumSonar();
        double rob_x = robot->getX() / 1000.0;
        double rob_y = robot->getY() / 1000.0;

        for(int i = 0; i < numSonar; i++) {
            ArSensorReading* reading = robot->getSonarReading(i);
            if(!reading || !reading->isNew()) continue;

            double range = reading->getRange() / 1000.0;
            if (range < threshold && range > 0.05) {
                double obs_x = reading->getX() / 1000.0;
                double obs_y = reading->getY() / 1000.0;
                
                // Vector Robot -> Obstacle
                double dx = obs_x - rob_x;
                double dy = obs_y - rob_y;
                double dist = std::sqrt(dx*dx + dy*dy);

                // Repulsive Vector (Obstacle -> Robot)
                // F = gain * (1/d - 1/thresh) * unit_vector_away
                if (dist > 0) {
                    double mag = gain * (1.0/dist - 1.0/threshold);
                    force.x -= (dx/dist) * mag;
                    force.y -= (dy/dist) * mag;
                }
            }
        }
        return force;
    }

    void setControlInput(Vector2D u) {
        // Inverse Feedback Linearization
        // v = u_x cos(th) + u_y sin(th)
        // w = (-u_x sin(th) + u_y cos(th)) / L
        
        double th = getHeading();
        double v = u.x * std::cos(th) + u.y * std::sin(th);
        double w = (-u.x * std::sin(th) + u.y * std::cos(th)) / L;

        // Saturation Limits
        const double MAX_V = 0.8; // m/s
        const double MAX_W = 1.2; // rad/s
        
        v = std::max(-MAX_V, std::min(MAX_V, v));
        w = std::max(-MAX_W, std::min(MAX_W, w));

        robot->setVel(v * 1000.0);
        robot->setRotVel(w * 180.0 / M_PI);
    }
};

// ============================================================================
// CONSENSUS CONTROLLER SYSTEM
// ============================================================================
class MultiRobotConsensus
{
private:
    std::vector<RobotAgent*> agents;
    std::vector<std::vector<int>> adjacencyMatrix;
    
    // Gains
    const double Kp = 1.5;           // Consensus Gain
    const double Obs_Gain = 1.2;     // Obstacle Gain
    const double Obs_Thresh = 0.8;   // Obstacle Threshold (m)
    
    // Trajectory
    const double R_orbit = 1.5;
    const double V_orbit = 0.4;

public:
    MultiRobotConsensus() {
        Aria::init();
    }

    ~MultiRobotConsensus() {
        for(auto agent : agents) delete agent;
        Aria::shutdown();
    }

    void addRobot(std::string ip, double offsetX, double offsetY) {
        int id = agents.size();
        agents.push_back(new RobotAgent(id, ip, {offsetX, offsetY}));
    }

    // Initialize a Ring Topology (or Line for 2 robots)
    void buildTopology() {
        int n = agents.size();
        adjacencyMatrix.assign(n, std::vector<int>(n, 0));
        
        // Fully Connected (or Ring)
        for(int i=0; i<n; i++) {
            for(int j=i+1; j<n; j++) {
                adjacencyMatrix[i][j] = 1;
                adjacencyMatrix[j][i] = 1;
            }
        }
        printf("Topology Built: Fully Connected Graph with %d agents.\n", n);
    }

    void run() {
        printf("Starting Consensus Loop...\n");
        ArUtil::sleep(1000);
        ArTime timer;
        timer.setToNow();

        while(true) {
            // Check connections
            bool allConnected = true;
            for(auto agent : agents) if(!agent->isConnected()) allConnected = false;
            if(!allConnected) break;

            double t = timer.mSecSince() / 1000.0;

            // 1. Calculate Global Reference Trajectory (Virtual Leader)
            double omega = V_orbit / R_orbit;
            Vector2D v_ref = {
                -V_orbit * std::sin(omega * t),
                 V_orbit * std::cos(omega * t)
            };

            // 2. Calculate Control Input for Each Agent
            for (int i = 0; i < agents.size(); ++i) {
                Vector2D u_consensus = {0, 0};
                Vector2D p_i = agents[i]->getLinearizedPosition();
                Vector2D d_i = agents[i]->offset;

                // Iterate Neighbors (Graph Theory Consensus)
                for (int j = 0; j < agents.size(); ++j) {
                    if (adjacencyMatrix[i][j] == 1) {
                        Vector2D p_j = agents[j]->getLinearizedPosition();
                        Vector2D d_j = agents[j]->offset;
                        
                        // Error Term: (p_i - p_j) - (d_i - d_j)
                        Vector2D error = (p_i - p_j) - (d_i - d_j);
                        
                        // Accumulate Consensus Force: -Kp * sum(error)
                        u_consensus = u_consensus - (error * Kp);
                    }
                }

                // Obstacle Avoidance Force
                Vector2D u_avoid = agents[i]->getObstacleAvoidanceForce(Obs_Gain, Obs_Thresh);

                // Total Control Input: Reference + Consensus + Avoidance
                Vector2D u_total = v_ref + u_consensus + u_avoid;

                // Apply
                agents[i]->setControlInput(u_total);
                
                // Debug Log (Agent 0)
                if(i == 0 && (int)(t*10)%10 == 0) {
                    printf("T:%.1f | Ag0 Avoid: [%.2f, %.2f]\n", t, u_avoid.x, u_avoid.y);
                }
            }

            ArUtil::sleep(100); // 10Hz
        }
    }
};

int main(int argc, char** argv) {
    ArSignalHandler::blockCommon();
    
    MultiRobotConsensus swarm;
    
    // Add Robots (IP, OffsetX, OffsetY)
    // Formation: Line (Side by Side) or Triangle
    swarm.addRobot("192.168.1.2", 0.0,  0.4); // Robot 1 (Left)
    swarm.addRobot("192.168.1.3", 0.0, -0.4); // Robot 2 (Right)
    
    // swarm.addRobot("192.168.1.4", 0.8, 0.0); // Easily add a 3rd robot!

    swarm.buildTopology();
    swarm.run();
    
    return 0;
}