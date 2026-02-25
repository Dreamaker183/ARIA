/*
Multi-Robot Consensus Navigation System for Pioneer 3DX
Copyright (C) 2024

Features:
- Decentralized behavior
- UDP state broadcasting
- Consensus logic (alignment, rendezvous)
- Nonholonomic mapping
- Sonar obstacle repulsion
*/

#include "Aria.h"
#include <vector>
#include <map>
#include <cmath>
#include <string>
#include <cstring>
#include <iostream>
#include <algorithm>

// Standard POSIX networking for robust UDP broadcasts
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

struct RobotStateMsg {
    int robot_id;
    int formation_type; // 0=Scatter, 1=Triangle, 2=Circle, 3=Line
    double x;
    double y;
    double theta;
    double v;
    double w;
    int movement_enabled;      // 0/1 synchronized movement command
    double group_speed_cmd;    // signed group speed command (mm/s)
    double sonar[8]; // Forward-facing sonars typically 0-7 on P3DX
};

// Formations
enum Formations {
    SCATTER = 0,
    TRIANGLE = 1,
    CIRCLE = 2,
    LINE_UP = 3
};

struct NeighborState {
    RobotStateMsg msg;
    ArTime lastReceived;
};

// Global list of all navigators so keypresses can be broadcasted to all robots
std::vector<class MultiAgentConsensus*> globalNavigators;

class MultiAgentConsensus
{
public:
    MultiAgentConsensus(ArRobot* robot, int robotId);
    ~MultiAgentConsensus();
    
    void run();
    void stop();
    void waitForStartCommand();
    
    // WiFi / Networking
    bool setupUdpNetwork();
    void broadcastState();
    void receiveNeighborStates();

    // WiFi module connection method
    bool discoverZLAN7104();
    bool connectToZLAN7104(const std::string& ip_address, int port);
    void scanForRobots();
    std::string getZLANIP();
    
    // Keyboard Handling
    void addKeyHandlers(ArKeyHandler* keyHandler);
    void keyTriangle();
    void keyCircle();
    void keyLine();
    void keyScatter();
    void keySpace();
    void keyForward();
    void keyBackward();
    void keyBrake();
    void keyQuit();
    
private:
    ArRobot* myRobot;
    int myId;
    int currentFormation;
    ArFunctorC<MultiAgentConsensus> myTaskCB;
    
    // Networking
    int udpSocket;
    struct sockaddr_in broadcastAddr;
    
    // Neighbors
    std::map<int, NeighborState> myNeighbors;

    // Movement state
    bool movementEnabled;
    double groupSpeed;  // 'f' key: group forward speed (mm/s), 0 = stationary formation
    bool systemReady;
    double lastCmdVel;
    double lastCmdRotVel;
    int formationStableTicks;
    
    // WiFi Connection
    std::string myZLANIP;
    int myZLANPort;
    bool myWiFiConnected;
    
    // Core parameters
    struct Parameters {
        double maxLinearVel = 400.0;    // mm/s
        double maxAngularVel = 60.0;    // deg/s
        double safeDistance = 800.0;    // mm
        double criticalDistance = 400.0;// mm
        double alpha = 0.6;             // Position consensus gain
        double beta = 0.4;              // Velocity consensus gain
        double gamma = 1.5;             // Obstacle repulsion gain
        double kV = 0.8;                // Linear velocity proportional gain
        double kW = 1.5;                // Angular velocity proportional gain
        double watchdogTimeout = 2.0;   // seconds
        int udpBasePort = 50000;  // Each robot uses 50000 + robotId
        double formationTolerance = 250.0;   // mm max relative error to consider formed
        int formationStableCycles = 8;       // number of control cycles to stay within tolerance
        double maxVelStep = 40.0;            // mm/s per control cycle command slew limit
        double maxRotStep = 8.0;             // deg/s per control cycle command slew limit
        double avoidTurnVel = 30.0;          // deg/s turn rate while obstacle-blocked
    } myParams;
    
    // Input Functors
    ArFunctorC<MultiAgentConsensus> myTriangleCB;
    ArFunctorC<MultiAgentConsensus> myCircleCB;
    ArFunctorC<MultiAgentConsensus> myLineCB;
    ArFunctorC<MultiAgentConsensus> myScatterCB;
    ArFunctorC<MultiAgentConsensus> mySpaceCB;
    ArFunctorC<MultiAgentConsensus> myForwardCB;
    ArFunctorC<MultiAgentConsensus> myBackwardCB;
    ArFunctorC<MultiAgentConsensus> myBrakeCB;
    ArFunctorC<MultiAgentConsensus> myQuitCB;

    // Formations
    std::pair<double, double> getFormationOffset(int formationId, int robotId);

    // Internal tasks
    void consensusTask();
    
    // Utilities
    double normalizeAngle(double angle);
    double clampDelta(double target, double current, double maxStep);
    int getLeaderId() const;
    void synchronizeCommandFromLeader();
    double computeFormationError();
    bool isMovementBlocked(double commandedVel, double& turnCmd);
    std::pair<double, double> computeConsensusVector();
    std::pair<double, double> computeObstacleRepulsion();
};

MultiAgentConsensus::MultiAgentConsensus(ArRobot* robot, int robotId) :
    myRobot(robot),
    myId(robotId),
    currentFormation(SCATTER),
    myTaskCB(this, &MultiAgentConsensus::consensusTask),
    udpSocket(-1),
    movementEnabled(false),
    groupSpeed(0.0),
    systemReady(false),
    lastCmdVel(0.0),
    lastCmdRotVel(0.0),
    formationStableTicks(0),
    myZLANPort(8101),
    myWiFiConnected(false),
    myTriangleCB(this, &MultiAgentConsensus::keyTriangle),
    myCircleCB(this, &MultiAgentConsensus::keyCircle),
    myLineCB(this, &MultiAgentConsensus::keyLine),
    myScatterCB(this, &MultiAgentConsensus::keyScatter),
    mySpaceCB(this, &MultiAgentConsensus::keySpace),
    myForwardCB(this, &MultiAgentConsensus::keyForward),
    myBackwardCB(this, &MultiAgentConsensus::keyBackward),
    myBrakeCB(this, &MultiAgentConsensus::keyBrake),
    myQuitCB(this, &MultiAgentConsensus::keyQuit)
{
    globalNavigators.push_back(this);
    setupUdpNetwork();
}

MultiAgentConsensus::~MultiAgentConsensus()
{
    // Remove from global list
    for (auto it = globalNavigators.begin(); it != globalNavigators.end(); ++it) {
        if (*it == this) {
            globalNavigators.erase(it);
            break;
        }
    }
    stop();
    if (udpSocket >= 0) close(udpSocket);
}

bool MultiAgentConsensus::setupUdpNetwork()
{
    int myPort = myParams.udpBasePort + myId;  // Each robot gets unique port
    
    udpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (udpSocket < 0) {
        ArLog::log(ArLog::Terse, "MultiAgentConsensus: Could not create UDP socket");
        return false;
    }

    int broadcastEnable = 1;
    setsockopt(udpSocket, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
    
    int reusePort = 1;
    setsockopt(udpSocket, SOL_SOCKET, SO_REUSEPORT, &reusePort, sizeof(reusePort));
    setsockopt(udpSocket, SOL_SOCKET, SO_REUSEADDR, &reusePort, sizeof(reusePort));

    // Non-blocking
    int flags = fcntl(udpSocket, F_GETFL, 0);
    fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in recvAddr;
    memset(&recvAddr, 0, sizeof(recvAddr));
    recvAddr.sin_family = AF_INET;
    recvAddr.sin_port = htons(myPort);
    recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udpSocket, (struct sockaddr*)&recvAddr, sizeof(recvAddr)) < 0) {
        ArLog::log(ArLog::Terse, "MultiAgentConsensus: Could not bind UDP socket on port %d", myPort);
    }

    // broadcastAddr is no longer used as a single target; we send to each robot's port
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(myPort);
    broadcastAddr.sin_addr.s_addr = inet_addr("127.0.0.1");

    ArLog::log(ArLog::Normal, "MultiAgentConsensus: Robot %d UDP listening on port %d", myId, myPort);
    return true;
}

void MultiAgentConsensus::run()
{
    myRobot->addUserTask("MultiAgentConsensus", 50, &myTaskCB);
    myRobot->lock();
    myRobot->enableMotors();
    myRobot->unlock();
    systemReady = true;
    ArLog::log(ArLog::Normal, "MultiAgentConsensus: Robot %d system ready, motors enabled", myId);
}

void MultiAgentConsensus::stop()
{
    myRobot->remUserTask(&myTaskCB);
    myRobot->stop();
}

void MultiAgentConsensus::waitForStartCommand()
{
    printf("\n🎮 MULTI-AGENT CONSENSUS READY!\n");
    printf("Robot ID: %d (Pioneer 3DX)\n", myId);
    printf("Press SPACE to start moving, 'q' to quit\n");
    printf("Formations: 't'=Triangle, 'c'=Circle, 'l'=Line, 's'=Scatter\n");
    fflush(stdout);
}

void MultiAgentConsensus::addKeyHandlers(ArKeyHandler* keyHandler)
{
    if (keyHandler == NULL) return;
    keyHandler->addKeyHandler(' ', &mySpaceCB);
    keyHandler->addKeyHandler('q', &myQuitCB);
    keyHandler->addKeyHandler('t', &myTriangleCB);
    keyHandler->addKeyHandler('c', &myCircleCB);
    keyHandler->addKeyHandler('l', &myLineCB);
    keyHandler->addKeyHandler('s', &myScatterCB);
    keyHandler->addKeyHandler('f', &myForwardCB);
    keyHandler->addKeyHandler('r', &myBackwardCB);
    keyHandler->addKeyHandler('b', &myBrakeCB);
}

void MultiAgentConsensus::keyTriangle() { for(auto n : globalNavigators) n->currentFormation = TRIANGLE; ArLog::log(ArLog::Normal, "All Robots Formation: TRIANGLE"); }
void MultiAgentConsensus::keyCircle()   { for(auto n : globalNavigators) n->currentFormation = CIRCLE;   ArLog::log(ArLog::Normal, "All Robots Formation: CIRCLE"); }
void MultiAgentConsensus::keyLine()     { for(auto n : globalNavigators) n->currentFormation = LINE_UP;  ArLog::log(ArLog::Normal, "All Robots Formation: LINE UP"); }
void MultiAgentConsensus::keyScatter()  { for(auto n : globalNavigators) n->currentFormation = SCATTER;  ArLog::log(ArLog::Normal, "All Robots Formation: SCATTER"); }
void MultiAgentConsensus::keySpace()
{
    bool targetEnabled = true;
    if (!globalNavigators.empty()) {
        targetEnabled = !globalNavigators.front()->movementEnabled;
    }
    for (auto n : globalNavigators) {
        n->movementEnabled = targetEnabled;
        n->formationStableTicks = 0;
    }
    ArLog::log(ArLog::Normal, "All Robots Movement: %s", targetEnabled ? "ENABLED" : "DISABLED");
}

void MultiAgentConsensus::keyForward()
{
    for (auto n : globalNavigators) {
        n->groupSpeed = 200.0;
        n->movementEnabled = true;
        n->formationStableTicks = 0;
    }
    ArLog::log(ArLog::Normal, ">>> ALL ROBOTS: FORWARD at 200mm/s IN FORMATION <<<");
}

void MultiAgentConsensus::keyBackward()
{
    for (auto n : globalNavigators) {
        n->groupSpeed = -200.0;
        n->movementEnabled = true;
        n->formationStableTicks = 0;
    }
    ArLog::log(ArLog::Normal, ">>> ALL ROBOTS: BACKWARD at 200mm/s IN FORMATION <<<");
}

void MultiAgentConsensus::keyBrake()
{
    for (auto n : globalNavigators) {
        n->groupSpeed = 0.0;
        n->movementEnabled = false;
        n->formationStableTicks = 0;
        n->lastCmdVel = 0.0;
        n->lastCmdRotVel = 0.0;
    }
    ArLog::log(ArLog::Normal, ">>> ALL ROBOTS: STOPPED <<<");
}
void MultiAgentConsensus::keyQuit()     { ArLog::log(ArLog::Normal, "Quit requested"); Aria::exit(0); }

void MultiAgentConsensus::broadcastState()
{
    if (udpSocket < 0) return;

    RobotStateMsg msg;
    msg.robot_id = myId;
    msg.formation_type = currentFormation;
    msg.x = myRobot->getX();
    msg.y = myRobot->getY();
    msg.theta = myRobot->getTh();
    msg.v = myRobot->getVel();
    msg.w = myRobot->getRotVel();
    msg.movement_enabled = movementEnabled ? 1 : 0;
    msg.group_speed_cmd = groupSpeed;

    for (int i = 0; i < 8; i++) {
        ArSensorReading* reading = myRobot->getSonarReading(i);
        msg.sonar[i] = reading ? reading->getRange() : 5000.0;
    }

    // Send to ALL other robots' UDP ports (localhost since all run in same process)
    for (int rid = 1; rid <= 3; rid++) {
        if (rid == myId) continue;  // Don't send to self
        struct sockaddr_in destAddr;
        memset(&destAddr, 0, sizeof(destAddr));
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(myParams.udpBasePort + rid);
        destAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
        sendto(udpSocket, &msg, sizeof(msg), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
    }
}

void MultiAgentConsensus::receiveNeighborStates()
{
    if (udpSocket < 0) return;

    RobotStateMsg msg;
    struct sockaddr_in senderAddr;
    socklen_t senderLen = sizeof(senderAddr);

    while (recvfrom(udpSocket, &msg, sizeof(msg), 0, (struct sockaddr*)&senderAddr, &senderLen) > 0) {
        if (msg.robot_id != myId) {
            NeighborState ns;
            ns.msg = msg;
            ns.lastReceived.setToNow();
            myNeighbors[msg.robot_id] = ns;
        }
    }

    // Prune old neighbors
    auto it = myNeighbors.begin();
    while (it != myNeighbors.end()) {
        if (it->second.lastReceived.secSince() >= myParams.watchdogTimeout) {
            ArLog::log(ArLog::Normal, "MultiAgentConsensus: Watchdog timeout for neighbor %d", it->first);
            it = myNeighbors.erase(it);
        } else {
            ++it;
        }
    }
}

double MultiAgentConsensus::normalizeAngle(double angle)
{
    while (angle > 180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

double MultiAgentConsensus::clampDelta(double target, double current, double maxStep)
{
    double delta = target - current;
    if (delta > maxStep) return current + maxStep;
    if (delta < -maxStep) return current - maxStep;
    return target;
}

int MultiAgentConsensus::getLeaderId() const
{
    int leaderId = myId;
    for (const auto& entry : myNeighbors) {
        if (entry.second.lastReceived.secSince() < myParams.watchdogTimeout) {
            leaderId = std::min(leaderId, entry.first);
        }
    }
    return leaderId;
}

void MultiAgentConsensus::synchronizeCommandFromLeader()
{
    const int leaderId = getLeaderId();
    if (leaderId == myId) return;

    auto it = myNeighbors.find(leaderId);
    if (it == myNeighbors.end()) return;
    if (it->second.lastReceived.secSince() >= myParams.watchdogTimeout) return;

    const RobotStateMsg& leaderMsg = it->second.msg;
    bool leaderMove = (leaderMsg.movement_enabled != 0);
    bool changed = false;

    if (currentFormation != leaderMsg.formation_type) {
        currentFormation = leaderMsg.formation_type;
        changed = true;
    }
    if (movementEnabled != leaderMove) {
        movementEnabled = leaderMove;
        changed = true;
    }
    if (fabs(groupSpeed - leaderMsg.group_speed_cmd) > 1.0) {
        groupSpeed = leaderMsg.group_speed_cmd;
        changed = true;
    }

    if (changed) {
        formationStableTicks = 0;
        ArLog::log(ArLog::Normal,
                   "Robot %d synced command from leader %d: formation=%d, move=%d, speed=%.1f",
                   myId, leaderId, currentFormation, movementEnabled ? 1 : 0, groupSpeed);
    }
}

double MultiAgentConsensus::computeFormationError()
{
    if (myNeighbors.empty()) return 0.0;

    const double myX = myRobot->getX();
    const double myY = myRobot->getY();
    const auto myOffset = getFormationOffset(currentFormation, myId);
    const double myDx = myOffset.first;
    const double myDy = myOffset.second;

    double maxError = 0.0;
    for (const auto& entry : myNeighbors) {
        const NeighborState& ns = entry.second;
        if (ns.lastReceived.secSince() >= myParams.watchdogTimeout) continue;

        const int nid = entry.first;
        const auto nOffset = getFormationOffset(currentFormation, nid);
        const double targetDx = nOffset.first - myDx;
        const double targetDy = nOffset.second - myDy;
        const double relXErr = (ns.msg.x - myX) - targetDx;
        const double relYErr = (ns.msg.y - myY) - targetDy;
        const double err = hypot(relXErr, relYErr);
        maxError = std::max(maxError, err);
    }

    return maxError;
}

bool MultiAgentConsensus::isMovementBlocked(double commandedVel, double& turnCmd)
{
    turnCmd = 0.0;
    if (fabs(commandedVel) < 1.0) return false;

    auto sanitizeRange = [](double r) -> double {
        return (r > 0.0) ? r : 5000.0;
    };

    const bool forwardMotion = (commandedVel > 0.0);
    const double frontPath = sanitizeRange(myRobot->getClosestSonarRange(-60.0, 60.0));
    const double rearLeft = sanitizeRange(myRobot->getClosestSonarRange(120.0, 179.0));
    const double rearRight = sanitizeRange(myRobot->getClosestSonarRange(-179.0, -120.0));
    const double rearPath = std::min(rearLeft, rearRight);
    const double pathDistance = forwardMotion ? frontPath : rearPath;

    if (pathDistance >= myParams.safeDistance) return false;

    double leftClear = 0.0;
    double rightClear = 0.0;
    if (forwardMotion) {
        leftClear = sanitizeRange(myRobot->getClosestSonarRange(10.0, 100.0));
        rightClear = sanitizeRange(myRobot->getClosestSonarRange(-100.0, -10.0));
    } else {
        leftClear = sanitizeRange(myRobot->getClosestSonarRange(80.0, 170.0));
        rightClear = sanitizeRange(myRobot->getClosestSonarRange(-170.0, -80.0));
    }

    turnCmd = (leftClear >= rightClear) ? myParams.avoidTurnVel : -myParams.avoidTurnVel;
    if (pathDistance < myParams.criticalDistance) {
        turnCmd *= 1.5;
        ArLog::log(ArLog::Terse, "Robot %d obstacle %.1fmm on %s path: hard block",
                   myId, pathDistance, forwardMotion ? "forward" : "reverse");
    }
    return true;
}

std::pair<double, double> MultiAgentConsensus::computeObstacleRepulsion()
{
    double repX = 0.0, repY = 0.0;
    
    double minFrontDist = 5000.0;
    double obstacleAngle = 0.0;
    
    // Very simple weighted average of front sonar for repulsion (like original code)
    double sumAngle = 0.0;
    double sumWeight = 0.0;

    for (int i = 0; i < myRobot->getNumSonar(); i++) {
        ArSensorReading* reading = myRobot->getSonarReading(i);
        if (reading != NULL) {
            double dist = reading->getRange();
            if (dist < myParams.safeDistance) {
                // Determine sensor angle
                double sAngle = (i * 22.5) - 180.0; // Appx mapping
                // We only care about front semi-circle for forward driving obstacle
                if (sAngle >= -90.0 && sAngle <= 90.0) {
                    minFrontDist = std::min(minFrontDist, dist);
                    double weight = 1.0 / (dist + 1.0);
                    sumAngle += sAngle * weight;
                    sumWeight += weight;
                }
            }
        }
    }

    if (sumWeight > 0.0 && minFrontDist < myParams.safeDistance) {
        obstacleAngle = sumAngle / sumWeight;
        
        // Compute repel force based on equation in specs
        double repelForce = myParams.gamma * (1.0 / minFrontDist - 1.0 / myParams.safeDistance) * 1000000.0; // scale up appropriately internally since mm
        
        // Push in opposite direction. Current heading is 0 relative because sonar angle is relative
        double repelAngleAbs = myRobot->getTh() + obstacleAngle + 180.0;
        double radAbs = repelAngleAbs * M_PI / 180.0;
        
        repX = repelForce * cos(radAbs);
        repY = repelForce * sin(radAbs);
        
        // Emergency stop if too close
        if (minFrontDist < myParams.criticalDistance) {
             ArLog::log(ArLog::Terse, "MultiAgentConsensus: Emergency obstacle distance %.1f", minFrontDist);
             // We can return a massive repel vector here to force turning away
             repX *= 10.0;
             repY *= 10.0;
        }
    }
    
    return {repX, repY};
}

std::pair<double, double> MultiAgentConsensus::getFormationOffset(int formationId, int targetRobotId)
{
    // Return relative offset expected for targetRobotId
    double dx = 0.0, dy = 0.0;
    
    if (formationId == TRIANGLE) {
        if (targetRobotId == 1) { dx = 0.0; dy = 0.0; }
        else if (targetRobotId == 2) { dx = -1000.0; dy = -1000.0; } // back and right
        else if (targetRobotId == 3) { dx = -1000.0; dy = 1000.0; }  // back and left
        else { dx = -1500.0 * (targetRobotId-1); dy = 0; } // Fallback tail
    } 
    else if (formationId == CIRCLE) {
        // Assume up to N robots spreading around a 1.5m radius circle
        double numBots = std::max(3, (int)myNeighbors.size() + 1);
        double angle = (2.0 * M_PI / numBots) * (targetRobotId - 1);
        dx = 1500.0 * cos(angle);
        dy = 1500.0 * sin(angle);
    } 
    else if (formationId == LINE_UP) {
        // Line along the Y axis
        dx = 0;
        dy = -1000.0 * (targetRobotId - 1);
    }
    
    return {dx, dy};
}

std::pair<double, double> MultiAgentConsensus::computeConsensusVector()
{
    double Ux = 0.0;
    double Uy = 0.0;
    
    double myX = myRobot->getX();
    double myY = myRobot->getY();
    double myThRad = myRobot->getTh() * M_PI / 180.0;
    double myV = myRobot->getVel();
    double myVx = myV * cos(myThRad);
    double myVy = myV * sin(myThRad);
    
    auto myOffset = getFormationOffset(currentFormation, myId);
    double myDx = myOffset.first;
    double myDy = myOffset.second;

    for (const auto& pair : myNeighbors) {
        int nid = pair.first;
        const NeighborState& ns = pair.second;
        
        double nx = ns.msg.x;
        double ny = ns.msg.y;
        double nThRad = ns.msg.theta * M_PI / 180.0;
        double nvx = ns.msg.v * cos(nThRad);
        double nvy = ns.msg.v * sin(nThRad);
        
        auto nOffset = getFormationOffset(currentFormation, nid);
        double nDx = nOffset.first;
        double nDy = nOffset.second;
        
        // Relative offset ∆_ij = ∆_j - ∆_i mapped roughly
        // The algorithm specifies (nx - my_x - DELTA_X) -> target offset from j to i
        double targetDx = nDx - myDx;
        double targetDy = nDy - myDy;
        
        // Position / Formation
        Ux += myParams.alpha * (nx - myX - targetDx);
        Uy += myParams.alpha * (ny - myY - targetDy);
        
        // Velocity Alignment
        Ux += myParams.beta * (nvx - myVx);
        Uy += myParams.beta * (nvy - myVy);
    }
    
    return {Ux, Uy};
}

void MultiAgentConsensus::consensusTask()
{
    broadcastState();
    receiveNeighborStates();
    
    if (!movementEnabled) {
        myRobot->setVel(0);
        myRobot->setRotVel(0);
        return;
    }
    
    // If no neighbors yet, drive forward slowly
    if (myNeighbors.empty()) {
        myRobot->setVel(groupSpeed > 0 ? groupSpeed : 200);
        myRobot->setRotVel(0);
        return;
    }

    std::pair<double, double> consU = computeConsensusVector();
    std::pair<double, double> repU = computeObstacleRepulsion();
    
    // Add group velocity bias: moves entire swarm forward in Robot 1's heading
    // Each robot adds a forward component in the average group heading
    double groupHeadingRad = myRobot->getTh() * M_PI / 180.0;
    double groupUx = groupSpeed * cos(groupHeadingRad);
    double groupUy = groupSpeed * sin(groupHeadingRad);
    
    // Blend: consensus (formation keeping) + obstacle avoidance + group movement
    double Ux = consU.first + repU.first + groupUx;
    double Uy = consU.second + repU.second + groupUy;
    
    // Nonholonomic Mapping
    double thetaDesRaw = atan2(Uy, Ux) * 180.0 / M_PI;
    double myTheta = myRobot->getTh();
    
    double eTheta = normalizeAngle(thetaDesRaw - myTheta);
    
    double magnitude = sqrt(Ux*Ux + Uy*Uy);
    // scale magnitude down to mm/s range roughly. Ux, Uy will be in mm territory from positions.
    // e.g. position diff of 1000mm * alpha(0.6) = 600.
    
    double eThetaRad = eTheta * M_PI / 180.0;
    
    double v = myParams.kV * magnitude * std::max(0.0, cos(eThetaRad)); // force forward only
    double w = myParams.kW * eTheta; // degrees / s
    
    // Apply speed limits
    v = std::min(std::max(v, 0.0), myParams.maxLinearVel);
    w = std::min(std::max(w, -myParams.maxAngularVel), myParams.maxAngularVel);
    
    myRobot->setVel(v);
    myRobot->setRotVel(w);

    static int log_counter = 0;
    if (++log_counter % 20 == 0) {
        ArLog::log(ArLog::Normal, "Consensus: Neighbors=%lu, U=(%.1f, %.1f), v=%.1f, w=%.1f, eTheta=%.1f", 
            myNeighbors.size(), Ux, Uy, v, w, eTheta);
    }
}

// Network setup / discovery
bool MultiAgentConsensus::discoverZLAN7104() {
    // Map each robot ID to its specific IP and port
    struct RobotConfig { std::string ip; int port; };
    std::map<int, RobotConfig> robot_configs = {
        {1, {"192.168.1.2", 8101}},
        {2, {"192.168.1.3", 8102}},
        {3, {"192.168.1.4", 8103}}
    };

    // Try the config assigned to this robot first
    auto it = robot_configs.find(myId);
    if (it != robot_configs.end()) {
        if (connectToZLAN7104(it->second.ip, it->second.port)) {
            myZLANIP = it->second.ip;
            myZLANPort = it->second.port;
            myWiFiConnected = true;
            ArLog::log(ArLog::Normal, "MultiAgentConsensus: Robot %d connected to %s:%d", myId, it->second.ip.c_str(), it->second.port);
            return true;
        }
    }

    // Fallback: try all configs if the assigned one fails
    for (const auto& pair : robot_configs) {
        if (connectToZLAN7104(pair.second.ip, pair.second.port)) {
            myZLANIP = pair.second.ip;
            myZLANPort = pair.second.port;
            myWiFiConnected = true;
            ArLog::log(ArLog::Normal, "MultiAgentConsensus: Robot %d fallback connected to %s:%d", myId, pair.second.ip.c_str(), pair.second.port);
            return true;
        }
    }
    return false;
}

bool MultiAgentConsensus::connectToZLAN7104(const std::string& ip_address, int port) {
    ArSocket testSocket;
    if (testSocket.connect(ip_address.c_str(), port, ArSocket::TCP, NULL)) {
        testSocket.close();
        return true;
    }
    return false;
}

void MultiAgentConsensus::scanForRobots() {
    if (discoverZLAN7104()) return;
    printf("\nPlease enter the ZLAN 7104 IP address: ");
    char ip_input[50];
    if (fgets(ip_input, sizeof(ip_input), stdin)) {
        ip_input[strcspn(ip_input, "\n")] = 0;
        if (connectToZLAN7104(ip_input, myZLANPort)) {
            myZLANIP = ip_input;
            myWiFiConnected = true;
        }
    }
}

std::string MultiAgentConsensus::getZLANIP() {
    return myZLANIP;
}

int main(int argc, char** argv)
{
    Aria::init();
    
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    
    bool use_wifi = parser.checkArgument("-wifi");
    
    // Single robot mode (original behavior with -id)
    int robotId = 1;
    bool has_id = parser.checkArgument("-id");
    if (has_id) {
        parser.checkParameterArgumentInteger("-id", &robotId);
    }
    
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    
    if (use_wifi && !has_id) {
        // ============================================================
        // MULTI-ROBOT MODE: Connect to all 3 robots simultaneously
        // WiFi Gateway: 192.168.1.1:4196
        // Robot 1 -> 192.168.1.2:8101
        // Robot 2 -> 192.168.1.3:8102
        // Robot 3 -> 192.168.1.4:8103
        // ============================================================
        
        struct RobotEntry {
            int id;
            std::string ip;
            int port;
            ArRobot* robot;
            ArSonarDevice* sonar;
            MultiAgentConsensus* navigator;
        };
        
        std::vector<RobotEntry> robots = {
            {1, "192.168.1.2", 8101, nullptr, nullptr, nullptr},
            {2, "192.168.1.3", 8102, nullptr, nullptr, nullptr},
            {3, "192.168.1.4", 8103, nullptr, nullptr, nullptr}
        };
        
        int connectedCount = 0;
        
        // Connect each robot, start async + navigator IMMEDIATELY to keep connection alive
        for (auto& entry : robots) {
            printf("\n--- Attempting Robot %d at %s:%d ---\n", entry.id, entry.ip.c_str(), entry.port);
            fflush(stdout);
            
            // Quick TCP probe: try connecting with 3s timeout to skip unreachable robots fast
            {
                int probe = socket(AF_INET, SOCK_STREAM, 0);
                struct timeval tv = {3, 0};  // 3 second timeout
                setsockopt(probe, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
                struct sockaddr_in addr;
                memset(&addr, 0, sizeof(addr));
                addr.sin_family = AF_INET;
                addr.sin_port = htons(entry.port);
                addr.sin_addr.s_addr = inet_addr(entry.ip.c_str());
                int result = connect(probe, (struct sockaddr*)&addr, sizeof(addr));
                ::close(probe);
                if (result != 0) {
                    printf("❌ Robot %d: Not reachable at %s:%d (skipping)\n", entry.id, entry.ip.c_str(), entry.port);
                    fflush(stdout);
                    continue;
                }
                printf("   TCP probe OK for Robot %d\n", entry.id);
                fflush(stdout);
            }
            
            entry.robot = new ArRobot();
            entry.sonar = new ArSonarDevice();
            entry.robot->addRangeDevice(entry.sonar);
            
            // Build persistent args for ArRobotConnector
            std::string portStr = std::to_string(entry.port);
            int* wargc = new int(5);
            char** wargv = new char*[5];
            wargv[0] = argv[0];
            wargv[1] = (char*)"-remoteHost";
            wargv[2] = strdup(entry.ip.c_str());
            wargv[3] = (char*)"-remoteRobotTcpPort";
            wargv[4] = strdup(portStr.c_str());
            
            ArArgumentParser* wp = new ArArgumentParser(wargc, wargv);
            wp->loadDefaultArguments();
            ArRobotConnector* wc = new ArRobotConnector(wp, entry.robot);
            
            printf("   ArRobotConnector connecting...\n");
            fflush(stdout);
            
            if (wc->connectRobot()) {
                printf("✅ Robot %d connected! (Name: %s)\n", entry.id, entry.robot->getRobotName());
                fflush(stdout);
                
                // Start async IMMEDIATELY to keep connection alive
                entry.robot->runAsync(true);
                printf("   Robot %d async started\n", entry.id);
                fflush(stdout);
                
                // Create navigator (per-robot UDP port)
                entry.navigator = new MultiAgentConsensus(entry.robot, entry.id);
                printf("   Robot %d navigator created\n", entry.id);
                fflush(stdout);
                
                // Start consensus task (motors enabled but movement waits for 'f' key)
                entry.navigator->run();
                printf("   Robot %d ready (waiting for 'f' to move)\n", entry.id);
                fflush(stdout);
                
                connectedCount++;
            } else {
                printf("❌ Robot %d at %s:%d failed to connect.\n", entry.id, entry.ip.c_str(), entry.port);
                fflush(stdout);
                delete entry.sonar;
                delete entry.robot;
                entry.robot = nullptr;
                entry.sonar = nullptr;
            }
        }
        
        if (connectedCount == 0) {
            printf("\n❌ No robots connected. Exiting.\n");
            Aria::exit(1);
        }
        
        printf("\n🎮 MULTI-ROBOT MODE: %d of 3 robots connected!\n", connectedCount);
        
        // Attach key handler to first robot, register for ALL navigators  
        bool keyHandlerAttached = false;
        for (auto& entry : robots) {
            if (entry.robot && entry.navigator) {
                if (!keyHandlerAttached) {
                    entry.robot->attachKeyHandler(&keyHandler);
                    keyHandlerAttached = true;
                }
                // Only register once per key (first navigator's globalNavigators loop handles all)
                if (!keyHandlerAttached) {
                    entry.navigator->addKeyHandlers(&keyHandler);
                }
            }
        }
        // Register key handlers from first navigator only (it broadcasts to all via globalNavigators)
        for (auto& entry : robots) {
            if (entry.navigator) {
                entry.navigator->addKeyHandlers(&keyHandler);
                break;  // Only register once — callbacks use globalNavigators
            }
        }
        
        printf("\n🎮 MULTI-AGENT CONSENSUS READY!\n");
        printf("Connected robots: %d/3\n", connectedCount);
        printf("Press SPACE to start moving, 'q' to quit\n");
        printf("Formations: 't'=Triangle, 'c'=Circle, 'l'=Line, 's'=Scatter\n");
        fflush(stdout);
        
        // Run until all robots disconnect
        bool anyConnected = true;
        while (anyConnected) {
            anyConnected = false;
            for (auto& entry : robots) {
                if (entry.robot && entry.robot->isConnected()) {
                    anyConnected = true;
                    break;
                }
            }
            ArUtil::sleep(100);
        }
        
        // Cleanup
        for (auto& entry : robots) {
            if (entry.navigator) delete entry.navigator;
            if (entry.sonar) delete entry.sonar;
            if (entry.robot) delete entry.robot;
        }
        
    } else {
        // ============================================================
        // SINGLE ROBOT MODE: Connect to one robot (original behavior)
        // Usage: ./multiAgentConsensus -id <N> [-wifi]
        // ============================================================
        
        ArRobot robot;
        robot.attachKeyHandler(&keyHandler);
        
        MultiAgentConsensus navigator(&robot, robotId);
        navigator.addKeyHandlers(&keyHandler);
        
        if (use_wifi) {
            navigator.scanForRobots();
            std::string wifi_ip = navigator.getZLANIP();
            
            if (!wifi_ip.empty()) {
                // Determine port based on robot ID
                int robot_port = 8101;
                if (robotId == 2) robot_port = 8102;
                else if (robotId == 3) robot_port = 8103;
                std::string portStr = std::to_string(robot_port);
                char* wifi_args[] = {
                    argv[0], (char*)"-remoteHost", (char*)wifi_ip.c_str(),
                    (char*)"-remoteRobotTcpPort", (char*)portStr.c_str()
                };
                int new_argc = 5;
                ArArgumentParser wifi_parser(&new_argc, wifi_args);
                wifi_parser.loadDefaultArguments();
                ArRobotConnector robotConnector(&wifi_parser, &robot);
                if (!robotConnector.connectRobot()) Aria::exit(1);
            } else {
                ArRobotConnector robotConnector(&parser, &robot);
                if (!robotConnector.connectRobot()) Aria::exit(1);
            }
        } else {
            ArRobotConnector robotConnector(&parser, &robot);
            if (!robotConnector.connectRobot()) Aria::exit(1);
        }
        
        ArSonarDevice sonarDev;
        robot.addRangeDevice(&sonarDev);
        
        robot.runAsync(true);
        ArUtil::sleep(2000);
        
        navigator.run();
        navigator.waitForStartCommand();
        
        while (robot.isConnected()) {
            ArUtil::sleep(100);
        }
    }
    
    Aria::exit(0);
    return 0;
}
