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
    bool systemReady;
    
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
        int udpPort = 50000;
    } myParams;
    
    // Input Functors
    ArFunctorC<MultiAgentConsensus> myTriangleCB;
    ArFunctorC<MultiAgentConsensus> myCircleCB;
    ArFunctorC<MultiAgentConsensus> myLineCB;
    ArFunctorC<MultiAgentConsensus> myScatterCB;
    ArFunctorC<MultiAgentConsensus> mySpaceCB;
    ArFunctorC<MultiAgentConsensus> myQuitCB;

    // Formations
    std::pair<double, double> getFormationOffset(int formationId, int robotId);

    // Internal tasks
    void consensusTask();
    
    // Utilities
    double normalizeAngle(double angle);
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
    systemReady(false),
    myZLANPort(5000),
    myWiFiConnected(false),
    myTriangleCB(this, &MultiAgentConsensus::keyTriangle),
    myCircleCB(this, &MultiAgentConsensus::keyCircle),
    myLineCB(this, &MultiAgentConsensus::keyLine),
    myScatterCB(this, &MultiAgentConsensus::keyScatter),
    mySpaceCB(this, &MultiAgentConsensus::keySpace),
    myQuitCB(this, &MultiAgentConsensus::keyQuit)
{
    setupUdpNetwork();
}

MultiAgentConsensus::~MultiAgentConsensus()
{
    stop();
    if (udpSocket >= 0) close(udpSocket);
}

bool MultiAgentConsensus::setupUdpNetwork()
{
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
    recvAddr.sin_port = htons(myParams.udpPort);
    recvAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(udpSocket, (struct sockaddr*)&recvAddr, sizeof(recvAddr)) < 0) {
        ArLog::log(ArLog::Terse, "MultiAgentConsensus: Could not bind UDP socket");
    }

    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(myParams.udpPort);
    broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255");

    ArLog::log(ArLog::Normal, "MultiAgentConsensus: UDP Network setup on port %d", myParams.udpPort);
    return true;
}

void MultiAgentConsensus::run()
{
    myRobot->addUserTask("MultiAgentConsensus", 50, &myTaskCB);
    myRobot->enableMotors();
    systemReady = true;
    ArLog::log(ArLog::Normal, "MultiAgentConsensus: System ready");
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
}

void MultiAgentConsensus::keyTriangle() { currentFormation = TRIANGLE; ArLog::log(ArLog::Normal, "Formation: TRIANGLE"); }
void MultiAgentConsensus::keyCircle()   { currentFormation = CIRCLE;   ArLog::log(ArLog::Normal, "Formation: CIRCLE"); }
void MultiAgentConsensus::keyLine()     { currentFormation = LINE_UP;  ArLog::log(ArLog::Normal, "Formation: LINE UP"); }
void MultiAgentConsensus::keyScatter()  { currentFormation = SCATTER;  ArLog::log(ArLog::Normal, "Formation: SCATTER"); }
void MultiAgentConsensus::keySpace()    { movementEnabled = true;      ArLog::log(ArLog::Normal, "Movement ENABLED via Spacebar"); }
void MultiAgentConsensus::keyQuit()     { ArLog::log(ArLog::Normal, "Quit requested"); myRobot->stopRunning(); Aria::exit(0); }

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

    // Populate 8 front sonars (assuming sensors 0-7 are front-facing or we just pull the first 8)
    for (int i = 0; i < 8; i++) {
        ArSensorReading* reading = myRobot->getSonarReading(i);
        msg.sonar[i] = reading ? reading->getRange() : 5000.0;
    }

    sendto(udpSocket, &msg, sizeof(msg), 0, (struct sockaddr*)&broadcastAddr, sizeof(broadcastAddr));
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
            
            // Sync formation if a neighbor changes it (naive max-wins to propagate new formation simply for demo, or match if different)
            // A clearer way: just adopt whatever formation the neighbor is broadcasting if they differ to auto-sync.
            if (msg.formation_type != SCATTER && currentFormation != msg.formation_type) {
                 ArLog::log(ArLog::Normal, "MultiAgentConsensus: Synced formation to %d from robot %d", msg.formation_type, msg.robot_id);
                 currentFormation = msg.formation_type;
            }
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
    
    if (!movementEnabled) return;
    
    // Safety check - if no neighbors, just stop or wander slightly
    if (myNeighbors.empty()) {
        myRobot->setVel(0);
        myRobot->setRotVel(0);
        return;
    }

    std::pair<double, double> consU = computeConsensusVector();
    std::pair<double, double> repU = computeObstacleRepulsion();
    
    // Blend them
    double Ux = consU.first + repU.first;
    double Uy = consU.second + repU.second;
    
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
    std::vector<std::string> common_ips = {"192.168.1.152", "192.168.1.100"};
    for (const auto& ip : common_ips) {
        if (connectToZLAN7104(ip, myZLANPort)) {
            myZLANIP = ip;
            myWiFiConnected = true;
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
    
    int robotId = 1;
    if (parser.checkArgument("-id")) {
        parser.checkParameterArgumentInteger("-id", &robotId);
    }
    
    ArRobot robot;
    
    ArKeyHandler keyHandler;
    Aria::setKeyHandler(&keyHandler);
    robot.attachKeyHandler(&keyHandler);
    
    MultiAgentConsensus navigator(&robot, robotId);
    navigator.addKeyHandlers(&keyHandler);
    
    bool use_wifi = parser.checkArgument("-wifi");
    std::string wifi_ip = "";
    
    if (use_wifi) {
        navigator.scanForRobots();
        wifi_ip = navigator.getZLANIP();
    }
    
    if (use_wifi && !wifi_ip.empty()) {
        argc = 1;
        char* wifi_args[] = {
            argv[0], (char*)"-remoteHost", (char*)wifi_ip.c_str(),
            (char*)"-remoteRobotTcpPort", (char*)"5000"
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
    
    ArSonarDevice sonarDev;
    robot.addRangeDevice(&sonarDev);
    
    robot.runAsync(true);
    ArUtil::sleep(2000);
    
    navigator.run();
    navigator.waitForStartCommand();
    
    while (robot.isConnected()) {
        ArUtil::sleep(100);
    }
    
    Aria::exit(0);
    return 0;
}
