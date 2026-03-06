/*
Synchronized multi-robot arrow-key teleop for Pioneer 3DX.

Controls:
- Up Arrow / w:    all robots forward
- Down Arrow / s:  all robots backward
- Left Arrow / a:  all robots rotate left
- Right Arrow / d: all robots rotate right
- Space / b:       stop all robots
- q:               quit
*/

#include "Aria.h"
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <iostream>

// Global recording state
static bool gIsRecording = false;
static int gNextRecordSlot = 1;
static std::string gSlotNames[3] = {"record_1", "record_2", "record_3"};
static std::string gStatusMsg = "Ready";
static volatile bool gPerRobotBlocked[4] = {false, false, false, false}; // per-robot sonar flags (index = robot ID)
static int gBlockingRobotId = 0;
static double gBlockingDistance = 0.0;
#include <algorithm>
#include <cstring>
#include <signal.h>

// Networking for TCP probe
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

class MultiAgentArrowControl;
static std::vector<MultiAgentArrowControl*> gControllers;
static volatile bool gShutdownRequested = false;
static volatile bool gAnyDisconnected = false;
static volatile int gDisconnectedRobotId = 0;

// UDP telemetry broadcast for 2D map visualizer
static int gUdpSocket = -1;
static struct sockaddr_in gVisualizerAddr;

// UDP command listener (receive key presses from Python visualizer)
static int gCmdSocket = -1;

struct RobotStateMsg {
  int robot_id;
  int boids_active;       // 1 = boids exploring, 0 = manual
  double x, y, theta;
  double v, w;
  double battery;         // battery voltage (normalized ~12V)
  double sonar[8];
};

static volatile bool gBoidsMode = false;

class MultiAgentArrowControl
{
  friend void processRemoteCommands();
public:
  MultiAgentArrowControl(ArRobot* robot, int robotId);
  ~MultiAgentArrowControl();

  void run();
  void stop();
  void addKeyHandlers(ArKeyHandler* keyHandler);

private:
  ArRobot* myRobot;
  int myId;

  bool myCommandEnabled;
  double myTargetVel;
  double myTargetRotVel;
  double myLastVel;
  double myLastRotVel;

  struct Parameters
  {
    double slowLinearVel = 60.0;   // mm/s
    double slowRotVel = 18.0;      // deg/s
    double maxLinearVel = 100.0;   // mm/s
    double maxRotVel = 25.0;       // deg/s
    double maxVelStep = 10.0;      // mm/s per cycle
    double maxRotStep = 3.0;       // deg/s per cycle
    double safeDistance = 250.0;   // mm — coordinated stop distance (front)
    double rearSafeDistance = 200.0; // mm — coordinated stop distance (rear)
    double criticalDistance = 300.0; // mm (unused but kept for reference)
    double avoidTurnVel = 16.0;    // deg/s
  } myParams;

  ArFunctorC<MultiAgentArrowControl> myTaskCB;
  ArFunctorC<MultiAgentArrowControl> myUpCB;
  ArFunctorC<MultiAgentArrowControl> myDownCB;
  ArFunctorC<MultiAgentArrowControl> myLeftCB;
  ArFunctorC<MultiAgentArrowControl> myRightCB;
  ArFunctorC<MultiAgentArrowControl> myStopCB;
  ArFunctorC<MultiAgentArrowControl> myQuitCB;

  ArFunctorC<MultiAgentArrowControl> myIndUpCB;
  ArFunctorC<MultiAgentArrowControl> myIndDownCB;
  ArFunctorC<MultiAgentArrowControl> myIndLeftCB;
  ArFunctorC<MultiAgentArrowControl> myIndRightCB;

  ArFunctorC<MultiAgentArrowControl> myInvUpCB;
  ArFunctorC<MultiAgentArrowControl> myInvDownCB;
  ArFunctorC<MultiAgentArrowControl> myInvLeftCB;
  ArFunctorC<MultiAgentArrowControl> myInvRightCB;

  ArFunctorC<MultiAgentArrowControl> myRecordCB;
  ArFunctorC<MultiAgentArrowControl> myPlay1CB;
  ArFunctorC<MultiAgentArrowControl> myPlay2CB;
  ArFunctorC<MultiAgentArrowControl> myPlay3CB;
  ArFunctorC<MultiAgentArrowControl> mySaveCB;
  ArFunctorC<MultiAgentArrowControl> myBoidsCB;

  void keyRecord();
  void keyPlay(int slot);
  void keyPlay1();
  void keyPlay2();
  void keyPlay3();
  void keySave();
  void keyBoids();

  struct RecordState {
    std::vector<std::pair<double, double>> history;
  };
  std::map<int, RecordState> myRecords;
  int myPlayingSlot;
  size_t myPlaybackIndex;

  void saveRecording(int slot, const std::string& name);
  void loadRecording(int slot, const std::string& name);

  void controlTask();

  void keyUp();
  void keyDown();
  void keyLeft();
  void keyRight();
  void keyStop();
  void keyQuit();

  // Individual Robot 1 controls (Arrow Keys)
  void keyIndUp();
  void keyIndDown();
  void keyIndLeft();
  void keyIndRight();

  // Inverted Robot 2 & 3 controls (TFGH)
  void keyInvUp();
  void keyInvDown();
  void keyInvLeft();
  void keyInvRight();

  void setAllCommands(double v, double w, bool enabled, const char* label);
  void setIndividualCommand(int targetId, double v, double w, bool enabled, const char* label);
  double clampDelta(double target, double current, double maxStep);
  bool applySonarSafety(double& targetV, double& targetW);

public:
  static void printUI();
};

MultiAgentArrowControl::MultiAgentArrowControl(ArRobot* robot, int robotId) :
  myRobot(robot),
  myId(robotId),
  myCommandEnabled(false),
  myTargetVel(0.0),
  myTargetRotVel(0.0),
  myLastVel(0.0),
  myLastRotVel(0.0),
  myTaskCB(this, &MultiAgentArrowControl::controlTask),
  myUpCB(this, &MultiAgentArrowControl::keyUp),
  myDownCB(this, &MultiAgentArrowControl::keyDown),
  myLeftCB(this, &MultiAgentArrowControl::keyLeft),
  myRightCB(this, &MultiAgentArrowControl::keyRight),
  myStopCB(this, &MultiAgentArrowControl::keyStop),
  myQuitCB(this, &MultiAgentArrowControl::keyQuit),
  myIndUpCB(this, &MultiAgentArrowControl::keyIndUp),
  myIndDownCB(this, &MultiAgentArrowControl::keyIndDown),
  myIndLeftCB(this, &MultiAgentArrowControl::keyIndLeft),
  myIndRightCB(this, &MultiAgentArrowControl::keyIndRight),
  myInvUpCB(this, &MultiAgentArrowControl::keyInvUp),
  myInvDownCB(this, &MultiAgentArrowControl::keyInvDown),
  myInvLeftCB(this, &MultiAgentArrowControl::keyInvLeft),
  myInvRightCB(this, &MultiAgentArrowControl::keyInvRight),
  myRecordCB(this, &MultiAgentArrowControl::keyRecord),
  myPlay1CB(this, &MultiAgentArrowControl::keyPlay1),
  myPlay2CB(this, &MultiAgentArrowControl::keyPlay2),
  myPlay3CB(this, &MultiAgentArrowControl::keyPlay3),
  mySaveCB(this, &MultiAgentArrowControl::keySave),
  myBoidsCB(this, &MultiAgentArrowControl::keyBoids),
  myPlayingSlot(-1),
  myPlaybackIndex(0)
{
  gControllers.push_back(this);
  loadRecording(1, gSlotNames[0]);
  loadRecording(2, gSlotNames[1]);
  loadRecording(3, gSlotNames[2]);
}

MultiAgentArrowControl::~MultiAgentArrowControl()
{
  for (std::vector<MultiAgentArrowControl*>::iterator it = gControllers.begin();
       it != gControllers.end(); ++it)
  {
    if (*it == this)
    {
      gControllers.erase(it);
      break;
    }
  }
  stop();
}

void MultiAgentArrowControl::run()
{
  myRobot->addUserTask("MultiAgentArrowControl", 100, &myTaskCB);
  myRobot->lock();
  myRobot->enableMotors();
  myRobot->enableSonar();
  myRobot->comInt(ArCommands::SONAR, 1); // Force-enable sonar via direct command
  myRobot->unlock();
  ArLog::log(ArLog::Normal, "ArrowControl: Robot %d ready (motors + sonar enabled, %d sonar sensors)", 
             myId, myRobot->getNumSonar());
}

void MultiAgentArrowControl::stop()
{
  myRobot->remUserTask(&myTaskCB);
  myRobot->setVel(0);
  myRobot->setRotVel(0);
  myLastVel = 0.0;
  myLastRotVel = 0.0;
}

void MultiAgentArrowControl::addKeyHandlers(ArKeyHandler* keyHandler)
{
  if (!keyHandler) return;

  keyHandler->addKeyHandler(ArKeyHandler::UP, &myIndUpCB);
  keyHandler->addKeyHandler(ArKeyHandler::DOWN, &myIndDownCB);
  keyHandler->addKeyHandler(ArKeyHandler::LEFT, &myIndLeftCB);
  keyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myIndRightCB);

  keyHandler->addKeyHandler('t', &myInvUpCB);
  keyHandler->addKeyHandler('g', &myInvDownCB);
  keyHandler->addKeyHandler('f', &myInvLeftCB);
  keyHandler->addKeyHandler('h', &myInvRightCB);

  keyHandler->addKeyHandler(ArKeyHandler::SPACE, &myStopCB);

  keyHandler->addKeyHandler('w', &myUpCB);
  keyHandler->addKeyHandler('s', &myDownCB);
  keyHandler->addKeyHandler('a', &myLeftCB);
  keyHandler->addKeyHandler('d', &myRightCB);
  keyHandler->addKeyHandler('b', &myStopCB);
  keyHandler->addKeyHandler('x', &myStopCB); // Added 'x' as emergency stop
  keyHandler->addKeyHandler('q', &myQuitCB);
  keyHandler->addKeyHandler('r', &myRecordCB);
  keyHandler->addKeyHandler('1', &myPlay1CB);
  keyHandler->addKeyHandler('2', &myPlay2CB);
  keyHandler->addKeyHandler('3', &myPlay3CB);
  keyHandler->addKeyHandler('n', &mySaveCB); // 'n' for Name/Save
  keyHandler->addKeyHandler('e', &myBoidsCB); // 'e' for Explore (boids)
}

void MultiAgentArrowControl::keyUp()
{
  setAllCommands(myParams.slowLinearVel, 0.0, true, "UP: forward");
}

void MultiAgentArrowControl::keyDown()
{
  setAllCommands(-myParams.slowLinearVel, 0.0, true, "DOWN: backward");
}

void MultiAgentArrowControl::keyLeft()
{
  setAllCommands(0.0, myParams.slowRotVel, true, "LEFT: rotate left");
}

void MultiAgentArrowControl::keyRight()
{
  setAllCommands(0.0, -myParams.slowRotVel, true, "RIGHT: rotate right");
}

void MultiAgentArrowControl::keyStop()
{
  if (gIsRecording) {
    gIsRecording = false;
    gStatusMsg = "RECORDING ABORTED.";
  }
  setAllCommands(0.0, 0.0, false, "STOP ALL");
  for (size_t i = 0; i < gControllers.size(); ++i) {
      gControllers[i]->myLastVel = 0;
      gControllers[i]->myLastRotVel = 0;
      gControllers[i]->myPlayingSlot = -1; // stop playback
  }
  printUI();
}

void MultiAgentArrowControl::keyQuit()
{
  ArLog::log(ArLog::Normal, "ArrowControl: quit requested");
  gShutdownRequested = true;
}

void MultiAgentArrowControl::setAllCommands(double v, double w, bool enabled, const char* label)
{
  for (size_t i = 0; i < gControllers.size(); ++i)
  {
    MultiAgentArrowControl* c = gControllers[i];
    c->myTargetVel = v;
    c->myTargetRotVel = w;
    c->myCommandEnabled = enabled;
  }
  ArLog::log(ArLog::Normal, "ArrowControl (ALL): %s  (v=%.1f, w=%.1f)", label, v, w);
}

void MultiAgentArrowControl::setIndividualCommand(int targetId, double v, double w, bool enabled, const char* label)
{
  for (size_t i = 0; i < gControllers.size(); ++i)
  {
    MultiAgentArrowControl* c = gControllers[i];
    if (c->myId == targetId)
    {
      c->myTargetVel = v;
      c->myTargetRotVel = w;
      c->myCommandEnabled = enabled;
      ArLog::log(ArLog::Normal, "ArrowControl (Robot %d): %s (v=%.1f, w=%.1f)", targetId, label, v, w);
      return;
    }
  }
}

void MultiAgentArrowControl::keyIndUp()    { setIndividualCommand(1, myParams.slowLinearVel, 0.0, true, "UP"); }
void MultiAgentArrowControl::keyIndDown()  { setIndividualCommand(1, -myParams.slowLinearVel, 0.0, true, "DOWN"); }
void MultiAgentArrowControl::keyIndLeft()  { setIndividualCommand(1, 0.0, myParams.slowRotVel, true, "LEFT"); }
void MultiAgentArrowControl::keyIndRight() { setIndividualCommand(1, 0.0, -myParams.slowRotVel, true, "RIGHT"); }

void MultiAgentArrowControl::keyInvUp()
{
  setIndividualCommand(2, myParams.slowLinearVel, 0.0, true, "TFGH_UP_R2");
  setIndividualCommand(3, -myParams.slowLinearVel, 0.0, true, "TFGH_UP_R3");
}
void MultiAgentArrowControl::keyInvDown()
{
  setIndividualCommand(2, -myParams.slowLinearVel, 0.0, true, "TFGH_DOWN_R2");
  setIndividualCommand(3, myParams.slowLinearVel, 0.0, true, "TFGH_DOWN_R3");
}
void MultiAgentArrowControl::keyInvLeft()
{
  setIndividualCommand(2, 0.0, myParams.slowRotVel, true, "TFGH_LEFT_R2");
  setIndividualCommand(3, 0.0, -myParams.slowRotVel, true, "TFGH_LEFT_R3");
}
void MultiAgentArrowControl::keyInvRight()
{
  setIndividualCommand(2, 0.0, -myParams.slowRotVel, true, "TFGH_RIGHT_R2");
  setIndividualCommand(3, 0.0, myParams.slowRotVel, true, "TFGH_RIGHT_R3");
}

void MultiAgentArrowControl::keyRecord()
{
  if (gIsRecording) {
    gIsRecording = false;
    gStatusMsg = "Saved loop to Slot " + std::to_string(gNextRecordSlot) + " (" + gSlotNames[gNextRecordSlot-1] + ")";
    
    for (size_t i = 0; i < gControllers.size(); ++i) {
      gControllers[i]->saveRecording(gNextRecordSlot, gSlotNames[gNextRecordSlot-1]);
    }

    gNextRecordSlot++;
    if (gNextRecordSlot > 3) gNextRecordSlot = 1;
  } else {
    for (size_t i = 0; i < gControllers.size(); ++i) {
      gControllers[i]->myRecords[gNextRecordSlot].history.clear();
      gControllers[i]->myPlayingSlot = -1; // stop playing
    }
    gIsRecording = true;
    gStatusMsg = "RECORDING to Slot " + std::to_string(gNextRecordSlot) + "...";
  }
  printUI();
}

void MultiAgentArrowControl::keyPlay(int slot)
{
  if (gIsRecording) {
    gStatusMsg = "Cannot play while recording. Press 'r' to stop first.";
    printUI();
    return;
  }
  gStatusMsg = "PLAYING Slot " + std::to_string(slot) + " (" + gSlotNames[slot-1] + ")";
  for (size_t i = 0; i < gControllers.size(); ++i) {
    gControllers[i]->myPlayingSlot = slot;
    gControllers[i]->myPlaybackIndex = 0;
  }
  printUI();
}
void MultiAgentArrowControl::keyPlay1() { keyPlay(1); }
void MultiAgentArrowControl::keyPlay2() { keyPlay(2); }
void MultiAgentArrowControl::keyPlay3() { keyPlay(3); }

void MultiAgentArrowControl::keySave()
{
  if (gIsRecording) {
      gStatusMsg = "Cannot rename while actively recording!";
      printUI();
      return;
  }

  // Clear previous lines to make console readable for input
  printf("\033[2J\033[H");
  std::cout << "--- Rename Saved Recording ---\n";
  std::cout << "Which slot to rename and save? (1, 2, or 3) [Enter 0 to cancel]: ";
  int slot = 0;
  std::cin >> slot;
  if(slot >= 1 && slot <= 3) {
      std::cout << "Enter new file name: ";
      std::string newName;
      std::cin >> newName;
      gSlotNames[slot-1] = newName;
      
      for (size_t i = 0; i < gControllers.size(); ++i) {
        gControllers[i]->saveRecording(slot, newName);
      }
      gStatusMsg = "Renamed Slot " + std::to_string(slot) + " to '" + newName + "'";
  } else {
      gStatusMsg = "Rename cancelled.";
  }
  printUI();
}

void MultiAgentArrowControl::keyBoids()
{
  gBoidsMode = !gBoidsMode;
  if (gBoidsMode) {
    gStatusMsg = "BOIDS EXPLORE MODE \u2014 Robots self-driving!";
    for (size_t i = 0; i < gControllers.size(); ++i) {
      gControllers[i]->myCommandEnabled = false;
      gControllers[i]->myPlayingSlot = -1;
    }
  } else {
    gStatusMsg = "Manual control resumed.";
    for (size_t i = 0; i < gControllers.size(); ++i) {
      gControllers[i]->myTargetVel = 0;
      gControllers[i]->myTargetRotVel = 0;
      gControllers[i]->myCommandEnabled = false;
      gControllers[i]->myLastVel = 0;
      gControllers[i]->myLastRotVel = 0;
    }
  }
  printUI();
}

void MultiAgentArrowControl::saveRecording(int slot, const std::string& name)
{
  std::string filename = "Robot" + std::to_string(myId) + "_" + name + ".txt";
  std::ofstream out(filename.c_str());
  if (out.is_open()) {
    for (size_t i = 0; i < myRecords[slot].history.size(); ++i) {
       out << myRecords[slot].history[i].first << " " << myRecords[slot].history[i].second << "\n";
    }
    out.close();
  }
}

void MultiAgentArrowControl::loadRecording(int slot, const std::string& name)
{
  std::string filename = "Robot" + std::to_string(myId) + "_" + name + ".txt";
  std::ifstream in(filename.c_str());
  if (in.is_open()) {
    myRecords[slot].history.clear();
    double v, w;
    while (in >> v >> w) {
      myRecords[slot].history.push_back(std::make_pair(v, w));
    }
    in.close();
  }
}

void MultiAgentArrowControl::printUI()
{
    // Clear screen and position cursor to top
    printf("\033[2J\033[H");
    printf("===================================================\n");
    printf("       SYNCHRONIZED SWARM ROBOT CONTROLLER\n");
    printf("===================================================\n\n");
    
    printf("--- CONTROLS ---\n");
    printf("   [Arrow Keys] : Move Robot 1 Individually\n");
    printf("   [ T F G H ]  : Move Robot 2 and 3 in OPPOSITE directions\n");
    printf("   [ W A S D ]  : Move ALL robots together\n");
    printf("   [ SPACE/X ]  : STOP ALL robots unconditionally\n");
    printf("   [    Q    ]  : Quit Program\n");
    printf("   [    E    ]  : Toggle Boids Explore Mode\n\n");
    
    printf("--- RECORD & PLAYBACK ---\n");
    printf("   [    R    ]  : Start/Stop sequence recording\n");
    printf("   [ 1, 2, 3 ]  : Playback recorded sequences\n");
    printf("   [    N    ]  : Save & Rename a recording slot\n\n");
    
    printf("--- LOADED RECORDINGS ---\n");
    for(int i = 0; i < 3; i++) {
        size_t len = 0;
        if (!gControllers.empty()) len = gControllers[0]->myRecords[i+1].history.size();
        printf("   Slot %d: %s (%zu frames)\n", i+1, gSlotNames[i].c_str(), len);
    }
    
    printf("\n--- ROBOTS ---\n");
    for(size_t i = 0; i < gControllers.size(); i++) {
        bool conn = gControllers[i]->myRobot->isConnected();
        int numSonar = gControllers[i]->myRobot->getNumSonar();
        double frontR = gControllers[i]->myRobot->getClosestSonarRange(-60.0, 60.0);
        double rearL = gControllers[i]->myRobot->getClosestSonarRange(120.0, 179.0);
        double rearR = gControllers[i]->myRobot->getClosestSonarRange(-179.0, -120.0);
        if (frontR <= 0) frontR = 9999;
        double rearD = std::min(rearL > 0 ? rearL : 9999.0, rearR > 0 ? rearR : 9999.0);
        double batt = gControllers[i]->myRobot->getBatteryVoltage();
        const char* blocked = gPerRobotBlocked[gControllers[i]->myId] ? " [BLOCKED]" : "";
        const char* battWarn = (batt > 0 && batt < 11.5) ? " [LOW BATT!]" : "";
        printf("   Robot %d: %s  Sonar:%d  F:%.0fmm R:%.0fmm  Batt:%.1fV%s%s\n", 
               gControllers[i]->myId, conn ? "OK" : "DOWN", numSonar, frontR, rearD, batt, blocked, battWarn);
    }
    
    printf("\n===================================================\n");
    printf(">> STATUS: %s\n", gStatusMsg.c_str());
    if (gIsRecording) {
        printf(">> [ RECORDING ACTIVE ] >> \n");
    } else if (gPerRobotBlocked[1] || gPerRobotBlocked[2] || gPerRobotBlocked[3]) {
        printf(">> [ SONAR BLOCKED — Robot %d detected obstacle at %.0fmm ] >> \n", gBlockingRobotId, gBlockingDistance);
    } else if (gBoidsMode) {
        printf(">> [ BOIDS EXPLORE MODE \u2014 Self-driving ] >> \n");
    } else if (gAnyDisconnected) {
        printf(">> [ CONNECTION LOST — Robot %d disconnected, ALL STOPPED ] >> \n", gDisconnectedRobotId);
    } else {
        printf("\n");
    }
    printf("===================================================\n");
    fflush(stdout);
}

double MultiAgentArrowControl::clampDelta(double target, double current, double maxStep)
{
  const double delta = target - current;
  if (delta > maxStep) return current + maxStep;
  if (delta < -maxStep) return current - maxStep;
  return target;
}

bool MultiAgentArrowControl::applySonarSafety(double& targetV, double& targetW)
{
  auto sanitizeRange = [](double r) -> double {
    return (r > 0.0) ? r : 5000.0;
  };

  bool thisRobotBlocked = false;

  // Always scan front sonar
  const double front = sanitizeRange(myRobot->getClosestSonarRange(-60.0, 60.0));
  if (front < myParams.safeDistance && targetV > 1.0)
  {
    thisRobotBlocked = true;
    gBlockingRobotId = myId;
    gBlockingDistance = front;
  }

  // Always scan rear sonar
  const double rearLeft = sanitizeRange(myRobot->getClosestSonarRange(120.0, 179.0));
  const double rearRight = sanitizeRange(myRobot->getClosestSonarRange(-179.0, -120.0));
  const double rear = std::min(rearLeft, rearRight);
  if (rear < myParams.rearSafeDistance && targetV < -1.0)
  {
    thisRobotBlocked = true;
    gBlockingRobotId = myId;
    gBlockingDistance = rear;
  }

  // Update THIS robot's flag (no race condition since each robot writes its own slot)
  gPerRobotBlocked[myId] = thisRobotBlocked;

  // Check if ANY robot is blocked
  bool anyBlocked = false;
  for (int i = 1; i <= 3; i++) {
    if (gPerRobotBlocked[i]) {
      anyBlocked = true;
      break;
    }
  }

  if (anyBlocked)
  {
    targetV = 0.0;
    targetW = 0.0;
    return true;
  }

  return false;
}

void MultiAgentArrowControl::controlTask()
{
  // ALWAYS broadcast telemetry first (even when stopped/blocked)
  if (gUdpSocket >= 0) {
    RobotStateMsg msg;
    msg.robot_id = myId;
    msg.boids_active = gBoidsMode ? 1 : 0;
    msg.x = myRobot->getX();
    msg.y = myRobot->getY();
    msg.theta = myRobot->getTh();
    msg.v = myRobot->getVel();
    msg.w = myRobot->getRotVel();
    msg.battery = myRobot->getBatteryVoltage();
    for (int s = 0; s < 8; s++) {
      ArSensorReading* r = myRobot->getSonarReading(s);
      msg.sonar[s] = r ? r->getRange() : 5000.0;
    }
    sendto(gUdpSocket, &msg, sizeof(msg), 0,
           (struct sockaddr*)&gVisualizerAddr, sizeof(gVisualizerAddr));
  }

  // Safety gate: stop all robots if any robot lost connection
  if (gAnyDisconnected) {
    myRobot->setVel(0);
    myRobot->setRotVel(0);
    return;
  }

  double targetV = myCommandEnabled ? myTargetVel : 0.0;
  double targetW = myCommandEnabled ? myTargetRotVel : 0.0;

  // === BOIDS AUTONOMOUS EXPLORATION MODE ===
  if (gBoidsMode && !gAnyDisconnected) {
    double boidsV = 40.0;  // Base forward speed (mm/s)
    double boidsW = 0.0;   // Rotation (deg/s)

    // 1. Obstacle avoidance from sonar
    double frontMin = 5000.0;
    double leftMin = 5000.0;
    double rightMin = 5000.0;
    for (int s = 0; s < 8; s++) {
      ArSensorReading* r = myRobot->getSonarReading(s);
      double d = r ? r->getRange() : 5000.0;
      if (d <= 0) d = 5000.0;
      double ang = r ? r->getSensorTh() : 0;
      if (ang > -45 && ang < 45) frontMin = std::min(frontMin, d);
      else if (ang >= 45) leftMin = std::min(leftMin, d);
      else rightMin = std::min(rightMin, d);
    }

    if (frontMin < 400.0) {
      // Obstacle ahead — stop and turn away
      boidsV = 0.0;
      boidsW = (leftMin > rightMin) ? 15.0 : -15.0;
    } else if (frontMin < 800.0) {
      // Obstacle approaching — slow down and start turning
      boidsV = 25.0;
      boidsW = (leftMin > rightMin) ? 8.0 : -8.0;
    }

    // 2. Separation — steer away from nearby robots
    double myX = myRobot->getX();
    double myY = myRobot->getY();
    double myTh = myRobot->getTh();
    double sepW = 0.0;
    for (size_t i = 0; i < gControllers.size(); i++) {
      if (gControllers[i]->myId == myId) continue;
      if (!gControllers[i]->myRobot->isConnected()) continue;
      double ox = gControllers[i]->myRobot->getX();
      double oy = gControllers[i]->myRobot->getY();
      double dx = ox - myX;
      double dy = oy - myY;
      double dist = sqrt(dx * dx + dy * dy);
      if (dist < 600.0 && dist > 0) {
        // Angle to the other robot relative to our heading
        double angleToOther = atan2(dy, dx) * 180.0 / M_PI;
        double relAngle = angleToOther - myTh;
        // Normalize to -180..180
        while (relAngle > 180) relAngle -= 360;
        while (relAngle < -180) relAngle += 360;
        // Turn away from the other robot
        double strength = (600.0 - dist) / 600.0;
        sepW += (relAngle > 0 ? -10.0 : 10.0) * strength;
      }
    }
    boidsW += sepW;

    // 3. Random wander — slight random turns to explore
    static int wanderCounter = 0;
    wanderCounter++;
    if (wanderCounter % 50 == 0) { // Every ~5 seconds
      boidsW += ((rand() % 20) - 10); // Random turn -10 to +10 deg/s
    }

    targetV = boidsV;
    targetW = std::max(-20.0, std::min(20.0, boidsW));
    myCommandEnabled = true; // ensure velocities are applied
  }

  // Playback logic overrides manual target
  if (myPlayingSlot != -1) {
    if (myPlaybackIndex < myRecords[myPlayingSlot].history.size()) {
       targetV = myRecords[myPlayingSlot].history[myPlaybackIndex].first;
       targetW = myRecords[myPlayingSlot].history[myPlaybackIndex].second;
       myPlaybackIndex++;
       myCommandEnabled = true;
    } else {
       myPlayingSlot = -1; // Finished
       targetV = 0.0;
       targetW = 0.0;
       myCommandEnabled = false;
       if (myId == 1) {
           gStatusMsg = "Playback Finished.";
           printUI();
       }
    }
  } else if (gIsRecording) {
    // Record current intended targets
    myRecords[gNextRecordSlot].history.push_back(std::make_pair(targetV, targetW));
  }

  // Sonar safety check — each robot updates its own flag, then checks ALL flags
  bool blocked = applySonarSafety(targetV, targetW);
  if (blocked) {
    // Update UI once per block event
    static bool wasBlocked = false;
    if (!wasBlocked) printUI();
    wasBlocked = true;
    myRobot->setVel(0);
    myRobot->setRotVel(0);
    return;
  } else {
    static bool wasBlocked = false;
    wasBlocked = false;
  }

  targetV = std::max(-myParams.maxLinearVel, std::min(myParams.maxLinearVel, targetV));
  targetW = std::max(-myParams.maxRotVel, std::min(myParams.maxRotVel, targetW));

  myLastVel = clampDelta(targetV, myLastVel, myParams.maxVelStep);
  myLastRotVel = clampDelta(targetW, myLastRotVel, myParams.maxRotStep);

  myRobot->setVel(myLastVel);
  myRobot->setRotVel(myLastRotVel);
}

// Helper to skip unreachable robots quickly
static bool probeTcpPort(const std::string& ip, int port)
{
  int probe = socket(AF_INET, SOCK_STREAM, 0);
  if (probe < 0) return false;

  struct timeval tv = {2, 0}; // 2 second timeout
  setsockopt(probe, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = inet_addr(ip.c_str());

  int result = connect(probe, (struct sockaddr*)&addr, sizeof(addr));
  ::close(probe);
  return (result == 0);
}

static void sigintHandler(int sig)
{
  (void)sig;
  printf("\nCtrl+C received, shutting down gracefully...\n");
  gShutdownRequested = true;
  system("pkill -f roomMapper2D.py 2>/dev/null");
}

// Process remote key commands received via UDP from the Python visualizer
void processRemoteCommands()
{
  if (gCmdSocket < 0 || gControllers.empty()) return;

  char buf[16];
  struct sockaddr_in sender;
  socklen_t slen = sizeof(sender);

  while (true) {
    ssize_t n = recvfrom(gCmdSocket, buf, sizeof(buf) - 1, 0,
                         (struct sockaddr*)&sender, &slen);
    if (n <= 0) break;
    buf[n] = '\0';
    std::string key(buf);

    MultiAgentArrowControl* c = gControllers[0];

    if (key == "UP")         c->keyUp();
    else if (key == "DOWN")  c->keyDown();
    else if (key == "LEFT")  c->keyLeft();
    else if (key == "RIGHT") c->keyRight();
    else if (key == "w")     c->keyUp();
    else if (key == "s")     c->keyDown();
    else if (key == "a")     c->keyLeft();
    else if (key == "d")     c->keyRight();
    else if (key == "t")     c->keyInvUp();
    else if (key == "g")     c->keyInvDown();
    else if (key == "f")     c->keyInvLeft();
    else if (key == "h")     c->keyInvRight();
    else if (key == "SPACE") c->keyStop();
    else if (key == "x")     c->keyStop();
    else if (key == "IUP")   c->keyIndUp();
    else if (key == "IDOWN") c->keyIndDown();
    else if (key == "ILEFT") c->keyIndLeft();
    else if (key == "IRIGHT")c->keyIndRight();
    else if (key == "r")     c->keyRecord();
    else if (key == "1")     c->keyPlay(1);
    else if (key == "2")     c->keyPlay(2);
    else if (key == "3")     c->keyPlay(3);
    else if (key == "e")     c->keyBoids();
    else if (key == "q")     c->keyQuit();
  }
}

int main(int argc, char** argv)
{
  Aria::init();
  signal(SIGINT, sigintHandler); // Early registration for connection phase

  ArArgumentParser parser(&argc, argv);
  parser.loadDefaultArguments();

  const bool useWifi = parser.checkArgument("-wifi");
  int robotId = 1;
  const bool hasId = parser.checkArgument("-id");
  if (hasId) parser.checkParameterArgumentInteger("-id", &robotId);

  ArKeyHandler keyHandler;
  Aria::setKeyHandler(&keyHandler);

  if (useWifi && !hasId)
  {
    struct RobotEntry
    {
      int id;
      std::string ip;
      int port;
      ArRobot* robot;
      ArSonarDevice* sonar;
      MultiAgentArrowControl* control;
      ArRobotConnector* connector;
      ArArgumentParser* parser;
    };

    std::vector<RobotEntry> robots;
    robots.push_back({1, "192.168.1.4", 8103, NULL, NULL, NULL, NULL, NULL});
    robots.push_back({2, "192.168.1.3", 8102, NULL, NULL, NULL, NULL, NULL});
    robots.push_back({3, "192.168.1.2", 8101, NULL, NULL, NULL, NULL, NULL});

    int connectedCount = 0;

    // Register SIGINT handler early so Ctrl+C during connection also cleans up
    signal(SIGINT, sigintHandler);

    for (size_t i = 0; i < robots.size(); ++i)
    {
      RobotEntry& r = robots[i];

      printf("\n--- Attempting Robot %d at %s:%d ---\n", r.id, r.ip.c_str(), r.port);
      fflush(stdout);

      if (!probeTcpPort(r.ip, r.port))
      {
        ArLog::log(ArLog::Terse, "ArrowControl: Robot %d not reachable at %s:%d (skipping)", r.id, r.ip.c_str(), r.port);
        continue;
      }

      r.robot = new ArRobot();
      r.sonar = new ArSonarDevice();
      r.robot->addRangeDevice(r.sonar);

      // Build persistent args for ArRobotConnector to avoid SIGSEGV
      std::string portStr = std::to_string(r.port);
      int* wargc = new int(5);
      char** wargv = new char*[5];
      wargv[0] = argv[0];
      wargv[1] = (char*)"-remoteHost";
      wargv[2] = strdup(r.ip.c_str());
      wargv[3] = (char*)"-remoteRobotTcpPort";
      wargv[4] = strdup(portStr.c_str());

      r.parser = new ArArgumentParser(wargc, wargv);
      r.parser->loadDefaultArguments();
      r.connector = new ArRobotConnector(r.parser, r.robot);

      if (r.connector->connectRobot())
      {
        ArLog::log(ArLog::Normal, "ArrowControl: Robot %d connected! (Name: %s)", r.id, r.robot->getRobotName());
        r.robot->setConnectionTimeoutTime(30000); // Tolerate up to 30sec of WiFi lag
        r.robot->runAsync(true);
        ArUtil::sleep(500); // Wait for connection to settle
        r.control = new MultiAgentArrowControl(r.robot, r.id);
        r.control->run();
        connectedCount++;
      }
      else
      {
        ArLog::log(ArLog::Terse, "ArrowControl: Robot %d connector failed", r.id);
        delete r.connector;
        delete r.parser;
        // wargv cleanup would go here but simple exit is usually fine for this script
        delete r.sonar;
        delete r.robot;
        r.connector = NULL;
        r.parser = NULL;
        r.sonar = NULL;
        r.robot = NULL;
      }
    }

    if (connectedCount == 0)
    {
      ArLog::log(ArLog::Terse, "ArrowControl: no robots connected");
      system("pkill -f roomMapper2D.py 2>/dev/null");
      Aria::exit(1);
      return 1;
    }

    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].robot)
      {
        robots[i].robot->attachKeyHandler(&keyHandler);
        break;
      }
    }

    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].control)
      {
        robots[i].control->addKeyHandlers(&keyHandler);
        break; // register key callbacks once
      }
    }

    ArLog::log(ArLog::Normal, "ArrowControl: %d robots connected", connectedCount);

    // Suppress verbose ArRobot timeout logging (reduces "Timed out" spam)
    ArLog::setLogLevel(ArLog::Terse);

    // Initialize UDP socket for broadcasting telemetry to Python visualizer
    gUdpSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (gUdpSocket >= 0) {
      memset(&gVisualizerAddr, 0, sizeof(gVisualizerAddr));
      gVisualizerAddr.sin_family = AF_INET;
      gVisualizerAddr.sin_port = htons(50000);
      gVisualizerAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
      ArLog::log(ArLog::Terse, "ArrowControl: UDP telemetry socket ready (port 50000)");
    }

    // Initialize UDP command listener (receive keys from Python visualizer)
    gCmdSocket = socket(AF_INET, SOCK_DGRAM, 0);
    if (gCmdSocket >= 0) {
      struct sockaddr_in cmdAddr;
      memset(&cmdAddr, 0, sizeof(cmdAddr));
      cmdAddr.sin_family = AF_INET;
      cmdAddr.sin_port = htons(50001);
      cmdAddr.sin_addr.s_addr = htonl(INADDR_ANY);
      bind(gCmdSocket, (struct sockaddr*)&cmdAddr, sizeof(cmdAddr));
      fcntl(gCmdSocket, F_SETFL, O_NONBLOCK); // Non-blocking reads
      ArLog::log(ArLog::Terse, "ArrowControl: UDP command listener ready (port 50001)");
    }

    // Auto-launch Python 2D room mapper visualizer (kill orphans first)
    system("pkill -f roomMapper2D.py 2>/dev/null");
    ArUtil::sleep(200);
    system("python3 \"FYP codes/roomMapper2D.py\" &");

    MultiAgentArrowControl::printUI();

    // Register our SIGINT handler AFTER attachKeyHandler so ARIA doesn't override it
    signal(SIGINT, sigintHandler);

    while (true)
    {
      if (gShutdownRequested) break;

      // Process any remote commands from Python visualizer
      processRemoteCommands();

      // --- Connection watchdog ---
      bool anyConnected = false;
      for (size_t i = 0; i < robots.size(); ++i)
      {
        if (robots[i].robot && robots[i].robot->isConnected())
        {
          anyConnected = true;
        }
        else if (robots[i].robot && robots[i].control && !gAnyDisconnected)
        {
          // This robot WAS connected but lost connection
          gAnyDisconnected = true;
          gDisconnectedRobotId = robots[i].id;
          gStatusMsg = "CONNECTION LOST — Reconnecting...";
          ArLog::log(ArLog::Terse, "ArrowControl: Robot %d DISCONNECTED!", robots[i].id);
              MultiAgentArrowControl::printUI();
        }
      }
      if (!anyConnected && !gAnyDisconnected) break;

      // --- Auto-reconnect loop ---
      if (gAnyDisconnected && !gShutdownRequested)
      {
        // Find the disconnected robot
        for (size_t i = 0; i < robots.size(); ++i)
        {
          if (robots[i].robot && !robots[i].robot->isConnected() && robots[i].control)
          {
            ArLog::log(ArLog::Terse, "ArrowControl: Attempting reconnect Robot %d at %s:%d...",
                       robots[i].id, robots[i].ip.c_str(), robots[i].port);

            // Wait 5 seconds before retry
            for (int w = 0; w < 50 && !gShutdownRequested; w++) ArUtil::sleep(100);
            if (gShutdownRequested) break;

            // Probe TCP first
            if (!probeTcpPort(robots[i].ip, robots[i].port))
            {
              ArLog::log(ArLog::Terse, "ArrowControl: Robot %d not reachable yet, will retry...", robots[i].id);
              continue;
            }

            // Clean up old objects
            robots[i].robot->stopRunning();
            robots[i].robot->waitForRunExit();
            robots[i].robot->disconnect();

            // Remove old controller from gControllers
            for (size_t c = 0; c < gControllers.size(); c++) {
              if (gControllers[c] == robots[i].control) {
                gControllers.erase(gControllers.begin() + c);
                break;
              }
            }
            delete robots[i].control;
            delete robots[i].connector;
            delete robots[i].parser;
            delete robots[i].sonar;
            delete robots[i].robot;

            // Create fresh objects
            robots[i].robot = new ArRobot();
            robots[i].sonar = new ArSonarDevice();
            robots[i].robot->addRangeDevice(robots[i].sonar);

            std::string portStr = std::to_string(robots[i].port);
            int* wargc = new int(5);
            char** wargv = new char*[5];
            wargv[0] = argv[0];
            wargv[1] = (char*)"-remoteHost";
            wargv[2] = strdup(robots[i].ip.c_str());
            wargv[3] = (char*)"-remoteRobotTcpPort";
            wargv[4] = strdup(portStr.c_str());

            robots[i].parser = new ArArgumentParser(wargc, wargv);
            robots[i].parser->loadDefaultArguments();
            robots[i].connector = new ArRobotConnector(robots[i].parser, robots[i].robot);

            if (robots[i].connector->connectRobot())
            {
              ArLog::log(ArLog::Terse, "ArrowControl: Robot %d RECONNECTED! (Name: %s)",
                         robots[i].id, robots[i].robot->getRobotName());
              robots[i].robot->setConnectionTimeoutTime(30000);
              robots[i].robot->runAsync(true);
              ArUtil::sleep(500);
              robots[i].control = new MultiAgentArrowControl(robots[i].robot, robots[i].id);
              robots[i].control->run();

              gAnyDisconnected = false;
              gDisconnectedRobotId = 0;
              gStatusMsg = "Robot reconnected! Resuming...";
              ArLog::setLogLevel(ArLog::Terse);
                  MultiAgentArrowControl::printUI();
            }
            else
            {
              ArLog::log(ArLog::Terse, "ArrowControl: Robot %d reconnect FAILED, will retry...", robots[i].id);
              delete robots[i].connector;
              delete robots[i].parser;
              delete robots[i].sonar;
              delete robots[i].robot;
              robots[i].connector = NULL;
              robots[i].parser = NULL;
              robots[i].sonar = NULL;
              robots[i].robot = NULL;
              robots[i].control = NULL;
            }
          }
        }
      }

      ArUtil::sleep(100);
    }

    printf("\nShutting down robots...\n");
    fflush(stdout);

    // Close UDP sockets and kill Python visualizer
    if (gUdpSocket >= 0) { close(gUdpSocket); gUdpSocket = -1; }
    if (gCmdSocket >= 0) { close(gCmdSocket); gCmdSocket = -1; }
    system("pkill -f roomMapper2D.py 2>/dev/null");

    Aria::setKeyHandler(NULL);

    // Step 1: Send stop/disable commands while async threads are still running
    // (so packet sender can actually deliver the commands to the robots)
    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].robot && robots[i].robot->isConnected()) {
        robots[i].robot->lock();
        robots[i].robot->setVel(0);
        robots[i].robot->setRotVel(0);
        robots[i].robot->disableMotors();
        robots[i].robot->disableSonar();
        robots[i].robot->unlock();
      }
    }
    ArUtil::sleep(500); // Give time for commands to be sent over WiFi

    // Step 2: Stop the async threads FIRST (prevents read-after-close crashes)
    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].robot) {
        robots[i].robot->stopRunning();
        robots[i].robot->waitForRunExit();
      }
    }

    // Step 3: Now safe to delete controllers and disconnect
    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].control) {
        delete robots[i].control;
        robots[i].control = NULL;
      }
      if (robots[i].robot) {
        robots[i].robot->disconnect();
      }
      if (robots[i].connector) delete robots[i].connector;
      if (robots[i].parser) delete robots[i].parser;
      if (robots[i].sonar) delete robots[i].sonar;
      if (robots[i].robot) delete robots[i].robot;
    }

    printf("All robots shut down cleanly.\n");
  }
  else
  {
    ArRobot robot;
    robot.attachKeyHandler(&keyHandler);

    if (useWifi)
    {
      std::string ip = "192.168.1.2";
      int port = 8101;
      if (robotId == 2) { ip = "192.168.1.3"; port = 8102; }
      else if (robotId == 3) { ip = "192.168.1.4"; port = 8103; }

      // Build persistent args for ArRobotConnector
      std::string portStr = std::to_string(port);
      int wargc = 5;
      char* wargv[5];
      wargv[0] = argv[0];
      wargv[1] = (char*)"-remoteHost";
      wargv[2] = (char*)ip.c_str();
      wargv[3] = (char*)"-remoteRobotTcpPort";
      wargv[4] = (char*)portStr.c_str();

      ArArgumentParser wp(&wargc, wargv);
      wp.loadDefaultArguments();
      ArRobotConnector wc(&wp, &robot);

      if (!wc.connectRobot())
      {
        ArLog::log(ArLog::Terse, "ArrowControl: single robot connect failed");
        Aria::exit(1);
        return 1;
      }
    }
    else
    {
      ArRobotConnector robotConnector(&parser, &robot);
      if (!robotConnector.connectRobot())
      {
        ArLog::log(ArLog::Terse, "ArrowControl: local/single robot connect failed");
        Aria::exit(1);
        return 1;
      }
    }

    ArSonarDevice sonarDev;
    robot.addRangeDevice(&sonarDev);
    robot.runAsync(true);

    MultiAgentArrowControl control(&robot, robotId);
    control.addKeyHandlers(&keyHandler);
    control.run();

    printf("\nArrow Control Ready (single robot)\n");
    printf("Arrow keys (or WASD): move\n");
    printf("SPACE or b: stop | q: quit\n\n");
    fflush(stdout);

    while (robot.isConnected() && !gShutdownRequested) ArUtil::sleep(100);

    robot.lock();
    robot.setVel(0);
    robot.setRotVel(0);
    robot.disableMotors();
    robot.disableSonar();
    robot.unlock();
    ArUtil::sleep(300);
    robot.stopRunning();
    robot.waitForRunExit();
    robot.disconnect();
  }

  printf("Program exiting.\n");
  fflush(stdout);
  _exit(0); // Hard exit - avoids ARIA double-cleanup crash
}
