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
#include <algorithm>
#include <cstring>

// Networking for TCP probe
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>

class MultiAgentArrowControl;
static std::vector<MultiAgentArrowControl*> gControllers;

class MultiAgentArrowControl
{
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
    double safeDistance = 800.0;   // mm
    double criticalDistance = 450.0; // mm
    double avoidTurnVel = 16.0;    // deg/s
  } myParams;

  ArFunctorC<MultiAgentArrowControl> myTaskCB;
  ArFunctorC<MultiAgentArrowControl> myUpCB;
  ArFunctorC<MultiAgentArrowControl> myDownCB;
  ArFunctorC<MultiAgentArrowControl> myLeftCB;
  ArFunctorC<MultiAgentArrowControl> myRightCB;
  ArFunctorC<MultiAgentArrowControl> myStopCB;
  ArFunctorC<MultiAgentArrowControl> myQuitCB;

  void controlTask();

  void keyUp();
  void keyDown();
  void keyLeft();
  void keyRight();
  void keyStop();
  void keyQuit();

  void setAllCommands(double v, double w, bool enabled, const char* label);
  double clampDelta(double target, double current, double maxStep);
  bool applySonarSafety(double& targetV, double& targetW);
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
  myQuitCB(this, &MultiAgentArrowControl::keyQuit)
{
  gControllers.push_back(this);
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
  myRobot->unlock();
  ArLog::log(ArLog::Normal, "ArrowControl: Robot %d ready", myId);
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

  keyHandler->addKeyHandler(ArKeyHandler::UP, &myUpCB);
  keyHandler->addKeyHandler(ArKeyHandler::DOWN, &myDownCB);
  keyHandler->addKeyHandler(ArKeyHandler::LEFT, &myLeftCB);
  keyHandler->addKeyHandler(ArKeyHandler::RIGHT, &myRightCB);
  keyHandler->addKeyHandler(ArKeyHandler::SPACE, &myStopCB);

  keyHandler->addKeyHandler('w', &myUpCB);
  keyHandler->addKeyHandler('s', &myDownCB);
  keyHandler->addKeyHandler('a', &myLeftCB);
  keyHandler->addKeyHandler('d', &myRightCB);
  keyHandler->addKeyHandler('b', &myStopCB);
  keyHandler->addKeyHandler('q', &myQuitCB);
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
  setAllCommands(0.0, 0.0, false, "STOP");
}

void MultiAgentArrowControl::keyQuit()
{
  ArLog::log(ArLog::Normal, "ArrowControl: quit requested");
  Aria::exit(0);
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
  ArLog::log(ArLog::Normal, "ArrowControl: %s  (v=%.1f, w=%.1f)", label, v, w);
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

  if (targetV > 1.0)
  {
    const double front = sanitizeRange(myRobot->getClosestSonarRange(-60.0, 60.0));
    if (front < myParams.safeDistance)
    {
      targetV = 0.0;
      if (front < myParams.criticalDistance && fabs(targetW) < 1.0)
      {
        const double left = sanitizeRange(myRobot->getClosestSonarRange(15.0, 100.0));
        const double right = sanitizeRange(myRobot->getClosestSonarRange(-100.0, -15.0));
        targetW = (left >= right) ? myParams.avoidTurnVel : -myParams.avoidTurnVel;
      }
      return true;
    }
  }
  else if (targetV < -1.0)
  {
    const double rearLeft = sanitizeRange(myRobot->getClosestSonarRange(120.0, 179.0));
    const double rearRight = sanitizeRange(myRobot->getClosestSonarRange(-179.0, -120.0));
    const double rear = std::min(rearLeft, rearRight);
    if (rear < myParams.safeDistance)
    {
      targetV = 0.0;
      if (rear < myParams.criticalDistance && fabs(targetW) < 1.0)
      {
        targetW = myParams.avoidTurnVel;
      }
      return true;
    }
  }

  return false;
}

void MultiAgentArrowControl::controlTask()
{
  double targetV = myCommandEnabled ? myTargetVel : 0.0;
  double targetW = myCommandEnabled ? myTargetRotVel : 0.0;

  applySonarSafety(targetV, targetW);

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

int main(int argc, char** argv)
{
  Aria::init();

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
    robots.push_back({1, "192.168.1.2", 8101, NULL, NULL, NULL, NULL, NULL});
    robots.push_back({2, "192.168.1.3", 8102, NULL, NULL, NULL, NULL, NULL});
    robots.push_back({3, "192.168.1.4", 8103, NULL, NULL, NULL, NULL, NULL});

    int connectedCount = 0;

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
    printf("\nSynchronized Arrow Control Ready\n");
    printf("Arrow keys (or WASD): move all robots together\n");
    printf("SPACE or b: stop all | q: quit\n\n");
    fflush(stdout);

    while (true)
    {
      bool anyConnected = false;
      for (size_t i = 0; i < robots.size(); ++i)
      {
        if (robots[i].robot && robots[i].robot->isConnected())
        {
          anyConnected = true;
          break;
        }
      }
      if (!anyConnected) break;
      ArUtil::sleep(100);
    }

    for (size_t i = 0; i < robots.size(); ++i)
    {
      if (robots[i].robot) {
        robots[i].robot->stopRunning();
        robots[i].robot->waitForRunExit();
      }
      if (robots[i].control) delete robots[i].control;
      if (robots[i].connector) delete robots[i].connector;
      if (robots[i].parser) delete robots[i].parser;
      if (robots[i].sonar) delete robots[i].sonar;
      if (robots[i].robot) delete robots[i].robot;
    }
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

    while (robot.isConnected()) ArUtil::sleep(100);
  }

  Aria::exit(0);
  return 0;
}
