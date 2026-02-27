/*
 * testRobot3Sonar.cpp
 * Simple standalone test: connects to Robot 3 only and prints all sonar readings live.
 * Usage: DYLD_LIBRARY_PATH=lib "./FYP codes/testRobot3Sonar"
 */
#include "Aria.h"
#include <cstdio>
#include <signal.h>

static volatile bool gQuit = false;

static void sigintHandler(int) {
  printf("\nQuitting...\n");
  gQuit = true;
}

int main(int argc, char** argv)
{
  Aria::init();
  signal(SIGINT, sigintHandler);

  ArRobot robot;
  ArSonarDevice sonar;
  robot.addRangeDevice(&sonar);

  // Connect to Robot 3 at 192.168.1.4:8103
  const char* ip = "192.168.1.4";
  int port = 8103;

  std::string portStr = std::to_string(port);
  int wargc = 5;
  char* wargv[5];
  wargv[0] = argv[0];
  wargv[1] = (char*)"-remoteHost";
  wargv[2] = (char*)ip;
  wargv[3] = (char*)"-remoteRobotTcpPort";
  wargv[4] = (char*)portStr.c_str();

  ArArgumentParser parser(&wargc, wargv);
  parser.loadDefaultArguments();
  ArRobotConnector connector(&parser, &robot);

  if (!connector.connectRobot()) {
    printf("Failed to connect to Robot 3 at %s:%d\n", ip, port);
    Aria::exit(1);
    return 1;
  }

  printf("Connected to Robot 3: %s\n", robot.getRobotName());
  robot.setConnectionTimeoutTime(15000);
  robot.runAsync(true);
  ArUtil::sleep(500);

  robot.lock();
  robot.enableMotors();
  robot.enableSonar();
  robot.comInt(ArCommands::SONAR, 1);
  robot.unlock();

  signal(SIGINT, sigintHandler); // Re-register after ARIA setup

  printf("\n=== ROBOT 3 SONAR TEST ===\n");
  printf("Showing live sonar readings. Press Ctrl+C to quit.\n\n");
  printf("Number of sonar sensors: %d\n\n", robot.getNumSonar());

  while (!gQuit && robot.isConnected())
  {
    robot.lock();
    int numSonar = robot.getNumSonar();

    printf("\033[2J\033[H"); // Clear screen
    printf("=== ROBOT 3 SONAR TEST (%s) ===\n\n", robot.getRobotName());
    printf("Sonar sensors configured: %d\n\n", numSonar);

    // Print each individual sonar reading
    printf("--- Individual Sonar Readings ---\n");
    for (int i = 0; i < numSonar; i++) {
      ArSensorReading* reading = robot.getSonarReading(i);
      if (reading) {
        printf("  Sonar %2d: %5d mm  (angle: %.0f deg)\n",
               i, reading->getRange(), reading->getSensorTh());
      } else {
        printf("  Sonar %2d: NO DATA\n", i);
      }
    }

    // Print grouped range readings
    printf("\n--- Grouped Ranges ---\n");
    double front = robot.getClosestSonarRange(-60.0, 60.0);
    double left  = robot.getClosestSonarRange(30.0, 120.0);
    double right = robot.getClosestSonarRange(-120.0, -30.0);
    double rearL = robot.getClosestSonarRange(120.0, 179.0);
    double rearR = robot.getClosestSonarRange(-179.0, -120.0);

    printf("  Front (-60 to +60): %.0f mm\n", front);
    printf("  Left  (+30 to +120): %.0f mm\n", left);
    printf("  Right (-120 to -30): %.0f mm\n", right);
    printf("  Rear-L (+120 to +179): %.0f mm\n", rearL);
    printf("  Rear-R (-179 to -120): %.0f mm\n", rearR);

    printf("\nPress Ctrl+C to quit.\n");

    robot.unlock();
    ArUtil::sleep(500); // Update every 500ms
  }

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

  printf("Robot 3 disconnected cleanly.\n");
  _exit(0);
}
