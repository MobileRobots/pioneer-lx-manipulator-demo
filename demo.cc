
#include <iostream>

#include "Aria.h"
#include "ArSystemStatus.h"
#include "ArNetworking.h"
#include "ArVideo.h"
#include "ArmDemoTask.h"


// This is meant to run on the secondary computer ("top" or "torso" computer) of
// a Pioneer LX Manipulator or similar robot.  It tries to connect any available
// Kinova arms, a pan/tilt unit (configure via command line arguments), and to 
// an ARNL server (configure via command line arguments).  It opens another
// ArNetwworking server for configuration and some visualization (to run on the
// same computer as the ARNL server instead, use the -serverPort argument to
// specify a different server port for this server. You will need to do this if
// you see an error like "ArSocket::open: could not bind... Address already in
// use ... Can't open server on port 7272 yet, waiting")
// 
// For example:
//    demo -host 192.168.0.32 -ptzType dpptu 
// or:
//    demo -host localhost -ptzType axis -serverPort 7070
//
// When ARNL reaches a goal starting with the prefix "Arm Demo" (or just the
// name "Arm Demo", it stops the tour of goals, performs
// the arm demo (depending on demo mode). 
// There are two goal modes:
//   * Reactive - put the arms in reactive (zero gravity) mode.  The PTU is
//   moved to track the left hand.  You must resome touring goals manually via
//   MobileEyes.
//   * CartesianPos - Do a sequence of cartesian positions. The PTU tracks the
//   left hand.  When done, automatically resumes touring goals.

// Return codes:
// 0 - Normal exit
// 2 - Error connecting to arms
// 3 - Error parsing command line arguments
// 4 - Error connecting to PTU/PTZ
// 5 - Error opening server
// 7 - Error connecting to ARNL server



int main(int argc, char **argv)
{

  Aria::init();
  ArVideo::init();

  /* Get arguments */

  ArArgumentParser argParser(&argc, argv);

  ArPTZConnector ptzConnector(&argParser, NULL);
  ArServerBase server;
  ArServerSimpleOpener openServer(&argParser);

  ArClientBase client;
  ArClientSimpleConnector clientConnector(&argParser);

  argParser.loadDefaultArguments();

  if(!Aria::parseArgs())
  {
    puts("error parsing args");
    Aria::exit(3);
  }

  if(!argParser.checkHelp())
  {
    Aria::logOptions();
    Aria::exit(0);
  }



  /* Connect to PTU */

  ptzConnector.connect();
  printf("Found %lu PTUs/cameras\n", ptzConnector.getNumPTZs());
  if(ptzConnector.getNumPTZs() <= 0)
  {
    ArLog::log(ArLog::Normal, "Error connecting to PTU. Specify type with -ptzType. Run with -help for additional options.");
    Aria::exit(4);
  }

  ArPTZ* ptu = ptzConnector.getPTZ(0);
  if(!ptu)
  {
    puts("no PTU");
    Aria::exit(4);
  }
  printf("PTU pan limits (%f, %f) tilt limits (%f, %f)\n",
    ptu->getMinPan(), ptu->getMaxPan(), 
    ptu->getMinTilt(), ptu->getMaxTilt() );

  // test lookat:
/*
  printf("TEST PANTILT\npan left %f\n", -45.0);
  ptu->panTilt(-45, 0);
  ArUtil::sleep(5000);
  printf("pan right %f\n", 45.0);
  ptu->panTilt(45, 0);
  ArUtil::sleep(5000);
  printf("tilt up %f\n", 45.0);
  ptu->panTilt(0, 45);
  ArUtil::sleep(5000);
  printf("tilt down -20\n");
  ptu->panTilt(0, -20);
*/
/*
  puts("TEST LOOKAT");
  ArUtil::sleep(5000);
  puts("straigt ahead");
  ptu_look_at_arm(*ptu, 0+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("left");
  ptu_look_at_arm(*ptu, 1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("right");
  ptu_look_at_arm(*ptu, -1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("up right");
  ptu_look_at_arm(*ptu, -1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("down left");
  ptu_look_at_arm(*ptu, 1+armOffset[LEFT].x, -1+armOffset[LEFT].y, -1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("down right");
  ptu_look_at_arm(*ptu, -1+armOffset[LEFT].x, -1+armOffset[LEFT].y, -1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("up left");
  ptu_look_at_arm(*ptu, 1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  Aria::exit(0);
*/


  /* Connect to ARNL */
  // TODO keep retrying to connect in runloop 
  if(!clientConnector.connectClient(&client))
  {
    ArLog::log(ArLog::Terse, "Could not connect to ARNL server. Specify server address or name with -host option. Run with -help for more options.");
    Aria::exit(7);
  }



  /* Init demo */
  ArmDemoTask armDemoTask(&client, ptu);
  ArLog::log(ArLog::Normal, "Connecting to arm(s)...");
  if(!armDemoTask.init_arms())
  {
    ArLog::log(ArLog::Terse, "Could not connect to arms.");
    Aria::exit(2);
  }
  armDemoTask.runAsync();



  /* Set up ArNetworking server */


  ArServerHandlerCommands commandsServer(&server);
 
#ifndef WIN32
  ArServerFileLister fileLister(&server, ".");
  ArServerFileToClient fileToClient(&server, ".");
  ArServerDeleteFileOnServer deleteFileOnServer(&server, ".");
#endif
   
  ArServerInfoStrings stringInfoServer(&server);

  Aria::getInfoGroup()->addAddStringCallback(stringInfoServer.getAddStringFunctor());
  ArSystemStatus::startPeriodicUpdate(); 
  Aria::getInfoGroup()->addStringDouble(
     "CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
 //  Aria::getInfoGroup()->addStringUnsignedLong(
 //    "Computer Uptime", 14, ArSystemStatus::getUptimeFunctor());
 //  Aria::getInfoGroup()->addStringUnsignedLong(
 //    "Program Uptime", 14, ArSystemStatus::getProgramUptimeFunctor());
  Aria::getInfoGroup()->addStringInt(
     "Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt(
     "Wireless Noise", 10, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
  Aria::getInfoGroup()->addStringInt(
     "Wireless Signal", 10, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");
  ArServerHandlerCommMonitor commMonitorServer(&server);

  // TODO create video server for Kinect

  ArServerInfoDrawings drawingsServer(&server);
  drawingsServer.addDrawing(
    new ArDrawingData("polyDots", ArColor(255, 0, 0), 40, 50), "armEE",
    new ArFunctor2C<ArmDemoTask, ArServerClient*, ArNetPacket*>(&armDemoTask, &ArmDemoTask::armEENetDrawingCallback));

  printf("Opening ArNetworking server...\n");
  if(!openServer.open(&server))
  {
    std::cout << "error opening ArNetworking server" << std::endl;
    Aria::exit(5);
    return 5;
  }
  server.runAsync();
  std::cout << "ArNetworking server running on port " << server.getTcpPort() << std::endl;


  /* Run */

  puts("Running...");
  client.run();

	Aria::exit(0);
}
