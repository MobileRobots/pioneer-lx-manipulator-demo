
#include <iostream>

#include "Aria.h"
#include "ArSystemStatus.h"
#include "ArNetworking.h"
#include "ArVideo.h"

#include "ArmDemoTask.h"
#include "KinectArVideoServer.h"

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
  ArLog::log(ArLog::Normal, "Parking arms");
  armDemoTask.park_arms();
  ArLog::log(ArLog::Normal, "Starting Arm Demo monitor task");
  armDemoTask.runAsync();

  // test lookat by looking at points 1m ahead (-1 on y), 1m to each side (x), 1m up/down (z), etc.
/*
  puts("TEST LOOKAT");
  ArUtil::sleep(5000);
  puts("straigt ahead");
  armDemoTask.ptu_look_at(0+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("left");
  armDemoTask.ptu_look_at(1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("right");
  armDemoTask.ptu_look_at(-1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 0+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("up right");
  armDemoTask.ptu_look_at(-1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("down left");
  armDemoTask.ptu_look_at(1+armOffset[LEFT].x, -1+armOffset[LEFT].y, -1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("down right");
  armDemoTask.ptu_look_at(-1+armOffset[LEFT].x, -1+armOffset[LEFT].y, -1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  puts("up left");
  armDemoTask.ptu_look_at(1+armOffset[LEFT].x, -1+armOffset[LEFT].y, 1+armOffset[LEFT].z);
  puts("");
  ArUtil::sleep(5000);
  Aria::exit(0);
*/


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


  ArServerInfoDrawings drawingsServer(&server);
  drawingsServer.addDrawing(
    new ArDrawingData("polyDots", ArColor(255, 0, 0), 120, 50), "armEE",
    new ArFunctor2C<ArmDemoTask, ArServerClient*, ArNetPacket*>(&armDemoTask, &ArmDemoTask::armEENetDrawingCallback));

  /* Kinect */
  KinectArVideoServer kinectVideoServer(&server);
  kinectVideoServer.runAsync();
  

  /* Start server */
  printf("Opening ArNetworking server...\n");
  if(!openServer.open(&server))
  {
    std::cout << "error opening ArNetworking server" << std::endl;
    Aria::exit(5);
    return 5;
  }
  server.runAsync();

  std::cout 
    << std::endl 
    << "--------------------------------------------" << std::endl
    << "ArNetworking server now running on port " << server.getTcpPort() << std::endl 
    << "--------------------------------------------" << std::endl
    << std::endl;


  /* Run */
  puts("Running...");
  client.run();

	Aria::exit(0);
}
