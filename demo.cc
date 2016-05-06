
#include <iostream>
#include <vector>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include "Aria.h"
#include "ArNetworking.h"
#include "ArVideo.h"

namespace Kinova {
#include "Kinova.API.CommLayerUbuntu.h"
#include "Kinova.API.UsbCommandLayerUbuntu.h"
#include "KinovaTypes.h"
};

typedef enum {
  CartesianVel,
  CartesianPos,
  Reactive,
  Idle
} DemoMode;

const DemoMode DEFAULT_MODE  = CartesianPos;
bool demoDone = false;
bool demoWaitingToFinish = false;
ArTime demoTime;


// These are initialized in init_demo():
DemoMode demoMode;
Kinova::CartesianInfo demoCartesianVelocities[12];
int numDemoCartesianVelocities = 0;
Kinova::TrajectoryPoint demoTrajectoryCommand;
Kinova::CartesianInfo demoCartesianPositions[12];
int numDemoCartesianPositions = 0;
Kinova::TrajectoryPoint demoPositionCommand;


Kinova::KinovaDevice armList[MAX_KINOVA_DEVICE];
int armCount = 0;

float panoffset = 0;   // offset of rotation point of pan joint on X axis (left/right)
float tiltoffset = 0;  // offset of rotation point of tilt joint on Z axis (up/down)

void clear_all_arm_trajectories()
{
  for(int i = 0; i < armCount; ++i)
  { 
    Kinova::SetActiveDevice(armList[i]);
    Kinova::EraseAllTrajectories();
  }
  puts("Cleared old arm trajectory commands.");
}

void set_demo_mode(DemoMode newMode)
{
  DemoMode oldMode = demoMode;

  if(newMode == CartesianPos)
  {
    puts("\nSet demo mode to CartesianPos.");
  }

  if(newMode == Reactive && oldMode != Reactive)
  {
    Kinova::StartForceControl();
    puts("\nSet demo mode to Reactive. Enabled reactive force control.");
  }
  else if(newMode != Reactive && oldMode == Reactive)
  {
    Kinova::StopForceControl();
    puts("\nDisabled reactive force control.");
  }

  demoMode = newMode;
  demoTime.setToNow();
  demoDone = false;
  demoWaitingToFinish = false;
  puts("Reset demo state.");
  
  clear_all_arm_trajectories();
}

void set_pose(Kinova::CartesianInfo& pos, float px, float py, float pz, float ox, float oy, float oz)
{
  pos.X = px;
  pos.Y = py;
  pos.Z = pz;
  pos.ThetaX = ox;
  pos.ThetaY = oy;
  pos.ThetaZ = oz;
}

void print_user_position(Kinova::UserPosition& p)
{
  if(p.Type == Kinova::CARTESIAN_POSITION)
    printf("CartesianPosition: Position: (%f, %f, %f), Orientation: (%f, %f, %f), Fingers: (%f, %f, %f)\n",
      p.CartesianPosition.X, p.CartesianPosition.Y, p.CartesianPosition.Z,
      p.CartesianPosition.ThetaX, p.CartesianPosition.ThetaY, p.CartesianPosition.ThetaZ,
      p.Fingers.Finger1, p.Fingers.Finger2, p.Fingers.Finger3);
  else if(p.Type == Kinova::CARTESIAN_VELOCITY)
    printf("CartesianVelocity: Position: (%f, %f, %f), Orientation: (%f, %f, %f), Fingers: (%f, %f, %f)\n",
      p.CartesianPosition.X, p.CartesianPosition.Y, p.CartesianPosition.Z,
      p.CartesianPosition.ThetaX, p.CartesianPosition.ThetaY, p.CartesianPosition.ThetaZ,
      p.Fingers.Finger1, p.Fingers.Finger2, p.Fingers.Finger3);
  /// todo angular pos
}

void set_fingers(Kinova::FingersPosition& f, float f1, float f2, float f3)
{
  f.Finger1 = f1;
  f.Finger2 = f2;
  f.Finger3 = f3;
}

void set_fingers_open(Kinova::FingersPosition& f)
{
  f.Finger1 = f.Finger2 = f.Finger3 = 0;
}

void set_fingers_closed(Kinova::FingersPosition& f)
{
  f.Finger1 = f.Finger2 = 4000;
  f.Finger3 = 3700;
}

void init_demo()
{
  set_demo_mode(DEFAULT_MODE);

  demoTrajectoryCommand.InitStruct();
  demoTrajectoryCommand.Position.Type = Kinova::CARTESIAN_VELOCITY;
  set_fingers(demoTrajectoryCommand.Position.Fingers, 0, 0, 0);

  numDemoCartesianVelocities = 4;
  // must be 12 or less
  set_pose(demoCartesianVelocities[0], 0, 0, 5, 0, 0, 0);
  set_pose(demoCartesianVelocities[1], 0, -5, -5, 0, 0, 0);
  set_pose(demoCartesianVelocities[2], 0, 5, 0, 0, 0, 0);
  set_pose(demoCartesianVelocities[3], 0, 0, 0, 0, 0, 0);

  demoPositionCommand.InitStruct();
  demoPositionCommand.Position.Type = Kinova::CARTESIAN_POSITION;

  // must be 12 or less positions
  int i = 0;
  set_pose(demoCartesianPositions[i++], -0.23, -0.69,  0.08,  1.136, -1.165,  2.445);
  set_pose(demoCartesianPositions[i++], -0.23, -0.69,  0.08,  2.954, -0.195, 2.445); // point down
  set_pose(demoCartesianPositions[i++], -0.23, -0.69, -0.15,  2.954, -0.195, 2.445); // move down
  set_pose(demoCartesianPositions[i++], -0.23, -0.69,  0.08,  2.954, -0.195, 2.445); // back up
  //set_pose(demoCartesianPositions[i++], -0.23, -0.69, -0.15,  2.950, -0.195,  2.445);
  set_pose(demoCartesianPositions[i++], -0.09, -0.69,  0.07,  2.950, -0.195,  2.445);
  set_pose(demoCartesianPositions[i++], -0.03, -0.32,  0.22,  2.950, -0.195,  2.445);
  set_pose(demoCartesianPositions[i++], -0.013,-0.53,  0.08,  0.054, -1.355, 0.960);
  set_pose(demoCartesianPositions[i++], -0.013, -0.53, 0.08,  1.855, 1.383, -2.711);
  numDemoCartesianPositions = i;
}
  

void close_arms_and_exit()
{
  puts("Closing Kinova API and exiting...");
  Kinova::CloseAPI();
  Aria::exit(0);
}


void rehome_all_arms()
{
  printf("\nRehoming all %d arms... ", armCount); fflush(stdout);
  for(int i = 0; i < armCount; ++i)
  {
    printf("#%d...", i); fflush(stdout);
    Kinova::SetActiveDevice(armList[i]);
    Kinova::MoveHome();
  }
  demoTime.setToNow();
  puts(""); fflush(stdout);
}

void toggle_mode()
{
  if(demoMode == CartesianPos)
    set_demo_mode(Reactive);
  else
    set_demo_mode(CartesianPos);
}


void rehome_signal_handler(int signum)
{
  rehome_all_arms();
}

void mode_signal_handler(int signum)
{
  toggle_mode();
}


// quick hack to look at where the end effector is. 
// * x, y and z are arm end effector position in arm coordinate system (-y // forward, +y back, +z up, -z down, -x right, +x left)
// * xoffset, yoffset and zoffset are position of camera relative to arm base  (coordinate system origin)
// * xoffset should be negative for an arm on the right side of the camera,
// positive for left arm.
// * yoffset should be positive if camera is behind arm bases, negative if in
// front.
// * zoffset should be positive if camera is mounted higher than arm bases,
// negative if lower.
void ptu_look_at_arm(ArPTZ& ptu, float x, float y, float z, float xoffset, float yoffset, float zoffset)
{
 //   todo also include vertical offset of ptu stage from middle of tilt joint
  float p, t;
  if( z == 0)
    p = 0;
  else
    p = -1 * ArMath::radToDeg( atan( -x / (y + panoffset) ) ) ;
  if(z == 0)
    t = 0;
  else
    t = -1 * ArMath::radToDeg( atan( x / (z + tiltoffset) ) ) ;
  //print 'ptu lookat: pos [%f, %f, %f] -> angles [%f, %f].' % (x, y, z, p, t)
  if(p >= ptu.getMaxPosPan() - 5)
    p = ptu.getMaxPosPan() - 15;
  else if(p <= ptu.getMaxNegPan() + 5)
    p = ptu.getMaxNegPan() + 15;
  if(t >= ptu.getMaxPosTilt() - 5)
    t = ptu.getMaxPosTilt() - 15;
  else if(t <= ptu.getMaxNegTilt() + 5)
    t = ptu.getMaxNegTilt() + 15;
  printf("[ptu lookat: pan % 2.0f, tilt % 2.0f]", p, t);
  ptu.panTilt(p, t);
}


/*
void setup_torso_protection_zone_for_left_arm()
{
  Kinova::ZoneList zones;
  zones.NbZones = 1;  // todo add second zone for kinect and third for robot base
  zones.Zones[0].zoneShape.shapeType = Kinova::PrismSquareBase_Z; 
  zones.Zones[0].zoneShape.Points[0].X = 
  zones.Zones[0].zoneShape.Points[0].Y = 
  zones.Zones[0].zoneShape.Points[0].Z = 
  zones.Zones[0].zoneShape.Points[1].X = 
  zones.Zones[0].zoneShape.Points[1].Y = 
  zones.Zones[0].zoneShape.Points[1].Z = 
  zones.Zones[0].zoneShape.Points[2].X = 
  zones.Zones[0].zoneShape.Points[2].Y = 
  zones.Zones[0].zoneShape.Points[2].Z = 
  zones.Zones[0].zoneShape.Points[3].X = 
  zones.Zones[0].zoneShape.Points[3].Y = 
  zones.Zones[0].zoneShape.Points[3].Z = 
  zones.Zones[0].zoneLimitation.speedParameter1 = 0;
  zones.Zones[0].zoneLimitation.speedParameter2 = 0;
  zones.Zones[0].zoneLimitation.speedParameter3 = 0;
  Kinova::SetActiveDevice(armList[0]);
  Kinova::SetProtectionZone(zones);
}

void setup_torso_protection_zone_for_right_arm()
{
  Kinova::ZoneList zones;

  Kinova::SetActiveDevice(armList[1]);
  Kinova::SetProtectionZone(zones);
}
*/


int main(int argc, char **argv)
{

  Aria::init();
  ArVideo::init();

  ArArgumentParser argParser(&argc, argv);
  argParser.loadDefaultArguments();

  ArPTZConnector ptzConnector(&argParser, NULL);


  int result;
  Kinova::CartesianPosition position;
  Kinova::AngularPosition torqueData; 

  result = Kinova::InitAPI();
  std::cout << "Kinova Arm Initialization result :" << result << std::endl;
  if(result != 1)
    Aria::exit(1);
  Aria::addExitCallback(new ArGlobalFunctor(&close_arms_and_exit));


  armCount = Kinova::GetDevices(armList, result);
  std::cout << "Found " << armCount << " arms" << std::endl;

  if(armCount <= 0)
    Aria::exit(2);

  for(int i = 0; i < armCount; ++i)
  {
    Kinova::SetActiveDevice(armList[i]);
    Kinova::InitFingers();
  }

  signal(SIGUSR1, rehome_signal_handler);
  signal(SIGUSR2, mode_signal_handler);


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

  ptzConnector.connect();
  printf("Found %lu PTUs/cameras\n", ptzConnector.getNumPTZs());
  //if(ptzConnector.getNumPTZs() <= 0)
  //  Aria::exit(3);

  ArPTZ* ptu = ptzConnector.getPTZ(0);

  init_demo();

  puts("Running...");
  while(true)
  {
    // do demo arm control:
    if(!demoDone)
    {

        
      if(demoMode == CartesianVel)
      {
        if(demoTime.secSince() >= 40) 
        {
          puts("\ndoing last cartesian velocity motion to stop it");
          demoTrajectoryCommand.Position.CartesianPosition = demoCartesianVelocities[3];
          // done.  this trajectory should have no motion.
          demoDone = true;
          demoWaitingToFinish = false;
          demoTime.setToNow();
        }
        else if(demoTime.secSince() >= 30)
        {
          puts("\ndoing third cartesian velocity motion");
          demoTrajectoryCommand.Position.CartesianPosition = demoCartesianVelocities[2];
        }
        else if(demoTime.secSince() >= 20)
        {
          puts("\ndoing second cartesian velocity motion");
          demoTrajectoryCommand.Position.CartesianPosition = demoCartesianVelocities[1];
        }
        else if(demoTime.secSince() >= 10)
        {
          puts("\ndoing first cartesian velocity motion");
          demoTrajectoryCommand.Position.CartesianPosition = demoCartesianVelocities[0];
        }

        Kinova::SendBasicTrajectory(demoTrajectoryCommand);

      }
      else if(demoMode == CartesianPos)
      {
        if(demoWaitingToFinish)
        {
          if(demoTime.secSince() >= 40)
            demoDone = true;
        }
        else
        {
          for(int i = 0; i < numDemoCartesianPositions; ++i)
          {
            demoPositionCommand.Position.CartesianPosition = demoCartesianPositions[i];
            if(i >= 2)
              set_fingers_closed(demoPositionCommand.Position.Fingers);
            else
              set_fingers_open(demoPositionCommand.Position.Fingers);
            printf("\n-> Sending position command %d: ", i);
            print_user_position(demoPositionCommand.Position);
            Kinova::SendBasicTrajectory(demoPositionCommand);
            ArUtil::sleep(5000);
          }
          demoWaitingToFinish = true;
        }
      }
    }

    // display data and if arm 0, point PTU at its current position.
		for(int i = 0; i < armCount; ++i)
		{
			Kinova::SetActiveDevice(armList[i]);

			Kinova::GetCartesianPosition(position);
      Kinova::GetAngularForce(torqueData);
      const float px = position.Coordinates.X;
      const float py = position.Coordinates.Y;
      const float pz = position.Coordinates.Z;
      const float ox = position.Coordinates.ThetaX;
      const float oy = position.Coordinates.ThetaY;
      const float oz = position.Coordinates.ThetaZ;
      std::vector<float> jointTorques(6);
      jointTorques[0] = torqueData.Actuators.Actuator1;
      jointTorques[1] = torqueData.Actuators.Actuator2;
      jointTorques[2] = torqueData.Actuators.Actuator3;
      jointTorques[3] = torqueData.Actuators.Actuator4;
      jointTorques[4] = torqueData.Actuators.Actuator5;
      jointTorques[5] = torqueData.Actuators.Actuator6;
      
			printf("Arm #%d: [x=% 2.2f, y=% 2.2f, z=% 2.2f, torques=%2.1f, %2.1f, %2.1f, %2.1f, %2.1f, %2.1f]  ", 
        i, px, py, pz, 
        jointTorques[0], 
        jointTorques[1], 
        jointTorques[2], 
        jointTorques[3], 
        jointTorques[4], 
        jointTorques[5]
      );
      fflush(stdout);

      if(i == 0 && ptu != NULL)
        ptu_look_at_arm(*ptu, px, py, pz, 0, 0, 0);

    }

    printf("\r");
    fflush(stdout);

    ArUtil::sleep(500);
    
	}
 

  Kinova::CloseAPI();

	Aria::exit(0);
}
