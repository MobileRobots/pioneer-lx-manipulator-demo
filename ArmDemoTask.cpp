
#include "ArmDemoTask.h"


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
//  set_pose(demoCartesianPositions[i++], -0.013, -0.53, 0.08,  1.855, 1.383, -2.711);
    set_pose(demoCartesianPositions[i++], -0.23, -0.69, -0.15,  2.954, -0.195, 2.445); // move down
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




/*
void setup_torso_protection_zone_for_left_arm()
{
  Kinova::ZoneList zones;
  zones.NbZones = 1;  // todo add second zone for kinect and third for robot base
  zones.Zones[0].zoneShape.shapeType = Kinova::PrismSquareBase_Z; 
  // TODO what do the Points mean? Location, width, height, depth?  Or
  // diagonaly opposed vertices to define the square prism?  Or specify each vertex?
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


void armEENetDrawingCallback(ArServerClient *client, ArNetPacket *pkt)
{
  ArNetPacket reply;
  reply.byte4ToBuf(armCount);
  for(int i = 0; i < armCount; ++i)
  {
    currentArmPositionMutex[i].lock();
    float ax = currentArmPositions[i].Coordinates.X;
    float ay = currentArmPositions[i].Coordinates.Y;
    currentArmPositionMutex[i].unlock();
    int rx = 1000.0 * (-1*armOffset[i].y + -1*ay);    // arm -Y m -> robot X mm
    int ry = 1000.0 * (-1*armOffset[i].x + -1*ax);    // arm -X m -> robot Y mm 
  printf("arm pos %d = %d, %d\n", i, rx, ry);
    reply.byte4ToBuf(rx);
    reply.byte4ToBuf(ry);
  }
  client->sendPacketUdp(&reply);
}




bool ArmDemo::init_arms()
{
  /* Connect to Arms */

  int result;
  Kinova::AngularPosition torqueData; 

  result = Kinova::InitAPI();
  std::cout << "Kinova Arm Initialization result :" << result << std::endl;
  if(result != 1)
  {
    std::cout << "Warning: error initializing arms!" << std::endl;
    Aria::exit(1);
  }
  Aria::addExitCallback(new ArGlobalFunctor(&close_arms_and_exit));


  armCount = Kinova::GetDevices(armList, result);
  std::cout << "Found " << armCount << " arms" << std::endl;

  if(armCount <= 0)
    Aria::exit(2);

  if(armCount > MAX_ARMS)
  {
    std::cout << "Too many arms, limiting to " << MAX_ARMS << std::endl;
    armCount = MAX_ARMS;
  }

  for(int i = 0; i < armCount; ++i)
  {
    Kinova::SetActiveDevice(armList[i]);
    Kinova::InitFingers();
  }

  // set up arm positions vs. camera, using arm axes, meters
  // so: arm below camera is -z, arm above camera is +z
  // arm behind camera is +y, arm in front of camera is -y
  // arm to the left of camera is +x, to the right is -x
  armOffset[LEFT].x = 0.1;
  armOffset[LEFT].y = 0;// 0.1;
  armOffset[LEFT].z = 0;// -0.1;
  armOffset[RIGHT].x = -0.1;
  armOffset[RIGHT].y = 0;// 0.1;
  armOffset[RIGHT].z = 0;// -0.1;
  // TODO move to call from main
}


void ArmDemo::runTask() 
{
  /* Run */

  Kinova::SetActiveDevice(armList[LEFT]);
  int i = LEFT;

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
            //ArUtil::sleep(1000);
          }
          demoWaitingToFinish = true;
        }
      }
    }

    // get data and if arm 0, point PTU at its current position.
		//for(int i = 0; i < armCount; ++i)
		{
			//Kinova::SetActiveDevice(armList[i]);

      currentArmPositionMutex[i].lock();
			Kinova::GetCartesianPosition(currentArmPositions[i]);
      const float px = currentArmPositions[i].Coordinates.X;
      const float py = currentArmPositions[i].Coordinates.Y;
      const float pz = currentArmPositions[i].Coordinates.Z;
      const float ox = currentArmPositions[i].Coordinates.ThetaX;
      const float oy = currentArmPositions[i].Coordinates.ThetaY;
      const float oz = currentArmPositions[i].Coordinates.ThetaZ;
      currentArmPositionMutex[i].unlock();

      Kinova::GetAngularForce(torqueData);
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
      {
        ptu_look_at_arm(*ptu, px+armOffset[i].x, py+armOffset[i].y, pz+armOffset[i].z);
      }

    }

    printf("\r");
    fflush(stdout);

    ArUtil::sleep(500);
    
	}
}
 
ArmDemo::~ArmDemo()
{

  Kinova::CloseAPI();

}
