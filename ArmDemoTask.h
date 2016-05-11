#ifndef ARMDEMOTASK_H
#define ARMDEMOTASK_H

#include <iostream>
#include <vector>
#include <stdio.h>
#include <signal.h>
#include <math.h>
#include "Aria.h"
#include "RemoteArnlASyncTask.h"

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

#define MAX_ARMS 2
#define LEFT 0
#define RIGHT 1

class ArmDemoTask: public virtual RemoteArnlASyncTask()
{
public:
  ArmDemoTask() : ArnlASyncTask(


private:
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


  Kinova::KinovaDevice armList[MAX_ARMS];
  int armCount = 0;

  typedef struct {
    float x;
    float y;
    float z;
  } PosData;
  PosData armOffset[MAX_ARMS]; // in arm coordinate system but relative to PTU

  Kinova::CartesianPosition currentArmPositions[MAX_ARMS];
  ArMutex currentArmPositionMutex[MAX_ARMS];



public:
  void init_demo();
  bool init_arms();
  void set_demo_mode(DemoMode newMode);
  void rehome_all_arms();

private:
  void clear_all_arm_trajectories();
  void set_pose(Kinova::CartesianInfo& pos, float px, float py, float pz, float ox, float oy, float oz);
  void print_user_position(Kinova::UserPosition& p);
  void set_fingers(Kinova::FingersPosition& f, float f1, float f2, float f3);
  void set_fingers_open(Kinova::FingersPosition& f);
  void set_fingers_closed(Kinova::FingersPosition& f);
  void setup_torso_protection_zone_for_left_arm();
  void setup_torso_protection_zone_for_right_arm();
  void armEENetDrawingCallback(ArServerClient *client, ArNetPacket *pkt);
  virtual void runTask();
  virtual ~ArmDemo();
};

#endif
