#ifndef REMOTEARNLTASK_H
#define REMOTEARNLTASK_H

#include "Aria.h"
#include "ArNetworking.h"
#include "ArClientHandlerRobotUpdate.h"


/** Monitor status of ARNL server and call a user-supplied virtual method based
    on status changed such as reaching a goal or failing to reach a goal.  
    Use this to perform some action based on these events.

  To define a task for your application, create a subclass of this class..
  This subclass should override the virtual getName() and goalReached() methods.
  You may also override the constructor, if needed, but be sure to call the 
  base class constructor.  Create an instance of this class in your program
  such that it will not be deleted for the duration of the process. For example,
  you could create it in your program's main() function, or allocate it with
  <tt>new</tt>.

  You can run this status monitor in a new thread by calling runAsync().  
  Handler methods such as goalReached(), goalFailed()
  etc. are called in this thread, so you can perform long running work in them.
  If the status changes before your handler method returns, a new one will be
  called but there is not currently a history or queue of events, only the most 
  recent status change is handled. (This may be changed in the future.)

  Or, instead of running in a new thread, you can call checkStatus() from your
  own program loop.

  C++ Code Example:

  @code{.cpp}
  class MyTask : public virtual RemoteArnlTask
  {
  public:
    MyTask(ArClientBase *client, ArArgumentParser *argParser = NULL)  :
      RemoteArnlTask(clint, argParser)
    {
    }

    virtual const char *getName() const 
    {
      return "My Example Task";
    }

    virtual void goalReached(const GoalInfo &g)
    {
      ArLog::log(ArLog::Normal, "%s: This is an example ARNL goal task.", getName());
      if(g.checkName("Pause Here"))
        ArUtil::sleep(5000);
      ArLog::log(ArLog::Normal, "%s: Task ended.", getName());
    }
  };

  ...

  void main(int argc, char** argv)
  {
    Aria::init();
    ...
    ArClientBase client;
    ...
    MyTask task(&client);
    task.runAsync();
    ...
  }
  @endcode

*/
class RemoteArnlTask : public virtual ArASyncTask
{
public:

  RemoteArnlTask(const char *name, ArClientBase *client,  ArArgumentParser *argParser = NULL) : 
    // TODO let someone pass in their own ArClientHandlerRobotUpdates 
    myName(name),
    myClient(client),
    myRobotUpdateHandler(client),
    myFirstCycleCB(this, &RemoteArnlTask::firstCycleCallback),
    myStatusChangedCB(this, &RemoteArnlTask::statusChanged)
	{	
    myRobotUpdateHandler.addStatusChangedCB(&myStatusChangedCB);
    //myClient->addConnectCB(&myClientConnectCB);
    // doesn't have a connect callback, do this instead:
    firstCycle = true;
    myClient->addCycleCallback(&myFirstCycleCB);
  }
  
  ~RemoteArnlTask()
  {
    myRobotUpdateHandler.stopUpdates();
    myRobotUpdateHandler.remStatusChangedCB(&myStatusChangedCB);
    myClient->remCycleCallback(&myFirstCycleCB);
  }

  virtual const char *getName() const  { return myName.c_str(); }

  static bool stringStartsWith(const std::string& s, const std::string& prefix)
  {
    return s.compare(0, prefix.size(), prefix) == 0;
  }

  class GoalInfo {
  public:
    bool hasName;
    std::string name;
    void setName(const std::string& n) {
      name = n;
      hasName = true;
    }
    bool checkName(const std::string &n) const {
      return (hasName && name == n);
    }
    bool checkNamePrefix(const std::string &prefix) const {
      return (hasName && stringStartsWith(name, prefix)); 
    }
    GoalInfo() : hasName(false) {}
    GoalInfo(const std::string& goalName) : hasName(true), name(goalName) {}
  };

  /** Override this method in your subclass. */
  virtual void goalReached(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Goal Reached", getName());
  }

  /** Override this method in your subclass. */
  virtual void goingToGoal(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Going to Goal", getName());
  }

  /** Override this method in your subclass. */
  virtual void goalFailed(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Goal Failed", getName());
  }

  /** Override this method in your subclass. */
  virtual void goalPointReached(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Goal Point Reached", getName());
  }

  /** Override this method in your subclass. */
  virtual void goingToGoalPoint(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Going to Goal Point", getName());
  }

  /** Override this method in your subclass. */
  virtual void goalPointFailed(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Goal Point Failed", getName());
  }

  /** Override this method in your subclass. */
  virtual void returningHome(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Going to Home", getName());
  }

  /** Override this method in your subclass. */
  virtual void returnedHome(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Returned Home", getName());
  }

  /** Override this method in your subclass. */
  virtual void homeFailed(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Home Failed", getName());
  }

  /** Override this method in your subclass. */
  virtual void touringToGoal(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Touring to goal", getName());
  }


  // Returns a reference to a copy of the most recently received basic robot
  // data from the server, including robot pose, status, mode, battery state.
  const ArClientHandlerRobotUpdate::RobotData& getRobotData() 
  {
    myRobotUpdateHandler.lock();
    myTmpData = myRobotUpdateHandler.getData();
    myRobotUpdateHandler.unlock();
    return myTmpData;
  }

  ArClientBase *getClient() const { return myClient; }

  void requestGoToGoal(const std::string& goalName)
  {
    myClient->requestOnceWithString("gotoGoal", goalName.c_str());
  }

protected:
  std::string myName;
  ArClientBase *myClient;
  ArClientHandlerRobotUpdate myRobotUpdateHandler;

private:
  ArClientHandlerRobotUpdate::RobotData myTmpData;

  ArFunctorC<RemoteArnlTask> myFirstCycleCB;
  ArFunctor2C<RemoteArnlTask, const char*, const char*> myStatusChangedCB;

  void clientConnected() {
    ArLog::log(ArLog::Normal, "%s: Requesting status updates from server...", getName());
    myRobotUpdateHandler.requestUpdates();
  }

  bool firstCycle;
  void firstCycleCallback() {
    if(firstCycle)
    {
      clientConnected();
      firstCycle = false;
    }
  }

  // TODO make this an event queue instead.
  bool myStatusChanged;
  ArCondition myStatusChangedCondition;
  ArMutex myStatusMutex;

  virtual void statusChanged(const char* m, const char *s) {
    printf("RemoteArnlTask: server status changed.  mode=%s, status=%s", m, s);
    myStatusChanged = true;
    myStatusChangedCondition.signal();
  }

  virtual void *runThread(void*)
  {
    ArLog::log(ArLog::Normal, "%s: Now monitoring server status...", getName());
    while(true)
    {
      myStatusChangedCondition.wait();
      checkStatus();
    }
  }

public:
  void checkStatus()
  {
    myStatusMutex.lock();
    if(!myStatusChanged)
    {
      myStatusMutex.unlock();
      return;
    }
    myStatusChanged = false;
    myStatusMutex.unlock();
    myRobotUpdateHandler.lock();
    const std::string mode = myRobotUpdateHandler.getMode();
    const std::string status = myRobotUpdateHandler.getStatus();;
    myRobotUpdateHandler.unlock();
    //printf("checkStatus(): mode=%s, status=%s\n", mode.c_str(), status.c_str());
    GoalInfo g;
    if(mode == "Goto goal")
    {
      if(stringStartsWith(status, "Arrived at "))
      {
        g.setName(status.substr(strlen("Arrived at ")));
        goalReached(g);
      }
      else if(stringStartsWith(status, "Failed"))
      {
        g.setName(status.substr(strlen("Failed to reach ")));
        goalFailed(g);
      }
      // TODO are there other statuses?
    }
    else if(mode == "Go home")
    {
      g.setName("Home");
      if(status == "Returned home")
        returnedHome(g);
      else if(status == "Returning home")
        returningHome(g);
      else if(status == "Failed to get home")
        homeFailed(g);
    }
    else if(mode == "Touring goals")
    {
      if(stringStartsWith(status, "Touring to "))
        g.setName(status.substr(strlen("Touring to ")));
      touringToGoal(g);
    }
    // TODO more modes
  }
  
};

#endif
