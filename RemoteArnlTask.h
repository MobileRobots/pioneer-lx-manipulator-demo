#ifndef ARNLASYNCTASK_H
#define ARNLASYNCTASK_H

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningTask.h"


/** When a remote ARNL server reaches a goal, call a user-supplied virtual
 * method

  To define a task for your application, create a subclass of this class..
  This subclass should override the virtual getName() and goalReached() methods.
  You may also override the constructor, if needed, but be sure to call the 
  base class constructor.  Create an instance of this class in your program
  such that it will not be deleted for the duration of the process. For example,
  you could create it in your program's main() function, or allocate it with
  <tt>new</tt>.

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
  @endcode


  Note one instance of this class is created for the
  whole program, but new threads may be created at any time (whenever ARNL happens
  to reach a goal), these threads are sharing access to the variables withhin the
  class, and so this access is synchronized using a mutex. Make certain you do
  not keep the mutex locked during any long running operations or operations of
  indeterminate duration, and that in all logical paths through the code, the
  mutex is eventually unlocked if locked.  


*/
class RemoteArnlTask 
{
public:

  RemoteArnlTask(ArClientBase *client,  ArArgumentParser *argParser = NULL) : 
    // TODO let someone pass in their own ArClientHandlerRobotUpdates 
    myClient(client),
    myClientConnectCB(this, &RemoteArnlTaskk::clientConnnected),
    myStatusChangedCB(this, &RemoteArnlTask::statusChanged)
	{	
    myRobotUpdateHandler.addStatusChangedCB(&myStatusChangedCB);
    myClient->addConnectCB(&myClientConnectCB);
  }
  
  ~RemoteArnlTask()
  {
    myRobotUpdateHandler.stopUpdates();
    myRobotUpdateHandler.remStatusChangedCB(&myStatusChangedCB);
  }

  /** Override this method in your subclass. */
  virtual const char *getName() const = 0;

  class GoalInfo {
  public:
    bool hasName;
    std::string name;
    bool checkName(const std::string &n) {
      return (hasName && name == n);
    }
    bool checkNamePrefix(const std::string &prefix) {
      return (hasName && name.compare(prefix)); // TODO
    }
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
  virtual void goingToHome(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Going to Home", getName());
  }

  /** Override this method in your subclass. */
  virtual void homeReached(const GoalInfo& goalInfo) 
  {
    ArLog::log(ArLog::Normal, "%s: Home Reached", getName());
  }

  /** Override this method in your subclass. */
  virtual void homeFailed() 
  {
    ArLog::log(ArLog::Normal, "%s: Home Failed", getName());
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

protected:
  ArClientBase *myClient;
  ArClientHandlerRobotUpdate myRobotUpdateHandler;

private:
  ArClientHandlerRobotUpdate::RobotData myTmpData;

  ArFunctorC<RemoteArnlTask> myClientConnectCB;
  ArFunctorC<RemoteArnlTask> myStatusChangedCB;

  void clientConnected() {
    myRobotUpdateHandler.requestUpdates();
  }

protected:
  virtual void statusChanged(const char* m, const char *s) {
    std::string mode = m;
    std::string status = s;
    GoalInfo g;
    if(mode == "go to goal")
    {
      if(status.substr(0, 10) == "Arrived at")
        goalReached(g);
    else if(status.substr(0, 6) == "Failed")
      goalFailed(g);
    }
    else if(mode == "Going home")
    {
      if(status == "Failed to get home")
        homeFailed(g);
    }
    // todo
  }
    
    
};

#endif
