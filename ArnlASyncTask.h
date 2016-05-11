#ifndef ARNLASYNCTASK_H
#define ARNLASYNCTASK_H

#include "Aria.h"
#include "ArNetworking.h"
#include "Arnl.h"
#include "ArPathPlanningTask.h"


/** an ArASyncTask subclass that runs new threads when ARNL reaches goals. 
  At each goal, a new thread is created which can perform some action
  asyncronously (while the ARNL path planning thread continues).

  To define a task for your application, create a subclass of ArnlASyncTask.
  This subclass should override the virtual getName() and runTask() methods.
  You may also override the constructor, if needed, but be sure to call the 
  base class constructor.  Create an instance of this class in your program
  such that it will not be deleted for the duration of the process. For example,
  you could create it in your program's main() function, or allocate it with
  <tt>new</tt>.

  The base class adds a config section named for the task containing a flag 
  to enable or disable the task.  You may add additional configuration 
  parameters to this section if desired by calling addConfigParam().
  
  C++ Code Example:

  @code{.cpp}
  class MyTask : public virtual ArnlASyncTask
  {
  public:
    MyTask(int defaultValue, ArPathPlanningTask *path, ArRobot *robot, ArArgumentParser *argParser = NULL)  :
      ArnlASyncTask(path, robot, argParser),
      myValue(defaultValue)
    {
      addConfigParam(ArConfigArg("Example Value", &myValue));
    }

    virtual const char *getName() const 
    {
      return "My Example Task";
    }

    virtual void runTask()
    {
      ArLog::log(ArLog::Normal, "%s: This is an example ARNL goal task.", getName());
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
class ArnlASyncTask : public virtual ArASyncTask
{
public:

  /** A callback is added that performs tasks at each goal, and some parameters
   * are addded to ArConfig. 
   */
  ArnlASyncTaskExample(ArPathPlanningTask *pp, ArRobot *robot, ArArgumentParser *argParser = NULL) : 
    myPathPlanningTask(pp), myGoalDoneCB(this, &ArnlASyncTaskExample::goalDone),
		myRobot(robot), myEnabled(true)
	{	
		ArConfig *config = Aria::getConfig();
		config->addParam(ArConfigArg("Enabled", &myEnabled, "Whether this task is enabled"), getConfigSectionName());
		myPathPlanningTask->addGoalDoneCB(&myGoalDoneCB);
  }

  /** Override this method in your subclass. */
  virtual void runTask()  = 0;

  /** Override this method in your subclass. */
  virtual const char *getName() const = 0;

  virtual const char *getConfigSectionName() const {
    return getName();
  }

  bool addConfigParam(ArConfigArg &arg) {
    return Aria::getConfig()->addParam(arg, getConfigSectionName());
  }

  void lock() {
    myMutex.lock();
  }

  void unlock() {
    myMutex.unlock();
  }

protected:
	ArPathPlanningTask *myPathPlanningTask;
  ArFunctor1C<ArnlASyncTaskExample, ArPose> myGoalDoneCB;
  ArRobot *myRobot;
  bool myEnabled;
  ArMutex myMutex;

  void waitForMoveDone() {
    while(!myRobot->isMoveDone())
      ArUtil::sleep(100);
  }



  /* This is the "goal done" callback called by the ARNL path planning thread
   * when the a goal point is sucessfully reached.  We run a new thread here to
   * perform our task.
   */
	void goalDone(ArPose pose)
	{
    if(myEnabled)
      runAsync();
	}

  void *runThread(void*)
  {
    runTask();
  }
};

#endif
