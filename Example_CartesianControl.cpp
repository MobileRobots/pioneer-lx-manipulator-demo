#include <iostream>
#include <dlfcn.h>
#include <vector>
#include "Kinova.API.CommLayerUbuntu.h"
#include "KinovaTypes.h"
#include <stdio.h>
#include <unistd.h>

using namespace std;

int main()
{
	int result;

	CartesianPosition currentCommand;

	//Handle for the library's command layer.
	void * commandLayer_handle;

	//Function pointers to the functions we need
	int (*MyInitAPI)();
	int (*MyCloseAPI)();
	int (*MySendBasicTrajectory)(TrajectoryPoint command);
	int (*MyGetDevices)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result);
	int (*MySetActiveDevice)(KinovaDevice device);
	int (*MyMoveHome)();
	int (*MyInitFingers)();
	int (*MyGetQuickStatus)(QuickStatus &);
	int (*MyGetCartesianCommand)(CartesianPosition &);

	//We load the library
	commandLayer_handle = dlopen("Kinova.API.USBCommandLayerUbuntu.so",RTLD_NOW|RTLD_GLOBAL);

	//We load the functions from the library (Under Windows, use GetProcAddress)
	MyInitAPI = (int (*)()) dlsym(commandLayer_handle,"InitAPI");
	MyCloseAPI = (int (*)()) dlsym(commandLayer_handle,"CloseAPI");
	MyMoveHome = (int (*)()) dlsym(commandLayer_handle,"MoveHome");
	MyInitFingers = (int (*)()) dlsym(commandLayer_handle,"InitFingers");
	MyGetDevices = (int (*)(KinovaDevice devices[MAX_KINOVA_DEVICE], int &result)) dlsym(commandLayer_handle,"GetDevices");
	MySetActiveDevice = (int (*)(KinovaDevice devices)) dlsym(commandLayer_handle,"SetActiveDevice");
	MySendBasicTrajectory = (int (*)(TrajectoryPoint)) dlsym(commandLayer_handle,"SendBasicTrajectory");
	MyGetQuickStatus = (int (*)(QuickStatus &)) dlsym(commandLayer_handle,"GetQuickStatus");
	MyGetCartesianCommand = (int (*)(CartesianPosition &)) dlsym(commandLayer_handle,"GetCartesianCommand");

	if((MyInitAPI == NULL) || (MyCloseAPI == NULL) ||
	   (MyGetQuickStatus == NULL) || (MySendBasicTrajectory == NULL) ||
	   (MySendBasicTrajectory == NULL) || (MyMoveHome == NULL) || (MyInitFingers == NULL) ||
	   (MyGetCartesianCommand == NULL))
	{
		cout << "* * *  E R R O R   D U R I N G   I N I T I A L I Z A T I O N  * * *" << endl;
	}
	else
	{
		cout << "I N I T I A L I Z A T I O N   C O M P L E T E D" << endl << endl;

		result = (*MyInitAPI)();

		cout << "Initialization's result :" << result << endl;

		KinovaDevice list[MAX_KINOVA_DEVICE];

		int devicesCount = MyGetDevices(list, result);

		for(int i = 0; i < devicesCount; i++)
		{
			cout << "Found a robot on the USB bus (" << list[i].SerialNumber << ")" << endl;

			//Setting the current device as the active device.
			MySetActiveDevice(list[i]);

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			cout << "Initializing the fingers" << endl;
			MyInitFingers();

			TrajectoryPoint pointToSend;
			pointToSend.InitStruct();

			//We specify that this point will be used an angular(joint by joint) velocity vector.
			pointToSend.Position.Type = CARTESIAN_VELOCITY;

			pointToSend.Position.CartesianPosition.X = 0;
			pointToSend.Position.CartesianPosition.Y = -0.15; //Move along Y axis at 20 cm per second
			pointToSend.Position.CartesianPosition.Z = 0;
			pointToSend.Position.CartesianPosition.ThetaX = 0;
			pointToSend.Position.CartesianPosition.ThetaY = 0;
			pointToSend.Position.CartesianPosition.ThetaZ = 0;

			pointToSend.Position.Fingers.Finger1 = 0;
			pointToSend.Position.Fingers.Finger2 = 0;
			pointToSend.Position.Fingers.Finger3 = 0;

			for(int i = 0; i < 200; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
				usleep(5000);
			}

			pointToSend.Position.CartesianPosition.Y = 0;
			pointToSend.Position.CartesianPosition.Z = 0.1;

			for(int i = 0; i < 200; i++)
			{
				//We send the velocity vector every 5 ms as long as we want the robot to move along that vector.
				MySendBasicTrajectory(pointToSend);
				usleep(5000);
			}

			cout << "Send the robot to HOME position" << endl;
			MyMoveHome();

			//We specify that this point will be an angular(joint by joint) position.
			pointToSend.Position.Type = CARTESIAN_POSITION;

			//We get the actual angular command of the robot.
			MyGetCartesianCommand(currentCommand);

			pointToSend.Position.CartesianPosition.X = currentCommand.Coordinates.X;
			pointToSend.Position.CartesianPosition.Y = currentCommand.Coordinates.Y - 0.1f;
			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z;
			pointToSend.Position.CartesianPosition.ThetaX = currentCommand.Coordinates.ThetaX;
			pointToSend.Position.CartesianPosition.ThetaY = currentCommand.Coordinates.ThetaY;
			pointToSend.Position.CartesianPosition.ThetaZ = currentCommand.Coordinates.ThetaZ;

			cout << "*********************************" << endl;
			cout << "Sending the first point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			pointToSend.Position.CartesianPosition.Z = currentCommand.Coordinates.Z + 0.1f;
			cout << "Sending the second point to the robot." << endl;
			MySendBasicTrajectory(pointToSend);

			cout << "*********************************" << endl << endl << endl;
		}

		cout << endl << "C L O S I N G   A P I" << endl;
		result = (*MyCloseAPI)();
	}

	dlclose(commandLayer_handle);

	return 0;
}
