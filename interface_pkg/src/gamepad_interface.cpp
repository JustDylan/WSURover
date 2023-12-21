#include <dlfcn.h>

#include <SDL.h>
#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "rover_pkg/UserInput.h"
#include "rover_pkg/CameraControlData.h"

using std::cout;
using std::endl;

int main(int argc, char *argv[])
{
	//initialize ros
	ros::init(argc, argv, "gamepad_interface");
	
	ros::NodeHandle nodeHandle;
	
	//game pad publisher
	ros::Publisher gpPublisher = 
		nodeHandle.advertise<rover_pkg::UserInput>(
			"user_input_stream", 
			10);
	
	//camera control publisher	
	ros::Publisher ccPub = 
		nodeHandle.advertise<rover_pkg::CameraControlData>(
			"camera_control_stream", 
			10);
			
	ros::Rate loop_rate(50);

	//dirty hack to prevent undefined udev symbols in SDL
	dlopen("/lib/x86_64-linux-gnu/libudev.so", RTLD_NOW|RTLD_GLOBAL);

	//initialize SDL
	if(SDL_Init(SDL_INIT_VIDEO | 
		SDL_INIT_JOYSTICK | 
		SDL_INIT_GAMECONTROLLER) < 0)
	{
		cout << "SDL Ran into an error" << endl;
		cout << SDL_GetError() << endl;
		
		return 0;
	}
	else
	{

		std::cout << SDL_GetError() << endl;

		SDL_GameController *controller = nullptr;

		//Look for game controller
		for(int i = 0; i < SDL_NumJoysticks(); i++)
		{
			//Make first controller found the one that's used
			if(SDL_IsGameController(i))
			{
				cout << "Controller found\n";
				
				controller = SDL_GameControllerOpen(i);
				if(controller == NULL)
					cout << "Error:" << SDL_GetError() << endl;
				else
				{
					char * controllerMap = 
						SDL_GameControllerMapping(controller);
						
					//check if controller map is returned
					if(controllerMap == NULL)
						cout << "Error:" << SDL_GetError() << endl;
					else
					{
						cout << "Controller map:" << endl;
						cout << controllerMap << endl;
					}
				}
				
				cout << "exit" << endl;
				
				break;
			}
		}
		
		if(controller == nullptr)
		{
			cout << "No controller found\n";
			return 0;
		}
		else
		{
			bool isRunning = true; 
			SDL_Event ev;

			//rover navigation control message
			rover_pkg::UserInput msg;
			
			msg.ljx = 0.0;
			msg.ljy = 0.0;
			msg.rjx = 0.0;
			msg.rjy = 0.0;
			
			//camera control message
			rover_pkg::CameraControlData ccMsg;
			
			ccMsg.ctt = 7800;
			ccMsg.ctp = 10400;
			ccMsg.QDC_SIG1 = 0;
			ccMsg.tool_select = 0;
			
			struct {
				float x;
				float y;
			} leftStick, rightStick;
			
			leftStick.x = 0.0;
			leftStick.y = 0.0;
			
			rightStick.x = 0.0;
			rightStick.y = 0.0;
			
			//Event polling loop
			while(isRunning && ros::ok())
			{
				
				
				msg.controlMode = "basic_controller";
			
				//iterate through all present events
				while(SDL_PollEvent(&ev) != 0)
				{
					//exit loop on quit event
					if(ev.type == SDL_QUIT)
						isRunning = false;
					//act on button press
					else if(ev.type == SDL_CONTROLLERBUTTONDOWN)
					{
						if(ev.cbutton.button == SDL_CONTROLLER_BUTTON_A)
							cout << "button A\n";
						else if(ev.cbutton.button == SDL_CONTROLLER_BUTTON_B)
							cout << "button B\n";
					}
					//act on joystick movement
					else if(ev.type == SDL_CONTROLLERAXISMOTION)
					{
						//print joystick activity
						/*
						cout << "Controller: " << ev.caxis.which << endl;
						cout << "Axis: " << (int)ev.caxis.axis << endl;
						cout << "Value: " << ev.caxis.value << endl;
						*/
						
						//update joystick axis
						switch((int)ev.caxis.axis)
						{
							case 0:
								leftStick.x = ((float)ev.caxis.value)/32768.0;
								break;
							case 1:
								leftStick.y = ((float)ev.caxis.value)/-32768.0;
								break;
							case 2:
								rightStick.x = ((float)ev.caxis.value)/32768.0;
								break;
							case 3:
								rightStick.y = ((float)ev.caxis.value)/-32768.0;
								break;
						}
					}
				}
				
				//left joystick magnitude
				float lStickMagnitude = sqrt(leftStick.x*leftStick.x + leftStick.y*leftStick.y);
				
				//right joystick magnitude
				float rStickMagnitude = sqrt(rightStick.x*rightStick.x + rightStick.y*rightStick.y);
				
				
				if(lStickMagnitude > 0.01)
				{
					ccMsg.ctt = -leftStick.x*15000 + 8500;
					if(ccMsg.ctt < 2400)
					{
						ccMsg.ctt = 2400;
					} else if (ccMsg.ctt > 17200)
					{
						ccMsg.ctt = 17200;
					}
					ccMsg.ctp = -leftStick.y*10400 + 10400;
					if(ccMsg.ctp < 2400)
					{
						ccMsg.ctp = 2400;
					} else if (ccMsg.ctp > 17200)
					{
						ccMsg.ctp = 17200;
					}
				}
				else
				{
					ccMsg.ctt = 8500;
					ccMsg.ctp = 10400;
				}
				
				
				if(rStickMagnitude > 0.01)
				{
					msg.ljy = cosf(copysignf(acosf(rightStick.x/rStickMagnitude), rightStick.y) - M_PI/4.0)*rStickMagnitude/1.3;
					msg.rjy = cosf(copysignf(acosf(rightStick.y/rStickMagnitude), rightStick.x) + M_PI/4.0)*rStickMagnitude/1.3;
				}
				else
				{
					msg.rjy = 0.0;
					msg.ljy = 0.0;
				}
				
				
				cout << "x1: " << leftStick.x << endl <<
					"y1: " << leftStick.y << endl <<
					"x2: " << rightStick.x << endl <<
					"y2: " << rightStick.y << endl <<
					"l1: " << lStickMagnitude << endl <<
					"r1: " << rStickMagnitude << "\n\n";
					
				
				
				
				//display contents of user input message
				cout << "ljx: " << msg.ljx << endl <<
					"ljy: " << msg.ljy << endl <<
					"rjx: " << msg.rjx << endl <<
					"rjy: " << msg.rjy << endl <<
					"ctt: " << ccMsg.ctt << endl <<
					"ctp: " << ccMsg.ctp << "\n\n";
				
				gpPublisher.publish(msg);
				
				ccPub.publish(ccMsg);
				
				ros::spinOnce();
				
				loop_rate.sleep();
			}

			if(controller != NULL)
				SDL_GameControllerClose(controller);

			SDL_Quit();

			return 0;
		}
	}
}
