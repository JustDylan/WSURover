#include <dlfcn.h>

#include <SDL.h>
#include <iostream>

#include "ros/ros.h"
#include "rover_pkg/UserInput.h"

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


			rover_pkg::UserInput msg;
			
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
						
						if((int)ev.caxis.axis == 1)
						{
							msg.ljy = ((float)ev.caxis.value)/32768.0;
						}
						else if((int)ev.caxis.axis == 3)
						{
							msg.rjy = ((float)ev.caxis.value)/32768.0;
						}
						
					}
				}
				
				//display contents of user input message
				cout << "ljx: " << msg.ljx << endl;
				cout << "ljy: " << msg.ljy << endl;
				cout << "rjx: " << msg.rjx << endl;
				cout << "rjy: " << msg.rjy << endl;
				
				gpPublisher.publish(msg);
				
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
