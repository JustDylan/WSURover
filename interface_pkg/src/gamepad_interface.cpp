//functions to manually load a dynamic library
#include <dlfcn.h>

//for HID input and creating application windows
#include <SDL.h>

#include <iostream>
#include <math.h>

//ros functions
#include "ros/ros.h"

//ros message for keyboard and joystick input
#include "rover_pkg/UserInput.h"

//ros message for camera orientation
#include "rover_pkg/CameraControlData.h"

//joystick smoothing variables
float const JOYSTICK_DEADZONE = 0.1f;
float const JOYSTICK_CURVE_COEFFICIENT = 4.0f;

//home orientation for tower camera
float const CAM_TILT_HOME = 8500;
float const CAM_PAN_HOME = 10400;

//maximum changes in tilt and pan angles of tower camera from home position
float const CAM_TILT_MAX_DELTA = 15000;
float const CAM_PAN_MAX_DELTA = 10400;

//minimum and maximum angles for camera tilt and pan
float const CAM_MIN = 2400;
float const CAM_MAX = 17200;

using std::cout;
using std::endl;

struct Point {
	float x;
	float y;
	
	Point(float x, float y) {
		this->x = x;
		this->y = y;
	}
};

//a positive only version of the modulus function
//returns positive remainder of x/y
float mod(float const x, float const y)
{
	if(x < 0.0f)
	{
		return fmodf(x, y) + y;
	}
	else
	{
		return fmodf(x, y);
	}
}

//applies an exponential curve to x or outputs zero if within deadzone
float joystickSmoothing(float x)
{
	float const c = 4.0f;
	return (powf(expf(JOYSTICK_CURVE_COEFFICIENT) + 1.0f, fabsf(x)) - 1.0f) /
		expf(JOYSTICK_CURVE_COEFFICIENT);
}

//cyclic pattern alternating between 1 and -1 with period of 4*PI
float h(float const x)
{
	return (mod(x + M_PI + M_PI/4.0f, 2.0f*M_PI) - 
		mod(x + M_PI/4.0f, 2.0f*M_PI)) / M_PI;
}

//A curve following a triangle wave pattern with a period of 4*PI
float g(float const x)
{
	return (mod(x + M_PI/4.0f, M_PI) * 4.0f/M_PI - 2.0f) * h(x);
}

//if side == false then velocity will be returned for left drive train
//v is the vector whose y axis determines velocity and 
//x axis determines angular velocity
float getVelocity(Point v, bool side)
{
	//v magnitude
	float vMagnitude = 
		sqrt(v.x*v.x + v.y*v.y);
		
	//angle of vector v
	float const vAngle = 
		copysignf(acosf(v.x/vMagnitude), v.y);
	
	//limit vector magnitude between deadzone and 1.0
	if(vMagnitude > 1.0f)
		vMagnitude = 1.0f;
	else if(vMagnitude < JOYSTICK_DEADZONE)
		return 0.0f;
		
	float b = 7.0f*M_PI/4.0f;
	if(side)
	{
		b *= -1.0f;
	}
	
	//calculate maximum magnitude for side
	float const g1 = g(vAngle - b);
	float const g2 = g(vAngle - b - M_PI/2.0f);
	float const maxMagnitude = (g1 - g2) / 2.0f;
	
	//velocity for selected side
	float const sideMagnitude = maxMagnitude * vMagnitude;
	
	cout << "side" << (int)side << "Angle: " << vAngle << endl <<
		"side" << (int)side << "Magnitude: " << sideMagnitude << endl;
	
	return sideMagnitude;
}

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
			
			msg.controlMode = "basic_controller";
			
			msg.ljx = 0.0;
			msg.ljy = 0.0;
			msg.rjx = 0.0;
			msg.rjy = 0.0;
			
			//camera control message
			rover_pkg::CameraControlData ccMsg;
			
			ccMsg.QDC_SIG1 = 0;
			ccMsg.tool_select = 0;
			
			Point leftStick = Point(0.0f, 0.0f);
			Point rightStick = Point(0.0f, 0.0f);
			
			//Event polling loop
			while(isRunning && ros::ok())
			{
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
				float lStickMagnitude = 
					sqrt(leftStick.x*leftStick.x + leftStick.y*leftStick.y);
				
				//set camera tower orientation when joystick is beyond deadzone
				if(lStickMagnitude > JOYSTICK_DEADZONE)
				{
					ccMsg.ctt = -leftStick.x*CAM_TILT_MAX_DELTA + CAM_TILT_HOME;
					if(ccMsg.ctt < CAM_MIN)
					{
						ccMsg.ctt = CAM_MIN;
					} else if (ccMsg.ctt > CAM_MAX)
					{
						ccMsg.ctt = CAM_MAX;
					}
					ccMsg.ctp = -leftStick.y*CAM_PAN_MAX_DELTA + CAM_PAN_HOME;
					if(ccMsg.ctp < CAM_MIN)
					{
						ccMsg.ctp = CAM_MIN;
					} else if (ccMsg.ctp > CAM_MAX)
					{
						ccMsg.ctp = CAM_MAX;
					}
				}
				//Set camera tower to home position
				else
				{
					ccMsg.ctt = CAM_TILT_HOME;
					ccMsg.ctp = CAM_PAN_HOME;
				}
				
				//Set drivetrain velocity
				msg.ljy = getVelocity(rightStick, false);
				msg.rjy = getVelocity(rightStick, true);
				
				
				cout << "x1: " << leftStick.x << endl <<
					"y1: " << leftStick.y << endl <<
					"x2: " << rightStick.x << endl <<
					"y2: " << rightStick.y << endl <<
					"l1: " << lStickMagnitude << "\n\n";
					
				
				
				
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
