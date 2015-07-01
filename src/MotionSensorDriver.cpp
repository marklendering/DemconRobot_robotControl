
#include <MotionSensorDriver.h>

using namespace std;

namespace DemconRobot
{
	MotionSensorDriver::MotionSensorDriver()
	{
		canDriver = new CanDriver();

		speedR_set = false;
		speedL_set = false;
		DISTR_set = false;
		DISTL_set = false;

		deltaDistanceRight = 0;
		deltaDistanceLeft = 0;
		totalDistanceRight = 0;
		totalDistanceLeft = 0;
		speedRight = 0.0;
		speedLeft = 0.0;

		//advertise wheel (current)velocities and distances
		WheelVelocities_pub = node_.advertise<beaglebone::WheelVelocities>("Wheel_Current_Velocities", 100);
		WheelDistances_pub = node_.advertise<beaglebone::WheelDistances>("Wheel_Delta_Distance", 100);

	}

	MotionSensorDriver::~MotionSensorDriver()
	{
		delete canDriver;
	}

	int MotionSensorDriver::start()
	{
		//open canbus then subscribe to wheel set velocities
		if(!canDriver->open_Bus((char*)"can1"))
		{
			return 1;
		}
		WheelVelocities_sub = node_.subscribe<beaglebone::WheelVelocities>("Wheel_Set_Velocity", 100, boost::bind(&MotionSensorDriver::setSpeed, this, _1));
		initRobot();
		return 0;
	}

	int MotionSensorDriver::stop()
	{
		return 0;
	}

	void MotionSensorDriver::doUpdate()
	{
		//get delta distance from wheels
		requestCANData('D', ALLMBEDS);
		getCanData();
		getCanData();
	}

	void MotionSensorDriver::requestCANData(int canId)
	{
		//request data from devices on canbus
		char buf[8];
		canDriver->write_Bus(buf, 0, canId);
	}

	void MotionSensorDriver::requestCANData(char command, int canId)
	{
		char buf[8];
		//set buffer to command settings
		buf[0] = '?';
		buf[1] = command;
		canDriver->write_Bus(buf, 2, canId);
	}


	void MotionSensorDriver::getCanData()
	{
		//read data from canbus and handle accordingly
		char readBuffer[8];
		int length = 0;
		int canId= 0;
		if(canDriver->read_Bus(readBuffer, length, canId))
		{
			switch(canId)
			{
				//motor distance left or right
				case ANS_DISTR: case ANS_DISTL:
					getMotorDistance(readBuffer, length, canId);
					break;
				//motor speed
				case BEAGLEBONE:
					getSpeed(readBuffer, length, canId);
					break;
				//default fail
				default:
					printf("incorrect canId: %d \r \n", canId);
					break;
			}
		}
	}


	void MotionSensorDriver::getMotorDistance(char readBuffer[8], int length, int canId)
	{
		//check if package is correct length
		if(length >= 5)
		{
			//convert data from buffer to integer type, so it can be transformed to a floating point value later one
			converter.Int = ((readBuffer[1]<<0) + (readBuffer[2]<<8) + (readBuffer[3]<<16) + (readBuffer[4]<<24));
			if(canId == ANS_DISTR)
			{
				DISTR_set = true;
				deltaDistanceRight = converter.Float;
			}
			else if(canId == ANS_DISTL)
			{
				DISTL_set = true;
				deltaDistanceLeft = converter.Float;
			}

			if(DISTR_set && DISTL_set)
			{
				beaglebone::WheelDistances msg;
				//~4% offset in measurement data and real world data. Add this to offset to correct values
				msg.right = deltaDistanceRight * 1.04;
				msg.left = deltaDistanceLeft * 1.04;
				WheelDistances_pub.publish(msg);
				DISTR_set = false;
				DISTL_set = false;
			}
		}
	}

	bool MotionSensorDriver::getSpeed(char readBuffer[8], int length, int canId)
	{
		char int_buffer[4];
		if(length >=5)
		{
			int_buffer[0] = readBuffer[1];
			int_buffer[1] = readBuffer[2];
			int_buffer[2] = readBuffer[3];
			int_buffer[3] = readBuffer[4];
			converter.Int = ((int_buffer[0] << 0) + (int_buffer[1] << 8) + (int_buffer[2] << 16) + (int_buffer[3] << 24));
			if(readBuffer[0] == MCRIGHT)
			{
				speedR_set = true;
				speedRight = converter.Float;
			}
			else if(readBuffer[0] == MCLEFT)
			{
				speedL_set = true;
				speedLeft = converter.Float;
				//set bool and variable
			}
			if(speedR_set && speedL_set)
			{
				speedL_set = false;
				speedR_set = false;
				beaglebone::WheelVelocities msg;
				msg.right = speedRight * TFACTOR;
				msg.left = speedLeft * TFACTOR;
				WheelVelocities_pub.publish(msg);
			}
		}
		return false;
	}

	void MotionSensorDriver::setSpeed(const beaglebone::WheelVelocities::ConstPtr& WheelVelocity)
	{
		//TODO: not very efficient
		char buffer[6];
		//left motor
		buffer[0]=COMMAND_WRITE;
		buffer[1]=COMMAND_SET_VELOCITY;

		converter.Float = WheelVelocity -> left;
		buffer[2] = ((converter.Int >> 0) & 0xff);
		buffer[3] = ((converter.Int >> 8) & 0xff);
		buffer[4] = ((converter.Int >> 16) & 0xff);
		buffer[5] = ((converter.Int >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, MCLEFT);

		//right motor
		converter.Float = WheelVelocity -> right;
		buffer[2] = ((converter.Int >> 0) & 0xff);
		buffer[3] = ((converter.Int >> 8) & 0xff);
		buffer[4] = ((converter.Int >> 16) & 0xff);
		buffer[5] = ((converter.Int >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, MCRIGHT);

		//set robot state to start
		setRobotState(START);

	}

	bool MotionSensorDriver::initRobot()
	{
		setRobotState(CONFIGURE);
		setRobotMode(1);
	}

	void MotionSensorDriver::setRobotMode(int mode)
	{
		char buffer[6];
		buffer[0] = '!';
		buffer[1] = 'm';
		buffer[2] = ((mode >> 0) & 0xff);
		buffer[3] = ((mode >> 8) & 0xff);
		buffer[4] = ((mode >> 16) & 0xff);
		buffer[5] = ((mode >> 24) & 0xff);
		canDriver->write_Bus(buffer, 6, ALLMBEDS); 
	}

	void MotionSensorDriver::setRobotState(int state)
	{
		char buffer[2];
		buffer[0] = '!';
		switch(state)
		{
			case CONFIGURE:
				buffer[1] = 'c';
				break;
			case START:
				buffer[1] = 's';
				break;

			case STOP:
				buffer[1] = 't';
				break;

			case RESET:
				buffer[1] = 'r';
				break;

			default:
				buffer[1] = 't';
				break;
		}
		canDriver->write_Bus(buffer, 2, ALLMBEDS);

	}
}
