





#include <assert.h>

#include <ros/ros.h>
#include <MotionSensorDriver.h>

#include <math.h>


using namespace std;
using namespace DemconRobot;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "MotionSensorDriver");

	MotionSensorDriver driver;
	ros::NodeHandle n;

	if(driver.start()!= 0){
		exit(-1);
	}


	ros::Rate loop_rate(20); //10hz


	while(n.ok()){
		driver.doUpdate();
		loop_rate.sleep();
		ros::spinOnce();
	}

	driver.stop();

	return(0);


}
