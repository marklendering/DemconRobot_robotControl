#ifndef MPU6000IMU_H_
#define MPU6000IMU_H_

#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define INT_ENABLE 0x38
#define USER_CTRL 0x6A
#define PWR_MGNT_1 0x6B

class MPU6000IMU
{
private:
	int I2CBus, I2CAddress, file;

	int initBus(int bus, int address);

	void configMPU(char value);
	void configGyro(char value);
	void configAcc(char value);

	void setInterrupts(char value);
	void setUserControl(char value);
	void setPowerManagement(char value);

	void writeData(char reg, char value);

public:
	 MPU6000IMU(int bus, int address);
	~ MPU6000IMU();

	void initModule();
	void getData(short *ACCx, short *ACCy, short *ACCz, short *GYRx, short *GYRy, short *GYRz, short *temp);

};


#endif
