#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <MPU6000IMU.h>

#include <math.h>
#include <iostream>
#include <stdio.h>
#include <sys/ioctl.h>

MPU6000IMU::MPU6000IMU(int bus, int address){
	I2CBus = bus;
	I2CAddress = address;
}

MPU6000IMU::~MPU6000IMU(){
	close(file);
}

void MPU6000IMU::initModule(){
	if(initBus(I2CBus, I2CAddress) != 0){
		printf("failed to set up connection with sensor\n");
	}
	else{
		configMPU(0x00);
		configGyro(0x00);
		configAcc(0x00);

		setInterrupts(0x00);
		setUserControl(0x00);
		setPowerManagement(0x00);
		printf("set up all configurations on sensor module, ready to read... \n");
	}
}

int MPU6000IMU::initBus(int bus, int address){
	char namebuf[64];
		snprintf(namebuf, sizeof(namebuf), "/dev/i2c-%d", I2CBus);
	if((file = open(namebuf, O_RDWR)) < 0){
		printf("failed to open MPU6000 sensor on %s I2C bus \n", namebuf);
		return(1);
	}
	if(ioctl(file, I2C_SLAVE, I2CAddress) < 0){
		printf("I2C_SLAVE address %s failed... \n", I2CAddress);
		return(2);
	}
	return(0);
}

void MPU6000IMU::configMPU(char value){
	writeData(CONFIG, value);
}

void MPU6000IMU::configGyro(char value){
	writeData(GYRO_CONFIG, value);
}

void MPU6000IMU::configAcc(char value){
	writeData(ACCEL_CONFIG, value);
}

void MPU6000IMU::setInterrupts(char value){
	writeData(INT_ENABLE, value);
}

void MPU6000IMU::setUserControl(char value){
	writeData(USER_CTRL, value);
}

void MPU6000IMU::setPowerManagement(char value){
	writeData(PWR_MGNT_1, value);
}

void MPU6000IMU::writeData(char reg, char value){
	char buffer[2];
	buffer[0] = reg;
	buffer[1] = value;
	if(write(file, buffer, 2) != 2){
		printf("failed to write data to sensor \n");
	}
}

void MPU6000IMU::getData(short *ACCx, short *ACCy, short *ACCz, short *GYRx, short *GYRy, short *GYRz, short *temp){
	char buffer[14];

	//ACCEL_XOUT_H, first register in list, will read 14 following registers
	buffer[0] = 0x3B;
	write(file, buffer, 1);
	if(read(file, buffer, 14) != 14){
		printf("failed to read data from sensor\n");
		return;
	}
	*ACCx = (buffer[0] << 8) | buffer[1];
	*ACCy = (buffer[2] << 8) | buffer[3];
	*ACCz = (buffer[4] << 8) | buffer[5];

	*temp = (buffer[6] << 8) | buffer[7];

	*GYRx = (buffer[8] << 8) | buffer[9];
	*GYRy = (buffer[10] << 8) | buffer[11];
	*GYRz = (buffer[12] << 8) | buffer[13];
}

