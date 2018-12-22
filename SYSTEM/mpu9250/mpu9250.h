#ifndef MPU9250_H
#define MPU9250_H

#include "main.h"
#include "JY901.h"

typedef struct
	{
	
	float stcGyroX;
	float stcGyroY;
	float stcGyroZ;
	
	}mpu9250_t;
	

void CopeSerial2Data(unsigned char ucData);
void mpu9250_assignment(void);

#endif

