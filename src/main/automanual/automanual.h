#pragma once

#include <stdlib.h>
#include <math.h>
#include "io/serial.h"
#include "flight/mixer.h"
#include "rx/rx.h"
#include "drivers/serial.h"
#include "build/debug.h"
#include <string.h>

/*****************************************************************************
* AUTO MANUAL HEADER FILE													 *
* Author: Jacob Crouse														 *
* Date Created: March 15, 2019												 *
* Date Modified: June 20, 2019												 *
*																			 *
* Purpose: This code is used to augment the preexisting betaflight			 *
* software to support sending the gyro and acceleration data				 *
* over a serial port, and receive motor PWMs from an external				 *
* but onboard autopilot software. To use these motor PWMs, this code		 *
* will implement an Auto/Manual switch which allows the piloting transmitter *
* to select whether the multirotor is being flown by the autonomously by     *
* the autopilot software or manually by the pilot.                           *
******************************************************************************/

#define MAX_READBUF_SIZE 1000
#define QUAD_SAS_MESSAGE_SYNC 0xa1
#define QUAD_SAS_MESSAGE_IMU 1
#define QUAD_SAS_MESSAGE_PWM_VALUES 2

/*function definitions*/
void initAHPort(void);
unsigned short quadSasCheckSumFletcher16(unsigned char *data, short len);
void sendMessage(float *rateData, float *accData, uint32_t currentTimeUs);
void resetBuffer(void);
unsigned int datalinkCheckSumCompute(unsigned char *buf, int byteCount);
void filterPacket(uint8_t *buffer);
char fillReadBuffer(uint32_t currentTimeUs);

/*message structure definitions*/
struct quadSasMessageHeader_ref {
	unsigned char sync;// = 0xa1;// :;
	unsigned char messageID;// = 0;// :id #;
	unsigned char messageSize;// = 0;// :including header;
};

struct quadSasMessagePwmValues_ref {
	unsigned char sync;// = 0xa1;
	unsigned char messageID;// = 0;// :id #;
	unsigned char messageSize;// = 0;// :including header;
	unsigned char count;// = 0;

	short pwm[7];// = { 0,0,0,0,0,0,0 };// :;

	unsigned short csum;// = 0;
};

struct quadSasMessageIMU_ref {
	unsigned char sync;
	unsigned char messageID;
	unsigned char messageSize;
	unsigned char count;
	short gyro[3];
	short accl[3];
signed short csum;
};

