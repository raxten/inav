//#pragma once
#include <stdlib.h>
#include <math.h>
#include "io/serial.h"
#include "flight/mixer.h"
#include "rx/rx.h"
#include "drivers/serial.h"
#include "build/debug.h"
#include <string.h>
#include "drivers/accgyro/accgyro.h"
#include "sensors/acceleration.h"
#include "automanual/automanual.h"

/*****************************************************************************
* AUTO MANUAL HEADER FILE													 *
* Author: Jacob Crouse														 *
* Date Created:  June 20, 2019												 *
* Date Modified: July 11, 2019												 *
*																			 *
* Purpose: This code is used to augment the preexisting betaflight			 *
* software to support sending the gyro and acceleration data				 *
* over a serial port, and receive motor PWMs from an external				 *
* but onboard autopilot software. To use these motor PWMs, this code		 *
* will implement an Auto/Manual switch which allows the piloting transmitter *
* to select whether the multirotor is being flown by the autonomously by     *
* the autopilot software or manually by the pilot.                           *
******************************************************************************/

#define MAX_READBUF_SIZE 100
#define FREQ 500 /*send loop frequency, the same as the PID loop frequency*/


/*global constant definitions*/
static serialPort_t *testPort = NULL;
//static int blah = 0;
//static int blah2 = 0;
//static int blah3 = 0;
static uint8_t GUSTreceiveBuf[MAX_READBUF_SIZE];
static int GUSTReceivePos = 0;
int AUTO = 0;
struct quadSasMessageHeader_ref headerVals;
struct quadSasMessagePwmValues_ref MESSAGE_PWM;	
struct quadSasMessageHeader_ref *header = &headerVals;
struct quadSasMessagePwmValues_ref *motorPWM = &MESSAGE_PWM;
extern int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
extern int16_t pilotInput[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float time = 0;

//Create a function that defines the testPort for use.
void initAHPort(void) {
	//testPort = openSerialPort(SERIAL_PORT_USART6, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, 0);
	testPort = openSerialPort(SERIAL_PORT_USART6, FUNCTION_NONE, NULL, NULL, 115200, MODE_RXTX, SERIAL_INVERTED);//Works with GUST. Inverted to handle the inversion by the usb-serial converter used for testing.
}

unsigned short quadSasCheckSumFletcher16(unsigned char *data, short len) {

	unsigned short check;
	unsigned short sum1 = 0xff, sum2 = 0xff;

	while (len) {
		short tlen = len > 21 ? 21 : len;
		len -= tlen;

		do {
			sum1 += *data++;
			sum2 += sum1;
		} while (--tlen);

		sum1 = (sum1 & 0xff) + (sum1 >> 8);
		sum2 = (sum2 & 0xff) + (sum2 >> 8);
	}

	// Second reduction step to reduce sums to 8 bits
	sum1 = (sum1 & 0xff) + (sum1 >> 8);
	sum2 = (sum2 & 0xff) + (sum2 >> 8);

	check = (unsigned short)(sum1 << 8) | (unsigned short)(sum2);

	return check;
}

long long timeAM;
int avgCntr;
float rateAvg[3], accAvg[3];

void sendMessage(float *rateData, float *accData, uint32_t currentTimeUs) {

	/*The purpose of this function is to take the current IMU data and package it up into something that will be successively
	 * written, once per pid loop, to create and send the IMU message to GUST. */
	timeAM += 1;
	avgCntr += 1;

	/*sum the IMU data until it is ready to be sent.*/
	for (int i = 0; i < 3; i++) {
		rateAvg[i] += rateData[i];
		accAvg[i] += accData[i];
	}

	/*100Hz stuff*/
	if (0 == (timeAM % (FREQ / 100))) {

		/*compute the average*/
		for (int i = 0; i < 3; i++) {
			rateAvg[i] /= avgCntr;
			accAvg[i] /= avgCntr;
		}

		struct quadSasMessageIMU_ref quadSasMessageIMU;
		struct quadSasMessageIMU_ref *quadIm = &quadSasMessageIMU;
		quadIm->sync = 0xa1;

#define sfr 0.01745*(1.0/8.7266667e-05) //converting deg/sec to rad/sec, accounting for scale factor in GUST
#define sfa 32.174*(1.0/0.010724667) //converting g to ft/s^2, accounting for scale factor in GUST
#define minPWM 1160
#define maxPWM 1841

		/* encode message */
		quadIm->messageID = (unsigned char)QUAD_SAS_MESSAGE_IMU;
		quadIm->messageSize = (unsigned char)sizeof(struct quadSasMessageIMU_ref);

		/*gyro data*/
		quadIm->gyro[0] = (short)(+rateAvg[0] * sfr);
		quadIm->gyro[1] = (short)(-rateAvg[1] * sfr);
		quadIm->gyro[2] = (short)(-rateAvg[2] * sfr);

		/*acceleration data*/
		quadIm->accl[0] = (short)( +(float)accAvg[0] * acc.dev.acc_1G_rec * sfa);
		quadIm->accl[1] = (short)( -(float)accAvg[1] * acc.dev.acc_1G_rec * sfa);
		quadIm->accl[2] = (short)( -(float)accAvg[2] * acc.dev.acc_1G_rec * sfa);

		quadIm->csum = quadSasCheckSumFletcher16((unsigned char *)quadIm, (short)(quadIm->messageSize - sizeof(unsigned short)));

		/* send message */
		serialWriteBuf(testPort, (unsigned char *)quadIm, quadIm->messageSize);

		/*reset the averaging of the IMU values*/
		avgCntr = 0;
		for (int j = 0; j < 3; j++) {
			rateAvg[j] = 0;
			accAvg[j] = 0;
		}

	}
	/*10Hz stuff*/
	if((FREQ/20) == (timeAM%(FREQ/10))) { /*FREQ/20 so the messages don't overlap*/
	//if(!timeAM) {

		struct quadSasMessagePwmValues_ref quadSasMessagePilot;
		struct quadSasMessagePwmValues_ref *pilotCommand = &quadSasMessagePilot;

		/*encode message*/
		pilotCommand->sync = 0xa1;
		pilotCommand->messageID = (unsigned char)QUAD_SAS_MESSAGE_PWM_VALUES;
		pilotCommand->messageSize = (unsigned char)sizeof(struct quadSasMessagePwmValues_ref);// = 0;// :including header;

		//pilotCommand->pwm[0] = (short)(minPWM + (maxPWM - pilotInput[0])); /*added to swap the direction*/
		pilotCommand->pwm[0] = (short)pilotInput[0]; /*added to swap the direction*/
		//pilotCommand->pwm[1] = (short)(minPWM + (maxPWM - pilotInput[1]));
		pilotCommand->pwm[1] = (short)pilotInput[1];
		//pilotCommand->pwm[2] = (short)(minPWM + (maxPWM - pilotInput[2]));
		pilotCommand->pwm[2] = (short)(pilotInput[2]);
		pilotCommand->pwm[3] = (short)pilotInput[3];
		pilotCommand->pwm[4] = (short)pilotInput[4];
		pilotCommand->pwm[5] = (short)pilotInput[5];
		pilotCommand->pwm[6] = (short)pilotInput[6];

		pilotCommand->csum = quadSasCheckSumFletcher16((unsigned char *)pilotCommand, (short)(pilotCommand->messageSize - sizeof(unsigned short)));

		/*send message*/
		serialWriteBuf(testPort,(unsigned char *)pilotCommand, pilotCommand->messageSize);
	}

}

/*Call this function to reset the serial receive buffer.*/
void resetBuffer(void) {
	GUSTReceivePos = 0;
	
}


unsigned int datalinkCheckSumCompute(unsigned char *buf, int byteCount) {

	unsigned int sum1 = 0xffff;
	unsigned int sum2 = 0xffff;
	unsigned int tlen = 0;
	unsigned int shortCount = byteCount / sizeof(short);
	unsigned int oddLength = byteCount % 2;

	/* this is Fletcher32 checksum modified to handle buffers with an odd number of bytes */

	while (shortCount) {
		/* 360 is the largest number of sums that can be performed without overflow */
		tlen = shortCount > 360 ? 360 : shortCount;
		shortCount -= tlen;
		do {
			sum1 += *buf++;
			sum1 += (*buf++ << 8);
			sum2 += sum1;
		} while (--tlen);

		/* add last byte if there's an odd number of bytes (equivalent to appending a zero-byte) */
		if ((oddLength == 1) && (shortCount < 1)) {
			sum1 += *buf++;
			sum2 += sum1;
		}

		sum1 = (sum1 & 0xffff) + (sum1 >> 16);
		sum2 = (sum2 & 0xffff) + (sum2 >> 16);
	}

	/* Second reduction step to reduce sums to 16 bits */
	sum1 = (sum1 & 0xffff) + (sum1 >> 16);
	sum2 = (sum2 & 0xffff) + (sum2 >> 16);

	return(sum2 << 16 | sum1);
}


void filterPacket(uint8_t *buffer) {
	int index, done, loc;
	unsigned char *bf;

	//int bytesread = (int)(MAX_READBUF_SIZE * sizeof(uint8_t));
	int bytesread = GUSTReceivePos; /* save the number of bytes read when the function was called */
	done = 0;
	index = 0;
	loc = 0;

	while ((index <= bytesread - (int)sizeof(struct quadSasMessageHeader_ref)) && !done) {

		if ((buffer[index] == QUAD_SAS_MESSAGE_SYNC)) {

			bf = &(buffer[index]);
			memcpy(header, bf, sizeof(struct quadSasMessageHeader_ref));
			if (header->messageSize >= sizeof(struct quadSasMessageHeader_ref) &&
				header->messageSize < MAX_READBUF_SIZE) {

				if (header->messageSize + index <= bytesread) {
					/* have read in the entire message */
					//DEBUG_SET(DEBUG_NONE, 2, header->messageSize);//We got to this point.
					/*((struct quadSasMessageHeader_ref *)bf)->hcsum = 0;*/
					//if (datalinkCheckSumCompute(&bf[sizeof(struct quadSasMessageHeader_ref)], header->messageSize - sizeof(struct quadSasMessageHeader_ref)) == header->csum) {
					if(1) {
						switch (header->messageID) {

							case QUAD_SAS_MESSAGE_PWM_VALUES:
								//DEBUG_SET(DEBUG_NONE, 2, blah2);
								//blah2++;
								loc = index;
								bf = &(buffer[loc]);/*Move the bf pointer to the beginning of the data message*/
								memcpy(motorPWM, bf, sizeof(struct quadSasMessagePwmValues_ref));
								/*checking to see if the system should be in auto mode*/
								/*DEBUG_SET(DEBUG_NONE, 0, pilotInput[0]);
								DEBUG_SET(DEBUG_NONE, 1, pilotInput[1]);
								DEBUG_SET(DEBUG_NONE, 2, pilotInput[2]);
								DEBUG_SET(DEBUG_NONE, 3, pilotInput[3]);*/
								/*DEBUG_SET(DEBUG_NONE, 0, motorPWM->pwm[0]);
								DEBUG_SET(DEBUG_NONE, 1, motorPWM->pwm[1]);
								DEBUG_SET(DEBUG_NONE, 2, motorPWM->pwm[2]);
								DEBUG_SET(DEBUG_NONE, 3, motorPWM->pwm[3]);*/
								if ( (pilotInput[4] > 1600) ) { /*assuming AUX1 is auto/manual switch*/
									AUTO = 1; /*auto mode*/
								}
								else {
									AUTO = 0; /*manual mode*/
								}
								//DEBUG_SET(DEBUG_NONE, 0, AUTO);
								//DEBUG_SET(DEBUG_NONE, 1, AUTO);
								//DEBUG_SET(DEBUG_NONE, 2, AUTO);
								//DEBUG_SET(DEBUG_NONE, 3, AUTO);

							break;

						default:
							/* unrecognized type */
							break;
						}

						//data->work->itime++;
						//*rxTime = nav->out->time;
					}
					else { /* checksum bad */
					 //data->work->badChecksums++;
					}
					index += header->messageSize - 1;

				}
				else {
					index--;
					done = 1;
				}
			}
			else { /* header checksum is bad */
				index += sizeof(struct quadSasMessageHeader_ref) - 1;
				//data->work->badHeaderChecksums++;
			}
		}
		index++; /* start seq not found, go to next byte */

		if (index < 0) index = MAX_READBUF_SIZE - 1;
	}
	//DEBUG_SET(DEBUG_NONE, 1, index);
	//DEBUG_SET(DEBUG_NONE, 2, GUSTReceivePos);
	/*if ((index >= (MAX_READBUF_SIZE - (int)sizeof(struct quadSasMessagePwmValues_ref))) || (GUSTReceivePos >= (MAX_READBUF_SIZE - 2))) {
		resetBuffer();
		DEBUG_SET(DEBUG_NONE, 0, blah);
		blah++;
		done = 1;
		//break;
	}*/
	//DEBUG_SET(DEBUG_NONE, 0, blah);
	//blah++;
	//DEBUG_SET(DEBUG_NONE, 1, index);
	//DEBUG_SET(DEBUG_NONE, 2, GUSTReceivePos);
	//if (GUSTReceivePos >= MAX_READBUF_SIZE - 5) {
	//	resetBuffer();
	//	index = 0;
	//}
	resetBuffer();
	//DEBUG_SET(DEBUG_NONE, 3, 123);
	//clearPort(port, index);


}

// returns completed response code or 0, JAKES ALTERED FUNCTION
char fillReadBuffer(uint32_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	uint8_t c;

	if (!testPort) {
		return 0;
	}
	
	while (serialRxBytesWaiting(testPort) && (GUSTReceivePos < MAX_READBUF_SIZE) ) {
		c = serialRead(testPort);
		//DEBUG_SET(DEBUG_NONE, 3, blah3);
		//blah3++;
		GUSTreceiveBuf[GUSTReceivePos++] = c;
		
		/* previously filterpacket was only called once the buffer was filled, but I'm just going to continuously call it, so we always get the most up-to-date data. */
		filterPacket(GUSTreceiveBuf);


		/*if (GUSTReceivePos == (MAX_READBUF_SIZE - 1)) {
			filterPacket(GUSTreceiveBuf);
			resetBuffer();
		}*/

	}

	return 0;//If there aren’t any bytes waiting, don’t keep the function running.
}
