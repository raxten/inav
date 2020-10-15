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
* Date Created: July 1, 2019												 *
* Date Modified: July 1, 2019												 *
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
#define FREQ 480 /*loop frequency*/


/*constant definitions*/
static serialPort_t *testPort = NULL;
static uint8_t GUSTreceiveBuf[MAX_READBUF_SIZE];
static int GUSTReceivePos = 0;
extern int16_t pilotMessage[MAX_SUPPORTED_RC_CHANNEL_COUNT];
float time = 0;

/*message structure definitions*/
#define QUAD_SAS_MESSAGE_SYNC 0xa1
struct quadSasMessageHeader_ref {
	unsigned char sync;// = 0xa1;// :;
	unsigned char messageID;// = 0;// :id #;
	unsigned char messageSize;// = 0;// :including header;
};

#define QUAD_SAS_MESSAGE_PWM_VALUES 2
struct quadSasMessagePwmValues_ref {
	unsigned char sync;// = 0xa1;
	unsigned char messageID;// = 0;// :id #;
	unsigned char messageSize;// = 0;// :including header;
	unsigned char count;// = 0;

	short pwm[7];// = { 0,0,0,0,0,0,0 };// :;

	unsigned short csum;// = 0;
};

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

void sendMessage(float *rateData, float *accData, uint32_t currentTimeUs) {

	/*The purpose of this function is to take the current IMU data and package it up into something that will be successively
	 * written, once per pid loop, to create and send the IMU message to GUST. */

	DEBUG_SET(DEBUG_NONE, 0, currentTimeUs);
	timeAM += 1;

	//if (timeAM % 0.041188f == 0) {
		//timeAM -= 0.041188f;
	if(0 == (timeAM%FREQ)) {

		//UPDATED IMU MESSAGE STRUCTURE
#define QUAD_SAS_MESSAGE_IMU 1
		struct quadSasMessageIMU_ref {
			unsigned char sync;
			unsigned char messageID;
			unsigned char messageSize;
			unsigned char count;
			short gyro[3];
			short accl[3];
			unsigned short csum;
		} quadSasMessageIMU;
		struct quadSasMessageIMU_ref *quadIm = &quadSasMessageIMU;
		quadIm->sync = 0xa1;

#define sfr (0.05) //(0.05/180.0*3.1416)
#define sfa (3.333/1000.0*1880.0) 
 
		/*/* encode message */
		quadIm->messageID = (unsigned char)QUAD_SAS_MESSAGE_IMU;
		quadIm->messageSize = (unsigned char)sizeof(struct quadSasMessageIMU_ref);
		quadIm->gyro[0] = (short)(+rateData[0] / sfr + 0.5);
		quadIm->gyro[1] = (short)(-rateData[1] / sfr + 0.5);
		quadIm->gyro[2] = (short)(-rateData[2] / sfr + 0.5);
		quadIm->accl[0] = (short)(-accData[0] / sfa + 0.5);
		quadIm->accl[1] = (short)(+accData[1] / sfa + 0.5);
		quadIm->accl[2] = (short)(+accData[2] / sfa + 0.5);
		quadIm->csum = quadSasCheckSumFletcher16((unsigned char *)quadIm, (short)(quadIm->messageSize - sizeof(unsigned short)));

		/* send message */
		serialWriteBuf(testPort, (unsigned char *)quadIm, quadIm->messageSize);
	}
	/*trying to send a tenth as often as the IMU data, roughly 10Hz*/
	if(17 == (timeAM%FREQ*10)) { /*17 so the messages don't overlap*/
		struct quadSasMessagePwmValues_ref quadSasMessagePilot;
		struct quadSasMessagePwmValues_ref *pilotCommand = &quadSasMessagePilot;

		/*encode message*/
		pilotCommand->sync = 0xa1;
		pilotCommand->messageID = (unsigned char)QUAD_SAS_MESSAGE_PWM_VALUES;
		pilotCommand->messageSize = (unsigned char)sizeof(struct quadSasMessagePwmValues_ref);// = 0;// :including header;
		//unsigned char count;// = 0;
		for(int i=0; i < 7; i++) {
			pilotCommand->pwm[i] = (short)pilotMessage[i];
		}
		pilotCommand->csum = quadSasCheckSumFletcher16((unsigned char *)pilotCommand, (short)(pilotCommand->messageSize - sizeof(unsigned short)));

		/*send message*/
		serialWriteBuf(testPort,(unsigned char *)pilotCommand, pilotCommand->messageSize);
	}

}

/*Call this function to reset the serial receive buffer.*/
void resetBuffer(void) {
	GUSTReceivePos = 0;
}


/**
 * @brief datalinkCheckSumCompute() calculates and returns the checksum of a
 * character buffer
 *
 * Calculates the checksum of a character buffer using a 32-bit Fletcher
 * checksum. Handles an odd number of bytes by calculating the checksum as if
 * there were an additional zero-byte appended to the end of the data.
 *
 * @param buf pointer to a character buffer array
 * @param byteCount the size in bytes of the character buffer
 */
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


static void filterPacket(uint8_t *buffer) {
	struct quadSasMessageHeader_ref headerVals;
	struct quadSasMessagePwmValues_ref MESSAGE_PWM;	

	//DEBUG_SET(DEBUG_NONE, 0, 444);//Means the function was called.
	struct quadSasMessageHeader_ref      *header = &headerVals;//Set the header pointer equal to allocated memory.
	struct quadSasMessagePwmValues_ref *motorPWM = &MESSAGE_PWM;//Same as above.
	int index, done, loc;
	unsigned char *bf;


	/*change all port->bytesread to (int)(MAX_RECBUF_SIZE*sizeof(uint8))*/
	/*change all port->buffer[] calls to buffer[]*/

	int bytesread = (int)(MAX_READBUF_SIZE * sizeof(uint8_t));

	//if (port->dataSource == PORT_OFF) return;

	done = 0;
	index = 0;
	loc = 0;

	while ((index <= bytesread - (int)sizeof(struct quadSasMessageHeader_ref)) && !done) {
		//DEBUG_SET(DEBUG_NONE, 1, 0);
		//DEBUG_SET(DEBUG_NONE, 2, 0);
		//DEBUG_SET(DEBUG_NONE, 3, 0);
		if ((buffer[index] == QUAD_SAS_MESSAGE_SYNC)) {

			bf = &(buffer[index]);

			memcpy(header, bf, sizeof(struct quadSasMessageHeader_ref));
			//DEBUG_SET(DEBUG_NONE, 1, header->messageID);

			//Next step is to see what happens when the header contains a 2 byte value. Will everything just "work"?
			//DEBUG_SET(DEBUG_NONE, 0, sizeof(struct quadSasMessageHeader_ref));
			/*if (datalinkCheckSumCompute(bf, sizeof(struct quadSasMessageHeader_ref) - sizeof(int) * 2) == header->hcsum &&
				header->messageSize >= sizeof(struct quadSasMessageHeader_ref) &&
				header->messageSize < MAX_READBUF_SIZE)*/

			if (header->messageSize >= sizeof(struct quadSasMessageHeader_ref) &&
				header->messageSize < MAX_READBUF_SIZE) {

				//DEBUG_SET(DEBUG_NONE, 2, 222);//Means we past the checksum test.
				if (header->messageSize + index <= bytesread) {
					/* have read in the entire message */
					//DEBUG_SET(DEBUG_NONE, 2, header->messageSize);//We got to this point.
					/*((struct quadSasMessageHeader_ref *)bf)->hcsum = 0;*/
					//if (datalinkCheckSumCompute(&bf[sizeof(struct quadSasMessageHeader_ref)], header->messageSize - sizeof(struct quadSasMessageHeader_ref)) == header->csum) {
					if(1) {
						switch (header->messageID) {

							case QUAD_SAS_MESSAGE_PWM_VALUES:
								//AUTO = 1;
								loc = index + (int)sizeof(struct quadSasMessageHeader_ref);
								bf = &(buffer[loc]);/*Move the bf pointer to the beginning of the data message*/
								memcpy(motorPWM, bf, sizeof(struct quadSasMessagePwmValues_ref));								//Try sending back an IMU command when a PWM command is received.
							break;

							//case QUAD_SAS_MESSAGE_PILOT_LINK:

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
	//DEBUG_SET(DEBUG_NONE, 3, 123);
	//clearPort(port, index);

}

// returns completed response code or 0, JAKES ALTERED FUNCTION
static char fillReadBuffer(uint32_t currentTimeUs)
{
	UNUSED(currentTimeUs);

	if (!testPort) {
		return 0;
	}
	//DEBUG_SET(DEBUG_NONE, 1, 222);//Got to this point.
	//DEBUG_SET(DEBUG_NONE, 1, GUSTReceivePos);
	//DEBUG_SET(DEBUG_NONE, 2, serialRxBytesWaiting(testPort));
	DEBUG_SET(DEBUG_NONE, 1, rcData[ROLL]);
	DEBUG_SET(DEBUG_NONE, 2, rcData[PITCH]);
	DEBUG_SET(DEBUG_NONE, 3, rcData[YAW]);
	while (serialRxBytesWaiting(testPort) && (GUSTReceivePos < MAX_READBUF_SIZE) ) {
		uint8_t c = serialRead(testPort);
		//DEBUG_SET(DEBUG_NONE, 0, c);
		GUSTreceiveBuf[GUSTReceivePos++] = c;
		
		if (GUSTReceivePos == (MAX_READBUF_SIZE - 1)) {
			/*Call some function to handle the received packet.*/
			//DEBUG_SET(DEBUG_NONE, 0, 111);//We called the function
			filterPacket(GUSTreceiveBuf);
			resetBuffer();
		}

	}

	return 0;//If there aren’t any bytes waiting, don’t keep the function running.
}
