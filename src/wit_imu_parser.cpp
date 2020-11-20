/*
Copyright (c) 2020 Balamurugan Kandan
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "wit_imu_parser.h"

CJY901 ::CJY901() : isMagEnabled(false), isPrsEnabled(false) {
	stcTime={0};
	stcAcc={0};
	stcGyro={0};
	stcAngle={0};
	stcMag={0};
	stcDStatus={0};
	stcDStatus={0};
	stcPress={0};
	stcLonLat={0};
	stcGPSV={0};
}

void CJY901::FetchData(unsigned char *data, int usLength) {
	unsigned char *ucRxBuffer = data;
	short temp1 = 0, temp2 = 0, temp3 = 0;
	while (usLength >= 11) {
		if (ucRxBuffer[0] != 0x55) {
			ucRxBuffer++;
			continue;
		}
		switch (ucRxBuffer[1]) {
			case 0x50: 
				stcTime.ucYear 		= ucRxBuffer[2];
				stcTime.ucMonth 	= ucRxBuffer[3];
				stcTime.ucDay 		= ucRxBuffer[4];
				stcTime.ucHour 		= ucRxBuffer[5];
				stcTime.ucMinute 	= ucRxBuffer[6];
				stcTime.ucSecond 	= ucRxBuffer[7];
				stcTime.usMiliSecond=((char)ucRxBuffer[9]<<8)|ucRxBuffer[8];
				break;
			case 0x51: // Accelation in ms^2
				stcAcc.a[0] = ((((char)ucRxBuffer[3]<<8)|ucRxBuffer[2]) / 32768.00) * 16 * 9.8;
				stcAcc.a[1] = ((((char)ucRxBuffer[5]<<8)|ucRxBuffer[4]) / 32768.00) * 16 * 9.8;
				stcAcc.a[2] = ((((char)ucRxBuffer[7]<<8)|ucRxBuffer[6]) / 32768.00) * 16 * 9.8;
				stcAcc.T = (((char)ucRxBuffer[9]<<8)|ucRxBuffer[8]) / 100.0;
				//printf("Accl : %f %f %f %f\n", stcAcc.a[0], stcAcc.a[1], stcAcc.a[2], stcAcc.T);
				break;
			case 0x52:	// Gyroscope values in Radians
				stcGyro.w[0] = ((((char)ucRxBuffer[3]<<8)|ucRxBuffer[2]) / 32768.00 * 2000) / 180 * 3.1415926;
				stcGyro.w[1] = ((((char)ucRxBuffer[5]<<8)|ucRxBuffer[4]) / 32768.00 * 2000) / 180 * 3.1415926;
				stcGyro.w[2] = ((((char)ucRxBuffer[7]<<8)|ucRxBuffer[6]) / 32768.00 * 2000) / 180 * 3.1415926;
				stcGyro.T = (((char)ucRxBuffer[9]<<8)|ucRxBuffer[8]) / 100.0;
				//printf("Gyro : %f %f %f %f\n", stcGyro.w[0], stcGyro.w[1], stcGyro.w[2], stcGyro.T);
				break;
			case 0x53:	// Angle
				stcAngle.Angle[0] = (ucRxBuffer[3]<<8)|ucRxBuffer[2];
				stcAngle.Angle[1] = (ucRxBuffer[5]<<8)|ucRxBuffer[4];
				stcAngle.Angle[2] = (ucRxBuffer[7]<<8)|ucRxBuffer[6];
				stcAngle.T = ((ucRxBuffer[9]<<8)|ucRxBuffer[8]) / 100.0;
				break;
			case 0x54: // Magnetometer values in micro-tesla
				stcMag.h[0] = (((char)ucRxBuffer[3]<<8)|ucRxBuffer[2]) * 0.015;
				stcMag.h[1] = (((char)ucRxBuffer[5]<<8)|ucRxBuffer[4]) * 0.015;
				stcMag.h[2] = (((char)ucRxBuffer[7]<<8)|ucRxBuffer[6]) * 0.015;
				stcMag.T = ((ucRxBuffer[9]<<8)|ucRxBuffer[8]) / 100.0;
				//printf("Magt : %f %f %f %f\n", stcMag.h[0], stcMag.h[1], stcMag.h[2], stcMag.T);
				isMagEnabled = true;
				break;
			//case 0x55:	// Ports
			//	stcDStatus.sDStatus[0] = ((char)ucRxBuffer[3]<<8)|ucRxBuffer[2];
			//	stcDStatus.sDStatus[1] = ((char)ucRxBuffer[5]<<8)|ucRxBuffer[4];
			//	stcDStatus.sDStatus[2] = ((char)ucRxBuffer[7]<<8)|ucRxBuffer[6];
			//	stcDStatus.sDStatus[3] = ((char)ucRxBuffer[9]<<8)|ucRxBuffer[8];
			//	break;
			case 0x56:	// Altitude and Pressure
				CharToLong(&stcPress.lPressure,(unsigned char*)&ucRxBuffer[2]);
				CharToLong(&stcPress.lAltitude,(unsigned char*)&ucRxBuffer[6]);
				isPrsEnabled = true;
				break;
			//case 0x57:	// GPS Lat / Long
			//	CharToLong(&stcLonLat.lLon,(unsigned char*)&ucRxBuffer[2]);
			//	CharToLong(&stcLonLat.lLat,(unsigned char*)&ucRxBuffer[6]);
			//	break;
			//case 0x58:	// Altimeter
			//	stcGPSV.sGPSHeight = ((char)ucRxBuffer[3]<<8)|ucRxBuffer[2];
			//	stcGPSV.sGPSYaw = ((char)ucRxBuffer[5]<<8)|ucRxBuffer[4];
			//	CharToLong(&stcGPSV.lGPSVelocity,(unsigned char*)&ucRxBuffer[6]);
			//	break;
			case 0x59: // Quaternion (unit less)
				stcQuat.x = (((char)ucRxBuffer[3]<<8)|ucRxBuffer[2]) / 32768.00;
				stcQuat.y = (((char)ucRxBuffer[5]<<8)|ucRxBuffer[4]) / 32768.00;
				stcQuat.z = (((char)ucRxBuffer[7]<<8)|ucRxBuffer[6]) / 32768.00;
				stcQuat.w = (((char)ucRxBuffer[9]<<8)|ucRxBuffer[8]) / 32768.00;
				//printf("Quat : %f %f %f %f\n", stcQuat.x, stcQuat.y, stcQuat.z, stcQuat.w);
				break;
		}
		usLength -= 11;
		ucRxBuffer += 11;
	}
}

void CJY901::CharToLong(long *value, unsigned char Source[]) {
	*value = ((unsigned short)Source[3]<<24 | (unsigned short)Source[2]<<16 |
			  (unsigned short)Source[1]<<8 | (unsigned short)Source[0]);
}
