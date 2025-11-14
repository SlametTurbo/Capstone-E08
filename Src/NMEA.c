/*
 * NMEA.c
 *
 *  Created on: 25-Feb-2022
 *      Author: controllerstech.com
 */


#include "NMEA.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"


int GMT = +530;



int inx = 0;
int hr=0,min=0,day=0,mon=0,yr=0;
int daychange = 0;

/* Decodes the GGA Data
   @GGAbuffer is the buffer which stores the GGA Data
   @GGASTRUCT is the pointer to the GGA Structure (in the GPS Structure)
   @Returns 0 on success
   @ returns 1, 2 depending on where the return statement is excuted, check function for more details
*/

int decodeGGA (char *GGAbuffer, GGASTRUCT *gga)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (GGAbuffer[inx] != ',') inx++;  // 1st ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // After time ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after latitude ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after NS ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after longitude ','
	inx++;
	while (GGAbuffer[inx] != ',') inx++;  // after EW ','
	inx++;  // reached the character to identify the fix
	if ((GGAbuffer[inx] == '1') || (GGAbuffer[inx] == '2') || (GGAbuffer[inx] == '6'))   // 0 indicates no fix yet
	{
		gga->isfixValid = 1;   // fix available
		inx = 0;   // reset the index. We will start from the inx=0 and extract information now
	}
	else
	{
		gga->isfixValid = 0;   // If the fix is not available
		return 1;  // return error
	}
	while (GGAbuffer[inx] != ',') inx++;  // 1st ','


/*********************** Get TIME ***************************/
//(Update the GMT Offset at the top of this file)

	inx++;   // reach the first number in time
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')  // copy upto the we reach the after time ','
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}

	hr = (atoi(buffer)/10000) + GMT/100;   // get the hours from the 6 digit number

	min = ((atoi(buffer)/100)%100) + GMT%100;  // get the minutes from the 6 digit number

	// adjust time.. This part still needs to be tested
	if (min > 59) 
	{
		min = min-60;
		hr++;
	}
	if (hr<0)
	{
		hr=24+hr;
		daychange--;
	}
	if (hr>=24)
	{
		hr=hr-24;
		daychange++;
	}

	// Store the time in the GGA structure
	gga->tim.hour = hr;
	gga->tim.min = min;
	gga->tim.sec = atoi(buffer)%100;

///***************** Get LATITUDE  **********************/
//	inx++;   // Reach the first number in the lattitude
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')   // copy upto the we reach the after lattitude ','
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	if (strlen(buffer) < 6) return 2;  // If the buffer length is not appropriate, return error
//	int16_t num = (atoi(buffer));   // change the buffer to the number. It will only convert upto decimal
//	int j = 0;
//	while (buffer[j] != '.') j++;   // Figure out how many digits before the decimal
//	j++;
//	int declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
//	int dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
//	float lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
//	gga->lcation.latitude = lat;  // save the lattitude data into the strucure
//	inx++;
//	gga->lcation.NS = GGAbuffer[inx];  // save the N/S into the structure

	/***************** Get LATITUDE  **********************/
	inx++;   // ke digit pertama latitude
	memset(buffer, 0, sizeof(buffer));
	i = 0;
	while (GGAbuffer[inx] != ',' && i < (int)sizeof(buffer)-1) {
	    buffer[i++] = GGAbuffer[inx++];
	}
	buffer[i] = '\0';
	if (i < 4) return 2;  // sanity check

	// konversi ddmm.mmmm -> derajat desimal
	float raw_lat = strtof(buffer, NULL);          // contoh: 0745.94648
	int   deg_lat = (int)(raw_lat / 100.0f);       // 07
	float min_lat = raw_lat - (deg_lat * 100.0f);  // 45.94648
	gga->lcation.latitude = deg_lat + (min_lat / 60.0f);

	inx++;                          // sekarang di N/S
	gga->lcation.NS = GGAbuffer[inx];
	if (gga->lcation.NS == 'S')     // beri tanda negatif untuk Selatan
	    gga->lcation.latitude = -gga->lcation.latitude;


///***********************  GET LONGITUDE **********************/
//	inx++;  // ',' after NS character
//	inx++;  // Reach the first number in the longitude
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')  // copy upto the we reach the after longitude ','
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	num = (atoi(buffer));  // change the buffer to the number. It will only convert upto decimal
//	j = 0;
//	while (buffer[j] != '.') j++;  // Figure out how many digits before the decimal
//	j++;
//	declen = (strlen(buffer))-j;  // calculate the number of digit after decimal
//	dec = atoi ((char *) buffer+j);  // conver the decimal part a a separate number
//	lat = (num/100.0) + (dec/pow(10, (declen+2)));  // 1234.56789 = 12.3456789
//	gga->lcation.longitude = lat;  // save the longitude data into the strucure
//	inx++;
//	gga->lcation.EW = GGAbuffer[inx];  // save the E/W into the structure

	/***********************  GET LONGITUDE **********************/
	inx++;  // lewati ',' setelah N/S
	inx++;  // ke digit pertama longitude
	memset(buffer, 0, sizeof(buffer));
	i = 0;
	while (GGAbuffer[inx] != ',' && i < (int)sizeof(buffer)-1) {
	    buffer[i++] = GGAbuffer[inx++];
	}
	buffer[i] = '\0';
	if (i < 5) return 3;  // sanity check (lon biasanya dddmm.mmmm)

	// konversi dddmm.mmmm -> derajat desimal
	float raw_lon = strtof(buffer, NULL);          // contoh: 11022.33525
	int   deg_lon = (int)(raw_lon / 100.0f);       // 110
	float min_lon = raw_lon - (deg_lon * 100.0f);  // 22.33525
	gga->lcation.longitude = deg_lon + (min_lon / 60.0f);

	inx++;                         // sekarang di E/W
	gga->lcation.EW = GGAbuffer[inx];
	if (gga->lcation.EW == 'W')    // beri tanda negatif untuk Barat
	    gga->lcation.longitude = -gga->lcation.longitude;

/**************************************************/
	// skip positition fix
	inx++;   // ',' after E/W
	inx++;   // position fix
	inx++;   // ',' after position fix;

	// number of sattelites
	inx++;  // Reach the first number in the satellites
	memset(buffer, '\0', 12);
	i=0;
	while (GGAbuffer[inx] != ',')  // copy upto the ',' after number of satellites
	{
		buffer[i] = GGAbuffer[inx];
		i++;
		inx++;
	}
	gga->numofsat = atoi(buffer);   // convert the buffer to number and save into the structure


	/***************** baca HDOP  *********************/
	inx++;  // pindah ke karakter pertama field HDOP
	memset(buffer, '\0', sizeof(buffer));
	i = 0;
	while (GGAbuffer[inx] != ',' && GGAbuffer[inx] != '*' && i < (int)sizeof(buffer)-1)
	{
	    buffer[i++] = GGAbuffer[inx++];
	}
	buffer[i] = '\0';

	// simpan; jika kosong jadikan -1.0f biar ketahuan "tidak tersedia"
	gga->hdop = (i > 0) ? strtof(buffer, NULL) : -1.0f;



//	/*************** Altitude calculation ********************/
//	inx++;
//	memset(buffer, '\0', 12);
//	i=0;
//	while (GGAbuffer[inx] != ',')
//	{
//		buffer[i] = GGAbuffer[inx];
//		i++;
//		inx++;
//	}
//	num = (atoi(buffer));
//	j = 0;
//	while (buffer[j] != '.') j++;
//	j++;
//	declen = (strlen(buffer))-j;
//	dec = atoi ((char *) buffer+j);
//	lat = (num) + (dec/pow(10, (declen)));
//	gga->alt.altitude = lat;
//
//	inx++;
//	gga->alt.unit = GGAbuffer[inx];

	/*************** Altitude calculation (simplified) ********************/
	inx++;  // pindah ke karakter pertama altitude
	memset(buffer, 0, sizeof(buffer));
	i = 0;
	while (GGAbuffer[inx] != ',' && GGAbuffer[inx] != '*' && i < (int)sizeof(buffer)-1) {
	    buffer[i++] = GGAbuffer[inx++];
	}
	buffer[i] = '\0';

	gga->alt.altitude = (i > 0) ? strtof(buffer, NULL) : 0.0f;

	inx++;  // sekarang pada unit 'M'
	gga->alt.unit = GGAbuffer[inx];

	return 0;

}


int decodeRMC (char *RMCbuffer, RMCSTRUCT *rmc)
{
	inx = 0;
	char buffer[12];
	int i = 0;
	while (RMCbuffer[inx] != ',') inx++;  // 1st ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // After time ,
	inx++;
	if (RMCbuffer[inx] == 'A')  // Here 'A' Indicates the data is valid, and 'V' indicates invalid data
	{
		rmc->isValid = 1;
	}
	else
	{
		rmc->isValid =0;
		return 1;
	}
	inx++;
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after latitude,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after NS ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after longitude ,
	inx++;
	while (RMCbuffer[inx] != ',') inx++;  // after EW ,

	// Get Speed
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){          // if the speed have some data
		int16_t num = (atoi(buffer));  // convert the data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->speed = lat;
	}
	else rmc->speed = 0;

	// Get Course
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	if (strlen (buffer) > 0){  // if the course have some data
		int16_t num = (atoi(buffer));   // convert the course data into the number
		int j = 0;
		while (buffer[j] != '.') j++;   // same as above
		j++;
		int declen = (strlen(buffer))-j;
		int dec = atoi ((char *) buffer+j);
		float lat = num + (dec/pow(10, (declen)));
		rmc->course = lat;
	}
	else
		{
			rmc->course = 0;
		}

	// Get Date
	inx++;
	i=0;
	memset(buffer, '\0', 12);
	while (RMCbuffer[inx] != ',')
	{
		buffer[i] = RMCbuffer[inx];
		i++;
		inx++;
	}

	// Date in the format 280222
	day = atoi(buffer)/10000;  // extract 28
	mon = (atoi(buffer)/100)%100;  // extract 02
	yr = atoi(buffer)%100;  // extract 22

	day = day+daychange;   // correction due to GMT shift

	// save the data into the structure
	rmc->date.Day = day;
	rmc->date.Mon = mon;
	rmc->date.Yr = yr;

	return 0;
}

