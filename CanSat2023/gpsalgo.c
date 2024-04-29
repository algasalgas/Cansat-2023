/*
 * gpsalgo.c
 *
 *  Created on: May 19, 2023
 *      Author: aldar
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#define RESERVE 0
int counter1 = 0;
int kolcbuff = 0;
int iteratorstate = 0;
uint8_t flashBlock[252] = {};
uint32_t counterPage = 0;
uint8_t counterPackInPage = 0;
int16_t AccData[3] = {};
int16_t GyroData[3] = {};
int16_t MagData[3] = {};
int32_t b;
uint8_t acknowledge = 0;

uint8_t tempp[4];
uint8_t tempp1[4];
uint8_t tempp2[4];
uint8_t tempp3[4];
uint8_t tempp4[4];
int32_t longitude;
int32_t latitude;
uint8_t appendbyte;
uint8_t packet[53];
uint8_t firstsend = 1;
uint32_t valueADC[5] = {0};
int gga = 0;
int k = 0;
int zapyat = 0;
char NMEA[246] = {};
char GGA[80] = {};
char GGAhelp[80] = {};
char str[512] = {};
uint8_t bufferia[512] = {};
iter1 = 0;
iter2 = 0;
struct GPGGA_Struct         // Структура �? данными GPS-ГЛО�?�?СС-модул�?
{
  	  char Time [sizeof("hhmmss.ss")+RESERVE];      // Врем�?
  	  char Latitude [sizeof("xxxx.yyyyyy")+RESERVE];    // Широта
  	  char NS[sizeof("N")+RESERVE];                            // Север-Юг
  	  char Longitude[sizeof("xxxxx.yyyyyy")+RESERVE]; // Долгота
  	  char WE[sizeof("W")+RESERVE];                            // Запад-Во�?ток
  	  char Qual[sizeof("1")+RESERVE];               // Режим работы приемника
  	  char Sats[sizeof("nn")+RESERVE];     // Количе�?тво �?путников в решении
  	  char Alt[sizeof("aaaaa.a")+RESERVE];    // Высота
  	  char Hdop[sizeof("xxx")+RESERVE];          // Horizontal dilution of precision
  	  char Units[sizeof("m")+RESERVE];         // Размерность высоты (m - meters)
};
struct GPGGA_Struct GPGGA = {
		"hhmmss.ss",
		"1111.111111",
		"S",
		"11111.111111",
		"W",
		"1",
		"nn",
		"aaaaa.a",
		"xxx",
		"m"
};
void NMEA_Handler(){
	if (iter1){
	    // Обработка оставшегося пакета
	    strncpy(GGA, GGAhelp, iter1);
	    for (int b = 0; b < 55-iter1+2; b++){
	        GGA[iter1+b-1] = NMEA[b];
	    }
	    gga = 1;
	    iter1 = 0;
	}
	if (!gga)
	for (int i = 0; i < 246; i++){
		if (counter1 == 6)
		{ // $xxGGA пойман
				if ((245 - i) > 55){
				    counter1 = 0;
					for (int b = 0; b < 55; b++){
						GGA[b] = NMEA[b+i+1];
					}
					gga = 1;
					iter1 = 0;
					break;
				} else {
					iter1 = 245 - i + 1;
					iteratorstate = 55-(245-iter1);
					for (int b = 0; b < 246-i; b++){
						GGA[b] = NMEA[i+b+1];
					}
					strncpy(GGAhelp, GGA, iter1);
					counter1 = 0;
                    break;
				}
		}
		if (counter1 == 0){
		    if (NMEA[i] == '$'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != '$'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 1) {counter1++; continue;}
		if (counter1 == 2) {counter1++; continue;}
		if (counter1 == 3){
		    if (NMEA[i] == 'G'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 4){
		    if (NMEA[i] == 'G'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'G'){
		        counter1 = 0;
		        continue;
		    }
		}
		if (counter1 == 5){
		    if (NMEA[i] == 'A'){
			    counter1++;
			    continue;
		    } else if (NMEA[i] != 'A'){
		        counter1 = 0;
		        continue;
		    }
		}
	}
	if (gga){
		// 9 ","
		uint8_t zapyat = 0;
		uint8_t k = 0;
		for (int i = 0; i < 56; i++){
			if (zapyat > 8){
				break;
			}
			if (GGA[i] == ','){
				zapyat += 1;
				k = 0;
				continue;
			}
			else{
				if (zapyat == 0 && k < sizeof("hhmmss.ss")+RESERVE-1){ GPGGA.Time[k] = GGA[i]; k++;}
				if (zapyat == 1 && k < sizeof("xxxx.yyyyyy")+RESERVE-1){ GPGGA.Latitude[k] = GGA[i]; k++;}
				if (zapyat == 2 && k < sizeof("N")+RESERVE-1){ GPGGA.NS[k] = GGA[i]; k++;}
				if (zapyat == 3 && k < sizeof("xxxxx.yyyyyy")+RESERVE-1){ GPGGA.Longitude[k] = GGA[i]; k++;}
				if (zapyat == 4 && k < sizeof("W")+RESERVE-1){ GPGGA.WE[k] = GGA[i]; k++;}
				if (zapyat == 5 && k < sizeof("1")+RESERVE-1){ GPGGA.Qual[k] = GGA[i]; k++;}
				if (zapyat == 6 && k < sizeof("nn")+RESERVE-1){ GPGGA.Sats[k] = GGA[i]; k++;}
				if (zapyat == 7 && k < sizeof("xxx")+RESERVE-1){ GPGGA.Hdop[k] = GGA[i]; k++;}
				if (zapyat == 8 && k < sizeof("aaaa.aa")+RESERVE-1){ GPGGA.Alt[k] = GGA[i]; k++;}
				if (zapyat == 9 && k < sizeof("m")+RESERVE-1){ GPGGA.Units[k] = GGA[i]; k++;}
			}
		}
		gga = 0;
		k = 0;
		zapyat = 0;
	}

}
int main()
{
    strcpy(NMEA,"$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*$GNGSA,A,3,19,,,,,,,,,,,,4.8,2.0,4.3,4*dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddff$GNGGA,102030.00,5546.959003,N,03740.692003,");
    NMEA_Handler();
    printf("%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
    strcpy(NMEA,"E,1,08,2.0,1424.00,M,0.0,M,,*$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*$GNGSA,A,3,19,  ,  ,  ,  ,  ,,,,,,,4.8,2.0,4.3,4*dddddddddff$GNGGA,102030.000,5546.95900,N,03740.69200,E,1,08,2.0,142.0,M,0.0,M,,*");
    NMEA_Handler();
    printf("%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
    strcpy(NMEA,"$GNGGA,102530.00,5546.75900,N,03740.39200,E,1,08,2.0,1452.00,M,0.0,M,,*$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
    NMEA_Handler();
    printf("%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
        strcpy(NMEA,"$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*$GNGSA,A,3,19,,,,,,,,,,,,4.8,2.0,4.3,4*dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddff$GNGGA,102030.00,5546.959003,N,03740.692003,");
    NMEA_Handler();
    printf("%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
    strcpy(NMEA,"E,1,08,2.0,1424.00,M,0.0,M,,*$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*$GNGSA,A,3,19,  ,  ,  ,  ,  ,,,,,,,4.8,2.0,4.3,4*dddddddddff$GNGGA,102030.000,5546.95900,N,03740.69200,E,1,08,2.0,142.0,M,0.0,M,,*");
    NMEA_Handler();
    printf("%s;%s;%s;%s;%s;%s;%s\n", GPGGA.Time, GPGGA.Latitude, GPGGA.NS, GPGGA.Longitude, GPGGA.WE, GPGGA.Sats, GPGGA.Alt);
    strcpy(NMEA,"$GNGGA,102530.00,5546.75900,N,03740.39200,E,1,08,2.0,1452.00,M,0.0,M,,*$GNGLL,5546.95900,N,03740.69200,E,102030.000,A,A*$GNGSA,A,3,10,16,18,20,26,27,,,,,,,4.8,2.0,4.3,1*dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd");
    return 0;
}


