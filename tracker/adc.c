#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <errno.h>
#include <wiringPiSPI.h>
#include <gertboard.h>
#include "hab.h"


void bmp085Calibration();
double bmp085GetTemperature(void);
double bmp085GetPressure(double Temperature);
short bmp085ReadInt(unsigned char address);
unsigned short bmp085ReadUT();
double bmp085ReadUP();


short ac1;
short ac2; 
short ac3; 
unsigned short ac4;
unsigned short ac5;
unsigned short ac6;
short B1; 
short B2;
short Mb;
short Mc;
short Md;
short fd;
// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
// int b5; 

struct HAB *HAB;

int AnalogRead (int chan)
{
  unsigned char spiData [3] ;
  unsigned char chanBits ;

  chanBits = 0xC0 | ((chan & 7) << 3);

  spiData [0] = chanBits ;
  spiData [1] = 0;
  spiData[2] = 0;

  wiringPiSPIDataRW (0, spiData, 3) ;

  return ((spiData[0] & 1) << 9) | (spiData[1] << 1) | (spiData[2] >> 7);
}

double GetVoltage(int chan, double FullScale)
{
	int RawValue, i;
        double Voltage;

        Voltage = 0;

        for (i=0; i<1000; i++)
        {
        	RawValue = AnalogRead(chan);
                Voltage += (double)RawValue * FullScale / (1024.0 * 1000.0);
        }

	return Voltage;
}

int main(int argc, char **argv)
{
	int id;
	key_t key;
	float InternalTemperature, Pressure, ExternalTemperature, GPUTemperature, BatteryVoltage;
	FILE *fp;

	printf("**** Sensor program ****\n");

        key = ftok("/home/pi/key",3);
        id = shmget(key, 300, IPC_CREAT);

        if (id == -1)
        {
                printf("shmget failed\n");
                exit(1);
        }

        printf("Identifier = %d\n", id);

        HAB = (struct HAB *)shmat(id, NULL, 0);

        if ((char *)HAB == (char *)(-1))
        {
                printf("shmat failed with error %d\n", errno);
                exit(1);
        }

        printf("Opening SPI ...\n");

        if (gertboardSPISetup () < 0)
        {
                printf("Failed to setup SPI\n");
        }

		
	while (1)
	{
		char line[128];
		long temp;

		// Battery voltage
        printf("Reading ADC ...\n");
		BatteryVoltage = GetVoltage(2, 4.0 * 3.3);
		printf("Battery Voltage = %5.2lf\n", BatteryVoltage);

		// Lock memory

		// Fill in values
		HAB->InternalTemperature = InternalTemperature;
		HAB->Pressure = Pressure;
		HAB->ExternalTemperature = ExternalTemperature;
		HAB->GPUTemperature = GPUTemperature;
		HAB->BatteryVoltage = BatteryVoltage;

		// Unlock

		// Now wait a while
		sleep(1);
	}

	return 0;
}

// Stores all of the bmp085's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  B1 = bmp085ReadInt(0xB6);
  B2 = bmp085ReadInt(0xB8);
  Mb = bmp085ReadInt(0xBA);
  Mc = bmp085ReadInt(0xBC);
  Md = bmp085ReadInt(0xBE);

printf ("Values are %d %d %d %u %u %u %d %d %d %d %d\n", ac1, ac2, ac3, ac4, ac5, ac6, B1, B2, Mb, Mc, Md);
}

// Calculate temperature given ut.
// Value returned will be in units of 0.1 deg C
double bmp085GetTemperature(void)
{
  double c5, alpha, mc, md;
	unsigned short ut;

	ut = bmp085ReadUT();
	printf("ut = %u\n", ut);

	c5 = (double)ac5 / (32768 * 160);
	printf ("c5 = %lf\n", c5);

	alpha = c5 * (double)(ut - ac6);
	printf("alpha = %lf\n", alpha);

	mc = 2048 * Mc / (160 * 160);
	printf("mc = %lf\n", mc);

	md = (double)Md / 160;
	printf("md = %lf\n", md);

	return alpha + mc / (alpha + md);
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
double bmp085GetPressure(double Temperature)
{
	double Pu, s, x0, x1, x2, c3, c4, b1, y0, y1, y2, p0, p1, p2, x, y, z, P;

	Pu = bmp085ReadUP();
	printf("Pu = %lf\n", Pu);

	s = Temperature - 25;
printf("s = %lf\n", s);
	x0 = ac1;
printf("x0 = %lf\n", x0);
	x1 = (double)ac2 * 160 / 8192;
printf("x1 = %lf\n", x1);
	x2 = (double)B2 * 160 * 160 / 33554432;
printf("x2 = %lf\n", x2);
	c3 = (double)ac3 * 160 / 32768;
	c4 = (double)ac4 / 32768000;
	b1 = (double)B1 * 160 * 160 /1073741824;
	y0 = c4 * 32768;
	y1 = c4 * c3;
	y2 = c4 * b1;
printf("y0=%lf, y1=%lf, y2=%lf\n", y0, y1, y2);
	p0 = (3791.0 - 8.0) / 1600.0;
	p1 = 1.0 - 7357.0 / 1048576.0;
	p2 = 303800.0 / 68719476740.0;
printf("p0=%lf, p1=%lf, p2=%lf\n", p0, p1, p2);
	x = x2 * s * s + x1 * s + x0;
printf("x = %lf\n", x);
	y = y2 * s * s + y1 * s + y0;
printf("y = %lf\n", y);
	z = (Pu - x) / y;
	P = p2 * z * z + p1 * z + p0;
printf("z=%lf, P=%lf\n", z, P);


	return P;
/*
  unsigned long up;
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  up = bmp085ReadUP();
  printf("up = %lu\n", up);

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (B2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = ((((long)ac1)*4 + x3) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (B1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * 50000);
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
*/
}

// Read 1 byte from the BMP085 at 'address'
unsigned char bmp085Read(unsigned char address)
{
	unsigned char buf[10];
 /* 
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;
    
  return Wire.receive();
*/
	buf[0] = address;
	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	if (read(fd, buf, 1) != 1) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}

	printf ("BMP085 address %d gives %02X\n", address, buf[0]);

	return buf[0];
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
short bmp085ReadInt(unsigned char address)
{
  unsigned char buf[10];
 /* 
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(address);
  Wire.endTransmission();
  
  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  
  return (int) msb<<8 | lsb;
*/

	buf[0] = address;

	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 2) != 2) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}

// printf("Address %d gives %02X %02X = %d\n", address, buf[0], buf[1], (short)(buf[0]<<8 | buf[1]));
	return (short) buf[0]<<8 | buf[1];
}

// Read the uncompensated temperature value
unsigned short bmp085ReadUT()
{
 	unsigned short ut;
	unsigned char buf[10];
  
  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
/*
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x2E);
  Wire.endTransmission();
  
  // Wait at least 4.5ms
  usleep(5000);
  
  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);


  return ut;
*/

	buf[0] = 0xF4;
	buf[1] = 0x2E;

	if ((write(fd, buf, 2)) != 2) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(5000);
	
	ut = bmp085ReadInt(0xF6);

	// printf("ut = %u\n", ut);

	return ut;
}

// Read the uncompensated pressure value
double bmp085ReadUP()
{
  unsigned char msb, lsb, xlsb;
  double up;
	unsigned char buf[10];
  
  // Write 0x34 into register 0xF4
  // Request a pressure reading w/ oversampling setting
/*
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF4);
  Wire.send(0x34);
  Wire.endTransmission();
  
  // Wait for conversion
  usleep(2000);
  
  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.send(0xF6);
  Wire.endTransmission();
  Wire.requestFrom(BMP085_ADDRESS, 3);
  
  // Wait for data to become available
  while(Wire.available() < 3)
    ;
  msb = Wire.receive();
  lsb = Wire.receive();
  xlsb = Wire.receive();
  
  up = (double)msb * 65536 + (double)lsb * 256 + (double)xlsb) / 256;
  
  return up;
*/
	buf[0] = 0xF4;
	buf[1] = 0x34;

	if ((write(fd, buf, 2)) != 2) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}

	usleep(3000);

	buf[0] = 0xF6;

	if ((write(fd, buf, 1)) != 1) {								// Send register we want to read from	
		printf("Error writing to i2c slave\n");
		exit(1);
	}
	
	if (read(fd, buf, 3) != 3) {								// Read back data into buf[]
		printf("Unable to read from slave\n");
		exit(1);
	}

	msb = buf[0];
	lsb = buf[1];
	xlsb = buf[2];

	// printf("Pressure values are %02X %02X %02X\n", msb, lsb, xlsb);

	up = (double)msb * 256 + (double)lsb + (double)xlsb / 256;


	return up;
}


