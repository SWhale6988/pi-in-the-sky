/*------------------------------------------------------------------\
/*------------------------------------------------------------------\
|                                                                   |
|                PI IN THE SKY TRACKER PROGRAM                      |
|                                                                   |
| This program is written for the Pi Telemetry Board                |
| produced by HAB Supplies Ltd.  No support is provided             |
| for use on other hardware. It does the following:                 |
|                                                                   |
| 1 - Sets up the hardware including putting the GPS in flight mode |
| 2 - Reads the current temperature                                 |
| 3 - Reads the current battery voltage                             |
| 4 - Reads the current GPS position                                |
| 5 - Builds a telemetry sentence to transmit                       |
| 6 - Converts that sentence to a sound file (WAV)                  |
| 7 - Plays that sound file                                         |
| 8 - repeats steps 2-7                                             |
|                                                                   |
|                                                                   |
|                                                                   |
\------------------------------------------------------------------*/

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <ctype.h>
#include <stdio.h>   	// Standard input/output definitions
#include <string.h>  	// String function definitions
#include <unistd.h>  	// UNIX standard function definitions
#include <fcntl.h>   	// File control definitions
#include <errno.h>   	// Error number definitions
#include <termios.h> 	// POSIX terminal control definitions
#include <stdint.h>
#include <stdlib.h>
#include <dirent.h>
#include <signal.h>
#include <math.h>
#include <pthread.h>
#include <wiringPi.h>

// Telemetry settings
#define PAYLOAD_ID	"PISKY"	// *** SET THIS TO YOUR OWN PAYLOAD ID BEFORE FLIGHT ***
#define RTTY_BAUD 600

// Pin allocations
#define NTX2B_ENABLE	0
#define UBLOX_ENABLE	2
#define LED_WARN		4
#define LED_OK			11


FILE *file;
int Records, FileNumber;
struct termios options;

char Hex[] = "0123456789ABCDEF"; 

struct TGPS
{
	float Time;
	float Longitude, Latitude;
	unsigned int Altitude;
	unsigned int Satellites;
	int Speed;
	int Direction;
} GPS;


void EscapeCharacters(char* source, char*target)
{
    for (; *source; source++)
	{
		if (*source == '$')
		{
			*target++ = '\\';
			*target++ = '$';
		}
		else if (*source == '\r')
		{
			*target++ = '\\';
			*target++ = 'r';
		}
		else if (*source == '\n')
		{
			*target++ = '\\';
			*target++ = 'n';
		}
		else
		{
			*target++ = *source;
		}
		
    }
	
	*target = '\0';
}

int GPSChecksumOK(unsigned char *Buffer, int Count)
{
  unsigned char XOR, i, c;
  
  XOR = 0;
  for (i = 1; i < (Count-4); i++)
  {
    c = Buffer[i];
    XOR ^= c;
  }
  
  return (Buffer[Count-4] == '*') && (Buffer[Count-3] == Hex[XOR >> 4]) && (Buffer[Count-2] == Hex[XOR & 15]);
}

float FixPosition(float Position)
{
	float Minutes, Seconds;
	
	Position = Position / 100;
	
	Minutes = trunc(Position);
	Seconds = fmod(Position, 1);
	
	return Minutes + Seconds * 5 / 3;
}

void ProcessLine(struct TGPS *GPS, char *Buffer, int *GotGGA, int *GotRMC)
{
    float utc_time, latitude, longitude, hdop, altitude, speed, course;
	int lock, satellites, date;
	char ns, ew, units;
	
	satellites = 0;
	
    if (sscanf(Buffer, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &latitude, &ns, &longitude, &ew, &lock, &satellites, &hdop, &altitude, &units) >= 1)
	{
	printf("%s\n", Buffer);
		// $GPGGA,124943.00,5157.01557,N,00232.66381,W,1,09,1.01,149.3,M,48.6,M,,*42
		*GotGGA = 1;
		if (satellites >= 4)
		{
			GPS->Time = utc_time;
			GPS->Latitude = FixPosition(latitude);
			if (ns == 'S') GPS->Latitude = -GPS->Latitude;
			GPS->Longitude = FixPosition(longitude);
			if (ew == 'W') GPS->Longitude = -GPS->Longitude;
			GPS->Altitude = altitude;
		}
		GPS->Satellites = satellites;
    }
    else if (sscanf(Buffer, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &utc_time, &latitude, &ns, &longitude, &ew, &speed, &course, &date) >= 1)
	{
		// $GPRMC,124943.00,A,5157.01557,N,00232.66381,W,0.039,,200314,,,A*6C
		GPS->Speed = (int)speed;
		GPS->Direction = (int)course;
        *GotRMC = 1;
    }
}
 
void GetPosition(int fd, struct TGPS *GPS)
{
    int Count, GotGGA, GotRMC;
    char Buffer[200]; 
    char Character;

    printf("NMEA...\r\n");

	GotGGA = 0;
	GotRMC = 0;
    Count = -1;

    while (!GotGGA || !GotRMC)
    {
        read(fd, &Character, 1);
		// printf ("%c", Character);

        if (Character == '$')
        {
            Count = 0;
        }
        else if (Count > 180)
        {
            Count = -1;
        }
        
        if ((Count >= 0) && (Count <= 180))
        {
            if (Character != '\r')
            {
                Buffer[Count++] = Character;
            }

            if (Character == '\n')
            {
				Buffer[Count] = '\0';
				if (GPSChecksumOK(Buffer, Count))
                {
					ProcessLine(GPS, Buffer, &GotGGA, &GotRMC);
				}
                Count = -1;
            }
        }
    }
}

void BuildSentence(char *TxLine, int SentenceCounter, struct TGPS *GPS)
{
    int Count, i, j;
    unsigned char c;
    unsigned int CRC, xPolynomial;
	float BatteryVoltage, Temperature;
	char TimeBuffer1[12], TimeBuffer2[10];
	
	BatteryVoltage = 0.0;
	Temperature = 0.0;
	
	sprintf(TimeBuffer1, "%06.0f", GPS->Time);
	TimeBuffer2[0] = TimeBuffer1[0];
	TimeBuffer2[1] = TimeBuffer1[1];
	TimeBuffer2[2] = ':';
	TimeBuffer2[3] = TimeBuffer1[2];
	TimeBuffer2[4] = TimeBuffer1[3];
	TimeBuffer2[5] = ':';
	TimeBuffer2[6] = TimeBuffer1[4];
	TimeBuffer2[7] = TimeBuffer1[5];
	TimeBuffer2[8] = '\0';
	
    sprintf(TxLine, "$$%s,%d,%s,%7.5lf,%7.5lf,%05.5u,%d,%d,%d,%3.1f,%3.1f,%3.1f",
            PAYLOAD_ID,
            SentenceCounter,
			TimeBuffer2,
            GPS->Latitude,
            GPS->Longitude,
            GPS->Altitude,
			(GPS->Speed * 13) / 7,
			GPS->Direction,
			GPS->Satellites,            
            Temperature,	// int
            Temperature,	// ext
            BatteryVoltage);

    Count = strlen(TxLine);

     CRC = 0xffff;           // Seed
     xPolynomial = 0x1021;
   
     for (i = 2; i < Count; i++)
     {   // For speed, repeat calculation instead of looping for each bit
        CRC ^= (((unsigned int)TxLine[i]) << 8);
        for (j=0; j<8; j++)
        {
            if (CRC & 0x8000)
                CRC = (CRC << 1) ^ 0x1021;
            else
                CRC <<= 1;
        }
     }

    TxLine[Count++] = '*';
    TxLine[Count++] = Hex[(CRC >> 12) & 15];
    TxLine[Count++] = Hex[(CRC >> 8) & 15];
    TxLine[Count++] = Hex[(CRC >> 4) & 15];
    TxLine[Count++] = Hex[CRC & 15];
	TxLine[Count++] = '\n';  
	TxLine[Count++] = '\0';  

    printf("%s", TxLine);
}


int OpenSerialPort(void)
{
	int fd;

    /* open the port */
    fd = open("/dev/ttyAMA0", O_RDWR | O_NOCTTY | O_NDELAY);
	if (fd >= 0)
	{
		fcntl(fd, F_SETFL, 0);

		/* get the current options */
		tcgetattr(fd, &options);

		/* set raw input */
		options.c_lflag &= ~ECHO;
		options.c_cc[VMIN]  = 0;
		options.c_cc[VTIME] = 10;
		options.c_cflag &= ~CSTOPB;
		cfsetispeed(&options, B9600);
		cfsetospeed(&options, B9600);
		tcsetattr(fd, TCSANOW, &options);
	}

	return fd;
}

/*
static void catch_function(int signo)
{
    digitalWrite (LED_WARN, 0);
    digitalWrite (LED_OK, 0);
    digitalWrite (NTX2B_ENABLE, 0);
	printf("Caught signal %d\n", signo);
	fflush(stdout);
    exit(1);
}
*/

void *GPSLoop(void *some_void_ptr)
{
	int fd;
	struct TGPS *GPS;

	if ((fd = OpenSerialPort()) < 0)
	{
		printf("Cannot open serial port - check documentation!\n");
		exit(1);
	}

	GPS = (struct TGPS *)some_void_ptr;
	
	while (1)
	{
		GetPosition(fd, GPS);
	}

	return NULL;
}

main()
{
	unsigned long Sentence_Counter = 0;
	char Sentence[100], EscapedSentence[120], Command[200];
	struct TGPS GPS;
	pthread_t GPSThread;
	
	GPS.Time = 0.0;
	GPS.Longitude = 0.0;
	GPS.Latitude = 0.0;
	GPS.Altitude = 0;
	GPS.Satellites = 0;
	GPS.Speed = 0.0;
	GPS.Direction = 0.0;
	
	// Set up I/O
	if (wiringPiSetup() == -1)
	{
		exit (1);
	}
	
	// We have 2 LED outputs and 2 enable outputs
	pinMode (NTX2B_ENABLE, OUTPUT);
	pinMode (UBLOX_ENABLE, OUTPUT);
	pinMode (LED_WARN, OUTPUT);
	pinMode (LED_OK, OUTPUT);

	// WARN stays on till we've got a lock
    digitalWrite (LED_WARN, 1);
	
	// Switch on the GPS
    digitalWrite (UBLOX_ENABLE, 0);
	
	// Switch on the radio
    digitalWrite (NTX2B_ENABLE, 1);
	
	// Redirect audio to GPIO PWM pin
	system("./pwm_switch");
	
	// Set volume to 92.6% (600Hz shift)
	system("amixer set PCM -- 92.6\%");
	
    /*
	if (signal(SIGINT, catch_function) == SIG_ERR)
	{
        fputs("An error occurred while setting a signal handler.\n", stderr);
        return EXIT_FAILURE;
    }
	*/
	
	if (pthread_create(&GPSThread, NULL, GPSLoop, &GPS))
	{
		fprintf(stderr, "Error creating thread\n");
		return 1;
	}
	
    printf("RASPBERRY PI-IN-THE-SKY FLIGHT COMPUTER\n");
    printf("-     Payload ID is %s\r\n", PAYLOAD_ID);
	if (strcmp(PAYLOAD_ID, "PIESKY") == 0)
	{
		printf("**** CHANGE THIS NOW!! ****\n");
	}
    printf("- RTTY Baud Rate is %d\r\n", RTTY_BAUD);
	
	while (1)
	{
		// GetPosition(fd, &GPS);
		
		digitalWrite (LED_WARN, GPS.Satellites < 4);
		digitalWrite (LED_OK, Sentence_Counter & 1);

		// Build sentence to send
		BuildSentence(Sentence, ++Sentence_Counter, &GPS);
		
		// Escape $, \r and \n
		EscapeCharacters(Sentence, EscapedSentence);
		
		// Convert it to RTTY in a WAV file
		sprintf(Command, "./makewav %d \"%s\" rtty.wav", RTTY_BAUD, EscapedSentence);
		system(Command);
		
		// And play it
		system("aplay rtty.wav");
    }
}
