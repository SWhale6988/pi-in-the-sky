#include <stdio.h>
#include <stdlib.h>
#include <string.h>
 
 
typedef unsigned int	UI;
typedef unsigned long int	UL;
typedef unsigned short int	US;
typedef unsigned char	UC;
typedef signed int		SI;
typedef signed long int	SL;
typedef signed short int	SS;
typedef signed char	SC;
 
 
#define attr(a) __attribute__((a))
 
#define packed attr(packed)
 
/* WAV header, 44-byte total */
typedef struct{
 UL riff	packed;
 UL len	packed;
 UL wave	packed;
 UL fmt	packed;
 UL flen	packed;
 US one	packed;
 US chan	packed;
 UL hz	packed;
 UL bpsec	packed;
 US bpsmp	packed;
 US bitpsmp	packed;
 UL dat	packed;
 UL dlen	packed;
}WAVHDR;
 
  
 
/* "converts" 4-char string to long int */
#define dw(a) (*(UL*)(a))
 
 
/* Makes 44-byte header for 8-bit WAV in memory
usage: wavhdr(pointer,sampleRate,dataLength) */
 
void wavhdr(void*m,UL hz,UL dlen){
 WAVHDR*p=m;
 p->riff=dw("RIFF");
 p->len=dlen+44;
 p->wave=dw("WAVE");
 p->fmt=dw("fmt ");
 p->flen=0x10;
 p->one=1;
 p->chan=1;
 p->hz=hz;
 p->bpsec=hz;
 p->bpsmp=1;
 p->bitpsmp=8;
 p->dat=dw("data");
 p->dlen=dlen;
}

void make_and_write_bit(FILE *f, UL cycles_per_bit, unsigned char Value)
{
	int i;
	
	if (Value)
	{
		Value = 255;
	}
	else
	{
		Value = 0;
	}
	
	for (i=0; i<cycles_per_bit; i++)
	{
		fwrite(&Value, 1, 1, f);
	}
}

 
void make_and_write_byte(FILE *f, UL cycles_per_bit, int DataBits, int StopBits, char Character)
{
	int i;
	
	// Start bit
	make_and_write_bit(f, cycles_per_bit, 0);

	// 7 or 8 data bits
	for (i=0; i<DataBits; i++)
	{
		make_and_write_bit(f, cycles_per_bit, (Character & (1 << i)));
	}
	
	for (i=0; i<StopBits; i++)
	{
		make_and_write_bit(f, cycles_per_bit, 1);
	}
}

void EscapeCharactersIn(char* buffer)
{
    char *source, *target;
	int Escaped=0;
	
    for (source=buffer, target=buffer; *source; source++)
	{
		if (*source == '\\')
		{
			Escaped = 1;
		}
		else if (Escaped)
		{
			if (*source == 'r')
			{
				*target++ = '\r';
			}
			else if (*source == 'n')
			{
				*target++ = '\r';
			}
			
			Escaped = 0;
		}
		else
		{
			*target++ = *source;
		}
		
    }
	
	*target ='\0';
}

 
/* makes wav file */
void makertty(UL freq, UL baud, int DataBits, int StopBits, char *Message, char *Filename)
{
	UL preamble_length, postamble_length, cycles_per_bit, cycles_per_byte, total_cycles, message_length;
	UC* m;
	FILE *f;
	int i, NumberOfBits;
	
	f = fopen(Filename,"wb");
	
	// Calculate size of file
	preamble_length = 20;
	postamble_length = 10;
	
	printf ("message=%s\n", Message);

	EscapeCharactersIn(Message);
	message_length = strlen(Message); 
	
	printf ("baud=%d\n", baud);
	printf ("freq=%d\n", freq);
	cycles_per_bit = freq / baud;
	printf ("cycles_per_bit=%d\n", cycles_per_bit);
	NumberOfBits = 1 + DataBits + StopBits;
	cycles_per_byte = cycles_per_bit * NumberOfBits;	// 1 stop 7 data 2 stop
	printf ("cycles_per_byte=%d\n", cycles_per_byte);
	total_cycles = (cycles_per_byte * message_length) + ((preamble_length + postamble_length) * cycles_per_bit);

	// Make header
	m = malloc(44);
	wavhdr(m, freq, total_cycles);
	
	// Write wav header
	fwrite(m, 1, 44, f);
	
	// Write preamble
	for (i=0; i< preamble_length; i++)
	{
		make_and_write_bit(f, cycles_per_bit, 1);
	}
	
	// Create and write wav data
	while (*Message)
	{
		make_and_write_byte(f, cycles_per_bit, DataBits, StopBits, *Message++);
	}

	// Write postamble
	for (i=0; i< postamble_length; i++)
	{
		make_and_write_bit(f, cycles_per_bit, 1);
	}
	
	fclose(f);
}


void main(int argc, char *argv[])
{
	if (argc == 4)
	{
		int baud;
		
		baud = atoi(argv[1]);
		
		makertty(38400, baud, 8, 2, argv[2], argv[3]);
	}
	else
	{
		printf("Usage: makewav <baud_rate> <\"some message\"> <filename>\n");
	}
	
}


