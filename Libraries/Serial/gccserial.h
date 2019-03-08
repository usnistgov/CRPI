#ifndef GCCSERIAL
#define GCCSERIAL
#if defined(__GNUC__)

#include <stdint.h>
typedef uint8_t BYTE;
typedef uint32_t DWORD;
typedef int32_t LONG;
#ifdef LONGLONG			/* FMP */
typedef int64_t LONGLONG;
#endif

typedef int                 BOOL;
typedef unsigned short      WORD;
typedef float               FLOAT;
typedef int HANDLE;
typedef BYTE   FCHAR;
typedef WORD   FSHORT;
typedef DWORD  FLONG;

typedef int DCB;
/*typedef struct _DCB {
	DWORD DCBlength;      /* sizeof(DCB)                     
	DWORD BaudRate;       /* Baudrate at which running       
	DWORD fBinary : 1;     /* Binary Mode (skip EOF check)    
	DWORD fParity : 1;     /* Enable parity checking          
	DWORD fOutxCtsFlow : 1; /* CTS handshaking on output       
	DWORD fOutxDsrFlow : 1; /* DSR handshaking on output       
	DWORD fDtrControl : 2;  /* DTR Flow control                
	DWORD fDsrSensitivity : 1; /* DSR Sensitivity              
	DWORD fTXContinueOnXoff : 1; /* Continue TX when Xoff sent 
	DWORD fOutX : 1;       /* Enable output X-ON/X-OFF        
	DWORD fInX : 1;        /* Enable input X-ON/X-OFF         
	DWORD fErrorChar : 1;  /* Enable Err Replacement          
	DWORD fNull : 1;       /* Enable Null stripping           
	DWORD fRtsControl : 2;  /* Rts Flow control                
	DWORD fAbortOnError : 1; /* Abort all reads and writes on Error 
	DWORD fDummy2 : 17;     /* Reserved                        
	WORD wReserved;       /* Not currently used              
	WORD XonLim;          /* Transmit X-ON threshold         
	WORD XoffLim;         /* Transmit X-OFF threshold        /
	BYTE ByteSize;        /* Number of bits/byte, 4-8        /
	BYTE Parity;          /* 0-4=None,Odd,Even,Mark,Space    /
	BYTE StopBits;        /* 0,1,2 = 1, 1.5, 2               /
	char XonChar;         /* Tx and Rx X-ON character        /
	char XoffChar;        /* Tx and Rx X-OFF character       /
	char ErrorChar;       /* Error replacement char          /
	char EofChar;         /* End of Input character          /
	char EvtChar;         /* Received Event character        /
	WORD wReserved1;      /* Fill for now.                   /
} DCB, *LPDCB;
*/

typedef struct _COMMTIMEOUTS {
	DWORD ReadIntervalTimeout;          /* Maximum time between read chars. */
	DWORD ReadTotalTimeoutMultiplier;   /* Multiplier of characters.        */
	DWORD ReadTotalTimeoutConstant;     /* Constant in milliseconds.        */
	DWORD WriteTotalTimeoutMultiplier;  /* Multiplier of characters.        */
	DWORD WriteTotalTimeoutConstant;    /* Constant in milliseconds.        */
} COMMTIMEOUTS, *LPCOMMTIMEOUTS;

#elif defined(_MSC_VER)
#include <windows.h>
#endif
#endif
