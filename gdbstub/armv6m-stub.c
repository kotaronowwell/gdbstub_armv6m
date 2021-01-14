/****************************************************************************

		THIS SOFTWARE IS NOT COPYRIGHTED

   HP offers the following for use in the public domain.  HP makes no
   warranty with regard to the software or it's performance and the
   user accepts the software "AS IS" with all faults.

   HP DISCLAIMS ANY WARRANTIES, EXPRESS OR IMPLIED, WITH REGARD
   TO THIS SOFTWARE INCLUDING BUT NOT LIMITED TO THE WARRANTIES
   OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.

****************************************************************************/

/****************************************************************************
 *  Header: remcom.c,v 1.34 91/03/09 12:29:49 glenne Exp $
 *
 *  Module name: remcom.c $
 *  Revision: 1.34 $
 *  Date: 91/03/09 12:29:49 $
 *  Contributor:     Lake Stevens Instrument Division$
 *
 *  Description:     low level support for gdb debugger. $
 *
 *  Considerations:  only works on target hardware $
 *
 *  Written by:      Glenn Engel $
 *  ModuleState:     Experimental $
 *
 *  NOTES:           See Below $
 *
 *  Modified for SPARC by Stu Grossman, Cygnus Support.
 *
 *  This code has been extensively tested on the Fujitsu SPARClite demo board.
 *
 *  To enable debugger support, two things need to happen.  One, a
 *  call to set_debug_traps() is necessary in order to allow any breakpoints
 *  or error conditions to be properly intercepted and reported to gdb.
 *  Two, a breakpoint needs to be generated to begin communication.  This
 *  is most easily accomplished by a call to breakpoint().  Breakpoint()
 *  simulates a breakpoint by executing a trap #1.
 *
 *************
 *
 *    The following gdb commands are supported:
 *
 * command          function                               Return value
 *
 *    g             return the value of the CPU registers  hex data or ENN
 *    G             set the value of the CPU registers     OK or ENN
 *
 *    mAA..AA,LLLL  Read LLLL bytes at address AA..AA      hex data or ENN
 *    MAA..AA,LLLL: Write LLLL bytes at address AA.AA      OK or ENN
 *
 *    c             Resume at current address              SNN   ( signal NN)
 *    cAA..AA       Continue at address AA..AA             SNN
 *
 *    s             Step one instruction                   SNN
 *    sAA..AA       Step one instruction from AA..AA       SNN
 *
 *    k             kill
 *
 *    ?             What was the last sigval ?             SNN   (signal NN)
 *
 * All commands and responses are sent with a packet which includes a
 * checksum.  A packet consists of
 *
 * $<packet info>#<checksum>.
 *
 * where
 * <packet info> :: <characters representing the command or response>
 * <checksum>    :: < two hex digits computed as modulo 256 sum of <packetinfo>>
 *
 * When a packet is received, it is first acknowledged with either '+' or '-'.
 * '+' indicates a successful transfer.  '-' indicates a failed transfer.
 *
 * Example:
 *
 * Host:                  Reply:
 * $m0,10#2a               +$00010203040506070809101112131415#42
 *
 ****************************************************************************/

#include <string.h>
#include <signal.h>

/************************************************************************
 *
 * external low-level support routines
 */

extern void putDebugChar();	/* write a single character      */
extern int getDebugChar();	/* read and return a single char */

/************************************************************************/
/* BUFMAX defines the maximum number of characters in inbound/outbound buffers*/
/* at least NUMREGBYTES*2 are needed for register packets */
#define BUFMAX 128

static int initialized = 0;	/* !0 means we've been initialized */

static const char hexchars[]="0123456789abcdef";

#define NUMREGS 26

/* Number of bytes of registers.  */
#define NUMREGBYTES (NUMREGS * 4)
/* defined at arm-tdep.c */
enum regnames { R0, R1, R2, R3, /*  0  1  2  3 */
                R4, R5, R6, R7, /*  4  5  6  7 */
                R8, R9, R10, R11,	/*  8  9 10 11 */
                R12, SP, LR, PC,	/* 12 13 14 15 */
                F0, F1, F2, F3,	/* 16 17 18 19 */
                F4, F5, F6, F7,	/* 20 21 22 23 */
                FPS, CPSR }; /* 24 25       */

#define NO_EXCEPTION  0
#define EXCEPTION_NMI 1
#define EXCEPTION_HARD_FAULT 2
#define EXECPTION_MEM_MANAGE 3
#define EXECPTION_BUS_FAULT 4
#define EXECPTION_USAGE_FAULT 5
#define EXECPTION_SVC 11
#define EXECPTION_DEBUG_MON 12
#define EXECPTION_PEND_SV 14
#define EXECPIONT_SYSTICK 15

/*! Definition of GDB target signals. Data taken from the GDB 6.8
    source. Only those we use defined here. */
enum target_signal {
  TARGET_SIGNAL_NONE =  0,
  TARGET_SIGNAL_INT  =  2,
  TARGET_SIGNAL_ILL  =  4,
  TARGET_SIGNAL_TRAP =  5,
  TARGET_SIGNAL_FPE  =  8,
  TARGET_SIGNAL_BUS  = 10,
  TARGET_SIGNAL_SEGV = 11,
  TARGET_SIGNAL_ALRM = 14,
  TARGET_SIGNAL_USR2 = 31,
  TARGET_SIGNAL_PWR  = 32
};

int registers[NUMREGS];

/***************************  ASSEMBLY CODE MACROS *************************/
/* 									   */
__attribute__((naked)) save_registers(void)
{
	asm volatile (
		/* save register values to stub register array */
		"LDR	R1, =registers\r\n"
		/* save registers R0-R3, R12, LR, PC, xPSR */
		"LDR	R0, [SP, #0x0]\r\n"		/* R0 */
		"STR	R0, [R1, #0x0]\r\n"
		"LDR	R0, [SP, #0x4]\r\n"		/* R1 */
		"STR	R0, [R1, #0x4]\r\n"
		"LDR	R0, [SP, #0x8]\r\n"		/* R2 */
		"STR	R0, [R1, #0x8]\r\n"
		"LDR	R0, [SP, #0xC]\r\n"		/* R3 */
		"STR	R0, [R1, #0xC]\r\n"
		"LDR	R0, [SP, #0x10]\r\n"	/* R12 */
		"STR	R0, [R1, #0x30]\r\n"
		"LDR	R0, [SP, #0x14]\r\n"	/* LR */
		"STR	R0, [R1, #0x38]\r\n"
		"LDR	R0, [SP, #0x18]\r\n"	/* PC */
		"STR	R0, [R1, #0x3C]\r\n"
		"LDR	R0, [SP, #0x1C]\r\n"	/* xPSR */
		"STR	R0, [R1, #0x64]\r\n"
		/* save other registers */
		"STR	R4, [R1, #0x10]\r\n"
		"STR	R5, [R1, #0x14]\r\n"
		"STR	R6, [R1, #0x18]\r\n"
		"STR	R7, [R1, #0x1C]\r\n"
		"MOV	R0, R8\r\n"
		"STR	R0, [R1, #0x20]\r\n"
		"MOV	R0, R9\r\n"
		"STR	R0, [R1, #0x24]\r\n"
		"MOV	R0, R10\r\n"
		"STR	R0, [R1, #0x28]\r\n"
		"MOV	R0, R11\r\n"
		"STR	R0, [R1, #0x2C]\r\n"
		"MOV	R0, R12\r\n"
		"STR	R0, [R1, #0x30]\r\n"
		"SUB	R0, R0, #0x20\r\n"
		"MOV	R0, SP\r\n"
		"STR	R0, [R1, #0x34]\r\n"
	);
}

/* Convert ch from a hex digit to an int */

static int
hex (unsigned char ch)
{
  if (ch >= 'a' && ch <= 'f')
    return ch-'a'+10;
  if (ch >= '0' && ch <= '9')
    return ch-'0';
  if (ch >= 'A' && ch <= 'F')
    return ch-'A'+10;
  return -1;
}

static char remcomInBuffer[BUFMAX];
static char remcomOutBuffer[BUFMAX];

/* scan for the sequence $<data>#<checksum>     */

unsigned char *
getpacket (void)
{
  unsigned char *buffer = &remcomInBuffer[0];
  unsigned char checksum;
  unsigned char xmitcsum;
  int count;
  char ch;

  while (1)
    {
      /* wait around for the start character, ignore all other characters */
      while ((ch = getDebugChar ()) != '$')
	;

retry:
      checksum = 0;
      xmitcsum = -1;
      count = 0;

      /* now, read until a # or end of buffer is found */
      while (count < BUFMAX - 1)
	{
	  ch = getDebugChar ();
          if (ch == '$')
            goto retry;
	  if (ch == '#')
	    break;
	  checksum = checksum + ch;
	  buffer[count] = ch;
	  count = count + 1;
	}
      buffer[count] = 0;

      if (ch == '#')
	{
	  ch = getDebugChar ();
	  xmitcsum = hex (ch) << 4;
	  ch = getDebugChar ();
	  xmitcsum += hex (ch);

	  if (checksum != xmitcsum)
	    {
	      putDebugChar ('-');	/* failed checksum */
	    }
	  else
	    {
	      putDebugChar ('+');	/* successful transfer */

	      /* if a sequence char is present, reply the sequence ID */
	      if (buffer[2] == ':')
		{
		  putDebugChar (buffer[0]);
		  putDebugChar (buffer[1]);

		  return &buffer[3];
		}

	      return &buffer[0];
	    }
	}
    }
}

/* send the packet in buffer.  */

static void
putpacket (unsigned char *buffer)
{
  unsigned char checksum;
  int count;
  unsigned char ch;

  /*  $<packet info>#<checksum>. */
  do
    {
      putDebugChar('$');
      checksum = 0;
      count = 0;

      while (ch = buffer[count])
	{
	  putDebugChar(ch);
	  checksum += ch;
	  count += 1;
	}

      putDebugChar('#');
      putDebugChar(hexchars[checksum >> 4]);
      putDebugChar(hexchars[checksum & 0xf]);

    }
  while (getDebugChar() != '+');
}

/* Indicate to caller of mem2hex or hex2mem that there has been an
   error.  */
static volatile int mem_err = 0;

/* Convert the memory pointed to by mem into hex, placing result in buf.
 * Return a pointer to the last char put in buf (null), in case of mem fault,
 * return 0.
 * If MAY_FAULT is non-zero, then we will handle memory faults by returning
 * a 0, else treat a fault like any other fault in the stub.
 */

static unsigned char *
mem2hex (unsigned char *mem, unsigned char *buf, int count, int may_fault)
{
  unsigned char ch;

  while (count-- > 0)
    {
      ch = *mem++;
      if (mem_err)
	return 0;
      *buf++ = hexchars[ch >> 4];
      *buf++ = hexchars[ch & 0xf];
    }

  *buf = 0;

  return buf;
}

/* convert the hex array pointed to by buf into binary to be placed in mem
 * return a pointer to the character AFTER the last byte written */

static char *
hex2mem (unsigned char *buf, unsigned char *mem, int count, int may_fault)
{
  int i;
  unsigned char ch;

  for (i=0; i<count; i++)
    {
      ch = hex(*buf++) << 4;
      ch |= hex(*buf++);
      *mem++ = ch;
      if (mem_err)
	      return 0;
    }

  return mem;
}

/* Convert the SPARC hardware trap type code to a unix signal number. */
static int
computeSignal (int exceptionVector)
{
  int sigval;

  switch (exceptionVector)
    {
    case NO_EXCEPTION:
      sigval = -1;
      break;
    case EXCEPTION_HARD_FAULT:
    case EXECPTION_USAGE_FAULT:
      sigval = TARGET_SIGNAL_ILL;
      break;
    case EXECPTION_BUS_FAULT:
      sigval = TARGET_SIGNAL_BUS;
      break;
    default:
      sigval = TARGET_SIGNAL_USR2;  /* "software generated"*/
      break;
    }
  return (sigval);
}

/*
 * While we find nice hex chars, build an int.
 * Return number of chars processed.
 */

static int
hexToInt(char **ptr, int *intValue)
{
  int numChars = 0;
  int hexValue;

  *intValue = 0;

  while (**ptr)
    {
      hexValue = hex(**ptr);
      if (hexValue < 0)
	break;

      *intValue = (*intValue << 4) | hexValue;
      numChars ++;

      (*ptr)++;
    }

  return (numChars);
}

/*
 * This function does all command procesing for interfacing to gdb.  It
 * returns 1 if you should skip the instruction at the trap address, 0
 * otherwise.
 */

extern void breakinst();

void
handle_exception (int exceptionVector)
{
  int sigval;
  int addr;
  int length;
  char *ptr;
  unsigned long *sp;

  sp = (unsigned long *)registers[SP];

  /* reply to host that an exception has occurred */
  sigval = computeSignal(exceptionVector);
  ptr = remcomOutBuffer;

  *ptr++ = 'T';
  *ptr++ = hexchars[sigval >> 4];
  *ptr++ = hexchars[sigval & 0xf];

  *ptr++ = hexchars[PC >> 4];
  *ptr++ = hexchars[PC & 0xf];
  *ptr++ = ':';
  ptr = mem2hex((char *)&registers[PC], ptr, 4, 0);
  *ptr++ = ';';

  *ptr++ = hexchars[SP >> 4];
  *ptr++ = hexchars[SP & 0xf];
  *ptr++ = ':';
  ptr = mem2hex((char *)&sp, ptr, 4, 0);
  *ptr++ = ';';

  *ptr++ = 0;

  putpacket(remcomOutBuffer);

  while (1)
    {
      remcomOutBuffer[0] = 0;

      ptr = getpacket();
      switch (*ptr++)
	{
	case '?':
	  remcomOutBuffer[0] = 'S';
	  remcomOutBuffer[1] = hexchars[sigval >> 4];
	  remcomOutBuffer[2] = hexchars[sigval & 0xf];
	  remcomOutBuffer[3] = 0;
	  break;

	case 'd':		/* toggle debug flag */
	  break;

	case 'g':		/* return the value of the CPU registers */
	  mem2hex ((char *) registers, remcomOutBuffer, NUMREGBYTES, 0);
	  break;
	case 'G':		/* set the value of the CPU registers - return OK */
	  hex2mem (ptr, (char *) registers, NUMREGBYTES, 0);
	  strcpy (remcomOutBuffer, "OK");
	  break;

	case 'm':	  /* mAA..AA,LLLL  Read LLLL bytes at address AA..AA */
	  /* Try to read %x,%x.  */

	  if (hexToInt(&ptr, &addr)
	      && *ptr++ == ','
	      && hexToInt(&ptr, &length))
	    {
	      if (mem2hex((char *)addr, remcomOutBuffer, length, 1))
		break;

	      strcpy (remcomOutBuffer, "E03");
	    }
	  else
	    strcpy(remcomOutBuffer,"E01");
	  break;

	case 'M': /* MAA..AA,LLLL: Write LLLL bytes at address AA.AA return OK */
	  /* Try to read '%x,%x:'.  */

	  if (hexToInt(&ptr, &addr)
	      && *ptr++ == ','
	      && hexToInt(&ptr, &length)
	      && *ptr++ == ':')
	    {
	      if (hex2mem(ptr, (char *)addr, length, 1))
		strcpy(remcomOutBuffer, "OK");
	      else
		strcpy(remcomOutBuffer, "E03");
	    }
	  else
	    strcpy(remcomOutBuffer, "E02");
	  break;

	case 'c':    /* cAA..AA    Continue at address AA..AA(optional) */
	  /* try to read optional parameter, pc unchanged if no parm */

	  if (hexToInt(&ptr, &addr))
	    {
	      registers[PC] = addr;
	    }

/* Need to flush the instruction cache here, as we may have deposited a
   breakpoint, and the icache probably has no way of knowing that a data ref to
   some location may have changed something that is in the instruction cache.
 */

	  return;

	  /* kill the program */
	case 'k' :		/* do nothing */
	  break;
	case 'r':		/* Reset */
#if 0 /* TODO */
	  asm ("call 0
		nop ");
#endif
	  break;
	}			/* switch */

      /* reply to the request */
      putpacket(remcomOutBuffer);
    }
}

/* This function will generate a breakpoint exception.  It is used at the
   beginning of a program to sync up with a debugger and can be used
   otherwise as a quick means to stop program execution and "break" into
   the debugger. */

void
breakpoint (void)
{
  if (!initialized)
    return;
#if 0 /* TODO */
  asm("	.globl _breakinst

	_breakinst: ta 1
      ");
#endif
}
