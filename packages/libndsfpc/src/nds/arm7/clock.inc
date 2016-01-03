<<<<<<< HEAD
<<<<<<< HEAD
{$ifndef ARM7}
  {$error The clock is only available on the ARM7}
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: clock.inc 25 2007-12-10 21:06:46Z p4p3r0 $
  ------------------------------------------------------------------------------
  	Copyright (C) 2005
  		Jason Rogers (dovoto)
  		Dave Murphy (WinterMute)
  	
  	This software is provided 'as-is', without any express or implied
  	warranty.  In no event will the authors be held liable for any
  	damages arising from the use of this software.
  	
  	Permission is granted to anyone to use this software for any
  	purpose, including commercial applications, and to alter it and
  	redistribute it freely, subject to the following restrictions:
  	
  	1.	The origin of this software must not be misrepresented; you
  		must not claim that you wrote the original software. If you use
  		this software in a product, an acknowledgment in the product
  		documentation would be appreciated but is not required.
  	
  	2.	Altered source versions must be plainly marked as such, and
  		must not be misrepresented as being the original software.
  	
  	3.	This notice may not be removed or altered from any source
  		distribution.
  ------------------------------------------------------------------------------
    
    
  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
   
  Copyright (C) 2006  Francesco Lombardi
  Check http://sourceforge.net/projects/libndsfpc for updates
   
  ------------------------------------------------------------------------------

  $Log$

*)

{$ifndef ARM7}
{$error The clock is only available on the ARM7}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif ARM7}

{$ifdef NDS_INTERFACE}
const
// RTC registers
  WRITE_STATUS_REG1  = $60;
  READ_STATUS_REG1   = $61;
  STATUS_POC       = (1 shl 7);  // read-only, cleared by reading (1 if just powered on)
  STATUS_BLD       = (1 shl 6);  // read-only, cleared by reading (1 if power dropped below the safety threshold)
  STATUS_INT2      = (1 shl 5);  // read-only, INT2 has occured
  STATUS_INT1      = (1 shl 4);  // read-only, INT1 has occured
  STATUS_SC1       = (1 shl 3);  // R/W scratch bit
  STATUS_SC0       = (1 shl 2);  // R/W scratch bit
  STATUS_24HRS     = (1 shl 1);  // 24 hour mode when 1, 12 hour mode when 0
  STATUS_RESET     = (1 shl 0);  // write-only, reset when 1 written

  WRITE_STATUS_REG2  = $62;
  READ_STATUS_REG2   = $63;
  STATUS_TEST      = (1 shl 7);  // 
  STATUS_INT2AE    = (1 shl 6);  // 
  STATUS_SC3       = (1 shl 5);  // R/W scratch bit
  STATUS_SC2       = (1 shl 4);  // R/W scratch bit

  STATUS_32kE      = (1 shl 3);  // Interrupt mode bits
  STATUS_INT1AE    = (1 shl 2);  // 
  STATUS_INT1ME    = (1 shl 1);  // 
  STATUS_INT1FE    = (1 shl 0);  // 

  // full 7 bytes for time and date
  WRITE_TIME_AND_DATE = $64;
  READ_TIME_AND_DATE = $65;

	// last 3 bytes of current time
  WRITE_TIME = $66;
  READ_TIME = $67;


  WRITE_INT_REG1     = $68;
  READ_INT_REG1      = $69;

  READ_INT_REG2      = $6A;
  WRITE_INT_REG2     = $6B;

  READ_CLOCK_ADJUST_REG  = $6C;
  WRITE_CLOCK_ADJUST_REG = $6D;
// clock-adjustment register

  READ_FREE_REG      = $6E;
  WRITE_FREE_REG     = $6F;


procedure rtcReset(); cdecl; external;
procedure rtcTransaction(command: pcuint8; commandLength: cuint32; result: pcuint8; resultLength: cuint32); cdecl; external;
procedure rtcGetTime(time: pcuint8); cdecl; external;
procedure rtcSetTime(time: pcuint8); cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD
=======
procedure rtcGetData(data: pcuint8; size: cuint32); cdecl; external;
>>>>>>> graemeg/fixes_2_2
=======
procedure rtcGetData(data: pcuint8; size: cuint32); cdecl; external;
>>>>>>> origin/fixes_2_2

procedure rtcGetTimeAndDate(time: pcuint8); cdecl; external;
procedure rtcSetTimeAndDate(time: pcuint8); cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
procedure rtcGetData(var data: cuint8; size: cuint32); cdecl; external;
procedure BCDToInteger(var data: cuint8; length: cuint32); cdecl; external;
procedure integerToBCD(var data: cuint8; length: cuint32); cdecl; external;

procedure initClockIRQ(); cdecl; external;
procedure resyncClock(); cdecl; external;
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
procedure BCDToInteger(data: pcuint8; length: cuint32); cdecl; external;
procedure integerToBCD(data: pcuint8; length: uint32); cdecl; external;

procedure initClockIRQ(); cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
=======
>>>>>>> origin/fixes_2.4
{$endif NDS_INTERFACE}
