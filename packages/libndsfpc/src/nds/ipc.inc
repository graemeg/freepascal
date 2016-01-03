(*
  $Id: ipc.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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

{$ifdef NDS_INTERFACE}
type
  sTransferSoundData = record
    data: pointer;//pcint;
    len: cuint32; 
    rate: cuint32; 
    vol: cuint8; 
    pan: cuint8; 
    format: cuint8; 
    PADDING: cuint8; 
  end;
  TransferSoundData = sTransferSoundData; 
  TTransferSoundData = TransferSoundData;
  PTransferSoundData = ^sTransferSoundData; 

<<<<<<< HEAD
<<<<<<< HEAD
  sTransferRegion = record
  	touchX,   touchY: cint16;
  	touchXpx, touchYpx: cint16;
  	touchZ1,  touchZ2: cint16;
 
  	buttons: cuint16;

    unixTime: cint32;
=======
=======
>>>>>>> origin/fixes_2_2
  sTransferSound = record
    data: array [0..15] of TransferSoundData;
    count: cuint8;
    PADDING: array [0..2] of cuint8;
  end; 
  TransferSound = sTransferSound;
  TTransferSound = TransferSound;
  PTransferSound = ^sTransferSound;
    
  sTransferRegion = record
  	touchX,   touchY: cint16;   // TSC X, Y
  	touchXpx, touchYpx: cint16; // TSC X, Y pixel values
  	touchZ1,  touchZ2: cint16;  // TSC x-panel measurements
  	tdiode1,  tdiode2: cuint16;  // TSC temperature diodes
  	temperature: cuint32;        // TSC computed temperature
 
  	buttons: cuint16;            // X, Y, /PENIRQ buttons

    time: record
      case integer of 
        0: (curtime: array [0..7] of cuint8);        // current time response from RTC
        1: (rtc: record
              command: cuint8;
              year: cuint8;           //add 2000 to get 4 digit year
              month: cuint8;          //1 to 12
              day: cuint8;            //1 to (days in month)  
             
              incr: cuint8;
              hours: cuint8;          //0 to 11 for AM, 52 to 63 for PM
              minutes: cuint8;        //0 to 59
              seconds: cuint8;        //0 to 59
            end;
           ); 
    end;
    unixTime: cint32;
    
  	battery: cuint16;            // battery life ??  hopefully.  :)
  	aux: cuint16;                // i have no idea...

  
    // Don't rely on these below, will change or be removed in the future
  	soundData: pTransferSound;
    mailAddr: cuint32;
    mailData: cuint32;
    mailRead: cuint8;
    mailBusy: cuint8;
    mailSize: cuint32;

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  TransferRegion = sTransferRegion; 
  TTransferRegion = TransferRegion;
  PTransferRegion = ^sTransferRegion; 


const
  //IPC : pTransferRegion = pointer($027FF000);

<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> origin/fixes_2_2
  IPC_PEN_DOWN = (1 shl 6);
  IPC_X = (1 shl 0);
  IPC_Y = (1 shl 1);
  IPC_LID_CLOSED = (1 shl 7);

<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
//---------------------------------------------------------------------------------
// Synchronization register
//---------------------------------------------------------------------------------
  REG_IPC_SYNC	: pcuint16 = pointer($04000180);

type
  IPC_SYNC_BITS = longint;
const
  IPC_SYNC_IRQ_ENABLE: IPC_SYNC_BITS =	(1 shl 14);
  IPC_SYNC_IRQ_REQUEST: IPC_SYNC_BITS	=	(1 shl 13);
<<<<<<< HEAD
<<<<<<< HEAD
  
  
function getIPC(): PTransferRegion; inline;
function IPC(): PTransferRegion; inline;
procedure IPC_SendSync(sync: cuint); inline; 
function IPC_GetSync(): cint; inline;

=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}  


{$ifdef NDS_IMPLEMENTATION}
function getIPC(): pTransferRegion; inline;
begin
	getIPC := pTransferRegion(pointer($027FF000));
end;

function IPC(): TransferRegion; 
begin
	IPC := getIPC()^;
end;
  
procedure IPC_SendSync(sync: cuint); inline; 
begin
	REG_IPC_SYNC^ := (REG_IPC_SYNC^ and $f0ff) or (((sync) and $0f) shl 8) or IPC_SYNC_IRQ_REQUEST;
end;

function IPC_GetSync(): cint; inline;
begin
	IPC_GetSync := REG_IPC_SYNC^ and $0f;
end;
{$endif NDS_IMPLEMENTATION}


{$ifdef NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
const
  REG_IPC_FIFO_TX		: pcuint32 = pointer($04000188);
  REG_IPC_FIFO_RX		: pcuint32 = pointer($04100000);
  REG_IPC_FIFO_CR		: pcuint16 = pointer($04000184);

type
  IPC_CONTROL_BITS = cint;

const
	IPC_FIFO_SEND_EMPTY  : IPC_CONTROL_BITS = (1 shl 0);
	IPC_FIFO_SEND_FULL   : IPC_CONTROL_BITS = (1 shl 1);
	IPC_FIFO_SEND_IRQ    : IPC_CONTROL_BITS = (1 shl 2);
	IPC_FIFO_SEND_CLEAR  : IPC_CONTROL_BITS = (1 shl 3);
	IPC_FIFO_RECV_EMPTY  : IPC_CONTROL_BITS	=	(1 shl 8);
	IPC_FIFO_RECV_FULL   : IPC_CONTROL_BITS = (1 shl 9);
	IPC_FIFO_RECV_IRQ    : IPC_CONTROL_BITS = (1 shl 10);
	IPC_FIFO_ERROR       : IPC_CONTROL_BITS = (1 shl 14);
	IPC_FIFO_ENABLE      : IPC_CONTROL_BITS = (1 shl 15);
{$endif NDS_INTERFACE}

<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_IMPLEMENTATION}
function getIPC(): PTransferRegion; inline;
begin
	getIPC := PTransferRegion(pointer($027FF000));
end;

function IPC(): PTransferRegion; inline;
begin
	IPC := getIPC();
end;
  
procedure IPC_SendSync(sync: cuint); inline; 
begin
	REG_IPC_SYNC^ := (REG_IPC_SYNC^ and $f0ff) or (((sync) and $0f) shl 8) or IPC_SYNC_IRQ_REQUEST;
end;

function IPC_GetSync(): cint; inline;
begin
	IPC_GetSync := REG_IPC_SYNC^ and $0f;
end;
{$endif NDS_IMPLEMENTATION}
=======
=======
>>>>>>> origin/fixes_2_2
{$ifdef NDS_INTERFACE}
function getIPC(): pTransferRegion; inline;
function IPC(): TransferRegion; 
procedure IPC_SendSync(sync: cuint); inline; 
function IPC_GetSync(): cint; inline;
{$endif NDS_INTERFACE}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
