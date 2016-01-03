<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}

type
  TGetHeaderCallback = function(source: pcuint8;  dest: pcuint16; r2: cuint32): pcint;
  getHeaderCallback = TGetHeaderCallback;
  TGetResultCallback = function(source: pcuint8): pcint;
  getResultCallback = TGetResultCallback; 
  TGetByteCallback = function(source: pcuint8): pcuint8; 
  getByteCallback = TGetByteCallback;

type
  DecompressionStream = packed record
    getSize: TGetHeaderCallback; 
    getResult: TGetResultCallback; 
    readByte: TGetByteCallback; 
=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: bios.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
  TGetSize = function(source: pcuint8;  dest: pcuint16; r2: cuint32): pcint; 
  TGetResult = function(source: pcuint8): pcint; 
  TReadByte = function(source: pcuint8): pcuint8; 


type
  DecompressionStream = packed record
    getSize: TGetSize; 
    getResult: TGetResult; 
    readByte: TReadByte; 
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
  end;
  TDecompressionStream = DecompressionStream;
  PDecompressionStream = ^TDecompressionStream;

  UnpackStruct = packed record
    sourceSize: cuint16; // in bytes
    sourceWidth: cuint8;
    destWidth: cuint8;
    dataOffset: cuint32;
  end;
  TUnpackStruct = UnpackStruct;
  PUnpackStruct = ^TUnpackStruct;


procedure swiSoftReset(); cdecl; external;

procedure swiDelay(duration: cuint32); cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
function swiDivide(numerator, divisor: cint): cint; cdecl; external;
function swiRemainder(numerator, divisor: cint): cint; cdecl; external;
procedure swiDivMod(numerator, divisor: cint; var result, remainder: cint); cdecl; external;
=======
=======
>>>>>>> origin/fixes_2_2
procedure swiIntrWait(waitForSet: cint; flags: cuint32); cdecl; external;

procedure swiWaitForVBlank(); cdecl; external;

=======
>>>>>>> origin/fixes_2.4
function swiDivide(numerator, divisor: cint): cint; cdecl; external;
function swiRemainder(numerator, divisor: cint): cint; cdecl; external;
procedure swiDivMod(numerator, divisor: cint; result, remainder: pcint); cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2

const
  COPY_MODE_HWORD = (0);
  COPY_MODE_WORD  = (1 shl 26);
  COPY_MODE_COPY  = (0);
  COPY_MODE_FILL  = (1 shl 24);

procedure swiCopy(const source: pointer; dest: pointer; flags: cint); cdecl; external;
procedure swiFastCopy(const source: pointer; dest: pointer; flags: cint); cdecl; external;

function swiSqrt(value: cint): cint; cdecl; external;

function swiCRC16(crc: cuint16; data: pointer; size: cuint32): cuint16; cdecl; external;

function swiIsDebugger(): cint; cdecl; external;

procedure swiUnpackBits(source: pcuint8; destination: pcuint32; params: PUnpackStruct); cdecl; external;

procedure swiDecompressLZSSWram(source, destination: pointer); cdecl; external;
function swiDecompressLZSSVram(source, destination: pointer; toGetSize: cuint32; stream: PDecompressionStream): cint; cdecl; external;

<<<<<<< HEAD
<<<<<<< HEAD
function swiDecompressLZSSVramNTR(source, destination: pointer; toGetSize: cuint32; stream: PDecompressionStream): cint; cdecl; external;
function swiDecompressLZSSVramTWL(source, destination: pointer; toGetSize: cuint32; stream: PDecompressionStream): cint; cdecl; external;

=======
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
function swiDecompressHuffman(source, destination: pointer; toGetSize: cuint32; stream: PDecompressionStream): cint; cdecl; external;

procedure swiDecompressRLEWram(source, destination: pointer); cdecl; external;
function swiDecompressRLEVram(source, destination: pointer; toGetSize: cuint32; stream: PDecompressionStream): cint; cdecl; external;
<<<<<<< HEAD
<<<<<<< HEAD

{$ifdef ARM9}
  procedure swiWaitForIRQ(); cdecl; external;
  procedure swiSetHaltCR(data: cuint32); cdecl; external;
  procedure swiDecodeDelta8(source, destination: pointer); cdecl; external;
  procedure swiDecodeDelta16(source, destination: pointer); cdecl; external;
{$endif ARM9}

{$ifdef ARM7}
  procedure swiHalt(); cdecl; external;
  procedure swiSleep(); cdecl; external;
  procedure swiSwitchToGBAMode(); cdecl; external;
  procedure swiSetHaltCR(data: cuint8); cdecl; external;
  function swiGetSineTable(index: cint): cuint16; cdecl; external;
  function swiGetPitchTable(index: cint): cuint16; cdecl; external;
  function swiGetVolumeTable(index: cint): cuint8; cdecl; external;
  procedure swiChangeSoundBias(enabled, delay: cint); cdecl; external;
{$endif ARM7}

=======
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}


{$ifdef NDS_INTERFACE}
{$ifdef ARM9}
procedure swiWaitForIRQ(); cdecl; external;
procedure swiSetHaltCR(data: cuint32); cdecl; external;
procedure swiDecodeDelta8(source, destination: pointer); cdecl; external;
procedure swiDecodeDelta16(source, destination: pointer); cdecl; external;
{$endif ARM9}

{$ifdef ARM7}
procedure swiHalt(); cdecl; external;
procedure swiSleep(); cdecl; external;
procedure swiSwitchToGBAMode(); cdecl; external;
procedure swiSetHaltCR(data: cuint8); cdecl; external;
function swiGetSineTable(index: cint): cuint16; cdecl; external;
function swiGetPitchTable(index: cint): cuint16; cdecl; external;
function swiGetVolumeTable(index: cint): cuint8; cdecl; external;
procedure swiChangeSoundBias(enabled, delay: cint); cdecl; external;
{$endif ARM7}
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}
