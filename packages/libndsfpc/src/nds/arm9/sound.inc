<<<<<<< HEAD
<<<<<<< HEAD
{$ifdef NDS_INTERFACE}

type
  MicCallback = procedure(completedBuffer: pointer; length: cint);

  SoundFormat = integer;
const
  SoundFormat_8Bit  : SoundFormat = 0;
  SoundFormat_16Bit : SoundFormat = 1;
  SoundFormat_ADPCM : SoundFormat = 2;
  SoundFormat_PSG   : SoundFormat = 3;

type
  MicFormat = integer;
const
  MicFormat_12Bit : MicFormat = 0;
  MicFormat_8Bit  : MicFormat = 1;
  
type
  DutyCycle = integer;
const
  DutyCycle_12  : DutyCycle = 0;
  DutyCycle_25  : DutyCycle = 1;
  DutyCycle_37  : DutyCycle = 2;
  DutyCycle_50  : DutyCycle = 3;
  DutyCycle_62  : DutyCycle = 4;
  DutyCycle_75  : DutyCycle = 5;
  DutyCycle_87  : DutyCycle = 6;
  DutyCycle_0   : DutyCycle = 7;
  
procedure soundEnable(); cdecl; external;

procedure soundDisable(); cdecl; external;

function soundPlaySample(const data: pointer; format: SoundFormat; dataSize: cuint32; freq: cuint16; volume, pan: cuint8; loop: cbool; loopPoint: cuint16): cint; cdecl; external; 

function soundPlayPSG(cycle: DutyCycle; freq: cuint16; volume, pan: cuint8): cint; cdecl; external;

function soundPlayNoise(freq: cuint16; volume, pan: cuint8): cint; cdecl; external;

procedure soundPause(soundId: cint); cdecl; external;

procedure soundSetWaveDuty(soundId: cint; cycle: DutyCycle); cdecl; external;

procedure soundKill(soundId: cint); cdecl; external;

procedure soundResume(soundId: cint); cdecl; external;

procedure soundSetVolume(soundId: cint; volume: cuint8); cdecl; external;

procedure soundSetPan(soundId: cint; pan: cuint8); cdecl; external;

procedure soundSetFreq(soundId: cint; freq: cuint16); cdecl; external;

function soundMicRecord(buffer: pointer; bufferLength: cuint32; format: MicFormat; freq: cint; callback: MicCallback): cint; cdecl; external;

procedure soundMicOff(); cdecl; external;

=======
=======
>>>>>>> origin/fixes_2_2
(*
  $Id: sound.inc 25 2007-12-10 21:06:46Z p4p3r0 $
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
procedure playSound(sound: pTransferSoundData); cdecl; external;
procedure setGenericSound(rate: cuint32; vol: cuint8; pan: cuint8; format: cuint8); cdecl; external;
procedure playGenericSound(const data: pointer; length: cuint32); cdecl; external;
<<<<<<< HEAD
>>>>>>> graemeg/fixes_2_2
=======
>>>>>>> origin/fixes_2_2
{$endif NDS_INTERFACE}
