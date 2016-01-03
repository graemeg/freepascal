(*
  gba_bios.pas  18/06/2006 4.17.48
  ------------------------------------------------------------------------------
  This lib is a raw porting of libgba library for gba (you can find it at
  http://www.devkitpro.org).
  
  As this is a direct port from c, I'm pretty sure that something could not work
  as you expect. I am even more sure that this code could be written better, so 
  if you think that I have made some mistakes or you have some better 
  implemented functions, let me know [francky74 (at) gmail (dot) com]
  Enjoy!

  Conversion by Legolas (http://itaprogaming.free.fr) for freepascal compiler
  (http://www.freepascal.org)
  
  Copyright (C) 2006  Francesco Lombardi
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
  
  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.
  
  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA
  ------------------------------------------------------------------------------
*)
unit gba_bios;
{$i def.inc}

interface

uses
  gba_types;
   
procedure SoftReset();cdecl; external;
procedure RegisterRamReset(flags: dword); cdecl; external;
procedure Halt(); cdecl; external;
procedure Stop(); cdecl; external;
procedure IntrWait(flagClear, irq: dword); cdecl; external;
procedure VBlankIntrWait(); cdecl; external;
function Divi(numerator, denominator: longint): longint;  cdecl; external;     
function DiviArm(denominator, numerator: longint): longint; cdecl; external;
function Sqrt(value: dword): dword; cdecl; external;
function ArcTan(ang: smallint): smallint; cdecl; external;
function ArcTan2(x, y: smallint): word; cdecl; external;
procedure CpuSet(source, dest: pointer; mode: dword); cdecl; external;
procedure CpuFastSet(source, dest: pointer; mode: dword); cdecl; external;
procedure ObjAffineSet(source: PObjAffineSource; dest: pointer; num: longint; offset: longint); cdecl; external;
procedure BgAffineSet(source: PBGAffineSource; dest: PBGAffineDest; num: longint); cdecl; external;
procedure BitUnPack(source: pointer; dest: pointer; _bup: PBUP); cdecl; external;
procedure LZ77UnCompWram(source, dest: pointer); cdecl; external;
procedure LZ77UnCompVram(source, dest: pointer); cdecl; external;
procedure HuffUnComp(source, dest: pointer); cdecl; external;
procedure RLUnCompWram(source, dest: pointer); cdecl; external;
procedure RLUnCompVram(source, dest: pointer); cdecl; external;
procedure Diff8bitUnFilterWram(source, dest: pointer); cdecl; external;
procedure Diff8bitUnFilterVram(source, dest: pointer); cdecl; external;
procedure Diff16bitUnFilter(source, dest: pointer); cdecl; external;
function MultiBoot(mb: PMultiBootParam; mode: dword): integer; cdecl; external;
procedure SoundBias(); cdecl; external;
procedure SoundDriverInit(); cdecl; external;
procedure SoundDriverMode(mode: dword); cdecl; external;
procedure SoundDriverMain(); cdecl; external;
procedure SoundDriverVSync(); cdecl; external;
procedure SoundChannelClear(); cdecl; external;
procedure MidiKey2Freq(); cdecl; external;
procedure SoundDriverVSyncOff(); cdecl; external;
procedure SoundDriverVSyncOn(); cdecl; external;
 
implementation

{$l bios_asm.o}

end.
