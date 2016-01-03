(*
  gba_input.pas 01/09/2006 19.57.16
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
unit gba_input;
{$i def.inc}
interface

uses 
  gba_types, gba_regs;



const
  KEY_A: TKeyPadBits       = (1 shl 0);
  KEY_B: TKeyPadBits       = (1 shl 1);
  KEY_SELECT: TKeyPadBits  = (1 shl 2);
  KEY_START: TKeyPadBits   = (1 shl 3);	
  KEY_RIGHT: TKeyPadBits   = (1 shl 4);
  KEY_LEFT: TKeyPadBits    = (1 shl 5);
  KEY_UP: TKeyPadBits      = (1 shl 6);
  KEY_DOWN: TKeyPadBits    = (1 shl 7);
  KEY_R: TKeyPadBits       = (1 shl 8);
  KEY_L: TKeyPadBits       = (1 shl 9);

  KEYIRQ_ENABLE: TKeyPadBits = (1 shl 14);
  KEYIRQ_OR: TKeyPadBits     = (0 shl 15);
  KEYIRQ_AND: TKeyPadBits    = (1 shl 15);
  DPAD: TKeyPadBits          = (1 shl 6) or (1 shl 7) or (1 shl 5) or (1 shl 4);



type
  KeyInput = packed record
    Up: word;
    Down: word;
    Held: word;
    Last: word;
    DownRepeat: word;
  end;
  TKeyInput = KeyInput;
//---------------------------------------------------------------------------------
// Global variables
//---------------------------------------------------------------------------------

var
  Keys: TKeyInput;
  delay: byte = 60;
  rept: byte = 30; 
  count: byte = 60;

procedure ScanKeys();
function KeysDown(): word;
function KeysDownRepeat(): word;
function KeysUp(): word;
function KeysHeld(): word;
procedure SetRepeat(SetDelay, SetRepeat: integer);




implementation

procedure SetRepeat(SetDelay, SetRepeat: integer);
begin
  delay := SetDelay;
  rept := SetRepeat;
end;

//---------------------------------------------------------------------------------
procedure ScanKeys();
var
  pressed, released: word;
begin
  Keys.Last := Keys.Held;
  Keys.Held := (REG_KEYINPUT^ and $03ff) xor $03ff; // upper 6 bits clear on hw not emulated


	pressed := Keys.Held and ( Keys.Last xor $03ff);

	Keys.DownRepeat	:= Keys.DownRepeat or pressed;
	Keys.Down := Keys.Down or pressed;


	released := ((Keys.Held xor $03ff) and Keys.Last);

	Keys.Up	:= Keys.Up or released;

	Keys.Down := Keys.Down and not released;
	Keys.DownRepeat := Keys.DownRepeat and not released;

	Keys.Up := Keys.Up and not pressed;

	if ( Keys.Last <> Keys.Held) then 
    count := delay;


	if ( delay <> 0) then
	begin
		dec(count);
		if (count = 0) then
		begin
			count := rept;
			Keys.DownRepeat := Keys.DownRepeat or Keys.Held;
		end;
	end;
end;

function KeysDownRepeat(): word;
var
	tmp: word; 
begin
  tmp := Keys.DownRepeat;
	Keys.DownRepeat := 0;

	KeysDownRepeat := tmp;
end;

function KeysDown(): word;
var
  tmp: word;
begin
  tmp := Keys.Down;
  Keys.Down := 0;

	KeysDown := tmp;
end;

function KeysUp(): word;
var
  tmp: word;
begin
  tmp := Keys.Up;
	Keys.Up := 0;

	KeysUp := tmp;
end;

function KeysHeld(): word;
begin
  KeysHeld := Keys.Held;
end;

end.
