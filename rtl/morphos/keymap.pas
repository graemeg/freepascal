{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by Karoly Balogh

    keymap.library interface unit for MorphOS/PowerPC
    Based on the Commodore Amiga/68k port by Nils Sjoholm

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
{$PACKRECORDS 2}
unit keymap;

INTERFACE

uses exec, inputevent;

Type

    pKeyMap = ^tKeyMap;
    tKeyMap = record
        km_LoKeyMapTypes        : Pointer;
        km_LoKeyMap             : Pointer;
        km_LoCapsable           : Pointer;
        km_LoRepeatable         : Pointer;
        km_HiKeyMapTypes        : Pointer;
        km_HiKeyMap             : Pointer;
        km_HiCapsable           : Pointer;
        km_HiRepeatable         : Pointer;
    end;


    pKeymapNode = ^tKeyMapNode;
    tKeyMapNode = record
        kn_Node         : tNode;         { including name of keymap }
        kn_KeyMap       : tKeyMap;
    end;

{ the structure of keymap.resource }

    pKeyMapResource = ^tKeyMapResource;
    tKeyMapResource = record
        kr_Node         : tNode;
        kr_List         : tList;         { a list of KeyMapNodes }
    end;


Const

{ Key Map Types }

    KC_NOQUAL           = 0;
    KC_VANILLA          = 7;    { note that SHIFT+ALT+CTRL is VANILLA }
    KCB_SHIFT           = 0;
    KCF_SHIFT           = $01;
    KCB_ALT             = 1;
    KCF_ALT             = $02;
    KCB_CONTROL         = 2;
    KCF_CONTROL         = $04;
    KCB_DOWNUP          = 3;
    KCF_DOWNUP          = $08;

    KCB_DEAD            = 5;    { may be dead or modified by dead key:  }
    KCF_DEAD            = $20;  {   use dead prefix bytes               }

    KCB_STRING          = 6;
    KCF_STRING          = $40;

    KCB_NOP             = 7;
    KCF_NOP             = $80;


{ Dead Prefix Bytes }

    DPB_MOD             = 0;
    DPF_MOD             = $01;
    DPB_DEAD            = 3;
    DPF_DEAD            = $08;

    DP_2DINDEXMASK      = $0f;  { mask for index for 1st of two dead keys }
    DP_2DFACSHIFT       = 4;    { shift for factor for 1st of two dead keys }

var 
  KeymapBase : pLibrary;

const
    KEYMAPNAME : PChar = 'keymap.library';

procedure SetKeyMapDefault(CONST keyMap : pKeyMap location 'a0'); 
SysCall KeymapBase 030;

function AskKeyMapDefault : pKeyMap;
SysCall KeymapBase 036;

function MapRawKey(CONST event : pInputEvent location 'a0'; buffer : pSHORTINT location 'a1'; length : longint location 'd1'; CONST keyMap : pKeyMap location 'a2') : INTEGER;
SysCall KeymapBase 042;

function MapANSI(CONST strg : pSHORTINT location 'a0'; count : longint location 'd0'; buffer : pSHORTINT location 'a1'; length : longint location 'd1'; CONST keyMap : pKeyMap location 'a2') : longint;
SysCall KeymapBase 048;

{ Helper calls }
function InitKeymapLibrary : boolean;

implementation

const
  { Change VERSION and LIBVERSION to proper values }
  VERSION : string[2] = '50';
  LIBVERSION : longword = 50;

var
  keymap_exit : Pointer;

procedure CloseKeymapLibrary;
begin
  ExitProc := keymap_exit;
  if KeymapBase <> nil then begin
    CloseLibrary(PLibrary(KeymapBase));
    KeymapBase := nil;
  end;
end;

function InitKeymapLibrary : boolean;
begin
  KeymapBase := nil;
  KeymapBase := OpenLibrary(KEYMAPNAME,LIBVERSION);
  if KeymapBase <> nil then begin
    keymap_exit := ExitProc;
    ExitProc := @CloseKeymapLibrary;
    InitKeymapLibrary:=True;
  end else begin
    InitKeymapLibrary:=False;
  end;
end;

end.
