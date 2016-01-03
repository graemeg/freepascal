{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 1998-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
        keymap.resource definitions and console.device key map definitions
}

{
    History:

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening
    of the library.
    14 Jan 2003.

    Changed integer > smallint,
            cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

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

VAR KeymapBase : pLibrary;

const
    KEYMAPNAME : PChar = 'keymap.library';

FUNCTION AskKeyMapDefault : pKeyMap;
FUNCTION MapANSI(thestring : pCHAR; count : LONGINT; buffer : pCHAR; length : LONGINT; keyMap : pKeyMap) : LONGINT;
FUNCTION MapRawKey(event : pInputEvent; buffer : pCHAR; length : LONGINT; keyMap : pKeyMap) : smallint;
PROCEDURE SetKeyMapDefault(keyMap : pKeyMap);

IMPLEMENTATION

uses msgbox;

FUNCTION AskKeyMapDefault : pKeyMap;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L KeymapBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MapANSI(thestring : pCHAR; count : LONGINT; buffer : pCHAR; length : LONGINT; keyMap : pKeyMap) : LONGINT;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L thestring,A0
    MOVE.L  count,D0
    MOVEA.L buffer,A1
    MOVE.L  length,D1
    MOVEA.L keyMap,A2
    MOVEA.L KeymapBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MapRawKey(event : pInputEvent; buffer : pCHAR; length : LONGINT; keyMap : pKeyMap) : smallint;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L event,A0
    MOVEA.L buffer,A1
    MOVE.L  length,D1
    MOVEA.L keyMap,A2
    MOVEA.L KeymapBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetKeyMapDefault(keyMap : pKeyMap);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L keyMap,A0
    MOVEA.L KeymapBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
  END;
END;

{$I useautoopenlib.inc}
{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of keymap.library}

var
    keymap_exit : Pointer;

procedure ClosekeymapLibrary;
begin
    ExitProc := keymap_exit;
    if KeymapBase <> nil then begin
        CloseLibrary(KeymapBase);
        KeymapBase := nil;
    end;
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

begin
    KeymapBase := nil;
    KeymapBase := OpenLibrary(KEYMAPNAME,LIBVERSION);
    if KeymapBase <> nil then begin
        keymap_exit := ExitProc;
        ExitProc := @ClosekeymapLibrary
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open keymap.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$else}
   {$Warning No autoopening of keymap.library compiled}
   {$Info Make sure you open keymap.library yourself}
{$endif use_auto_openlib}


END. (* UNIT KEYMAP *)



