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

interface

uses
  exec, inputevent;

type
  PKeyMap = ^TKeyMap;
  TKeyMap = record
    km_LoKeyMapTypes: PByte;
    km_LoKeyMap: PLongWord;
    km_LoCapsable: PByte;
    km_LoRepeatable: PByte;
    km_HiKeyMapTypes: PByte;
    km_HiKeyMap: PLongWord;
    km_HiCapsable: PByte;
    km_HiRepeatable: PByte;
  end;

  PKeymapNode = ^TKeyMapNode;
  TKeyMapNode = record
    kn_Node: TNode;      // including name of keymap
    kn_KeyMap: TKeyMap;
  end;

  PExtendedKeyMapNode = ^TExtendedKeyMapNode;
  TExtendedKeyMapNode = record
    ekn_Node: TNode;
    ekn_KeyMap: TKeyMap;
    pad1: Word; 
    ekn_Seglist: BPTR;
    ekn_Resident: PResident;
    ekn_Future0: APTR;        // keep 0 for now
  end;

// the structure of keymap.resource
  PKeyMapResource = ^TKeyMapResource;
  TKeyMapResource = record
    kr_Node: TNode;
    kr_List: TList;  // a list of KeyMapNodes
  end;

const
// Key Map Types
  KC_NOQUAL   = 0;
  KC_VANILLA  = 7;   // note that SHIFT+ALT+CTRL is VANILLA

  KCB_SHIFT   = 0;
  KCF_SHIFT   = 1 shl KCB_SHIFT;
  KCB_ALT     = 1;
  KCF_ALT     = 1 shl KCB_ALT;
  KCB_CONTROL = 2;
  KCF_CONTROL = 1 shl KCB_CONTROL;
  KCB_DOWNUP  = 3;
  KCF_DOWNUP  = 1 shl KCB_DOWNUP;

  KCB_DEAD    = 5;              // may be dead or modified by dead key:
  KCF_DEAD    = 1 shl KCB_DEAD; // use dead prefix bytes

  KCB_STRING  = 6;
  KCF_STRING  = 1 shl KCB_STRING;

  KCB_NOP     = 7;
  KCF_NOP     = 1 shl KCB_NOP;

  // Dead Prefix Bytes
  DPB_MOD  = 0;
  DPF_MOD  = 1 shl DPB_MOD;
  DPB_DEAD = 3;
  DPF_DEAD = 1 shl DPB_DEAD;

  DP_2DINDEXMASK = $0f; // mask for index for 1st of two dead keys
  DP_2DFACSHIFT  = 4;   // shift for factor for 1st of two dead keys

type
  PUCS4_ConvTable = ^TUCS4_ConvTable;
  TUCS4_ConvTable = record
    FirstChar: Word;
    LastChar: Word;
    ConvTable: APTR; // Either pointer to Byte or LongWord
  end;

  PUCS4_CharsetCode = ^TUCS4_CharsetCode;
  TUCS4_CharsetCode = record
    UCS4: LongWord;
    CharsetCode: LongWord;
  end;

  PUCS4_CharsetConvTable = ^TUCS4_CharsetConvTable;
  TUCS4_CharsetConvTable = record
    Mapping: PUCS4_CharsetCode;  // An optional array, terminated with  (0, 0) entry
    ConvTables: array[0..0] of TUCS4_ConvTable; // 0 sized array
  end;

var
  KeyMapBase: PLibrary = nil;

const
  KEYMAPNAME: PChar = 'keymap.library';

procedure SetKeyMapDefault(const KeyMap: PKeyMap location 'a0'); SysCall KeyMapBase 030;
function AskKeyMapDefault: PKeyMap; SysCall KeyMapBase 036;
function MapRawKey(const Event: PInputEvent location 'a0'; Buffer: STRPTR location 'a1'; Length: LongInt location 'd1'; const KeyMap: PKeyMap location 'a2'): LongInt; SysCall KeyMapBase 042;
function MapANSI(const Strg: STRPTR location 'a0'; Count: LongInt location 'd0'; Buffer: STRPTR location 'a1'; Length: LongInt location 'd1'; const KeyMap: PKeyMap location 'a2'): LongInt; SysCall KeyMapBase 048;
function MapRawKeyUCS4(const Event: PInputEvent location 'a0'; Buffer: WSTRPTR location 'a1'; Length: LongInt location 'd1'; const KeyMap: PKeyMap location 'a2'): LongInt; SysCall KeyMapBase 54;
function MapUCS4(const Strg: WSTRPTR location 'a0'; Count: LongInt location 'd0'; Buffer: STRPTR location 'a1'; Length: LongInt location 'd1'; const KeyMap: PKeyMap location 'a2'): LongInt; SysCall KeyMapBase 60;
function ToANSI(UCS4Char: WideChar location 'a0'; const KeyMap: PKeyMap location 'a1'): Char; SysCall KeyMapBase 66;
function ToUCS4(ASCIIChar: Char location 'a0'; const KeyMap: PKeyMap location 'a1'): WideChar; SysCall KeyMapBase 72;
function GetKeyMapCodePage(const KeyMap: PKeyMap location 'a0'): STRPTR; SysCall KeyMapBase 78;

// Helper calls
function InitKeymapLibrary : boolean;

implementation

const
  LIBVERSION: LongWord = 50;

function InitKeymapLibrary: boolean;
begin
  InitKeyMapLibrary := Assigned(KeyMapBase);
end;

initialization
  KeyMapBase := OpenLibrary(KEYMAPNAME, LIBVERSION);
finalization
  if Assigned(KeyMapBase) then
    CloseLibrary(PLibrary(KeyMapBase));
end.
