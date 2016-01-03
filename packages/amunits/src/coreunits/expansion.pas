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
    History:

    Added the defines use_amiga_smartlink and
    use_auto_openlib. Implemented autoopening of
    the library.
    14 Jan 2003.

    Update for AmigaOS 3.9.
    Changed start code for unit.
    01 Feb 2003.

    Changed cardinal > longword.
    09 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT expansion;

INTERFACE
USES exec, configvars, amigados;

Const
    EXPANSIONNAME       : PChar = 'expansion.library';

{ flags for the AddDosNode() call }
    ADNB_STARTPROC      = 0;

    ADNF_STARTPROC      = 1;

VAR ExpansionBase : pLibrary;

FUNCTION AddBootNode(bootPri : LONGINT; flags : ULONG; deviceNode : pDeviceNode; configDev : pConfigDev) : BOOLEAN;
PROCEDURE AddConfigDev(configDev : pConfigDev);
FUNCTION AddDosNode(bootPri : LONGINT; flags : ULONG; deviceNode : pDeviceNode) : BOOLEAN;
PROCEDURE AllocBoardMem(slotSpec : ULONG);
FUNCTION AllocConfigDev : pConfigDev;
FUNCTION AllocExpansionMem(numSlots : ULONG; slotAlign : ULONG) : POINTER;
PROCEDURE ConfigBoard(board : POINTER; configDev : pConfigDev);
PROCEDURE ConfigChain(baseAddr : POINTER);
FUNCTION FindConfigDev(const oldConfigDev : pConfigDev; manufacturer : LONGINT; product : LONGINT) : pConfigDev;
PROCEDURE FreeBoardMem(startSlot : ULONG; slotSpec : ULONG);
PROCEDURE FreeConfigDev(configDev : pConfigDev);
PROCEDURE FreeExpansionMem(startSlot : ULONG; numSlots : ULONG);
FUNCTION GetCurrentBinding(const currentBinding : pCurrentBinding; bindingSize : ULONG) : ULONG;
FUNCTION MakeDosNode(const parmPacket : POINTER) : pDeviceNode;
PROCEDURE ObtainConfigBinding;
FUNCTION ReadExpansionByte(const board : POINTER; offset : ULONG) : BYTE;
PROCEDURE ReadExpansionRom(const board : POINTER; configDev : pConfigDev);
PROCEDURE ReleaseConfigBinding;
PROCEDURE RemConfigDev(configDev : pConfigDev);
PROCEDURE SetCurrentBinding(currentBinding : pCurrentBinding; bindingSize : ULONG);
PROCEDURE WriteExpansionByte(board : POINTER; offset : ULONG; byte : ULONG);

{Here we read how to compile this unit}
{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitEXPANSIONLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    EXPANSIONIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox;
{$endif dont_use_openlib}

FUNCTION AddBootNode(bootPri : LONGINT; flags : ULONG; deviceNode : pDeviceNode; configDev : pConfigDev) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  bootPri,D0
    MOVE.L  flags,D1
    MOVEA.L deviceNode,A0
    MOVEA.L configDev,A1
    MOVEA.L ExpansionBase,A6
    JSR -036(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE AddConfigDev(configDev : pConfigDev);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L configDev,A0
    MOVEA.L ExpansionBase,A6
    JSR -030(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AddDosNode(bootPri : LONGINT; flags : ULONG; deviceNode : pDeviceNode) : BOOLEAN;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  bootPri,D0
    MOVE.L  flags,D1
    MOVEA.L deviceNode,A0
    MOVEA.L ExpansionBase,A6
    JSR -150(A6)
    MOVEA.L (A7)+,A6
    TST.W   D0
    BEQ.B   @end
    MOVEQ   #1,D0
  @end: MOVE.B  D0,@RESULT
  END;
END;

PROCEDURE AllocBoardMem(slotSpec : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  slotSpec,D0
    MOVEA.L ExpansionBase,A6
    JSR -042(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION AllocConfigDev : pConfigDev;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ExpansionBase,A6
    JSR -048(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION AllocExpansionMem(numSlots : ULONG; slotAlign : ULONG) : POINTER;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  numSlots,D0
    MOVE.L  slotAlign,D1
    MOVEA.L ExpansionBase,A6
    JSR -054(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ConfigBoard(board : POINTER; configDev : pConfigDev);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L board,A0
    MOVEA.L configDev,A1
    MOVEA.L ExpansionBase,A6
    JSR -060(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ConfigChain(baseAddr : POINTER);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L baseAddr,A0
    MOVEA.L ExpansionBase,A6
    JSR -066(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION FindConfigDev(const oldConfigDev : pConfigDev; manufacturer : LONGINT; product : LONGINT) : pConfigDev;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L oldConfigDev,A0
    MOVE.L  manufacturer,D0
    MOVE.L  product,D1
    MOVEA.L ExpansionBase,A6
    JSR -072(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FreeBoardMem(startSlot : ULONG; slotSpec : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  startSlot,D0
    MOVE.L  slotSpec,D1
    MOVEA.L ExpansionBase,A6
    JSR -078(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeConfigDev(configDev : pConfigDev);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L configDev,A0
    MOVEA.L ExpansionBase,A6
    JSR -084(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE FreeExpansionMem(startSlot : ULONG; numSlots : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVE.L  startSlot,D0
    MOVE.L  numSlots,D1
    MOVEA.L ExpansionBase,A6
    JSR -090(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetCurrentBinding(const currentBinding : pCurrentBinding; bindingSize : ULONG) : ULONG;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L currentBinding,A0
    MOVE.L  bindingSize,D0
    MOVEA.L ExpansionBase,A6
    JSR -138(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

FUNCTION MakeDosNode(const parmPacket : POINTER) : pDeviceNode;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L parmPacket,A0
    MOVEA.L ExpansionBase,A6
    JSR -144(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ObtainConfigBinding;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ExpansionBase,A6
    JSR -120(A6)
    MOVEA.L (A7)+,A6
  END;
END;

FUNCTION ReadExpansionByte(const board : POINTER; offset : ULONG) : BYTE;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L board,A0
    MOVE.L  offset,D0
    MOVEA.L ExpansionBase,A6
    JSR -096(A6)
    MOVEA.L (A7)+,A6
    MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReadExpansionRom(const board : POINTER; configDev : pConfigDev);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L board,A0
    MOVEA.L configDev,A1
    MOVEA.L ExpansionBase,A6
    JSR -102(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE ReleaseConfigBinding;
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L ExpansionBase,A6
    JSR -126(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE RemConfigDev(configDev : pConfigDev);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L configDev,A0
    MOVEA.L ExpansionBase,A6
    JSR -108(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SetCurrentBinding(currentBinding : pCurrentBinding; bindingSize : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L currentBinding,A0
    MOVE.L  bindingSize,D0
    MOVEA.L ExpansionBase,A6
    JSR -132(A6)
    MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE WriteExpansionByte(board : POINTER; offset : ULONG; byte : ULONG);
BEGIN
  ASM
    MOVE.L  A6,-(A7)
    MOVEA.L board,A0
    MOVE.L  offset,D0
    MOVE.L  byte,D1
    MOVEA.L ExpansionBase,A6
    JSR -114(A6)
    MOVEA.L (A7)+,A6
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of expansion.library}
  {$Info don't forget to use InitEXPANSIONLibrary in the beginning of your program}

var
    expansion_exit : Pointer;

procedure CloseexpansionLibrary;
begin
    ExitProc := expansion_exit;
    if ExpansionBase <> nil then begin
        CloseLibrary(ExpansionBase);
        ExpansionBase := nil;
    end;
end;

procedure InitEXPANSIONLibrary;
begin
    ExpansionBase := nil;
    ExpansionBase := OpenLibrary(EXPANSIONNAME,LIBVERSION);
    if ExpansionBase <> nil then begin
        expansion_exit := ExitProc;
        ExitProc := @CloseexpansionLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open expansion.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    EXPANSIONIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of expansion.library}

var
    expansion_exit : Pointer;

procedure CloseexpansionLibrary;
begin
    ExitProc := expansion_exit;
    if ExpansionBase <> nil then begin
        CloseLibrary(ExpansionBase);
        ExpansionBase := nil;
    end;
end;

begin
    ExpansionBase := nil;
    ExpansionBase := OpenLibrary(EXPANSIONNAME,LIBVERSION);
    if ExpansionBase <> nil then begin
        expansion_exit := ExitProc;
        ExitProc := @CloseexpansionLibrary;
        EXPANSIONIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open expansion.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    EXPANSIONIsCompiledHow := 3;
   {$Warning No autoopening of expansion.library compiled}
   {$Warning Make sure you open expansion.library yourself}
{$endif dont_use_openlib}


END. (* UNIT EXPANSION *)



