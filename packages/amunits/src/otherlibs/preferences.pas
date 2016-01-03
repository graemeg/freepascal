{
  This file is part of the Free Pascal run time library.

  A file in Amiga system run time library.
  Copyright (c) 2003 by Nils Sjöholm.
  member of the Amiga RTL development team.

  This is a unit for preferences.library

  See the file COPYING.FPC, included in this distribution,
  for details about the copyright.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

**********************************************************************}
{
  History:

  First version of this unit.
  17 Jan 2003.

  Changed cardinal > longword.
  Changed startcode for unit.
  12 Feb 2003.

  nils.sjoholm@mailbox.swipnet.se Nils Sjoholm
}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT PREFERENCES;

INTERFACE
USES Exec;

VAR PreferencesBase : pLibrary;

const
    PREFERENCESNAME : PChar = 'preferences.library';


  {
      $VER: preferences.h 39.6 (26.07.2000)

          preferences.library include

      (C) Copyright 2000 Satanic Dreams Software
      All Rights Reserved
   }
  { followed by the data, a total of ps_Size bytes  }

  type
     PPrefsStruct = ^tPrefsStruct;
     tPrefsStruct = record
          ps_Size : WORD;
       end;

  { -- Scalos specific Prefs: }
  { ------------------ ScalosPrefs ------------------------- }
  { Name: "Scalos" or "ScalosPrefs" }
  { ID: "MAIN" }
  { struct [8] }

  const
     SCP_IconOffsets = $80000001;
  { word }
     SCP_IconNormFrame = $80000002;
  { word }
     SCP_IconSelFrame = $80000003;
  { word }
     SCP_IconTextMode = $80000004;
  { word }
     SCP_IconSecLine = $80000005;
  { word }
     SCP_IconTextSkip = $80000006;
  { byte image or image+text }
     SCP_BobsType = $80000010;
  { byte system or custom }
     SCP_BobsMethod = $80000011;
  { byte solid or transparent }
     SCP_BobsTranspMode = $80000012;
  { byte ghosted or real transparent }
     SCP_BobsTranspType = $80000013;
  { long when transparent }
     SCP_BobsTransp = $80000014;
  { string }
     SCP_ScreenTitle = $80000020;
  { string }
     SCP_RootWinTitle = $80000021;
  { string }
     SCP_WindowTitle = $80000022;
  { byte }
     SCP_Separator = $80000023;
  { byte }
     SCP_TitleRefresh = $80000024;
  { string }
     SCP_PathsDefIcons = $80000025;
  { string }
     SCP_PathsDiskCopy = $80000026;
  { string }
     SCP_PathsWBStartup = $80000027;
  { string }
     SCP_PathsHome = $80000028;
  { byte }
     SCP_MiscAutoRemove = $80000029;
  { byte }
     SCP_MiscClickTransp = $8000002a;
  { byte }
     SCP_MiscHardEmulation = $8000002b;
  { byte }
     SCP_MiscUseExAll = $8000002c;
  { byte }
     SCP_MiscWindowType = $8000002d;
  { byte }
     SCP_MiscDoWaitDelay = $8000002e;
  { byte delay }
     SCP_MiscDiskiconsRefresh = $8000002f;
  { byte }
     SCP_MiscMenuCurrentDir = $80000030;
  { byte }
     SCP_NewIconsTransparent = $80000031;
  { long }
     SCP_NewIconsPrecision = $80000032;
  { string }
     SCP_TextModeFont = $80000033;
  { byte }
     SCP_TextModeDateFormat = $80000034;
  { list }
     SCP_PlugInList = $80000035;



FUNCTION AllocPrefsHandle(name : pCHAR) : POINTER;
FUNCTION FindPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword) : pPrefsStruct;
PROCEDURE FreePrefsHandle(PrefsHandle : POINTER);
FUNCTION GetEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD; Entry : longword) : longword;
FUNCTION GetPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD) : longword;
PROCEDURE ReadPrefsHandle(PrefsHandle : POINTER; Filename : pCHAR);
FUNCTION RemEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; Entry : longword) : longword;
PROCEDURE SetEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD; Entry : longword);
PROCEDURE SetPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD);
PROCEDURE WritePrefsHandle(PrefsHandle : POINTER; Filename : pCHAR);

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitPREFERENCESLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    PREFERENCESIsCompiledHow : longint;


IMPLEMENTATION

{$ifndef dont_use_openlib}
uses msgbox;
{$endif dont_use_openlib}

FUNCTION AllocPrefsHandle(name : pCHAR) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L PreferencesBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION FindPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword) : pPrefsStruct;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVEA.L PreferencesBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE FreePrefsHandle(PrefsHandle : POINTER);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVEA.L PreferencesBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION GetEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD; Entry : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVEA.L a1arg,A1
        MOVE.L  Struct_Size,D2
        MOVE.L  Entry,D3
        MOVEA.L PreferencesBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION GetPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVEA.L a1arg,A1
        MOVE.L  Struct_Size,D2
        MOVEA.L PreferencesBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE ReadPrefsHandle(PrefsHandle : POINTER; Filename : pCHAR);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVEA.L Filename,A1
        MOVEA.L PreferencesBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION RemEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; Entry : longword) : longword;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVE.L  Entry,D2
        MOVEA.L PreferencesBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE SetEntry(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD; Entry : longword);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVEA.L a1arg,A1
        MOVE.L  Struct_Size,D2
        MOVE.L  Entry,D3
        MOVEA.L PreferencesBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE SetPreferences(PrefsHandle : POINTER; ID : longword; d1arg : longword; a1arg : POINTER; Struct_Size : WORD);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVE.L  ID,D0
        MOVE.L  d1arg,D1
        MOVEA.L a1arg,A1
        MOVE.L  Struct_Size,D2
        MOVEA.L PreferencesBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE WritePrefsHandle(PrefsHandle : POINTER; Filename : pCHAR);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PrefsHandle,A0
        MOVEA.L Filename,A1
        MOVEA.L PreferencesBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
  END;
END;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of preferences.library}
  {$Info don't forget to use InitPREFERENCESLibrary in the beginning of your program}

var
    preferences_exit : Pointer;

procedure ClosepreferencesLibrary;
begin
    ExitProc := preferences_exit;
    if PreferencesBase <> nil then begin
        CloseLibrary(PreferencesBase);
        PreferencesBase := nil;
    end;
end;

procedure InitPREFERENCESLibrary;
begin
    PreferencesBase := nil;
    PreferencesBase := OpenLibrary(PREFERENCESNAME,LIBVERSION);
    if PreferencesBase <> nil then begin
        preferences_exit := ExitProc;
        ExitProc := @ClosepreferencesLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open preferences.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    PREFERENCESIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of preferences.library}

var
    preferences_exit : Pointer;

procedure ClosepreferencesLibrary;
begin
    ExitProc := preferences_exit;
    if PreferencesBase <> nil then begin
        CloseLibrary(PreferencesBase);
        PreferencesBase := nil;
    end;
end;

begin
    PreferencesBase := nil;
    PreferencesBase := OpenLibrary(PREFERENCESNAME,LIBVERSION);
    if PreferencesBase <> nil then begin
        preferences_exit := ExitProc;
        ExitProc := @ClosepreferencesLibrary;
        PREFERENCESIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open preferences.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    PREFERENCESIsCompiledHow := 3;
   {$Warning No autoopening of preferences.library compiled}
   {$Warning Make sure you open preferences.library yourself}
{$endif dont_use_openlib}


END. (* UNIT PREFERENCES *)



