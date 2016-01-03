{
    This file is part of the Free Pascal run time library.

    A file in Amiga system run time library.
    Copyright (c) 2000-2003 by Nils Sjoholm
    member of the Amiga RTL development team.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

{
    History:

    First version.
    15 Oct 2000.

    Updated for fpc 1.0.7
    16 Jan 2003.

    Changed integer > smallint.
    Changed startcode for unit.
    12 Feb 2003.

    nils.sjoholm@mailbox.swipnet.se

}

{$I useamigasmartlink.inc}
{$ifdef use_amiga_smartlink}
   {$smartlink on}
{$endif use_amiga_smartlink}

UNIT ptreplay;

INTERFACE

USES Exec;

 const
     PTREPLAYNAME : PChar = 'ptreplay.library';
  { The rest is private for now, but more details may be released later.  }

  type
     PModule = ^TModule;
     TModule = record
          mod_Name : PChar;
     { The rest is private for now, but more details may be released later.  }
     end;

     { This structure is returned by GetSample function  }
     PPTSample = ^TPTSample;
     TPTSample = record
          Name     : array[0..21] of Char; { Null terminated string with samplename  }
          Length   :  WORD;                { Sample length in words  }
          FineTune : BYTE;                 { FineTune of sample in lower 4 bits  }
          Volume   : BYTE;                 { Volume of sample  }
          Repeat_  : WORD;                 { Repeat start in number of words  }
          Replen   : WORD;                 { Repeat length in number of words  }
       end;

VAR PTReplayBase : pLibrary;

FUNCTION PTLoadModule(name : pCHAR) : pModule;
PROCEDURE PTUnloadModule(module : pModule);
FUNCTION PTPlay(module : pModule) : ULONG;
FUNCTION PTStop(module : pModule) : ULONG;
FUNCTION PTPause(module : pModule) : ULONG;
FUNCTION PTResume(module : pModule) : ULONG;
PROCEDURE PTFade(module : pModule; speed : BYTE);
PROCEDURE PTSetVolume(module : pModule; vol : BYTE);
FUNCTION PTSongPos(module : pModule) : BYTE;
FUNCTION PTSongLen(module : pModule) : BYTE;
FUNCTION PTSongPattern(module : pModule; Pos : WORD) : BYTE;
FUNCTION PTPatternPos(Module : pModule) : BYTE;
FUNCTION PTPatternData(Module : pModule; Pattern : BYTE; Row : BYTE) : POINTER;
PROCEDURE PTInstallBits(Module : pModule; Restart : SHORTINT; NextPattern : SHORTINT; NextRow : SHORTINT; Fade : SHORTINT);
FUNCTION PTSetupMod(ModuleFile : POINTER) : pModule;
PROCEDURE PTFreeMod(Module : pModule);
PROCEDURE PTStartFade(Module : pModule; speed : BYTE);
PROCEDURE PTOnChannel(Module : pModule; Channels : SHORTINT);
PROCEDURE PTOffChannel(Module : pModule; Channels : SHORTINT);
PROCEDURE PTSetPos(Module : pModule; Pos : BYTE);
PROCEDURE PTSetPri(Pri : SHORTINT);
FUNCTION PTGetPri : SHORTINT;
FUNCTION PTGetChan : SHORTINT;
FUNCTION PTGetSample(Module : pModule; Nr : smallint) : pPTSample;

FUNCTION PTLoadModule(name : string) : pModule;

{You can remove this include and use a define instead}
{$I useautoopenlib.inc}
{$ifdef use_init_openlib}
procedure InitPTREPLAYLibrary;
{$endif use_init_openlib}

{This is a variable that knows how the unit is compiled}
var
    PTREPLAYIsCompiledHow : longint;

IMPLEMENTATION

uses
{$ifndef dont_use_openlib}
msgbox,
{$endif dont_use_openlib}
pastoc;

FUNCTION PTLoadModule(name : pCHAR) : pModule;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L name,A0
        MOVEA.L PTReplayBase,A6
        JSR     -030(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE PTUnloadModule(module : pModule);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -036(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION PTPlay(module : pModule) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -042(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PTStop(module : pModule) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -048(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PTPause(module : pModule) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -054(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PTResume(module : pModule) : ULONG;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -060(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE PTFade(module : pModule; speed : BYTE);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVE.L  speed,D0
        MOVEA.L PTReplayBase,A6
        JSR     -066(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTSetVolume(module : pModule; vol : BYTE);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVE.L  vol,D0
        MOVEA.L PTReplayBase,A6
        JSR     -072(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION PTSongPos(module : pModule) : BYTE;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -078(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTSongLen(module : pModule) : BYTE;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -084(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTSongPattern(module : pModule; Pos : WORD) : BYTE;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L module,A0
        MOVE.L  Pos,D0
        MOVEA.L PTReplayBase,A6
        JSR     -090(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTPatternPos(Module : pModule) : BYTE;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -096(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTPatternData(Module : pModule; Pattern : BYTE; Row : BYTE) : POINTER;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  Pattern,D0
        MOVE.L  Row,D1
        MOVEA.L PTReplayBase,A6
        JSR     -102(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE PTInstallBits(Module : pModule; Restart : SHORTINT; NextPattern : SHORTINT; NextRow : SHORTINT; Fade : SHORTINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  Restart,D0
        MOVE.L  NextPattern,D1
        MOVE.L  NextRow,D2
        MOVE.L  Fade,D3
        MOVEA.L PTReplayBase,A6
        JSR     -108(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION PTSetupMod(ModuleFile : POINTER) : pModule;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L ModuleFile,A0
        MOVEA.L PTReplayBase,A6
        JSR     -114(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

PROCEDURE PTFreeMod(Module : pModule);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVEA.L PTReplayBase,A6
        JSR     -120(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTStartFade(Module : pModule; speed : BYTE);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  speed,D0
        MOVEA.L PTReplayBase,A6
        JSR     -126(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTOnChannel(Module : pModule; Channels : SHORTINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  Channels,D0
        MOVEA.L PTReplayBase,A6
        JSR     -132(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTOffChannel(Module : pModule; Channels : SHORTINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  Channels,D0
        MOVEA.L PTReplayBase,A6
        JSR     -138(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTSetPos(Module : pModule; Pos : BYTE);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.L  Pos,D0
        MOVEA.L PTReplayBase,A6
        JSR     -144(A6)
        MOVEA.L (A7)+,A6
  END;
END;

PROCEDURE PTSetPri(Pri : SHORTINT);
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVE.L  Pri,D0
        MOVEA.L PTReplayBase,A6
        JSR     -150(A6)
        MOVEA.L (A7)+,A6
  END;
END;

FUNCTION PTGetPri : SHORTINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PTReplayBase,A6
        JSR     -156(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTGetChan : SHORTINT;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L PTReplayBase,A6
        JSR     -162(A6)
        MOVEA.L (A7)+,A6
        MOVE.B  D0,@RESULT
  END;
END;

FUNCTION PTGetSample(Module : pModule; Nr : smallint) : pPTSample;
BEGIN
  ASM
        MOVE.L  A6,-(A7)
        MOVEA.L Module,A0
        MOVE.W  Nr,D0
        MOVEA.L PTReplayBase,A6
        JSR     -168(A6)
        MOVEA.L (A7)+,A6
        MOVE.L  D0,@RESULT
  END;
END;

FUNCTION PTLoadModule(name : string) : pModule;
begin
    PTLoadModule := PTLoadModule(pas2c(name));
end;

const
    { Change VERSION and LIBVERSION to proper values }

    VERSION : string[2] = '0';
    LIBVERSION : longword = 0;

{$ifdef use_init_openlib}
  {$Info Compiling initopening of ptreplay.library}
  {$Info don't forget to use InitPTREPLAYLibrary in the beginning of your program}

var
    ptreplay_exit : Pointer;

procedure CloseptreplayLibrary;
begin
    ExitProc := ptreplay_exit;
    if PTReplayBase <> nil then begin
        CloseLibrary(PTReplayBase);
        PTReplayBase := nil;
    end;
end;

procedure InitPTREPLAYLibrary;
begin
    PTReplayBase := nil;
    PTReplayBase := OpenLibrary(PTREPLAYNAME,LIBVERSION);
    if PTReplayBase <> nil then begin
        ptreplay_exit := ExitProc;
        ExitProc := @CloseptreplayLibrary;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ptreplay.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;
end;

begin
    PTREPLAYIsCompiledHow := 2;
{$endif use_init_openlib}

{$ifdef use_auto_openlib}
  {$Info Compiling autoopening of ptreplay.library}

var
    ptreplay_exit : Pointer;

procedure CloseptreplayLibrary;
begin
    ExitProc := ptreplay_exit;
    if PTReplayBase <> nil then begin
        CloseLibrary(PTReplayBase);
        PTReplayBase := nil;
    end;
end;

begin
    PTReplayBase := nil;
    PTReplayBase := OpenLibrary(PTREPLAYNAME,LIBVERSION);
    if PTReplayBase <> nil then begin
        ptreplay_exit := ExitProc;
        ExitProc := @CloseptreplayLibrary;
        PTREPLAYIsCompiledHow := 1;
    end else begin
        MessageBox('FPC Pascal Error',
        'Can''t open ptreplay.library version ' + VERSION + #10 +
        'Deallocating resources and closing down',
        'Oops');
        halt(20);
    end;

{$endif use_auto_openlib}

{$ifdef dont_use_openlib}
begin
    PTREPLAYIsCompiledHow := 3;
   {$Warning No autoopening of ptreplay.library compiled}
   {$Warning Make sure you open ptreplay.library yourself}
{$endif dont_use_openlib}


END. (* UNIT PTREPLAY *)






