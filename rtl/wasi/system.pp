{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2020,2021 by the Free Pascal development team.

    System unit for The WebAssembly System Interface (WASI).

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}

unit system;

interface

{$define FPC_IS_SYSTEM}

{$I systemh.inc}

const
  LineEnding = #10;
  LFNSupport = true;
  DirectorySeparator = '/';
  DriveSeparator = '';
  ExtensionSeparator = '.';
  PathSeparator = ':';
  AllowDirectorySeparators : set of char = ['\','/'];
  AllowDriveSeparators : set of char = [];
{  FileNameCaseSensitive and FileNameCasePreserving are defined below! }
  maxExitCode = 65535;
  MaxPathLen = 4096;
  AllFilesMask = '*';

const
  UnusedHandle    = -1;
  StdInputHandle  = 0;
  StdOutputHandle = 1;
  StdErrorHandle  = 2;

  FileNameCaseSensitive : boolean = true;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = false; (* #26 not considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsLF;

var
  argc: longint;
  argv: PPChar;

procedure DebugWrite(const P: PChar);
procedure DebugWriteLn(const P: PChar);
procedure DebugWriteChar(Ch: Char);
procedure DebugWriteHexDigit(d: Byte);
procedure DebugWriteHexByte(b: Byte);
procedure DebugWriteHexWord(w: Word);
procedure DebugWriteHexLongWord(lw: LongWord);

implementation

{$I wasitypes.inc}
{$I wasiprocs.inc}

{$I system.inc}

var
  argv_size,
  argv_buf_size: __wasi_size_t;
  argv_buf: Pointer;

function GetProcessID: SizeUInt;
begin
end;

Procedure Randomize;
Begin
End;

procedure System_exit;
begin
  __wasi_proc_exit(ExitCode);
End;

procedure Setup_PreopenedDirs;
var
  fd: __wasi_fd_t;
  prestat: __wasi_prestat_t;
  res: __wasi_errno_t;
  prestat_dir_name: PChar;
begin
  fd:=3;
  repeat
    res:=__wasi_fd_prestat_get(fd, @prestat);
    if res=__WASI_ERRNO_SUCCESS then
    begin
      if (prestat.tag=__WASI_PREOPENTYPE_DIR) and (prestat.u.dir.pr_name_len>0) then
      begin
        //GetMem(prestat_dir_name,prestat.u.dir.pr_name_len+1);
        //if __wasi_fd_prestat_dir_name(fd,PByte(prestat_dir_name),prestat.u.dir.pr_name_len)=__WASI_ERRNO_SUCCESS then
        //begin
        //  prestat_dir_name[prestat.u.dir.pr_name_len]:=#0;
        //  //Writeln(prestat_dir_name);
        //end
        //else
        //  FreeMem(prestat_dir_name,prestat.u.dir.pr_name_len+1);
      end;
    end;
    Inc(fd);
  until res<>__WASI_ERRNO_SUCCESS;
end;

procedure setup_arguments;
begin
  if argv<>nil then
    exit;
  if __wasi_args_sizes_get(@argc, @argv_buf_size)<>__WASI_ERRNO_SUCCESS then
  begin
    argc:=0;
    argv:=nil;
    exit;
  end;
  argv_size:=(argc+1)*SizeOf(PChar);
  GetMem(argv, argv_size);
  GetMem(argv_buf, argv_buf_size);
  if __wasi_args_get(Pointer(argv), argv_buf)<>__WASI_ERRNO_SUCCESS then
  begin
    FreeMem(argv, argv_size);
    FreeMem(argv_buf, argv_buf_size);
    argc:=0;
    argv:=nil;
  end;
end;

Function ParamCount: Longint;
Begin
  if argv=nil then
    setup_arguments;
  paramcount := argc - 1;
End;

function paramstr(l: longint) : string;
begin
  if argv=nil then
    setup_arguments;
  if (l>=0) and (l+1<=argc) then
    paramstr:=strpas(argv[l])
  else
    paramstr:='';
end;

procedure SysInitStdIO;
begin
  OpenStdIO(Input,fmInput,StdInputHandle);
  OpenStdIO(Output,fmOutput,StdOutputHandle);
  OpenStdIO(ErrOutput,fmOutput,StdErrorHandle);
  OpenStdIO(StdOut,fmOutput,StdOutputHandle);
  OpenStdIO(StdErr,fmOutput,StdErrorHandle);
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
end;

procedure DebugWrite(const P: PChar);
var
  our_iov: __wasi_ciovec_t;
  our_nwritten: longint;
begin
  our_iov.buf := PByte(P);
  our_iov.buf_len := StrLen(P);
  __wasi_fd_write(1, @our_iov, 1, @our_nwritten);
end;

procedure DebugWriteLn(const P: PChar);
begin
  DebugWrite(P);
  DebugWriteChar(#10);
end;

procedure DebugWriteChar(Ch: Char);
var
  CharArr: array [0..1] of Char;
begin
  CharArr[0] := Ch;
  CharArr[1] := #0;
  DebugWrite(@CharArr);
end;

procedure DebugWriteHexDigit(d: Byte);
const
  HexDigits: array [0..15] of Char = '0123456789ABCDEF';
begin
  DebugWriteChar(HexDigits[d]);
end;

procedure DebugWriteHexByte(b: Byte);
begin
  DebugWriteHexDigit(b shr 4);
  DebugWriteHexDigit(b and 15);
end;

procedure DebugWriteHexWord(w: Word);
begin
  DebugWriteHexByte(w shr 8);
  DebugWriteHexByte(Byte(w));
end;

procedure DebugWriteHexLongWord(lw: LongWord);
begin
  DebugWriteHexWord(lw shr 16);
  DebugWriteHexWord(Word(lw));
end;

begin
  { To be set if this is a GUI or console application }
  IsConsole := TRUE;
{$ifdef FPC_HAS_FEATURE_DYNLIBS}
  { If dynlibs feature is disabled,
    IsLibrary is a constant, which can thus not be set to a value }
  { To be set if this is a library and not a program  }
  IsLibrary := FALSE;
{$endif def FPC_HAS_FEATURE_DYNLIBS}
  { Setup heap }
  InitHeap;
  SysInitExceptions;
  initunicodestringmanager;
  { Setup stdin, stdout and stderr }
  SysInitStdIO;
  { Reset IO Error }
  InOutRes:=0;
{$ifdef FPC_HAS_FEATURE_THREADING}
  InitSystemThreads;
{$endif}
  Setup_PreopenedDirs
end.
