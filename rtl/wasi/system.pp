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
  preopened_dirs_count: longint;
  preopened_dirs: PPChar;
  drives_count: longint;
  current_dirs: PPChar;
  current_dir_fds: Plongint;
  current_drive: longint;

function ConvertToFdRelativePath(path: PChar; out fd: LongInt; out relfd_path: PChar): Boolean;

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

function HasDriveLetter(const path: PChar): Boolean;
begin
  HasDriveLetter:=(path<>nil) and (UpCase(path[0]) in ['A'..'Z']) and (path[1] = ':');
end;

function ConvertToFdRelativePath(path: PChar; out fd: LongInt; out relfd_path: PChar): Boolean;
var
  drive_nr,I,pdir_drive,longest_match,pdir_length: longint;
  IsAbsolutePath: Boolean;
  pdir, savepath: PChar;
begin
  fd:=0;
  relfd_path:=nil;

  if HasDriveLetter(path) then
  begin
    drive_nr:=Ord(UpCase(path[0]))-(Ord('A')-1);
    inc(path,2);
  end
  else
    drive_nr:=current_drive;
  if path[0] in ['/','\'] then
  begin
    { path is absolute. Try to find it in the preopened dirs array }
    InOutRes:=3;
    ConvertToFdRelativePath:=false;
    longest_match:=0;
    savepath:=path;
    for I:=0 to preopened_dirs_count-1 do
    begin
      path:=savepath;
      pdir:=preopened_dirs[I];
      if HasDriveLetter(pdir) then
      begin
        pdir_drive:=Ord(UpCase(pdir[0]))-(Ord('A')-1);
        Inc(pdir,2);
      end
      else
        pdir_drive:=0;
      if pdir_drive<>drive_nr then
        continue;
      pdir_length:=StrLen(pdir);
      if pdir_length>StrLen(path) then
        continue;
      if CompareByte(path^,pdir^,pdir_length)<>0 then
        continue;
      Inc(path,pdir_length);
      if not (path^ in [#0,'/','\']) then
        continue;
      if pdir_length>longest_match then
      begin
        longest_match:=pdir_length;
        while path^ in ['/','\'] do
          Inc(path);
        fd:=I+3;
        FreeMem(relfd_path);
        relfd_path:=GetMem(StrLen(path)+1);
        Move(path^,relfd_path^,StrLen(path)+1);
        InOutRes:=0;
        ConvertToFdRelativePath:=true;
      end;
    end;
  end
  else
  begin
    { path is relative to a current directory }
    if (drive_nr>=drives_count) or (current_dirs[drive_nr]=nil) then
    begin
      InOutRes:=15;
      ConvertToFdRelativePath:=false;
      exit;
    end;
    fd:=current_dir_fds[drive_nr];
    relfd_path:=GetMem(1+StrLen(path));
    Move(path^,relfd_path^,1+StrLen(path));
    ConvertToFdRelativePath:=true;
  end;
end;

procedure Setup_PreopenedDirs;
var
  fd: __wasi_fd_t;
  prestat: __wasi_prestat_t;
  res: __wasi_errno_t;
  prestat_dir_name: PChar;
  drive_nr: longint;
begin
  preopened_dirs_count:=0;
  preopened_dirs:=nil;
  drives_count:=0;
  current_dirs:=nil;
  current_dir_fds:=nil;
  current_drive:=0;
  fd:=3;
  repeat
    res:=__wasi_fd_prestat_get(fd, @prestat);
    if res=__WASI_ERRNO_SUCCESS then
    begin
      if (prestat.tag=__WASI_PREOPENTYPE_DIR) and (prestat.u.dir.pr_name_len>0) then
      begin
        GetMem(prestat_dir_name,prestat.u.dir.pr_name_len+1);
        if __wasi_fd_prestat_dir_name(fd,PByte(prestat_dir_name),prestat.u.dir.pr_name_len)=__WASI_ERRNO_SUCCESS then
        begin
          prestat_dir_name[prestat.u.dir.pr_name_len]:=#0;
          Inc(preopened_dirs_count);
          if preopened_dirs=nil then
            preopened_dirs:=AllocMem(preopened_dirs_count*SizeOf(PChar))
          else
            ReAllocMem(preopened_dirs, preopened_dirs_count*SizeOf(PChar));
          preopened_dirs[preopened_dirs_count-1]:=prestat_dir_name;
          if HasDriveLetter(prestat_dir_name) then
            drive_nr:=Ord(UpCase(prestat_dir_name[0]))-(Ord('A')-1)
          else
            drive_nr:=0;
          if (drive_nr+1)>drives_count then
          begin
            drives_count:=drive_nr+1;
            if current_dirs=nil then
            begin
              current_dirs:=AllocMem(drives_count*SizeOf(PChar));
              current_dir_fds:=AllocMem(drives_count*SizeOf(longint));
            end
            else
            begin
              ReAllocMem(current_dirs,drives_count*SizeOf(PChar));
              ReAllocMem(current_dir_fds,drives_count*SizeOf(longint));
            end;
          end;
          if current_dirs[drive_nr]=nil then
          begin
            current_dirs[drive_nr]:=GetMem(1+StrLen(prestat_dir_name));
            Move(prestat_dir_name^,current_dirs[drive_nr]^,StrLen(prestat_dir_name)+1);
            current_dir_fds[drive_nr]:=fd;
          end;
        end
        else
          FreeMem(prestat_dir_name,prestat.u.dir.pr_name_len+1);
      end;
    end;
    Inc(fd);
  until res<>__WASI_ERRNO_SUCCESS;
  while (current_drive<drives_count) and (current_dirs[current_drive]=nil) do
    Inc(current_drive);
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
