{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2009 by Sven Barth

    FPC Pascal system unit for the WinNT API.

    See the file COPYING.FPC, included in this distribution,
    for details about the copyright.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

 **********************************************************************}
unit System;
interface

{$define FPC_IS_SYSTEM}
{$ifdef SYSTEMDEBUG}
  {$define SYSTEMEXCEPTIONDEBUG}
{$endif SYSTEMDEBUG}

{$ifdef cpui386}
  {$define Set_i386_Exception_handler}
{$endif cpui386}

{$define DISABLE_NO_THREAD_MANAGER}

{$ifdef KMODE}
  // in KernelMode we need use a memory manager that just wraps the routines
  // provided by the NT Executive and allows to select whether we want to use
  // paged or non-paged (use sparely!) memory
  {$define HAS_MEMORYMANAGER}
{$endif KMODE}

{ include system-independent routine headers }
{$I systemh.inc}

var
  CurrentPeb: Pointer;
  IsDeviceDriver: Boolean = False;

const
 LineEnding = #13#10;
 LFNSupport = true;
 DirectorySeparator = '\';
 DriveSeparator = '\';
 ExtensionSeparator = '.';
 PathSeparator = ';';
 AllowDirectorySeparators : set of char = ['\'];
 AllowDriveSeparators : set of char = [];

{ FileNameCaseSensitive and FileNameCasePreserving are defined separately below!!! }
 maxExitCode = High(ErrorCode);
 MaxPathLen = High(Word);
 AllFilesMask = '*';

type
   PEXCEPTION_FRAME = ^TEXCEPTION_FRAME;
   TEXCEPTION_FRAME = record
     next : PEXCEPTION_FRAME;
     handler : pointer;
   end;

var
{ C compatible arguments }
  argc: LongWord;
  argvw: PPWideChar;
  argv: PPChar;

const
{ Default filehandles }
  UnusedHandle    : THandle = 0;
  StdInputHandle  : THandle = 0;
  StdOutputHandle : THandle = 0;
  StdErrorHandle  : THandle = 0;

{$ifndef kmode}
type
  TDLL_Entry_Hook = procedure (dllparam : longint);

const
  Dll_Process_Detach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Attach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Detach_Hook : TDLL_Entry_Hook = nil;
{$endif}

const
  // NT is case sensitive
  FileNameCaseSensitive : boolean = true;
  FileNameCasePreserving: boolean = true;
  // todo: check whether this is really the case on NT
  CtrlZMarksEOF: boolean = true; (* #26 is considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

  System_exception_frame : PEXCEPTION_FRAME =nil;

implementation

{ include system independent routines }
{$I system.inc}

function fpc_pwidechar_length(p: PWideChar): SizeInt; external name 'FPC_PWIDECHAR_LENGTH';

{ based on setup_arguments from Win32 RTL }
procedure setup_arguments;
var
  i,len,
  arglen,
  count   : longint;
  argstart,
  pc,arg  : pwidechar;
  pc2     : pchar;
  quote   : Boolean;
  argvlen : longint;
  params  : PRTLUserProcessParameters;

  procedure allocarg(idx,len:longint);
    var
      oldargvlen : longint;
    begin
      if idx>=argvlen then
       begin
         oldargvlen:=argvlen;
         argvlen:=(idx+8) and (not 7);
         sysreallocmem(argvw,argvlen*sizeof(pointer));
         fillchar(argvw[oldargvlen],(argvlen-oldargvlen)*sizeof(pointer),0);
       end;
      { use realloc to reuse already existing memory }
      { always allocate, even if length is zero, since }
      { the arg. is still present!                     }
      sysreallocmem(argvw[idx],len*sizeof(widechar)+2);
    end;

begin
  { create commandline, it starts with the executed filename which is argvw[0] }
  { NativeNT passes inside the PEB which is passed on startup }
  argvw:=nil;
  argv:=nil;
  argvlen:=0;
  params:=PSimplePEB(CurrentPEB)^.ProcessParameters;
  ArgLen:=params^.ImagePathName.Length + 1;
  allocarg(0,arglen);
  move(params^.ImagePathName.Buffer^,argvw[0]^,arglen*sizeof(widechar)+1);
  { Setup cmdline variable }
  { cmdline is a PChar, but NT uses PWideChar... don't set cmdline for now }
  {$message warning 'cmdline is not set'}
//  cmdline:=GetCommandLine;
  { the first argument isn't the image file name, so start at 1 }
  count:=1;
  { process arguments }
  pc:=params^.CommandLine.Buffer;
  while pc^<>#0 do
   begin
     { skip leading spaces }
     while (Ord(pc^) >= 1) and (Ord(pc^) <= 32) {pc^ in [#1..#32]} do
      inc(pc);
     if pc^=#0 then
      break;
     { calc argument length }
     quote:=False;
     argstart:=pc;
     arglen:=0;
     while pc^<>#0 do
      begin
        case pc^ of
          #1..#32 :
            begin
              if quote then
               inc(arglen)
              else
               break;
            end;
          '"' :
            if pc[1]<>'"' then
              quote := not quote
              else
              inc(pc);
          else
            inc(arglen);
        end;
        inc(pc);
      end;
     { copy argument }
     { Don't copy the first one, it is already there.}
     If Count<>0 then
      begin
        allocarg(count,arglen);
        quote:=False;
        pc:=argstart;
        arg:=argvw[count];
        while (pc^<>#0) do
         begin
           case pc^ of
             #1..#32 :
               begin
                 if quote then
                  begin
                    arg^:=pc^;
                    inc(arg);
                  end
                 else
                  break;
               end;
             '"' :
               if pc[1]<>'"' then
                 quote := not quote
                  else
                inc(pc);
             else
               begin
                 arg^:=pc^;
                 inc(arg);
               end;
           end;
           inc(pc);
         end;
        arg^:=#0;
      end;
     inc(count);
   end;
  { get argc }
  argc:=count;
  { free unused memory, leaving a nil entry at the end }
  sysreallocmem(argvw,(count+1)*sizeof(pointer));
  argvw[count] := nil;
  { now we need to fill argv with UTF8 encoded arguments }
  sysreallocmem(argv,(count+1)*sizeof(pointer));
  fillchar(argv^,(count+1)*sizeof(pointer),0);
  for i := 0 to count - 1 do begin
    len := fpc_pwidechar_length(argvw[i]);
    pc := argvw[i];
    argv[i]:=nil;
    sysreallocmem(argv[i],len+1);
    pc2 := argv[i];
    {$message warning 'Use UnicodeToUTF8 for argument conversion'}
    while Ord(pc^) > 0  do begin
      if word(pc^) < 127 then
        pc2^ := Char(word(pc^))
      else
        pc2^ := '?';
      Inc(pc);
      Inc(pc2);
    end;
    pc2^ := #0;
  end;
end;

function paramcount : longint;
begin
  paramcount := argc - 1;
end;

function paramstr(l : longint) : string;
begin
  if (l>=0) and (l<argc) then
    paramstr:=strpas(argv[l])
  else
    paramstr:='';
end;

procedure KeQueryTickCount(TickCount: PLargeInteger); stdcall; external ntdll name 'KeQueryTickCount';

procedure randomize;
var
  tc: PLargeInteger;
begin
  FillChar(tc, SizeOf(TLargeInteger), 0);
  KeQueryTickCount(@tc);
  // the lower part should differ most on system startup
  randseed := tc^.LowPart;
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

procedure PascalMain;external name 'PASCALMAIN';

{$ifndef KMODE}
function NtTerminateProcess(aProcess: THandle; aStatus: LongInt): LongInt; stdcall; external ntdll name 'NtTerminateProcess';
{$endif KMODE}

Procedure system_exit;
begin
  if IsLibrary or IsDeviceDriver then
    Exit;
{$ifndef KMODE}
  NtTerminateProcess(THandle(-1), ExitCode);
{$endif KMODE}
end;

{$ifdef kmode}
function FPCDriverStartup(aDriverObject: Pointer; aRegistryPath: Pointer): LongInt; [public, alias: 'FPC_DriverStartup'];
begin
  IsDeviceDriver := True;
  IsConsole := True;
  IsLibrary := True;

  SysDriverObject := aDriverObject;
  SysRegistryPath := aRegistryPath;

  PASCALMAIN;

  SysDriverObject := Nil;
  SysRegistryPath := Nil;

  Result := ExitCode;
end;
{$else}

const
   DLL_PROCESS_ATTACH = 1;
   DLL_THREAD_ATTACH = 2;
   DLL_PROCESS_DETACH = 0;
   DLL_THREAD_DETACH = 3;

function FPCDLLEntry(aHInstance: Pointer; aDLLReason: LongInt; aDLLParam: LongInt): LongBool; [public, alias: 'FPC_DLLEntry'];
begin
  IsLibrary := True;
  FPCDLLEntry := True;
  case aDLLReason of
    DLL_PROCESS_ATTACH: begin
      PascalMain;
      FPCDLLEntry := ExitCode = 0;
    end;
    DLL_THREAD_ATTACH: begin
      if Dll_Thread_Attach_Hook <> Nil then
        Dll_Thread_Attach_Hook(aDllParam);
    end;
    DLL_THREAD_DETACH: begin
      if Dll_Thread_Detach_Hook <> Nil then
        Dll_Thread_Detach_Hook(aDllParam);
    end;
    DLL_PROCESS_DETACH: begin
      if Dll_Process_Detach_Hook <> Nil then
        Dll_Process_Detach_Hook(aDllParam);
      // finalize units
      internal_do_exit;
    end;
  end;
end;

procedure FPCProcessStartup(aArgument: Pointer);[public, alias: 'FPC_ProcessStartup'];
begin
  IsConsole := True;
  IsLibrary := False;
  CurrentPeb := aArgument;

  PASCALMAIN;

  system_exit;
end;
{$endif}

{$ifdef kmode}

// Kernel Mode Entry Point

function NtDriverEntry( aDriverObject: Pointer; aRegistryPath: Pointer ): LongInt; stdcall; [public, alias: '_NtDriverEntry'];
begin
  NtDriverEntry := FPCDriverStartup(aDriverObject, aRegistryPath);
end;
{$else}

// User Mode Entry Points

procedure NtProcessStartup( aArgument: Pointer ); stdcall; [public, alias: '_NtProcessStartup'];
begin
  FPCProcessStartup(aArgument);
end;

function DLLMainStartup( aHInstance: Pointer; aDLLReason, aDLLParam: LongInt ): LongBool; stdcall; [public, alias: '_DLLMainStartup'];
begin
  DLLMainStartup := FPCDLLEntry(aHInstance, aDLLReason, aDLLParam);
end;
{$endif}

procedure SysInitStdIO;
begin
  { This function is currently only called if the RTL is compiled for Usermode;
    one could think about adding a text driver that outputs using DbgPrint }
{$ifndef KMODE}
  with PSimplePEB(CurrentPEB)^.ProcessParameters^ do begin
    StdInputHandle := StandardInput;
    StdOutputHandle := StandardOutput;
    StdErrorHandle := StandardError;
  end;
  if StdInputHandle <> 0 then
    OpenStdIO(Input, fmInput, StdInputHandle)
  else
    Assign(Input, '');
  if StdOutputHandle <> 0 then begin
    OpenStdIO(Output, fmOutput, StdOutputHandle);
    OpenStdIO(StdOut, fmOutput, StdOutputHandle);
  end else begin
    Assign(Output, '');
    Assign(StdOut, '');
  end;
  if StdErrorHandle <> 0 then begin
    OpenStdIO(ErrOutput, fmOutput, StdErrorHandle);
    OpenStdIO(StdErr, fmOutput, StdErrorHandle);
  end else begin
    Assign(ErrOutput, '');
    Assign(StdErr, '');
  end;
{$endif}
end;

function GetProcessID: SizeUInt;
begin
{$ifdef kmode}
  // it might be that we can detect the user process that called us,
  // but that needs to be checked... so for now just return 0
  Result := 0;
{$else}
  Result := NtCurrentTEB^.ClientID.UniqueProcess;
{$endif}
end;

begin
{$if not defined(KMODE) and not defined(HAS_MEMORYMANAGER)}
  { Setup heap }
  InitHeap;
{$endif ndef KMODE and ndef HAS_MEMORYMANAGER}
  SysInitExceptions;
  { we do not use winlike widestrings and also the RTL can't be compiled with
    2.2, so we can savely use the UnicodeString manager only. }
  initunicodestringmanager;
{$ifndef KMODE}
  SysInitStdIO;
  { Arguments }
  setup_arguments;
{$endif}
  InOutRes := 0;
  InitSystemThreads;
  errno := 0;
end.

