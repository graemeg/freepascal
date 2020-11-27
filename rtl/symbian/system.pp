{
    This file is part of the Free Pascal run time library.
    Copyright (c) 2007 by contributors of the Free Pascal Compiler

    Pascal system unit for the Symbian OS

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

{ include system-independent routine headers }
{$I systemh.inc}

const
  LineEnding = #13#10;
  LFNSupport = true;
  DirectorySeparator = '\';
  DriveSeparator = ':';
  ExtensionSeparator = '.';
  PathSeparator = ';';
  AllowDirectorySeparators : set of char = ['\','/'];
  AllowDriveSeparators : set of char = [':'];
  { FileNameCaseSensitive and FileNameCasePreserving are defined separately below }
  maxExitCode = 65535;
  MaxPathLen = 260;
  AllFilesMask = '*';

type
   PEXCEPTION_FRAME = ^TEXCEPTION_FRAME;
   TEXCEPTION_FRAME = record
     next : PEXCEPTION_FRAME;
     handler : pointer;
   end;

const
{ Default filehandles }
  UnusedHandle    : THandle = -1;
  StdInputHandle  : THandle = 0;
  StdOutputHandle : THandle = 0;
  StdErrorHandle  : THandle = 0;

  FileNameCaseSensitive : boolean = false;
  FileNameCasePreserving: boolean = true;
  CtrlZMarksEOF: boolean = true; (* #26 not considered as end of file *)

  sLineBreak = LineEnding;
  DefaultTextLineBreakStyle : TTextLineBreakStyle = tlbsCRLF;

  System_exception_frame : PEXCEPTION_FRAME =nil;

type
  TStartupInfo=packed record
    cb : longint;
    lpReserved : Pointer;
    lpDesktop : Pointer;
    lpTitle : Pointer;
    dwX : longint;
    dwY : longint;
    dwXSize : longint;
    dwYSize : longint;
    dwXCountChars : longint;
    dwYCountChars : longint;
    dwFillAttribute : longint;
    dwFlags : longint;
    wShowWindow : Word;
    cbReserved2 : Word;
    lpReserved2 : Pointer;
    hStdInput : longint;
    hStdOutput : longint;
    hStdError : longint;
  end;

var
{ C compatible arguments }
  argc : longint;
  argv : ppchar;
{ Win32 Info }
  startupinfo : tstartupinfo;
  hprevinst,
  MainInstance,
  cmdshow     : longint;
  DLLreason,DLLparam:longint;
type
  TDLL_Process_Entry_Hook = function (dllparam : longint) : longbool;
  TDLL_Entry_Hook = procedure (dllparam : longint);

const
  Dll_Process_Attach_Hook : TDLL_Process_Entry_Hook = nil;
  Dll_Process_Detach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Attach_Hook : TDLL_Entry_Hook = nil;
  Dll_Thread_Detach_Hook : TDLL_Entry_Hook = nil;

implementation

var
  SysInstance: Longint; public name '_FPC_SysInstance';

{ include system independent routines }
{$I system.inc}

{*****************************************************************************
                              Minimum Symbian API declarations
*****************************************************************************}
const KErrNone=0;

{*****************************************************************************
                              Parameter Handling
*****************************************************************************}

var
  ModuleName : array[0..255] of char;

function GetCommandFile:pchar;
begin

end;


procedure setup_arguments;
begin

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


procedure randomize;
begin
//  randseed:=GetTickCount;
end;

{*****************************************************************************
                         System Dependent Exit code
*****************************************************************************}

//procedure PascalMain; external name 'PASCALMAIN';
//procedure fpc_do_exit; stdcall; external name 'FPC_DO_EXIT';

Procedure system_exit;
begin

end;

var
  { value of the stack segment
    to check if the call stack can be written on exceptions }
  _SS : Cardinal;

procedure pascalmain; external name 'PASCALMAIN';

{ Entry point for the pascal code }
function Pascal_E32Main: Integer; cdecl; [public, alias: '_Pascal_E32Main'];
var
  ST : pointer;
begin
  PascalMain;

  { if we pass here there was no error }
  system_exit;

  Result := KErrNone;
end;

procedure SysInitStdIO;
begin

end;

(* ProcessID cached to avoid repeated calls to GetCurrentProcess. *)

var
  ProcessID: SizeUInt;

function GetProcessID: SizeUInt;
begin
  GetProcessID := ProcessID;
end;

function CheckInitialStkLen(stklen : SizeUInt) : SizeUInt;
begin
  result := stklen;
end;

{
const
   Exe_entry_code : pointer = @Exe_entry;
   Dll_entry_code : pointer = @Dll_entry;
}

begin
end.
